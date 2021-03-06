#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netlink.h>
#include <linux/timer.h>
#include <linux/export.h>
#include <net/genetlink.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/skbuff.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/spinlock.h>

#include "genl_ex.h"
#include "hcsr04_structures.h"

#define ALERT
#define DEBUG
static DEFINE_SPINLOCK(sp_lock_irq);

#if defined(ALERT)
    #define DALERT(fmt, args...) printk(KERN_ALERT "INFO:"fmt, ##args)
#else
    #define DALERT(fmt, args...) 
#endif

#if defined(DEBUG)
    #define DPRINTK(fmt, args...) printk(""fmt, ##args)
#else
    #define DPRINTK(fmt, args...) 
#endif

#define HCSR_DEVICE_NAME "HCSR04"
#define LED_DEVICE_NAME "MAX7219"

/*Struct to keep track of the netlink family properties*/
static struct genl_family genl_netlink_family;

static struct spi_device_id spi_deviceId_table[]={
    {"MAX7219", 0}
};

struct hcsr04_dev *hcsr04_devp;

int configureDotMatrix(void);
static void send_message_to_user_processes(unsigned int group);
static void send_distance(unsigned int group, int distance);
static void terminate_operation(unsigned int group);
static void send_pattern(uint16_t *pattern);

/*Function to keep the most recently measured data*/
void writeToCircularBuffer(long distance, long long timestamp, struct hcsr04_dev *hcsr04_devp){
    DPRINTK("Writing into the circular buffer \n");
    hcsr04_devp->list = addNode(hcsr04_devp->list, initNode(distance, timestamp));
    // hcsr04_devp->isEnabled = 0;
    // mutex_unlock(&hcsr04_devp->sampleRunning);
}

/*Function to clear Buffer*/
void clearBuffer(struct hcsr04_dev *hcsr04_devp){
    /*
    *  Initializing the list to null and the number of samples measured to 0
    */
    DPRINTK("Clearing the current Buffer \n");
    hcsr04_devp->list = NULL;
    resetBuffer();
}

/*Function to trigger 8 40kHz pulse from trigger pin */
void triggerPulse(struct hcsr04_dev *hcsr04_devp){
    gpio_set_value_cansleep(hcsr04_devp->trigger_pin.gpio_pin,0);
    udelay(5);
	gpio_set_value_cansleep(hcsr04_devp->trigger_pin.gpio_pin,1);
	udelay(10);
	gpio_set_value_cansleep(hcsr04_devp->trigger_pin.gpio_pin,0);
}

/*Function to perform the distance measurement operation for the given criteria*/
void measure_distance(struct work_struct *hcsr04_wq){
    int numIterations=0;
    for(numIterations=0; numIterations < hcsr04_devp->timeToStopMeasurment; numIterations++){
       
        int i = 0;
        int sum = 0;
        long long time = 0;
        long instance_of_sample = 0;
        long divisor = 58;
        long outlier_min = LONG_MAX;
        long outlier_max = LONG_MIN;
        int average_distance;

        struct work_queue_hcsr *wq = container_of(hcsr04_wq, struct work_queue_hcsr, work);
        struct hcsr04_dev *hcsr04_devp = wq->parameter;
        //  mutex_lock(&hcsr04_devp->lock);
        for(i=0;i<hcsr04_devp->num_samples_per_measurement+2;i++){
            /*Trigger pulse to take nth Measurement*/
            triggerPulse(hcsr04_devp);
            /*Sleeping for 65ms to receive the echo pulse(5ms extra compared to the documentation)*/
            msleep(65);

            /*Check if the trigger and echo event occured. If not discard the sample*/
            if(hcsr04_devp->trig_time == 0 || hcsr04_devp->echo_time == 0){
                DALERT("Echo wasn't recieved. Skipping the sample \n");
                hcsr04_devp->trig_time = 0;
                hcsr04_devp->echo_time = 0;
                mdelay(hcsr04_devp->sampling_period);
                // mutex_unlock(&hcsr04_devp->lock);
                continue;
            }
            
            /*
            *  Finding the time Difference between the clock cycles.
            *   Multiplying it with 10^6 to find the result in uS.
            *   Dividing it with the max frequency of the board. 4 * 10^6
            *   Dividing the uS with 58 to find the result in cm as per the documentation.
            */
            time = hcsr04_devp->echo_time - hcsr04_devp->trig_time;
            if(time < 0){
                /*Possibility of some stale echo trigger*/
                DALERT("Not a valid echo value. Skipping the sample \n");
                hcsr04_devp->trig_time = 0;
                hcsr04_devp->echo_time = 0;
                mdelay(hcsr04_devp->sampling_period);
                // mutex_unlock(&hcsr04_devp->lock);
                continue;
            }
            DPRINTK("Time Diff %lld \t ", time);
            instance_of_sample = div_u64(time,400);;
            instance_of_sample = div_u64(instance_of_sample, divisor);
            DPRINTK("Distance#%d: %ld \n", i, instance_of_sample);
            sum = sum + instance_of_sample;
            
            /* Finding the min-distance instance*/
            if(instance_of_sample < outlier_min){
                outlier_min = instance_of_sample;
            }

            /* Finding the max-distance instance*/
            if(instance_of_sample > outlier_max){
                outlier_max = instance_of_sample;
            }

            hcsr04_devp->trig_time = 0;
            hcsr04_devp->echo_time = 0;
            mdelay(hcsr04_devp->sampling_period);
        }

        sum = sum - outlier_min - outlier_max;
        DALERT("#SUM: %d \t\t", sum);
        sum = div_u64(sum, hcsr04_devp->num_samples_per_measurement);
        average_distance = sum;
        DALERT("#AVERAGE_DISTANCE: %d \n", average_distance);
        send_distance(GROUP0, average_distance); /*Relaying back to the user*/
    }
    terminate_operation(GROUP0); /*Termination command to indicate that measurement is done */
}


/*Function to spawn k-thread*/
static int spawn_thread_distance_measurement(struct hcsr04_dev *hcsr04_devp){
    /*Initialize the work and assign the requried parameterstruct hcsr04_dev *hcsr04_devps*/
    INIT_WORK(&hcsr04_devp->test_hcsr_wq->work, measure_distance);
    hcsr04_devp->test_hcsr_wq->parameter = hcsr04_devp;
    DALERT("About to spawn thread \n");
    schedule_work(&hcsr04_devp->test_hcsr_wq->work);
    hcsr04_devp->isWorkInitialized=1;

    return 0;
}

/*Function to pull out the read value from the buffer*/
void removeReadMeasurement(struct hcsr04_dev *hcsr04_devp){
    hcsr04_devp->list = deleteHead(hcsr04_devp->list);
}

/*Function to change th sampling period and number of sampels*/
int configure_measurement_parameters(setparam_input *input, struct hcsr04_dev *fileData){
    DPRINTK("Configuring Measurment Parameters: Num Samples: %d Period: %d \n", input->num_samples,input->sampling_period);

    if(input->sampling_period < 60){
        DPRINTK("The sampling frequency must be greater than 60 \n");
        return -EINVAL;
    }
    fileData->num_samples_per_measurement = input->num_samples;
    fileData->sampling_period = input->sampling_period;

    return 0;
}

/*
* Check for the validity of the given gpio by the user
* gpio is the MUX, pinType --> 0 for echo_pin and 1 for trigger_pin
*/
bool isValidPin(int pinNum, int isTriggerPin){
    /*
    * Chekc if the pins are greater than -1 and less than or equal to 13.
    * Also it must not be 7th or 8th pin 
    */
    if(isTriggerPin){
        if(pinNum > 0 && pinNum != 7 && pinNum != 8 && pinNum <=13 && validTriggerPins[pinNum]){
            return true;
        }
    }else{
        if(pinNum > 0 && pinNum != 7 && pinNum != 8 && pinNum <=13 && validEchoPins[pinNum]){
            return true;
        }
    }

    return false;
}

/* A Callback function to handle triggered Interrupts */
static irq_handler_t handleEchoTriggerRisingAndFalling(unsigned int irq, void *dev_id){
    struct hcsr04_dev *fileData = (struct hcsr04_dev *)dev_id;
    unsigned long flags;
    spin_lock_irqsave(&sp_lock_irq, flags);
    if(gpio_get_value(fileData->echo_pin.gpio_pin)){
        fileData->trig_time = native_read_tsc();
    }else{
        fileData->echo_time = native_read_tsc();
    }
    spin_unlock_irqrestore(&sp_lock_irq, flags);
    return (irq_handler_t) IRQ_HANDLED;
}

/*Configuring Interrupt Request Handler to keep track of edge changes in the echo pin*/
int configure_irq_handler(struct hcsr04_dev *fileData){
    fileData->irq_number = gpio_to_irq(fileData->echo_pin.gpio_pin);
    
    DPRINTK("Configuring IRQ for pin: %d\n", fileData->echo_pin.gpio_pin);
    if(fileData->irq_number < 0){
        DALERT("Configuring interrup failed \n");
        return -EINVAL;
    }

    if(request_irq(fileData->irq_number, (irq_handler_t) handleEchoTriggerRisingAndFalling, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, fileData->name, (void *) fileData) < 0){
        DALERT("Registering IRQ failed \n");
        return -EINVAL;
    }
    DPRINTK("IRQ Configuration successful \n");

    return 0;
}

/*Unregistering the gpio pins*/
void unregister_pin(struct hcsr04_dev *fileData, int select){
    switch (select)
    {
    case 0:
        free_irq(fileData->irq_number, fileData);
        if(fileData->echo_pin.gpio_pin >= 0){
            gpio_free(fileData->echo_pin.gpio_pin);
        }
        if(fileData->echo_pin.mux1 >= 0){
            gpio_free(fileData->echo_pin.mux1);
        }
        if(fileData->echo_pin.mux2 >= 0){
            gpio_free(fileData->echo_pin.mux2);
        }
        if(fileData->echo_pin.pull >= 0){
            gpio_free(fileData->echo_pin.pull);
        }
        if(fileData->echo_pin.shift_pin >= 0){
            gpio_free(fileData->echo_pin.shift_pin);
        }
        break;
    case 1:
        if(fileData->trigger_pin.gpio_pin >= 0){
            gpio_free(fileData->trigger_pin.gpio_pin);
        }
        if(fileData->trigger_pin.mux1 >= 0){
            gpio_free(fileData->trigger_pin.mux1);
        }
        if(fileData->trigger_pin.mux2 >= 0){
            gpio_free(fileData->trigger_pin.mux2);
        }
        if(fileData->trigger_pin.pull >= 0){
            gpio_free(fileData->trigger_pin.pull);
        }
        if(fileData->trigger_pin.shift_pin >= 0){
            gpio_free(fileData->trigger_pin.shift_pin);
        }
        break;
    case 2:
        if(fileData->chip_select_pin.gpio_pin >= 0){
            gpio_free(fileData->chip_select_pin.gpio_pin);
        }
        if(fileData->chip_select_pin.mux1 >= 0){
            gpio_free(fileData->chip_select_pin.mux1);
        }
        if(fileData->chip_select_pin.mux2 >= 0){
            gpio_free(fileData->chip_select_pin.mux2);
        }
        if(fileData->chip_select_pin.pull >= 0){
            gpio_free(fileData->chip_select_pin.pull);
        }
        if(fileData->chip_select_pin.shift_pin >= 0){
            gpio_free(fileData->chip_select_pin.shift_pin);
        }
        break;
    default:

        break;
    }
}

/*Handling the registration of gpio pins*/
int configure_pin_mux(int gpio, struct hcsr04_dev *fileData, int isTrigger){
 
    int ret;
    int gpio_pin = lookupTable[0][gpio];
    int shift_pin = lookupTable[1][gpio];
    int pull_pin = lookupTable[2][gpio];
    int mux_pin = lookupTable[3][gpio];

    char pin_var[20];

    if(isTrigger){
        if(fileData->trigger_pin.digitalPin == gpio){
            DALERT("Requested PIN already set as trigger for this specific device \n");
            return 0;
        }

        if(fileData->trigger_pin.digitalPin > -1){
            unregister_pin(fileData, 1);
        }

        if(gpio_pin >= 0){
            sprintf(pin_var, "%s%d", "gpio", gpio_pin);
            if(gpio_request(gpio_pin, pin_var)){
                DPRINTK("Unable to register pin: %d \n", gpio_pin);
                return -EINVAL;
            }
            DPRINTK("GPIO %d got configured \n", gpio_pin);
            
            ret = gpio_direction_output(gpio_pin, 0);
            if(ret){
                DPRINTK("Unable to set as Output pin (trigger pin)");
                return -EINVAL;
            }
        }

        if(shift_pin >= 0){
            sprintf(pin_var, "%s%d", "direction", shift_pin);
            if(gpio_request(shift_pin, pin_var)){
                DPRINTK("Unable to register pin: %d\n", shift_pin);
                return -EINVAL;
            }
            DPRINTK("SHIFT %d got configured %s\n", shift_pin, pin_var);

            ret = gpio_direction_output(shift_pin, 0);
            if(ret){
                DPRINTK("Unable to set as Output pin (trigger pin)");
                return -EINVAL;
            }
        }

        if(mux_pin >= 0){
            sprintf(pin_var, "%s%d", "mux", mux_pin);
            if(gpio_request(mux_pin, pin_var)){
                DPRINTK("Unable to register pin: %d \n", mux_pin);
                return -EINVAL;
            }
            DPRINTK("MUX %d got configured \n", mux_pin);
            gpio_set_value_cansleep(mux_pin, 0);
            if(mux_pin == 76){
                if(gpio_request(64, pin_var)){
                    DPRINTK("Unable to register pin: 64 \n");
                    return -EINVAL;
                }
                DPRINTK("GPIO 64 got configured \n");
                gpio_set_value_cansleep(76, 0);
                fileData->trigger_pin.mux2 = 64;
            }else if(mux_pin == 44){
                if(gpio_request(72, pin_var)){
                    DPRINTK("Unable to register pin: 72 \n");
                    return -EINVAL;
                }
                DPRINTK("GPIO 72 got configured \n");
                gpio_set_value_cansleep(72, 0);
                fileData->trigger_pin.mux2 = 72;
            }else{
                fileData->trigger_pin.mux2 = -1;
            }
        }
    }else{
        if(fileData->echo_pin.digitalPin == gpio){
            DALERT("Requested PIN already set as echo for this specific device \n");
            return 0;
        }

        if(fileData->echo_pin.digitalPin > -1){
            unregister_pin(fileData, 0);
        }

        if(gpio_pin >= 0){
            sprintf(pin_var, "%s%d", "gpio", gpio_pin);
            if(gpio_request(gpio_pin, pin_var)){
                DPRINTK("Unable to register pin: %d \n", gpio_pin);
                return -EINVAL;
            }
            DPRINTK("GPIO %d got configured \n", gpio_pin);
        }
        ret = gpio_direction_input(gpio_pin);
        
        if(ret){
            DPRINTK("Unable to set as Input pin");
            return -EINVAL;
        }
        DPRINTK("Setting gpio: %d as Input\n",gpio_pin);

        if(shift_pin >= 0){
            sprintf(pin_var, "%s%d", "direction", shift_pin);
            if(gpio_request(shift_pin, pin_var)){
                DPRINTK("Unable to register pin: %d \n", shift_pin);
                return -EINVAL;
            }
            DPRINTK("SHIFT %d got configured \n", shift_pin);
            gpio_set_value_cansleep(shift_pin,1);
        }

        if(pull_pin >= 0){
            sprintf(pin_var, "%s%d", "pull", pull_pin);
            if(gpio_request(pull_pin, pin_var)){
                DPRINTK("Unable to register pin: %d \n", pull_pin);
                return -EINVAL;
            }
            gpio_set_value_cansleep(pull_pin, 0);
            DPRINTK("PULL %d got configured \n", pull_pin);
        }

        if(mux_pin >= 0){
            sprintf(pin_var, "%s%d", "mux", mux_pin);
            if(gpio_request(mux_pin, pin_var)){
                DPRINTK("Unable to register pin: %d \n", mux_pin);
                return -EINVAL;
            }
            DPRINTK("MUX %d got configured \n", mux_pin);
            gpio_set_value_cansleep(mux_pin, 0);
            
            if(mux_pin == 76){
                if(gpio_request(64, pin_var)){
                    DPRINTK("Unable to register pin: 64 \n");
                    return -EINVAL;
                }
                DPRINTK("MUX 64 got configured \n");
                gpio_set_value_cansleep(76, 0);

                fileData->echo_pin.mux2 = 64;
            }else if(mux_pin == 44){
                if(gpio_request(72, pin_var)){
                    DPRINTK("Unable to register pin: 72 \n");
                    return -EINVAL;
                }
                DPRINTK("MUX 72 got configured \n");
                gpio_set_value_cansleep(72, 0);
                fileData->echo_pin.mux2 = 72;
            }else{
                fileData->echo_pin.mux2 = -1;
            }
        }

        /*Configuring IRQ to the echo PIN*/
        if(configure_irq_handler(fileData)){
            DPRINTK("Interrupt Configuration failed");

            return -EINVAL; 
        }
    }
    
     if(isTrigger){
        fileData->trigger_pin.gpio_pin = gpio_pin;
        fileData->trigger_pin.mux1 = mux_pin;
        fileData->trigger_pin.pull = pull_pin;
        fileData->trigger_pin.shift_pin = shift_pin;
        fileData->trigger_pin.digitalPin = gpio;
    }else{
        fileData->echo_pin.gpio_pin = gpio_pin;
        fileData->echo_pin.mux1 = mux_pin;
        fileData->echo_pin.pull = pull_pin;
        fileData->echo_pin.shift_pin = shift_pin;
        fileData->echo_pin.digitalPin = gpio;
    }
    return 0;
}

/*Function to register gpio based on user input */
int configure_IO_pins(config_input *input,struct hcsr04_dev *fileData){
    DPRINTK("Configuring I/O Pins: Trigger:  %d  Echo: %d \n", input->trigger_pin, input->echo_pin);
    
    /*Base check to make sure both the pins are not the same since it's program triggered */
    if(input->trigger_pin == input->echo_pin){
        DALERT("Input and Output pins cannot be the same \n");
        return -EINVAL;
    }

    /* Check for validity of both the pins */
    if(isValidPin(input->trigger_pin, 1) && isValidPin(input->echo_pin, 0)){
        if(configure_pin_mux(input->echo_pin, fileData, 0)){
            DALERT("Couldn't configure echo_pin\n");
            return -EINVAL;
        }

        if(configure_pin_mux(input->trigger_pin, fileData, 1)){
            DALERT("Couldn't configure trigger_pin \n");
            return -EINVAL;
        }

        DPRINTK("Configuring echo: %d\n", fileData->echo_pin.gpio_pin);
        DPRINTK("Configuring Trigger: %d\n", fileData->trigger_pin.gpio_pin);
        return 0;
    }
    return -EINVAL;
}

/*Method to send distance to the user*/
static void send_distance(unsigned int group, int distance){
    void *header;
    int res, flags = GFP_ATOMIC;
    char msg[MAX_BUF_LENGTH];
    struct sk_buff* sk_buf = genlmsg_new(NLMSG_DEFAULT_SIZE, flags);

    if (!sk_buf) {
        DALERT("Failed to create a new generic message. \n");
        return;
    }

    header = genlmsg_put(sk_buf, 0, 0, &genl_netlink_family, flags, CONFIGURE_HCSR04);
    if (!header) {
        DALERT("Error in creating the sk_buf header \n");
        return;
    }
    snprintf(msg, MAX_BUF_LENGTH, "Sending to the group %s\n", genl_test_mcgrp_names[group]);
    nla_put_flag(sk_buf, RET_VAL_SUCCESS_ATTR);
    res = nla_put_u32(sk_buf,DISTANCE_MEASURE_ATTR, distance);
    if (res) {
        DALERT("%d: err %d ", __LINE__, res);
        return;
    }

    genlmsg_end(sk_buf, header);
    genlmsg_multicast(&genl_netlink_family, sk_buf, 0, group, flags);
    return;
}

static void terminate_operation(unsigned int group){
    void *header;
    int res, flags = GFP_ATOMIC;
    struct sk_buff* sk_buf = genlmsg_new(NLMSG_DEFAULT_SIZE, flags);

    if (!sk_buf) {
        DALERT("Failed to create a new generic message. \n");
        return;
    }

    header = genlmsg_put(sk_buf, 0, 0, &genl_netlink_family, flags, CONFIGURE_HCSR04);
    if (!header) {
        DALERT("Error in creating the sk_buf header \n");
        return;
    }
    res = nla_put_flag(sk_buf, RET_VAL_SUCCESS_ATTR);
    res = nla_put_flag(sk_buf, TERMINATE_OPERATION_ATTR);
    if (res) {
        DALERT("%d: err %d ", __LINE__, res);
        return;
    }

    genlmsg_end(sk_buf, header);
    genlmsg_multicast(&genl_netlink_family, sk_buf, 0, group, flags);
    return;
}


static void send_message_to_user_processes(unsigned int group){
    void *header;
    int res, flags = GFP_ATOMIC;
    struct sk_buff* sk_buf = genlmsg_new(NLMSG_DEFAULT_SIZE, flags);

    if (!sk_buf) {
        DALERT("Failed to create a new generic message. \n");
        return;
    }

    header = genlmsg_put(sk_buf, 0, 0, &genl_netlink_family, flags, CONFIGURE_HCSR04);
    if (!header) {
        DALERT("Error in creating the sk_buf header \n");
        goto nlmsg_fail;
    }

    res = nla_put_flag(sk_buf, RET_VAL_SUCCESS_ATTR);
    if (res) {
        DALERT("%d: err %d ", __LINE__, res);
        goto nlmsg_fail;
    }

    genlmsg_end(sk_buf, header);
    genlmsg_multicast(&genl_netlink_family, sk_buf, 0, group, flags);
    return;

nlmsg_fail:
    genlmsg_cancel(sk_buf, header);
    nlmsg_free(sk_buf);
    return;
}

static int configure_hcsr04_device (struct sk_buff* sk_buf, struct genl_info* info){
    config_input input_pins;
    setparam_input input_params;
    if(!info->attrs[HCSR04_TRIGGER_PIN_ATTR] || !info->attrs[HCSR04_ECHO_PIN_ATTR] || !info->attrs[HCSR04_SAMPLING_PERIOD_ATTR] || !info->attrs[HCSRO4_NUMBER_SAMPLES_ATTR]){
        DALERT("Empty data is sent from the user. Exiting... \n");
        return -EINVAL;
    }

    input_pins.echo_pin = nla_get_u32(info->attrs[HCSR04_ECHO_PIN_ATTR]);
    input_pins.trigger_pin =  nla_get_u32(info->attrs[HCSR04_TRIGGER_PIN_ATTR]);
    input_params.sampling_period = nla_get_u32(info->attrs[HCSR04_SAMPLING_PERIOD_ATTR]);
    input_params.num_samples = nla_get_u32(info->attrs[HCSRO4_NUMBER_SAMPLES_ATTR]);

    if(configure_IO_pins(&input_pins, hcsr04_devp) < 0){
        DALERT("Configuring HCSR04 I/O pins failed");
        return -EINVAL;
    }
    if(configure_measurement_parameters(&input_params, hcsr04_devp) < 0){
        DALERT("Configuring HCSR04 parameters failed");
        return -EINVAL;
    }
    send_message_to_user_processes(GROUP0);
    return 0;
}

static void transfer_complete(void *context){
    DALERT("The transfer is completed successfully");
}

/*Clear the matrix Display*/
static void clear_display(struct spi_device *spi){
    uint16_t pattern1[16]={0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700,0x0800};
    int i=0;
    int cs_pin = 0;
    uint16_t send[1];
    struct spi_message sp_msg;
    struct spi_transfer spi_transfer1={
        .tx_buf = send,
        .rx_buf = 0,
        .len = 2,
        .bits_per_word = 16,
        .speed_hz = 10000000,
        .delay_usecs = 1,
        .cs_change = 1,
    };
    
    cs_pin = hcsr04_devp->chip_select_pin.gpio_pin;
    spi_message_init(&sp_msg);
    sp_msg.complete=&transfer_complete;
    spi_message_add_tail((void *)&spi_transfer1, &sp_msg);

	spi_sync(spi, &sp_msg);
    for(i=0;i<8;i++){
        send[0] = pattern1[0];
        gpio_set_value(cs_pin,0);       
        spi_sync(spi, &sp_msg);
        gpio_set_value(cs_pin,1);
    }
}

/*Send pattern to the matrix led*/
void send_pattern(uint16_t *pattern){
    uint16_t send[1];
    struct spi_message sp_msg;
    int i=0;
    int cs_pin;
    struct spi_transfer spi_transfer={
        .tx_buf = send,
        .rx_buf = 0,
        .len = 2,
        .bits_per_word = 16,
        .speed_hz = 10000000,
        .delay_usecs = 1,
        .cs_change = 1,
    };
    cs_pin = hcsr04_devp->chip_select_pin.gpio_pin;
    
    spi_message_init(&sp_msg);
    sp_msg.complete=&transfer_complete;
    spi_message_add_tail((void *)&spi_transfer, &sp_msg);

	spi_sync(hcsr04_devp->found_device, &sp_msg);
    
    for(i=0;i<8;i++){
        send[0] = pattern[i];
        gpio_set_value(cs_pin,0);       
        spi_sync(hcsr04_devp->found_device, &sp_msg);
        gpio_set_value(cs_pin,1);
    }
    // mdelay(100);
    DALERT("Done sending the messages %d %d %d ", sp_msg.status, sp_msg.actual_length, sp_msg.frame_length);
    // mutex_unlock(&hcsr04_devp->lock);
}


/*Base matrix setup to configure the properties*/
static int setup_device(struct spi_device *spi){
    struct spi_message sp_msg;
    int cs_pin;
    uint16_t addr_intensity = 0x0900;
    uint16_t addr_decode = 0x0A0F;
    uint16_t addr_scan_limit = 0x0B07;
    uint16_t addr_shutdown = 0x0C01;

	uint16_t sendData[1];    		
    
    /*Setting up the base transfer operation properties*/
    struct spi_transfer spi_transfer = {
        .tx_buf = sendData,
        .rx_buf = 0,
        .len = 2,
        .bits_per_word = 16,
        .speed_hz = 10000000,
        .delay_usecs = 1,
        .cs_change = 1,
    };
    cs_pin = hcsr04_devp->chip_select_pin.gpio_pin;
    spi_message_init(&sp_msg);
    sp_msg.complete=&transfer_complete;
    spi_message_add_tail((void *)&spi_transfer, &sp_msg);
	spi_sync(spi, &sp_msg);

    /*Setting up the shutdown register to normal mode*/
    sendData[0] = addr_shutdown;
    gpio_set_value(cs_pin,0);       
    spi_sync(spi, &sp_msg);
    gpio_set_value(cs_pin,1);

    /*Setting up the address register to maximun */
    sendData[0] = addr_intensity;
    gpio_set_value(cs_pin,0);       
    spi_sync(spi, &sp_msg);
    gpio_set_value(cs_pin,1);

    /*setting up the address decode register */
    sendData[0] = addr_decode;
    gpio_set_value(cs_pin,0);       
    spi_sync(spi, &sp_msg);
    gpio_set_value(cs_pin,1);

    /*Setting up the scan limit register */
    sendData[0] = addr_scan_limit;
    gpio_set_value(cs_pin,0);       
    spi_sync(spi, &sp_msg);
    gpio_set_value(cs_pin,1);	

    /*Clearing the display*/
    clear_display(spi);

    return 0;
}

/*Configuring the CS pin*/
static int configure_chip_select(int cs_pin){
    int gpio_pin = lookupTable[0][cs_pin];
    int shift_pin = lookupTable[1][cs_pin];
    int pull_pin = lookupTable[2][cs_pin];
    int mux_pin = lookupTable[3][cs_pin];
    char pin_var[20];

    if(cs_pin == hcsr04_devp->chip_select_pin.digitalPin){
        DALERT("Required pin is already set. No changes needed. Exiting... \n");
        return 0;
    }

    if(hcsr04_devp->chip_select_pin.digitalPin > -1){
        unregister_pin(hcsr04_devp, 2);
    }

    if(shift_pin > -1){
         sprintf(pin_var, "%s%d", "gpio", shift_pin);
        if(gpio_request(shift_pin, pin_var)){
            return -EINVAL;
        }
        gpio_direction_output(shift_pin,0);
        DPRINTK("Registerd shift: %d \n", shift_pin);
    }

    if(pull_pin > -1){
         sprintf(pin_var, "%s%d", "gpio", pull_pin);
        if(gpio_request(pull_pin, pin_var)){
            return -EINVAL;
            
        }
        gpio_direction_output(pull_pin,0);
        DPRINTK("Registerd pull: %d \n", pull_pin);
    }
     if(gpio_pin > -1){
         sprintf(pin_var, "%s%d", "gpio", gpio_pin);
        if(gpio_request(gpio_pin, pin_var)){
            return -EINVAL;
        }
        gpio_direction_output(gpio_pin,0);
        DPRINTK("Registerd gpio: %d \n", gpio_pin);
    }

    if(mux_pin > -1){
        sprintf(pin_var, "%s%d", "gpio", mux_pin);
        if(gpio_request(mux_pin, "")){
            return -EINVAL;
        }
        DPRINTK("Registerd mux: %d \n", mux_pin);

        gpio_direction_output(mux_pin,0);
        if(mux_pin == 76){
                sprintf(pin_var, "%s%d", "gpio", 64);
                if(gpio_request(64, pin_var)){
                    DPRINTK("Unable to register pin: 64 \n");
                    return -EINVAL;
                }
                DPRINTK("MUX 64 got configured \n");
                gpio_direction_output(76, 0);
                hcsr04_devp->chip_select_pin.mux2 = 64;
                printk("Registerd gpio: 64 \n");
        }else if(mux_pin == 44){
                sprintf(pin_var, "%s%d", "gpio", 72);
                if(gpio_request(72, pin_var)){
                    DPRINTK("Unable to register pin: 72 \n");
                    return -EINVAL;
                }
                DPRINTK("MUX 72 got configured \n");
                gpio_direction_output(72, 0);
                hcsr04_devp->chip_select_pin.mux2 = 72;
                printk("Registerd gpio: 72 \n");
        }else{
            hcsr04_devp->chip_select_pin.mux2 = -1;
        }
    }

    
    /*Initializing the found pins*/
    hcsr04_devp->chip_select_pin.digitalPin = cs_pin;
    hcsr04_devp->chip_select_pin.gpio_pin = gpio_pin;
    hcsr04_devp->chip_select_pin.mux1 = mux_pin;
    hcsr04_devp->chip_select_pin.pull = pull_pin;
    hcsr04_devp->chip_select_pin.shift_pin = shift_pin;

    /*Setting up the found spi_device(Matrix Led)*/
    if(setup_device(hcsr04_devp->found_device)){
        DALERT("Error in setting up the device \n");
        return -EINVAL;
    }

    return 0;
}

/*Callback to configure the matrix led device */
static int configure_max7219_device (struct sk_buff* sk_buf, struct genl_info* info){
    int chip_select;
    printk("About to configure max_7219 \n");
    if(!info->attrs[MAX7219_CS_PIN_ATTR]){
        DALERT("Pin number received \n");
        return -EINVAL;
    }

    chip_select = nla_get_u32(info->attrs[MAX7219_CS_PIN_ATTR]);

    if(configure_chip_select(chip_select)){
        DALERT("Failed to configure chip select pins. \n");
        return -EINVAL;
    }
    DALERT("Chip Select Pin Configured Successfully \n");
    send_message_to_user_processes(GROUP0);
    return 0;
}

/*Callback to initiate the distance measurement*/
static int start_distance_measurement (struct sk_buff* sk_buf, struct genl_info* info){
    int iterationsToRun;
    DALERT("About to start the distance measurment \n");

    if(!info->attrs[HCSR04_SECONDS_TO_RUN_ATTR]){
        iterationsToRun = 20;  //Assigning a default value if there is no data is sent.
    }else{  
        iterationsToRun = nla_get_u32(info->attrs[HCSR04_SECONDS_TO_RUN_ATTR]);
    }

    /*Number of iterations to be taken*/
    hcsr04_devp->timeToStopMeasurment = iterationsToRun;

    /*Initializing it to the work queue*/
    spawn_thread_distance_measurement(hcsr04_devp);
    return 0;
}

/*Callaback to receive the patter from led and send back to the matrix led*/
static int send_pattern_to_matrix_led (struct sk_buff* sk_buf, struct genl_info* info){
    uint16_t *data;

    if(!info->attrs[DISPLAY_PATTERN_ATTR]){
        DALERT("No Pattern has been received \n");
        return 0;
    }

    data = (uint16_t *)nla_data(info->attrs[DISPLAY_PATTERN_ATTR]);

    send_pattern(data);
    return 0;
}

/*Setting up the general operations/commands for the netlink family*/
static const struct genl_ops genl_netlink_ops[] = {
    {
        .cmd = CONFIGURE_HCSR04,
        .policy = genl_test_policy,
        .doit = configure_hcsr04_device,
        .dumpit = NULL,
    },
    {
        .cmd = CONFIGURE_MAX7219,
        .policy = genl_test_policy,
        .doit = configure_max7219_device,
        .dumpit = NULL,
    },
    {
        .cmd = INITIATE_MEASUREMENT,
        .policy = genl_test_policy,
        .doit = start_distance_measurement,
        .dumpit = NULL,
    },
    {
        .cmd = DISPLAY_PATTERN,
        .policy = genl_test_policy,
        .doit = send_pattern_to_matrix_led,
        .dumpit = NULL,
    },
};

/*Setting up the base Clock and DIN pins */
int set_gpio_pins_for_max7219(void){
    if(gpio_request(24, "gpio24")){
        DALERT("Configuration failed \n");
        return -EINVAL;
    }
    if(gpio_request(44, "gpio44")){
        DALERT("Configuration failed \n");
        return -EINVAL;
    }
    if(gpio_request(72, "gpio72")){
        DALERT("Configuration failed \n");
        return -EINVAL;
    }
    if(gpio_request(25, "gpio25")){
        DALERT("Configuration failed \n");
        return -EINVAL;
    }
    gpio_direction_output(44,1);
    gpio_direction_output(24,0);
    gpio_direction_output(25,0);
    gpio_set_value_cansleep(72, 0);


    if(gpio_request(46, "gpio46")){
        DALERT("Configuration failed \n");
        return -EINVAL;
    }
    if(gpio_request(30, "gpio30")){
        DALERT("Configuration failed \n");
        return -EINVAL;
    }
    if(gpio_request(31, "gpio31")){
        DALERT("Configuration failed \n");
        return -EINVAL;
    }
	
    gpio_direction_output(46,1);
	gpio_direction_output(30,0);
	gpio_direction_output(31,0);
    return 0;
}

/*Decongifure the default pins of matrix led*/
void deconfigure_max7219(void){
    gpio_free(44);
    gpio_free(24);
    gpio_free(25);
    gpio_free(72);

    gpio_free(46);
    gpio_free(30);
    gpio_free(31);
}


/*Keeping track of the multicast groups */
static const struct genl_multicast_group genl_netlink_mcgrps[] = {
    [GROUP0] = { .name = MULTICAST_GROUP0, },
    [GROUP1] = { .name = MULTICAST_GROUP1, },
    [GROUP2] = { .name = MULTICAST_GROUP2, },
};


/*The primary configuration the genl_netlink_family*/
static struct genl_family genl_netlink_family = {
    .name = NETLINK_FAMILY_NAME,
    .version = 1,
    .maxattr = GENL_TEST_ATTR_MAX,
    .netnsok = false,
    .module = THIS_MODULE,
    .ops = genl_netlink_ops,
    .n_ops = ARRAY_SIZE(genl_netlink_ops),
    .mcgrps = genl_netlink_mcgrps,
    .n_mcgrps = ARRAY_SIZE(genl_netlink_mcgrps),
};


/*Initializing the found spi device*/
static int initializeDevice(void){
    /*Setting up the base gpio pins for the matrix led*/
    if(set_gpio_pins_for_max7219()){
        DALERT("Failed register gpio pins \n");
        return -EINVAL;
    }
    DALERT("GPIO pins are configured successfully for Led Matrix device\n");
    return 0;
}

/*On Removal removing the configured pins of the spi device. Only the primary pins */
static int removeDevice(void){
    deconfigure_max7219();
    return 0;
}

/*Probe function to handle the found spi devices */
static int spi_driver_probe(struct spi_device *spi){
    DALERT("Found a matching device. Device name: %s \n", spi->modalias);
    hcsr04_devp->found_device = spi;
    
    if(initializeDevice()){
        DALERT("Failed to Initialize the found device. \n");
        return -EINVAL;
    }

    DALERT("SPI Driver initialized successfully.\n");
    return 0;
}

/*Remove function to handle the removal of spi device */
static int spi_driver_remove(struct spi_device *spi){
    DALERT("Removing the device. \n");

    removeDevice();

    DALERT("SPI Driver removed successfully.\n");
    return 0;
}


/*
*Base configuration of the spi driver, to keep track of the list of possible spi-devices through id_table
*/
static struct spi_driver spi_driver = {
    .id_table=spi_deviceId_table,
    .probe=spi_driver_probe,
    .remove=spi_driver_remove,
    .driver={
        .name = LED_DEVICE_NAME,
        .owner = THIS_MODULE,
    }
};


/*Kernel Module Initialization*/
static int __init spi_netlink_init(void){
    /*Initializing the netlink family for communication */
    if(genl_register_family(&genl_netlink_family)){
        DALERT("Netlink family registration failed. \n");
        return -EINVAL;
    }
    DALERT("Generic Netlink family registered successfully. \n");
    
    hcsr04_devp = kmalloc(sizeof(struct hcsr04_dev), GFP_KERNEL);
    
    if(!hcsr04_devp) {
        DALERT("Unable to allocate memory \n");
        return -ENOMEM;
    }

    memset(hcsr04_devp, 0, sizeof (struct hcsr04_dev));

    sprintf(hcsr04_devp->name, "%s",HCSR_DEVICE_NAME);

    /*Initializing the variables */
    hcsr04_devp->irq_number = 0;
    hcsr04_devp->isWorkInitialized = 0;
    hcsr04_devp->chip_select_pin.digitalPin = -1;
    hcsr04_devp->echo_pin.digitalPin = -1;
    hcsr04_devp->trigger_pin.digitalPin = -1;

    /*Initialize work queue*/
    hcsr04_devp->test_hcsr_wq = kmalloc(sizeof(struct work_queue_hcsr *), GFP_KERNEL);

    /*Initializing mutexes*/
    mutex_init(&hcsr04_devp->lock);
    mutex_init(&hcsr04_devp->sampleRunning);

    /*Registering the spi_driver */
    if(spi_register_driver(&spi_driver)){
        DALERT("Failed to register spi driver. \n");
        return -EINVAL;
    }
    DALERT("SPI Driver registered successfully. \n");
    return 0;
}

/*Unmounting the kernel module */
static void spi_netlink_exit(void){
    DALERT("About to unregister the driver and the genl family \n");
    genl_unregister_family(&genl_netlink_family);

    spi_unregister_driver(&spi_driver);
    DALERT("Driver and Generic unregistration successful.\n");

    unregister_pin(hcsr04_devp, 0);
    unregister_pin(hcsr04_devp, 1);
    unregister_pin(hcsr04_devp, 2);
}

module_init(spi_netlink_init);
module_exit(spi_netlink_exit);

MODULE_AUTHOR("Keerthivasan");
MODULE_LICENSE("GPL");