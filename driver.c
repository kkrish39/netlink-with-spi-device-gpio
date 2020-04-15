#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netlink.h>
#include <linux/timer.h>
#include <linux/export.h>
#include <net/genetlink.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/skbuff.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>

#include "genl_ex.h"
#include "hcsr04_structures.h"

#define ALERT

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
static struct genl_family genl_netlink_family;

static struct spi_device_id spi_deviceId_table[]={
    {"MAX7219", 0}
};


struct work_queue {
	struct work_struct work;
	struct hcsr04_dev *parameter;
} work_queue;

struct hcsr04_dev *hcsr04_devp;
/*static configuration of I/O to gpio pin MUX */
/*Considering only digital I/O pins*/
static int lookupTable[4][20]= {
    {11,12,61,14,6,0,1,38,40,4,10,5,15,7},
    {32,28,-1,16,36,18,20,-1,-1,22,26,24,42,30},
    {33,29,35,17,37,19,21,39,41,23,27,25,43,31},
    {-1,45,77,76,-1,66,68,-1,-1,70,74,44,-1,46}
};

/*Valid Echo pins with R/F/B Interrupt modes*/
static int validEchoPins[14] = {0,0,1,1,1,1,1,0,0,1,0,1,0,1};

/*Valid Trigger pins with L/H/R/F Interrupt modes*/
static int validTriggerPins[14] = {1,1,1,1,1,1,1,0,0,1,1,1,1,1};

struct spi_device *found_device;
int configureDotMatrix(void);


/*Function to keep the most recently measured data*/
void writeToCircularBuffer(long distance, long long timestamp, struct hcsr04_dev *hcsr04_devp){
    DPRINTK("Writing into the circular buffer \n");
    hcsr04_devp->list = addNode(hcsr04_devp->list, initNode(distance, timestamp));
    // hcsr04_devp->isEnabled = 0;
    mutex_unlock(&hcsr04_devp->sampleRunning);
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
    mutex_lock(&hcsr04_devp->lock);
    gpio_set_value_cansleep(hcsr04_devp->trigger_pin.gpio_pin,0);
    udelay(5);
	gpio_set_value_cansleep(hcsr04_devp->trigger_pin.gpio_pin,1);
	udelay(10);
	gpio_set_value_cansleep(hcsr04_devp->trigger_pin.gpio_pin,0);
    mutex_unlock(&hcsr04_devp->lock);
}

/*Function to perform the distance measurement operation for the given criteria*/
void measureDistance(struct work_struct *hcsr04_wq){
    int i = 0;
    int sum = 0;
    long long time = 0;
    long instance_of_sample = 0;
    long divisor = 58;
    long outlier_min = LONG_MAX;
    long outlier_max = LONG_MIN;
    int average_distance;

    struct work_queue *wq = container_of(hcsr04_wq, struct work_queue, work);
    struct hcsr04_dev *hcsr04_devp = wq->parameter;

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
    writeToCircularBuffer(average_distance, native_read_tsc(),hcsr04_devp);
}

/*Function to spawn k-thread*/
static int spawnThread(struct hcsr04_dev *hcsr04_devp){
    /*Initialize the work and assign the requried parameters*/
    INIT_WORK(&hcsr04_devp->test_wq->work, measureDistance);
    hcsr04_devp->test_wq->parameter = hcsr04_devp;

    schedule_work(&hcsr04_devp->test_wq->work);
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
    
    if(gpio_get_value(fileData->echo_pin.gpio_pin)){
        fileData->trig_time = native_read_tsc();
    }else{
        fileData->echo_time = native_read_tsc();
    }

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

void unregister_pin(struct hcsr04_dev *fileData, int isTrigger){
    if(isTrigger){
        if(fileData->trigger_pin.gpio_pin >= 0){
            gpio_free(fileData->trigger_pin.gpio_pin);
        }
        if(fileData->trigger_pin.mux1 >= 0){
            gpio_free(fileData->trigger_pin.mux1);
        }
        if(fileData->trigger_pin.pull >= 0){
            gpio_free(fileData->trigger_pin.pull);
        }
        if(fileData->trigger_pin.shift_pin >= 0){
            gpio_free(fileData->trigger_pin.shift_pin);
        }
    }else{
        free_irq(fileData->irq_number, fileData);
        if(fileData->echo_pin.gpio_pin >= 0){
            gpio_free(fileData->echo_pin.gpio_pin);
        }
        if(fileData->echo_pin.mux1 >= 0){
            gpio_free(fileData->echo_pin.mux1);
        }
        if(fileData->echo_pin.pull >= 0){
            gpio_free(fileData->echo_pin.pull);
        }
        if(fileData->echo_pin.shift_pin >= 0){
            gpio_free(fileData->echo_pin.shift_pin);
        }
    }
}

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
            }else if(mux_pin == 44){
                if(gpio_request(72, pin_var)){
                    DPRINTK("Unable to register pin: 72 \n");
                    return -EINVAL;
                }
                DPRINTK("GPIO 72 got configured \n");
                gpio_set_value_cansleep(72, 0);
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
            }else if(mux_pin == 44){
                if(gpio_request(72, pin_var)){
                    DPRINTK("Unable to register pin: 72 \n");
                    return -EINVAL;
                }
                DPRINTK("MUX 72 got configured \n");
                gpio_set_value_cansleep(72, 0);
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
        fileData->trigger_pin.mux2 = mux_pin;
        fileData->trigger_pin.pull = pull_pin;
        fileData->trigger_pin.shift_pin = shift_pin;
        fileData->trigger_pin.digitalPin = gpio;
    }else{
        fileData->echo_pin.gpio_pin = gpio_pin;
        fileData->echo_pin.mux1 = mux_pin;
        fileData->echo_pin.mux2 = mux_pin;
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


static void send_message_to_user_processes(unsigned int group){
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
        goto nlmsg_fail;
    }

    snprintf(msg, MAX_BUF_LENGTH, "Hello group %s\n", genl_test_mcgrp_names[group]);
    nla_put_flag(sk_buf, RET_VAL_SUCCESS);
    res = nla_put_string(sk_buf, CALLBACK_IDENTIFIER, msg);
    if (res) {
        printk(KERN_ERR "%d: err %d ", __LINE__, res);
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
    DALERT("About to configure the devices \n");
    if(!info->attrs[HCSR04_TRIGGER_PIN] || !info->attrs[HCSR04_ECHO_PIN] || !info->attrs[HCSR04_SAMPLING_PERIOD] || !info->attrs[HCSRO4_NUMBER_SAMPLES]){
        DALERT("Empty data is sent from the user. Exiting... \n");
        return -EINVAL;
    }

    input_pins.echo_pin = nla_get_u32(info->attrs[HCSR04_ECHO_PIN]);
    input_pins.trigger_pin =  nla_get_u32(info->attrs[HCSR04_TRIGGER_PIN]);
    input_params.sampling_period = nla_get_u32(info->attrs[HCSR04_SAMPLING_PERIOD]);
    input_params.num_samples = nla_get_u32(info->attrs[HCSRO4_NUMBER_SAMPLES]);

    DALERT("%d  %d  %d  %d", input_pins.echo_pin, input_pins.trigger_pin, input_params.sampling_period, input_params.num_samples);
    // if(configure_IO_pins(&input_pins, hcsr04_devp) < 0){
    //     DALERT("Configuring HCSR04 I/O pins failed");
    //     return -EINVAL;
    // }
    // if(configure_measurement_parameters(&input_params, hcsr04_devp) < 0){
    //     DALERT("Configuring HCSR04 parameters failed");
    //     return -EINVAL;
    // }
    send_message_to_user_processes(GROUP0);
    return 0;
}

static void transfer_complete(void *context){
    DALERT("The transfer is completed successfully");
}
static int setup_device(struct spi_device *spi){
    uint16_t trans_arr = 0x0F01;
    struct spi_message msg;
	    		
	

    //for message exchange
    struct spi_transfer spi_trans = {
        .tx_buf = &trans_arr,
        .rx_buf = 0,
        .len = 2,
        .bits_per_word = 16,
        .speed_hz = 10000000,
        .delay_usecs = 1,
        .cs_change = 1,
    };

    printk("Lighting up LEDS - display Test mode!!");
	// trans_arr[0] = 0x0F;
	// trans_arr[1] = 0x01;
    spi_message_init(&msg);
    msg.complete = &transfer_complete;
	spi_message_add_tail((void *)&spi_trans, &msg);
	gpio_set_value(15,0);
	spi_sync(spi, &msg);
	gpio_set_value(15,1);	
	return 0;


    // struct spi_message *sp_msg;
    // struct spi_transfer sp_tf1;
    // struct spi_transfer *sp_tf2;
    // struct spi_transfer *sp_tf3;
    // struct spi_transfer *sp_tf4;
    // struct spi_transfer *sp_tf5;
    // struct spi_transfer *sp_tf6;
    // int ret = 0;
    // u16 d1=0x0F01, d2, d3, d4, d5;

    // static u16 pat[8] = {0x0110,0x0209, 0x030F, 0x0408, 0x0508, 0x06EC, 0x07FB, 0x0819};
    // u8 bits = 16;
    // u32 speed = 10000000;
    // u16 delay = 0;
    // // sp_tf1 = kmalloc(sizeof(struct spi_transfer), GFP_KERNEL);
    // sp_tf2 = kmalloc(sizeof(struct spi_transfer), GFP_KERNEL);
    // sp_tf3 = kmalloc(sizeof(struct spi_transfer), GFP_KERNEL);
    // sp_tf4 = kmalloc(sizeof(struct spi_transfer), GFP_KERNEL);
    // sp_tf5 = kmalloc(sizeof(struct spi_transfer), GFP_KERNEL);
    // sp_tf6 = kmalloc(sizeof(struct spi_transfer), GFP_KERNEL);

    // // d1={0x0F,0x01};
    // // d3=0x0B07;
    // // d4=0x0C01;
    // // d5=0x0F01;

    // // memset(sp_tf1, 0, sizeof(struct spi_transfer));
    // memset(sp_tf2, 0, sizeof(struct spi_transfer));
    // memset(sp_tf3, 0, sizeof(struct spi_transfer));
    // memset(sp_tf4, 0, sizeof(struct spi_transfer));
    // memset(sp_tf5, 0, sizeof(struct spi_transfer));
    // memset(sp_tf6, 0, sizeof(struct spi_transfer));

    // sp_tf1.tx_buf=&d1;
    // sp_tf1.rx_buf=NULL;
    // sp_tf1.len=2;
    // sp_tf1.speed_hz=speed;
    // sp_tf1.bits_per_word = bits;
	// sp_tf1.delay_usecs = delay;

    // sp_tf2->tx_buf=&d2;
    // sp_tf2->rx_buf=NULL;
    // sp_tf2->len=2;
    // sp_tf2->speed_hz=speed;
    // sp_tf2->bits_per_word = bits;
	// sp_tf2->delay_usecs = delay;

    // sp_tf3->tx_buf=&d3;
    // sp_tf3->rx_buf=NULL;
    // sp_tf3->len=2;
    // sp_tf3->speed_hz=speed;
    // sp_tf3->bits_per_word = bits;
	// sp_tf3->delay_usecs = delay;

    // sp_tf4->tx_buf=&d4;
    // sp_tf4->rx_buf=NULL;
    // sp_tf4->len=2;
    // sp_tf4->speed_hz=speed;
    // sp_tf4->bits_per_word = bits;
	// sp_tf4->delay_usecs = delay;

    // sp_tf5->tx_buf=&d5;
    // sp_tf5->rx_buf=NULL;
    // sp_tf5->len=2;
    // sp_tf5->speed_hz=speed;
    // sp_tf5->bits_per_word = bits;
	// sp_tf5->delay_usecs = delay;


    // sp_tf6->tx_buf=&pat;
    // sp_tf6->rx_buf=NULL;
    // sp_tf6->len=16;
    // sp_tf6->speed_hz=speed;
    // sp_tf6->bits_per_word = bits;
	// sp_tf6->delay_usecs = delay;

    // sp_msg = kmalloc(sizeof(struct spi_message), GFP_KERNEL);
    // spi_message_init(sp_msg);
   
    // spi_message_add_tail(&sp_tf1, sp_msg);
    // // spi_message_add_tail(sp_tf2, sp_msg);
    // // spi_message_add_tail(sp_tf3, sp_msg);
    // // spi_message_add_tail(sp_tf4, sp_msg);
    // // spi_message_add_tail(sp_tf5, sp_msg);
    // // spi_message_add_tail(sp_tf6, sp_msg);
    // ret = spi_sync(spi, sp_msg);

    // mdelay(100);
    DALERT("Transfer Complete");
    return 0;
}
static int configure_max7219_device (struct sk_buff* sk_buf, struct genl_info* info){
    printk("About to configure max7219 \n");
    return 0;
}
static int start_distance_measurement (struct sk_buff* sk_buf, struct genl_info* info){
    spawnThread(hcsr04_devp);
    return 0;
}

static int send_pattern_to_matrix_led (struct sk_buff* sk_buf, struct genl_info* info){

    return 0;
}
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

int configureDotMatrix(void){

printk(KERN_INFO" initializing the gpio pins for the spi LED matrix\n");
	// //IO11 is conencted to DIn fo dot matrix LED
	// gpio_export(mosi_ls,true);
	// gpio_export(mosi_pullup,true);
	// gpio_export(mosi_mux1,true);
	// gpio_export(mosi_mux2,true);
	// gpio_export(mosi_gpio,true);
	// gpio_direction_output(mosi_gpio,1);
	// gpio_set_value(mosi_gpio,1);
	// gpio_set_value(mosi_mux2,0);
	// gpio_direction_output(mosi_ls,1);
	// gpio_set_value(mosi_ls,0);
	// gpio_direction_output(mosi_pullup,1);
	// gpio_set_value(mosi_pullup,0);
	// gpio_direction_output(mosi_mux1,1);
	// gpio_set_value(mosi_mux1,1);

	// //IO13 is connected to CLK pin of dot matrix LED
	// gpio_export(sck_ls,true);
	// gpio_export(sck_pullup,true);
	// gpio_export(sck_mux1,true);
	// gpio_export(sck_gpio,true);
	// gpio_direction_output(sck_gpio,1);
	// gpio_set_value(sck_gpio,1);
	// gpio_direction_output(sck_mux1,1);
	// gpio_set_value(sck_mux1,1);
	// gpio_direction_output(sck_ls,1);
	// gpio_set_value(sck_ls,0);
	// gpio_direction_output(sck_pullup,1);
	// gpio_set_value(sck_pullup,0);

	// //IO12 is connected to CS pin of dot matrix LED
	// gpio_export(42,true);
	// gpio_export(43,true);
	// gpio_export(15,true);
	// gpio_direction_output(42,1);
	// gpio_set_value(42,0);
	// gpio_direction_output(43,1);
	// gpio_set_value(43,0);
	// gpio_direction_output(15,1);
	// gpio_set_value(15,0);
    
    
    if(gpio_request(24, "gpio24")){
        DALERT("Configuration failed \n");
    }
    if(gpio_request(44, "gpio44")){
        DALERT("Configuration failed \n");
    }
    if(gpio_request(72, "gpio72")){
        DALERT("Configuration failed \n");
    }
    if(gpio_request(25, "gpio25")){
        DALERT("Configuration failed \n");
    }
    // if(gpio_request(5, "gpio5")){
    //     DALERT("Configuration failed \n");
    // }



    gpio_direction_output(44,1);
    // gpio_direction_output(5,0);
    gpio_direction_output(24,0);
    gpio_direction_output(25,0);
    // gpio_set_value_cansleep(5, 1);
    // gpio_set_value_cansleep(44, 1);
    gpio_set_value_cansleep(72, 0);
    // gpio_set_value_cansleep(24, 0);
    // gpio_set_value_cansleep(25, 0);


    if(gpio_request(46, "gpio46")){
        DALERT("Configuration failed \n");
    }
    if(gpio_request(30, "gpio30")){
        DALERT("Configuration failed \n");
    }
    if(gpio_request(31, "gpio31")){
        DALERT("Configuration failed \n");
    }
    // if(gpio_request(7, "gpio7")){
    //     DALERT("Configuration failed \n");
    // }

    // gpio_direction_output(7,1);
	// gpio_set_value_cansleep(7,1);
	gpio_direction_output(46,1);
	// gpio_set_value_cansleep(46,1);
	gpio_direction_output(30,0);
	// gpio_set_value_cansleep(30,0);
	gpio_direction_output(31,0);
	// gpio_set_value_cansleep(31,0);


    if(gpio_request(42, "gpio42")){
        DALERT("Configuration failed \n");
    }
    if(gpio_request(43, "gpio43")){
        DALERT("Configuration failed \n");
    }
    if(gpio_request(15, "gpio15")){
        DALERT("Configuration failed \n");
    }

    gpio_direction_output(42,0);
	// gpio_set_value_cansleep(42,0);
	gpio_direction_output(43,0);
	// gpio_set_value_cansleep(43,0);
	gpio_direction_output(15,0);
	// gpio_set_value_cansleep(15,0);
    // gpio_direction_output(26,0);
    // gpio_set_value_cansleep(74, 1);
    // gpio_direction_input(27);

    // if(gpio_request(30, "gpio26")){
    //     DALERT("Configuration failed \n");
    // }
    // if(gpio_request(46, "gpio74")){
    //     DALERT("Configuration failed \n");
    // }
    // if(gpio_request(31, "gpio27")){
    //     DALERT("Configuration failed \n");
    // }

    // gpio_direction_output(30,0);
    // gpio_set_value_cansleep(46, 1);
    // gpio_direction_input(31);

    DALERT("Configuration Successful pins \n");

    return 0;
}

void deConfigureDotMatrix(void){
    gpio_free(26);
    gpio_free(74);
    gpio_free(27);
    gpio_free(24);
    gpio_free(44);
    gpio_free(72);
    gpio_free(25);
    gpio_free(46);
    gpio_free(30);
    gpio_free(31);
}
static const struct genl_multicast_group genl_netlink_mcgrps[] = {
    [GROUP0] = { .name = MULTICAST_GROUP0, },
    [GROUP1] = { .name = MULTICAST_GROUP1, },
    [GROUP2] = { .name = MULTICAST_GROUP2, },
};

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



/************************************************************************************************************************************************************/

static int initializeDevice(void){
    // configureDotMatrix();
    return 0;
}

static int removeDevice(void){
    deConfigureDotMatrix();
    return 0;
}

static int spi_driver_probe(struct spi_device *spi){
    DALERT("Found a matching device. About to Initialize \n");
    found_device = spi;
    configureDotMatrix();

    if(setup_device(found_device)){
        DALERT("Failed to initialize the found device\n");
        return -EINVAL;
    }
    
    initializeDevice();
    DALERT("SPI Driver initialized successfully.\n");
    return 0;
}

static int spi_driver_remove(struct spi_device *spi){
    DALERT("Removing the device. \n");

    if(removeDevice()){ 
        DALERT("Failed to remove the device\n");
        return -EINVAL; 
    }
    DALERT("SPI Driver removed successfully.\n");
    return 0;
}


/*
*
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

/************************************************************************************************************************************************************/

static int __init spi_netlink_init(void){
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

    if(spi_register_driver(&spi_driver)){
        DALERT("Failed to register spi driver. \n");
        return -EINVAL;
    }
    DALERT("SPI Driver registered successfully. \n");
    return 0;
}

static void spi_netlink_exit(void){
    DALERT("About to unregister the driver and the genl family \n");
    genl_unregister_family(&genl_netlink_family);

    spi_unregister_driver(&spi_driver);
    DALERT("Driver and Generic unregistration successful.\n");
}

module_init(spi_netlink_init);
module_exit(spi_netlink_exit);

MODULE_AUTHOR("Keerthivasan");
MODULE_LICENSE("GPL");