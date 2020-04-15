#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/math64.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include "hcsr04_structures.h"
#include "fifo.h"

#define ALERT

#if defined(ALERT)
    #define DALERT(fmt, args...) printk(KERN_ALERT "ALERT:"fmt, ##args)
#else
    #define DALERT(fmt, args...) 
#endif

#if defined(INFO)
    #define DPRINTK(fmt, args...) printk(""fmt, ##args)
#else
    #define DPRINTK(fmt, args...) 
#endif

#define DEVICE_NAME "HCSR_" //


struct work_queue {
	struct work_struct work;
	struct hcsr04_dev *parameter;
} work_queue;

/* per device structure */
struct hcsr04_dev {
	char name[20]; /* Name of the device */
    struct miscdevice misc_device; /* Miscellaneous character driver*/
    pins echo_pin; /*pin structure to keep track of related gpio pins for echo pin*/
    pins trigger_pin; /*pin structure to keep track of related gpio pins for echo pin*/
    unsigned long long time_stamp; /*time when the distance is recorded */
    int distance;/*The distance measure in centimeter*/
    circular_buf *list; /*Kernel fifo buffer*/
    int num_samples_per_measurement;/*Number of samples per  measurement */
    int sampling_period;/*Sampling period*/
    unsigned long long trig_time; /*Time when the Trigger pin is triggered*/
    unsigned long long echo_time; /*Time when the echo pin receives an echo*/
    int isWorkInitialized; /*Flag to keep track whether workQueue is initialized*/
    int irq_number; /*IRQ number that is being generated for the given echo pin*/
    struct work_queue *test_wq; /*Workqueue specific for each device */
    struct mutex lock, sampleRunning; /*per-device locks to enforce synchronization*/
}*hcsr04_devp[MAX_DEVICE_LIMIT];

/*Number of gpio devices requested by the user*/
static int n_gpio = 0;

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

/*Get Number of devices from the user*/
module_param(n_gpio, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(n_gpio, "Number of GPIO");


/*
* Function to open the given device file.
*/
int hcsr04_driver_open(struct inode *inode, struct file *file){
    /*The file pointer will already be pointing to the misc device when it's rgistered */
    struct hcsr04_dev *hcsr04_devp;
    hcsr04_devp = container_of(file->private_data, struct hcsr04_dev,misc_device);

    DALERT("%s is opening \n", hcsr04_devp->name);
    return 0;
}

/*
* Function to release the given device file once all the operations are done.
*/
int hcsr04_driver_release(struct inode *inode, struct file *file){
    struct hcsr04_dev *hcsr04_devp;
    hcsr04_devp = container_of(file->private_data, struct hcsr04_dev,misc_device);
    
    DALERT("%s is closing \n", hcsr04_devp->name);
    return 0;
}

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

/* Function to handle write function for the device */
ssize_t hcsr04_driver_write(struct file *file, const char *buf, size_t count, loff_t *ppos){
    int *input_val, lock;
    int ret;
    struct hcsr04_dev *hcsr04_devp;
    hcsr04_devp = container_of(file->private_data, struct hcsr04_dev,misc_device);
    
    /*Check if there is any ongoing measurement*/
    lock = mutex_trylock(&hcsr04_devp->sampleRunning);
    
    /*Enforcing non-blocking write. Exiting if there is an ongoing measurement*/
    if(!lock){
        DALERT("****There is an Ongoing Measurement****\n");
        return -EINVAL;
    }
    copy_from_user(&input_val, (int *)buf, sizeof(int));

    /*If the input is 0, clear the buffer and start the measurement.*/
    if(input_val == 0){
        clearBuffer(hcsr04_devp);
    }
    /*Start the Measurement*/
    DALERT("Triggering measurment from Write Function \n");
    ret =  spawnThread(hcsr04_devp);
    return ret;
}

/*Function to pull out the read value from the buffer*/
void removeReadMeasurement(struct hcsr04_dev *hcsr04_devp){
    hcsr04_devp->list = deleteHead(hcsr04_devp->list);
}

/* Function to handle read function for the device */
ssize_t hcsr04_driver_read(struct file *file, char *buf, size_t count, loff_t *ppos){
    struct hcsr04_dev *hcsr04_devp;
    struct circular_buffer data;
    int lock;
    hcsr04_devp = container_of(file->private_data, struct hcsr04_dev,misc_device);

    /*Check if the lock is free*/
    lock = mutex_trylock(&hcsr04_devp->sampleRunning);
    /*If there is a lock already, wait till we acquire the lock and return the data*/
    /*Blocking Read*/
    if(!lock){
        DPRINTK("Waiting for the Ongoing Measurement \n");
        mutex_lock(&hcsr04_devp->sampleRunning);
    }
    
    /*If there is no lock and no measurements, trigger a measurement*/
    if(hcsr04_devp->list == NULL){
        DALERT("Triggering measurment from Read Function \n");
        spawnThread(hcsr04_devp);
        mutex_lock(&hcsr04_devp->sampleRunning);
    }

    DALERT("About to send distace:%ld Timestamp:%lld \n", hcsr04_devp->list->distance, hcsr04_devp->list->timestamp);
    data.distance = hcsr04_devp->list->distance;
    data.timestamp = hcsr04_devp->list->timestamp;
    copy_to_user((struct circular_buffer *)buf, &data, sizeof(struct circular_buffer));
    
    /*Removing the currently read value from the buffer*/
    hcsr04_devp->list = deleteHead(hcsr04_devp->list);
    mutex_unlock(&hcsr04_devp->sampleRunning);
    
    return sizeof(circular_buffer);
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

/*Function to perform IOCTL operation requested by the user*/
static long hcsr04_driver_ioctl(struct file *file, unsigned int cmd, unsigned long arg){
    struct hcsr04_dev *hcsr04_devp;
    void * input;

    hcsr04_devp = container_of(file->private_data, struct hcsr04_dev, misc_device);

    switch(cmd){
        /*Setting up the I/O pins*/
        case CONFIG_PINS:
            input = (config_input *)arg;
            if(configure_IO_pins(input, hcsr04_devp) < 0){
                return -EINVAL;
            }
            break;
        /*Setting up the measurement parameters*/
        case SET_PARAMETERS:
            input = (setparam_input *)arg;
            if(configure_measurement_parameters(input, hcsr04_devp)){
                return -EINVAL;
            }
            break;
        default:
            break;
    };

    return 0;
}

/*List of file operations to be implemented by the misc_devices*/
static struct file_operations hcsr04_fops = {
    .owner          = THIS_MODULE,
    .open           = hcsr04_driver_open,
    .release        = hcsr04_driver_release,
    .write          = hcsr04_driver_write,
    .read           = hcsr04_driver_read,
    .unlocked_ioctl = hcsr04_driver_ioctl
};

/*Module Entry function*/
int __init hcsr04_driver_init(void){
    int ret;
    int i;

    for(i=0;i<n_gpio;i++){
        hcsr04_devp[i] = kmalloc(sizeof(struct hcsr04_dev), GFP_KERNEL);
        if(!hcsr04_devp[i]) {
            DALERT("Unable to allocate memory \n");
            return -ENOMEM;
        }

        memset(hcsr04_devp[i], 0, sizeof (struct hcsr04_dev));

        sprintf(hcsr04_devp[i]->name, "%s%d",DEVICE_NAME,i+1);

        /*Allocate Miscellaneous device structure*/
        // (minor_number, device_name, file_operations) 
        hcsr04_devp[i]->misc_device.minor = MISC_DYNAMIC_MINOR; //Dynamically allocated
        hcsr04_devp[i]->misc_device.name = hcsr04_devp[i]->name;
        hcsr04_devp[i]->misc_device.fops = &hcsr04_fops;
        
        /*Initializing the variables */
        hcsr04_devp[i]->irq_number = 0;
        hcsr04_devp[i]->isWorkInitialized = 0;
        
        /*Initialize work queue*/
        hcsr04_devp[i]->test_wq = kmalloc(sizeof(struct work_queue *), GFP_KERNEL);

        /*Initializing mutexes*/
        mutex_init(&hcsr04_devp[i]->lock);
        mutex_init(&hcsr04_devp[i]->sampleRunning);
        /*Reigster the miscellaneous device */
        ret = misc_register(&hcsr04_devp[i]->misc_device);


        if(ret){
            DALERT(KERN_DEBUG "Can't register miscellaneous device\n");
            return ret;
        }
    }
    DALERT("hcsr04 drive initialized. \n");   
    return 0;
}

/*Module exit function*/
void __exit hcsr04_driver_exit(void){
    int i;

    /*Free the memory for each device created */
    for(i=0;i<n_gpio;i++){
        DALERT("********CLEARING %s********", hcsr04_devp[i]->name);
        if(hcsr04_devp[i]->isWorkInitialized)
            flush_work(&hcsr04_devp[i]->test_wq->work);
        DALERT("Freeing IRQ \n");
        if(hcsr04_devp[i]->irq_number)
            free_irq(hcsr04_devp[i]->irq_number, hcsr04_devp[i]);
        
        DALERT("Freeing Echo pin I/O related gpio pins \n");
        if(hcsr04_devp[i]->echo_pin.gpio_pin >= 0){
            gpio_free(hcsr04_devp[i]->echo_pin.gpio_pin);
        }
        if(hcsr04_devp[i]->echo_pin.mux1 >= 0){
            gpio_free(hcsr04_devp[i]->echo_pin.mux1);
        }
        if(hcsr04_devp[i]->echo_pin.pull >= 0){
            gpio_free(hcsr04_devp[i]->echo_pin.pull);
        }
        if(hcsr04_devp[i]->echo_pin.shift_pin >= 0){
            gpio_free(hcsr04_devp[i]->echo_pin.shift_pin);
        }

        DALERT("Freeing Trigger pin I/O related gpio pins \n");
        if(hcsr04_devp[i]->trigger_pin.gpio_pin >= 0){
            gpio_free(hcsr04_devp[i]->trigger_pin.gpio_pin);
        }
        if(hcsr04_devp[i]->trigger_pin.mux1 >= 0){
            gpio_free(hcsr04_devp[i]->trigger_pin.mux1);
        }
        if(hcsr04_devp[i]->trigger_pin.pull >= 0){
            gpio_free(hcsr04_devp[i]->trigger_pin.pull);
        }
        if(hcsr04_devp[i]->trigger_pin.shift_pin >= 0){
            gpio_free(hcsr04_devp[i]->trigger_pin.shift_pin);
        }
       
        /*unregister the miscellaneous devices */
        misc_deregister(&hcsr04_devp[i]->misc_device);

        /*Free the allocated memory for each device */
        kfree(hcsr04_devp[i]);
    }
    DALERT("hcsr04 driver removed \n");
}

module_init(hcsr04_driver_init);
module_exit(hcsr04_driver_exit);

MODULE_LICENSE("GPL v2");