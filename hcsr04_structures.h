#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>
#include "fifo.h"

#define MAX_DEVICE_LIMIT 15
#define CIRCULAR_BUF_LENGTH 5

/* Input structrue to configure pins for a device */
typedef struct config_input{
    unsigned trigger_pin;
    unsigned echo_pin;
} config_input;

/* Input structrue to change the metrics of data measurement */ 
typedef struct setparam_input{
    int num_samples;
    int sampling_period;
} setparam_input;

/*Circular Buffer to keep track of the measurements*/
typedef struct circular_buffer{
    long distance;
    unsigned long long timestamp;
} circular_buffer;

/*List of gpio pins that may be associated with the a given I/O pin */
typedef struct pins_to_configure {
    int digitalPin; /*   Digital pin */
    int gpio_pin;   /*   GPIO pin    */
    int shift_pin;  /*   Shift pin   */
    int pull;       /*   Pull pin    */
    int mux1;       /*   Mux1 pin    */
    int mux2;       /*   Mux2 pin    */
} pins;

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


/* per device structure */
struct hcsr04_dev {
	char name[20]; /* Name of the device */
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
    unsigned long long timeToStopMeasurment; /*Keep track the time to halt the distance measurement*/
    struct spi_device *found_device;
};