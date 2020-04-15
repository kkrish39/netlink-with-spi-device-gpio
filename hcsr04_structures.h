#include <linux/ioctl.h>
#include <linux/miscdevice.h>
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
    int digitalPin;
    int gpio_pin;   /* GPIO  Pin*/
    int shift_pin;  /*shift pin*/
    int pull;       /*pull pin*/
    int mux1;       /*mux1_pin*/
    int mux2;       /*mux2_pin*/
} pins;

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
};