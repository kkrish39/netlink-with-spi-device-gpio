#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h>

static struct spi_device *matrix_led;

static struct spi_board_info board_info = {
   .modalias="MAX7219",
   .max_speed_hz= 10000000, //10MHz
   .bus_num = 1,
   .chip_select = 1,
   .mode = SPI_MODE_0
};


static int spi_device_init(void){
    struct spi_master *master_device;

    /*
    * @board_info.bus_num - the master's bus number
    * 
    * @return 
    *   #success - spi_master structure will be returned
    *   #failure - NULL
    */
    master_device = spi_busnum_to_master(board_info.bus_num);

    if(!master_device){
        printk(KERN_ALERT "Unable to register a bus number. Exiting... \n");
        return -EINVAL;
    }

    /*
    * @master_device - Controller to which the device is connected.
    * @board_info - describing the SPI device.
    * 
    * @return
    *   #success - a new spi_device will be returned
    *   #failure - NULL
    */
    matrix_led = spi_new_device(master_device, &board_info);

    if(!matrix_led){
        printk(KERN_ALERT "Unable to allocate/get a new device. Exiting... \n");
        return -EINVAL;
    }

    matrix_led->bits_per_word = 16;

    /*
    *   @matrix_led - SPI Device
    *   
    *   @return
    *       #success - 0 is returned.
    *       #failure - error value will be returned. Non-zero value.
    */
    if(spi_setup(matrix_led)){
        printk(KERN_ALERT "Setting up spi device failed \n");
        return -EINVAL;
    }

    printk(KERN_ALERT"SPI Device setup successful");

    return 0;
}

static void spi_device_exit(void){
    printk(KERN_ALERT "Unregistering the device \n");

    /*
    * @matrix_led - SPI device that needs to be unregistered/removed.
    */
    spi_unregister_device(matrix_led);
}

module_init(spi_device_init);
module_exit(spi_device_exit);

MODULE_AUTHOR("Keerthivasan");
MODULE_LICENSE("GPL");