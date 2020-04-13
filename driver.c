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

#include "genl_ex.h"

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


#define DEVICE_NAME "MAX7219"
static struct genl_family genl_netlink_family;

static struct spi_device_id spi_deviceId_table[]={
    {"MAX7219", 0}
};

static void manage_messages(unsigned int group){
    void *header;
    int res, flags = GFP_ATOMIC;
    char msg[MAX_BUF_LENGTH];
    struct sk_buff* sk_buf = genlmsg_new(NLMSG_DEFAULT_SIZE, flags);

    if (!sk_buf) {
        printk(KERN_ERR "%d: OOM!!", __LINE__);
        return;
    }

    header = genlmsg_put(sk_buf, 0, 0, &genl_netlink_family, flags, CONFIGURE_DEVICE);
    if (!header) {
        printk(KERN_ERR "%d: Unknown err !", __LINE__);
        goto nlmsg_fail;
    }

    snprintf(msg, MAX_BUF_LENGTH, "Hello group %s\n", genl_test_mcgrp_names[group]);

    res = nla_put_string(sk_buf, GENL_TEST_ATTR_MSG, msg);
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

static int genl_netlink_rx_msg(struct sk_buff* sk_buf, struct genl_info* info)
{
    struct sample *buf ;
    if (!info->attrs[1]) {
        printk(KERN_ERR "empty message from %d!!\n",
            info->snd_portid);
        printk(KERN_ERR "%p\n", info->attrs[1]);
        return -EINVAL;
    }
    buf = (struct sample *)nla_data(info->attrs[1]);
    printk(KERN_NOTICE "%u says %d \n", info->snd_portid,
        buf->data2);
    
    manage_messages(GROUP0);
    return 0;
}

static int configure_device (struct sk_buff* sk_buf, struct genl_info* info){
    int echo_pin, trigger_pin, sampling_period, num_samples;
    DALERT("About to configure the devices \n");
    if(!info->attrs[HCSR04_TRIGGER_PIN] || !info->attrs[HCSR04_ECHO_PIN] || !info->attrs[HCSR04_SAMPLING_PERIOD] || !info->attrs[HCSRO4_NUMBER_SAMPLES]){
        DALERT("Empty data is sent from the user. Exiting... \n");
        return -EINVAL;
    }

    echo_pin = nla_get_u32(info->attrs[HCSR04_ECHO_PIN]);
    trigger_pin =  nla_get_u32(info->attrs[HCSR04_TRIGGER_PIN]);
    sampling_period = nla_get_u32(info->attrs[HCSR04_SAMPLING_PERIOD]);
    num_samples = nla_get_u32(info->attrs[HCSRO4_NUMBER_SAMPLES]);

    DALERT("%d  %d  %d  %d\n", echo_pin, trigger_pin, sampling_period, num_samples);
    manage_messages(GROUP0);
    return 0;
}

static int start_distance_measurement (struct sk_buff* sk_buf, struct genl_info* info){

    return 0;
}

static int send_pattern_to_matrix_led (struct sk_buff* sk_buf, struct genl_info* info){

    return 0;
}
static const struct genl_ops genl_netlink_ops[] = {
    {
        .cmd = GENL_TEST_C_MSG,
        .policy = genl_test_policy,
        .doit = genl_netlink_rx_msg,
        .dumpit = NULL,
    },
    {
        .cmd = CONFIGURE_DEVICE,
        .policy = genl_test_policy,
        .doit = configure_device,
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

    gpio_request(26, "gpio26");
    gpio_request(74, "gpio74");
    gpio_request(27, "gpio27");

    gpio_set_value_cansleep(26, 0);
    gpio_set_value_cansleep(74, 0);
    gpio_direction_input(27);


    gpio_request(24, "gpio24");
    gpio_request(44, "gpio44");
    gpio_request(72, "gpio72");
    gpio_request(25, "gpio25");

    gpio_set_value_cansleep(24, 0);
    gpio_set_value_cansleep(44, 1);
    gpio_set_value_cansleep(72, 0);
    gpio_direction_input(25);

    gpio_request(42, "gpio42");
    gpio_request(43, "gpio43");

    gpio_set_value_cansleep(42, 0);
    gpio_direction_input(43);

    gpio_request(30, "gpio26");
    gpio_request(46, "gpio74");
    gpio_request(31, "gpio27");

    gpio_set_value_cansleep(30, 0);
    gpio_set_value_cansleep(46, 1);
    gpio_direction_input(31);

    return 0;
}

void deConfigureDotMatrix(void){
    gpio_free(26);
    gpio_free(74);
    gpio_free(27);
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

    return 0;
}

static int removeDevice(void){
    deConfigureDotMatrix();
    return 0;
}

static int spi_driver_probe(struct spi_device *spi){
    DALERT("Found a matching device. About to Initialize \n");

    if(initializeDevice()){
        DALERT("Failed to initialize the found device\n");
        return -EINVAL;
    }
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
        .name = DEVICE_NAME,
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
    

    if(spi_register_driver(&spi_driver)){
        DALERT("Failed to register spi driver. \n");
        return -EINVAL;
    }
    DALERT("SPI Driver registered successfully. \n");

    configureDotMatrix();
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