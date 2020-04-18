#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <pthread.h>
#include <time.h>
#include <sched.h>
#include <netlink/msg.h>
#include <netlink/attr.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>

#include "genl_ex.h"

#define USER_SAMPLING_PERIOD 80
#define USER_NUM_SAMPLES 5
#define USER_NUM_SECONDS_TO_RUN 10

#ifndef GRADING
#define MAX7219_CS_PIN 12
#define HCSR04_TRIGGER_PIN 1
#define HCSR04_ECHO_PIN 5
#endif

static struct nl_sock *netlink_socket;
static struct nl_msg *msg;
static struct nl_cb *cb = NULL;
pthread_mutex_t read_write_lock, termination_flag_lock;


static int should_send_recv = 1;
static int measured_distance;
static int delay_time;
static int isInitialMeasured = 0;

/*Dummy Callaback to get notified that a message is recieved from the kernel*/
static int message_entry(struct nl_msg *msg, void *arg){
    printf("Received message: \n");
    return NL_OK;
}


static int callback_handler(struct nl_msg *msg, void* arg)
{
	struct nlattr *attr[GENL_TEST_ATTR_MAX+1];

	genlmsg_parse(nlmsg_hdr(msg), 0, attr, GENL_TEST_ATTR_MAX, genl_test_policy);

    if(!nla_get_flag(attr[RET_VAL_SUCCESS_ATTR])){
        printf("***************************Encountered an error*************************** \n");
        if(attr[OPTIONAL_ERROR_MESSAGE_ATTR]){
            printf("The Following error occured: %s \n", nla_get_string(attr[OPTIONAL_ERROR_MESSAGE_ATTR]));
        }
        return -1;
    }

    if(attr[TERMINATE_OPERATION_ATTR]){
        printf("\t\t\t\tReceived a termination command. Exiting... \n");
        pthread_mutex_lock(&termination_flag_lock);
        should_send_recv = 0;
        pthread_mutex_unlock(&termination_flag_lock);
        return 0;
    }

    if(attr[DISTANCE_MEASURE_ATTR]){
        pthread_mutex_lock(&read_write_lock);
        measured_distance = nla_get_u32(attr[DISTANCE_MEASURE_ATTR]);
        if(isInitialMeasured == 0){
            isInitialMeasured = 1;
        }
        printf("\t\t\t\tReceived the distance measure: %d \n", nla_get_u32(attr[DISTANCE_MEASURE_ATTR]));
        pthread_mutex_unlock(&read_write_lock);
    }
	return NL_OK;
}

void *sendPattern(){
    printf("\t\t\t\tAbout to relay the pattern \n");
    /*Keep track of rate of change relative to the distance*/
    double rate_of_change = 0;

    /*default time delay between the patterns */
    int milli_seconds = 2000000; 
    
    /*Pattern to clear the display*/
    u_int16_t clearDisplay[8] = {0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700,0x0800};

    /*Pattern to lighten the entire display*/
    u_int16_t completeDisplay[8] = {0x01FF,0x02FF,0x03FF,0x04FF,0x05FF,0x06FF,0x07FF,0x08FF};

    /*Pattern to display the left dance pose of the human */
    u_int16_t pattern1[8] = {0x0100, 0x0240,0x0360, 0x0427,0x05F8,0x06A4,0x0732,0x0811};

    /*Pattern to display the right dance pose of the human */
    u_int16_t pattern3[8] = {0x0100, 0x0211,0x0332, 0x04A4,0x05F8,0x0627,0x0760,0x0840};

    /*Clear the display in the beginning of the pattern display*/
    nla_put(msg,DISPLAY_PATTERN_ATTR,sizeof(clearDisplay),(void *)clearDisplay);
    nl_send_auto(netlink_socket, msg);
    

    /*Keep relaying the pattern until the termination condition is met*/
    while(1){
        /*check if the termination command is met or not */
        pthread_mutex_lock(&termination_flag_lock);
            if(!should_send_recv) break;
        pthread_mutex_unlock(&termination_flag_lock);

        /*Access the received distance measure and update the rate of display*/
        pthread_mutex_lock(&read_write_lock);

        /*Initially print the complete display before receiving a valid distance from the HCSR device*/
        if(!isInitialMeasured){
            nla_put(msg,DISPLAY_PATTERN_ATTR,sizeof(completeDisplay),(void *)completeDisplay);
            nl_send_auto(netlink_socket, msg);
            pthread_mutex_unlock(&read_write_lock);
            continue;
        }

        pthread_mutex_unlock(&read_write_lock);
        /*Calculating the rate of change based on upper bound as 400cms*/
        rate_of_change = (double)measured_distance/(double)400;
        
        /*Calculating a relative sleep time based on distance */
        delay_time = milli_seconds * rate_of_change;

        /*Handle the edge case in case of discrepency in the distance measurment.
            defaulting it to the base sleeptime*/
        if(delay_time == 0 || delay_time > 2000000){
            delay_time = milli_seconds;
        }

        /*Sending the first pattern*/
        nla_put(msg,DISPLAY_PATTERN_ATTR,sizeof(pattern1),(void *)pattern1);
        nl_send_auto(netlink_socket, msg);

        /*Sleep for a requried amount of time*/
        usleep(delay_time);

        /*Send back the next pattern */
        nla_put(msg,DISPLAY_PATTERN_ATTR,sizeof(pattern3),(void *)pattern3);
        nl_send_auto(netlink_socket, msg);
        
        /*Sleep for a required amout of time*/
        usleep(delay_time);
    }

    /*Unlock when the termination condition is met*/
    pthread_mutex_unlock(&termination_flag_lock);

    /* Clearing the display once the termination condition is met. */
    nla_put(msg,DISPLAY_PATTERN_ATTR,sizeof(clearDisplay),(void *)clearDisplay);
    nl_send_auto(netlink_socket, msg);

    
    /*Thread exit*/
    pthread_exit(NULL);
}

static int skip_seq_check(struct nl_msg *msg, void *arg)
{
	return NL_OK;
}

/*Receive the measurment continously from the hcsr04 device*/
void *receiveMeasurements(){

    /*Receive till the last message is received */
    while(1){
        pthread_mutex_lock(&termination_flag_lock);
            if(!should_send_recv) break;
        pthread_mutex_unlock(&termination_flag_lock);
        if(nl_recvmsgs(netlink_socket, cb)){
            printf("Error in receiving from the kernel. \n");
            break;
        }
    }
    pthread_mutex_unlock(&termination_flag_lock);
    /*Thread exit once the termination condition is met.*/
    
    pthread_exit(NULL);
}

int main() {
    /*Keep track of group_id, family_number*/
    int group_id, family_num, return_value;

    /*Thread id's for distacne and pattern thread*/
    pthread_t thread_dev1, thread_dev2;

    /*Initializing the netlink socket*/
    netlink_socket = nl_socket_alloc();


    /*initializing the mutexes*/
    pthread_mutex_init(&termination_flag_lock, NULL);
    pthread_mutex_init(&read_write_lock, NULL);
    /*Check for allocation failure*/
    if(!netlink_socket){
        printf("Failed to allocate netlink socket. Exiting... \n");
        return -1;
    }
    

    /*Disabling the sequence check and the automatic acknowledgement*/
    nl_socket_disable_seq_check(netlink_socket);
    nl_socket_disable_auto_ack(netlink_socket);
    
    /*Connecting a generic netlink socket.*/
    if(genl_connect(netlink_socket)){
        printf("Failed to initalize the generic netlink. Exiting... \n");
        return -1;
    }

    /*Resolve the family name to number and check for it's validity*/

    family_num = genl_ctrl_resolve(netlink_socket, NETLINK_FAMILY_NAME);
    
    if(family_num < 0){
        printf("Invalid Family Name. Exiting... \n");
        return -1;
    }

    /*resolve generic group. Initialized only one group. But programmed it as a multicast system*/
    group_id = genl_ctrl_resolve_grp(netlink_socket, NETLINK_FAMILY_NAME, genl_test_mcgrp_names[0]);

    /*Checking the validity of the group*/
    if(group_id < 0){
        printf("Invalid group id \b");
        return -1;
    }

    /*Socket membership addition to intiate the communication*/
    if(nl_socket_add_membership(netlink_socket, group_id)){
        printf("Failed to register membership \n");
        return -1;
    }

    /*Allocating the message to be sent*/
    msg = nlmsg_alloc();
    if(!msg){
        printf("Failed to allocate message \n");
        return -1;
    }

    /*Command to configure the HCSR04 device */
    if(!genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, family_num, 0, NLM_F_REQUEST, CONFIGURE_HCSR04, 0)){
        printf("Failed to put netlink header. Exiting... \n");
        goto failure;
    }

     /* Prepare the callback functions to receive data from the kernel*/
	cb = nl_cb_alloc(NL_CB_DEFAULT);
	nl_cb_set(cb, NL_CB_SEQ_CHECK, NL_CB_CUSTOM, skip_seq_check, NULL);
    nl_cb_set(cb, NL_CB_MSG_IN, NL_CB_CUSTOM, message_entry, NULL);
	nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, callback_handler, NULL);

    /*Construct the attributes to configure HCSR04 device */
    /*ECHO/TRIGGER/SAMPLING PERIOD/NUMBER_OF_SAMPLES. Initialized to default*/
    nla_put_u32(msg, HCSR04_ECHO_PIN_ATTR, HCSR04_ECHO_PIN);
    nla_put_u32(msg, HCSR04_TRIGGER_PIN_ATTR, HCSR04_TRIGGER_PIN);
    nla_put_u32(msg, HCSR04_SAMPLING_PERIOD_ATTR,USER_SAMPLING_PERIOD);
    nla_put_u32(msg, HCSRO4_NUMBER_SAMPLES_ATTR, USER_NUM_SAMPLES);

    /*Send Request to configure the HCSR04 device*/
    nl_send_auto(netlink_socket, msg);
	
    /*Check for any configuration errors */
    if(nl_recvmsgs(netlink_socket, cb)){
        goto failure;
    }

    printf("\t\t\t\tAbout to configure matrix led \n");
    
    nlmsg_free(msg);
    msg = nlmsg_alloc();

    /*Preparing the next message command to condigure MAX 7219 device */
    if(!genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, family_num, 0, NLM_F_REQUEST, CONFIGURE_MAX7219, 0)){
        printf("Failed to put netlink header. Exiting... \n");
        return -1;
    }

    /*Constructing attributes. Added CS_PIN*/
    nla_put_u32(msg, MAX7219_CS_PIN_ATTR, MAX7219_CS_PIN);

    nl_send_auto(netlink_socket, msg);
    
     /*Check for any configuration errors */
    if(nl_recvmsgs(netlink_socket, cb)){
        goto failure;
    }

    printf("\t\t\t\tAbout to Initiate measurement \n");
    
    nlmsg_free(msg);
    msg = nlmsg_alloc();


    /*Command to initiate the distance measurement operation*/
    nla_put_u32(msg, HCSR04_SECONDS_TO_RUN_ATTR, USER_NUM_SECONDS_TO_RUN);
    if(!genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, family_num, 0, NLM_F_REQUEST, INITIATE_MEASUREMENT, 0)){
        printf("Failed to put netlink header. Exiting... \n");
        return -1;
    }

    nl_send_auto(netlink_socket, msg);

    /*Creating the thread to listen to the messages that are being sent by the kernel*/
    return_value = pthread_create(&thread_dev1, NULL, receiveMeasurements, (void *)netlink_socket);
  
    if(return_value){
        printf("Error in creating thread for device 1\n");
        goto failure;
    }
    
    printf("Thread creation successful to receive messages...\n");
    printf("\t\t\t\tAbout to relay the display patterns \n");
    nlmsg_free(msg);
    msg = nlmsg_alloc();

    /*Command to start sending the display pattern*/
    if(!genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, family_num, 0, NLM_F_REQUEST, DISPLAY_PATTERN, 0)){
        printf("Failed to put netlink header. Exiting... \n");
        return -1;
    }

    /*Thread to start relaying the display pattern. Manipulations to the pattern will be handled in the thread*/
    return_value = pthread_create(&thread_dev2, NULL, sendPattern, (void *)netlink_socket);
  
    if(return_value){
        printf("Error in creating thread for device 1\n");
        goto failure;
    }


    printf("Thread creation successful to send the patterns...\n");

    /*Joining the distance and display threads after successful completion*/

    pthread_join(thread_dev1, NULL);
    pthread_join(thread_dev2, NULL);
	nl_cb_put(cb);

failure:
    /* Destroying the message */
    nlmsg_free(msg);

    /* Destroy the socket */
    nl_socket_free(netlink_socket);

    /*destroying mutex*/
    pthread_mutex_destroy(&termination_flag_lock);
    pthread_mutex_destroy(&read_write_lock);

    printf("Exiting Application \n");
    return 0;
}