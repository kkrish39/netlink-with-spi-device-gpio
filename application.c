#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netlink/msg.h>
#include <netlink/attr.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>

#include "genl_ex.h"

#define USER_HCSR04_ECHO_PIN 5
#define USER_HCSR04_TRIGGER_PIN 1
#define USER_SAMPLING_PERIOD 80
#define USER_NUM_SAMPLES 8

static int skip_seq_check(struct nl_msg *msg, void *arg)
{
	return NL_OK;
}

static int entry_message(struct nl_msg *msg, void *arg){
    printf("Received a message \n");
    return NL_OK;
}
static int print_rx_msg(struct nl_msg *msg, void* arg)
{
	struct nlattr *attr[GENL_TEST_ATTR_MAX+1];

	genlmsg_parse(nlmsg_hdr(msg), 0, attr, GENL_TEST_ATTR_MAX, genl_test_policy);

	if (!attr[GENL_TEST_ATTR_MSG]) {
		fprintf(stdout, "Kernel sent empty message!!\n");
		return NL_OK;
	}

	fprintf(stdout, "Kernel says: %s \n", nla_get_string(attr[GENL_TEST_ATTR_MSG]));
	return NL_OK;
}

int main() {
    struct nl_sock *netlink_socket;
    struct nl_msg *msg;
    struct nl_cb *cb = NULL;
    int group_id;

    int family_num;
    int ret;
    /*Initializing the netlink socket*/
    netlink_socket = nl_socket_alloc();

    if(!netlink_socket){
        printf("Failed to allocate netlink socket. Exiting... \n");
        return -1;
    }

    nl_socket_disable_seq_check(netlink_socket);
    nl_socket_disable_auto_ack(netlink_socket);
    
    if(genl_connect(netlink_socket)){
        printf("Failed to initalize the generic netlink. Exiting... \n");
        return -1;
    }

    family_num = genl_ctrl_resolve(netlink_socket, NETLINK_FAMILY_NAME);
    if(family_num < 0){
        printf("Invalid Family Name. Exiting... \n");
        return -1;
    }

    /*resolve group*/
    group_id = genl_ctrl_resolve_grp(netlink_socket, NETLINK_FAMILY_NAME, genl_test_mcgrp_names[0]);

    if(group_id < 0){
        printf("Invalid group id \b");
        return -1;
    }

    if(nl_socket_add_membership(netlink_socket, group_id)){
        printf("Failed to register membership \n");
        return -1;
    }

    msg = nlmsg_alloc();
    if(!msg){
        printf("Failed to allocate message \n");
        return -1;
    }
    /*
    * msg -> Message to be sent
    * NL_AUTO_PORT - Netlink port to be set by the netlink socket before sending
    * NL_AUTO_SEQ - Sequence number which should be set automatically
    * family_num - Registered familyid
    * headerLength - 0
    * Additional Flags
    * cmd - message specific command
    * version - Interface version
    */
    // if(!genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, family_num, 0, NLM_F_REQUEST, GENL_TEST_C_MSG, 0)){
    //     printf("Failed to put netlink header. Exiting... \n");
    //     return -1;
    // }

    // ret = nla_put_string(msg, GENL_TEST_ATTR_MSG, messageBuffer);
    // if(ret){
    //     printf("Failed to put nl string. Exiting... \n");
    //     return -1;
    // }


    // ret = nla_put_string(msg, 20, messageBuffer);
    // if(ret){
    //     printf("Failed to put nl string. Exiting... \n");
    //     return -1;
    // }

    /* 
    * nl_send_auto(struct *nl_sock, struct *nl_msg) 
    * Finalize and Transmit Netlink message 
    */
    ret = nl_send_auto(netlink_socket, msg);
    if(ret < 0){    
        printf("Failed to send message. Exiting... \n");
        return -1;
    }   

    /* prep the cb */
	cb = nl_cb_alloc(NL_CB_DEFAULT);
	nl_cb_set(cb, NL_CB_SEQ_CHECK, NL_CB_CUSTOM, skip_seq_check, NULL);
    nl_cb_set(cb, NL_CB_MSG_IN, NL_CB_CUSTOM, entry_message, NULL);
	nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, print_rx_msg, NULL);
	
	ret = nl_recvmsgs(netlink_socket, cb);
    
    if(!ret){
        nlmsg_free(msg);
        msg = nlmsg_alloc();
        if(!genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, family_num, 0, NLM_F_REQUEST, CONFIGURE_DEVICE, 0)){
            printf("Failed to put netlink header. Exiting... \n");
            return -1;
        }

        nla_put_u32(msg, HCSR04_ECHO_PIN, USER_HCSR04_ECHO_PIN);
        nla_put_u32(msg, HCSR04_TRIGGER_PIN, USER_HCSR04_TRIGGER_PIN);
        nla_put_u32(msg, HCSR04_SAMPLING_PERIOD,USER_SAMPLING_PERIOD);
        nla_put_u32(msg, HCSRO4_NUMBER_SAMPLES, USER_NUM_SAMPLES);

        nl_send_auto(netlink_socket, msg);
    }else{

    }

    ret = nl_recvmsgs(netlink_socket, cb);

    if(!ret){

    }else{

    }

	nl_cb_put(cb);

    /* Destroying the message */
    nlmsg_free(msg);
    

    /*Preparing for the callback*/

    /* Destroy the socket */
    nl_socket_free(netlink_socket);
    printf("Exiting Application \n");
    return 0;
}