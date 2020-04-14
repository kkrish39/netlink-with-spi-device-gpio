
#ifndef GENL_TEST_H
#define GENL_TEST_H

#include <linux/netlink.h>

#ifndef __KERNEL__
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#endif

#define NETLINK_FAMILY_NAME	"genl_test"

#define MULTICAST_GROUP0 "group0"
#define MULTICAST_GROUP1 "group1"
#define MULTICAST_GROUP2 "group2"

#define MAX_BUF_LENGTH		256

#define GENL_TEST_MCGRP_MAX 3

enum {
	GENL_TEST_C_UNSPEC,		/* Must NOT use element 0 */
	CONFIGURE_HCSR04,
	CONFIGURE_MAX7219,
	INITIATE_MEASUREMENT,
	DISPLAY_PATTERN
};

enum genl_test_multicast_groups { GROUP0, GROUP1, GROUP2 };

static char* genl_test_mcgrp_names[GENL_TEST_MCGRP_MAX] = {
	MULTICAST_GROUP0,
	MULTICAST_GROUP1,
	MULTICAST_GROUP2
};

enum genl_test_attrs {
	GENL_TEST_ATTR_UNSPEC,		/* Must NOT use element 0 */
	HCSR04_ECHO_PIN,
	HCSR04_TRIGGER_PIN,
	HCSR04_SAMPLING_PERIOD,
	HCSRO4_NUMBER_SAMPLES,
	CALLBACK_IDENTIFIER,
	RET_VAL_SUCCESS,
	RET_VAL_FAILURE,
	OPTIONAL_ERROR_MESSAGE,
	__GENL_TEST_ATTR__MAX
};

#define GENL_TEST_ATTR_MAX (__GENL_TEST_ATTR__MAX - 1)

static struct nla_policy genl_test_policy[GENL_TEST_ATTR_MAX+1] = {
	[HCSR04_ECHO_PIN] = {
		.type = NLA_U32
	},
	[HCSR04_TRIGGER_PIN] = {
		.type = NLA_U32
	},
	[HCSR04_SAMPLING_PERIOD] = {
		.type = NLA_U32
	},
	[HCSRO4_NUMBER_SAMPLES] = {
		.type = NLA_U32
	},
	[CALLBACK_IDENTIFIER] = {
		.type = NLA_U32
	},
	[RET_VAL_FAILURE] = {
		.type = NLA_FLAG
	},
	[RET_VAL_SUCCESS] = {
		.type = NLA_FLAG
	},
	[OPTIONAL_ERROR_MESSAGE] ={
		.type = NLA_STRING,
#ifdef __KERNEL__
		.len = MAX_BUF_LENGTH
#else
		.maxlen = MAX_BUF_LENGTH
#endif
	}
};

#endif