/*
* © Copyright Black Sesame Technologies (Shanghai)Ltd. Co., 2020. All rights reserved.
*
* This file contains proprietary information that is the sole intellectual property of
* Black Sesame Technologies (Shanghai)Ltd. Co.
*
* No part of this material or its documentation may be reproduced, distributed,
* transmitted, displayed or published in any manner without the written permission
* of Black Sesame Technologies (Shanghai)Ltd. Co.
*/

#include <linux/slab.h>
#include <linux/tee_drv.h>
#include <linux/types.h>

#include "optee_private.h"
#include "call_sec.h"

int tee_get_cpu_id(uint64_t *id)
{
    int rc;
    struct optee_msg_arg *arg;

    arg = kzalloc(OPTEE_MSG_GET_ARG_SIZE(1), GFP_KERNEL);
	if (!arg)
		return -ENOMEM;

    arg->func = eDEVID_GET;
    arg->num_params = 1;
    arg->params[0].attr = OPTEE_MSG_ATTR_TYPE_TMEM_OUTPUT;
    arg->params[0].u.tmem.buf_ptr = (uint64_t)id;
    arg->params[0].u.tmem.size = 16;

    rc = optee_call_secure(NULL, arg);
    kfree(arg);

    return rc;
}
EXPORT_SYMBOL(tee_get_cpu_id);

