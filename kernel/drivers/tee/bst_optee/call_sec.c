/*
* This file contains proprietary information that is the sole intellectual 
* property of Black Sesame Technologies, Inc. and its affiliates. 
* No portions of this material may be reproduced in any 
* form without the written permission of: 
* Black Sesame Technologies, Inc. and its affiliates 
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050 
* Copyright @2016: all right reserved. 
*/

#include <linux/printk.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/tee_drv.h>
#include <linux/types.h>
#include <linux/ipc_interface.h>
#include <linux/delay.h>
#include "optee_private.h"
#include "call_sec.h"

#define CALL_MAX_RETRY 100
#define CALL_SEC_ERR -1

T_IN_BUFF t_in_buf[4] = {0};
extern struct optee *optee_svc;

static int save_input_buff(struct optee_msg_arg *msg_arg, struct optee *optee)
{
    struct tee_shm *shm;
    phys_addr_t in_pa;
    void *in_va;
    int in_size, total_size = 0, i = 0;
    int rc;
    size_t n;

    for (n = 0; n < msg_arg->num_params; n++) {
        if (msg_arg->params[n].attr == OPTEE_MSG_ATTR_TYPE_TMEM_INPUT) {
            shm = (struct tee_shm *)msg_arg->params[n].u.tmem.shm_ref;
            in_pa = (phys_addr_t)msg_arg->params[n].u.tmem.buf_ptr;
            in_size = msg_arg->params[n].u.tmem.size;
            rc = tee_shm_pa2va(shm, in_pa, &in_va);
            memcpy(optee->dma_vaddr + SECURE_REQ_OBJ_SIZE + total_size, in_va, in_size);
            t_in_buf[i].buff_paddr = optee->dma_phy_addr + SECURE_REQ_OBJ_SIZE + total_size;
            t_in_buf[i].buff_size = in_size;
            i += 1;
            total_size += in_size;
        }
    }
    return 0;
}

int optee_call_secure(struct tee_context *ctx, struct optee_msg_arg *msg_arg)
{
    int rc;
    size_t n;
    T_RING_OBJ req_obj;
    u32 out_size = 0, req_ack[64];
    struct tee_shm *shm_out;
    phys_addr_t buff_pa;
    void *buff_va = NULL;
    ipc_msg send_msg, recv_msg;
    struct optee *optee;

    if (ctx)
        optee= tee_get_drvdata(ctx->teedev);
    else
        optee = optee_svc;
    if (!optee) {
        return CALL_SEC_ERR;    
    }

    if (optee->used) {
        u32 send_cnt = CALL_MAX_RETRY;
        while(send_cnt--) {
            if (!optee->used) {
                break;
            }
            msleep(1);
        }
    }
    if (optee->used) {
        pr_err("call secure fail");
        return CALL_SEC_ERR;
    }
    optee->used = 1;
    save_input_buff(msg_arg, optee);
    memset(&req_obj, 0, SECURE_REQ_OBJ_SIZE);

    switch(msg_arg->func){
        case eDEVID_GET:{
            req_obj.cpu_idx = IPC_CORE_ARM4;
            req_obj.is_secure_idle = 0;
            req_obj.req_stage = 0;
            req_obj.req.req_type = eDEVID_GET;
            req_obj.req.idx = 0;
        }
        break;
        case eHASH:{
            req_obj.cpu_idx = IPC_CORE_ARM4;
            req_obj.is_secure_idle = 0;
            req_obj.req_stage = 0;
            req_obj.req.req_type = eHASH;
            req_obj.req.idx = 0;
            req_obj.req.para1 = t_in_buf[0].buff_paddr;
            req_obj.req.para2 = t_in_buf[0].buff_size;
            req_obj.req.para3 = msg_arg->params[1].u.value.a;
        }
        break;
        case eTRNG:{
            req_obj.cpu_idx = IPC_CORE_ARM4;
            req_obj.is_secure_idle = 0;
            req_obj.req_stage = 0;
            req_obj.req.req_type = eTRNG;
            req_obj.req.idx = 0;
            req_obj.req.para1 = (u32)msg_arg->params[0].u.tmem.size;
        }
        break;

        default:{
            pr_err("call sec func is invalid");
            rc = CALL_SEC_ERR;
            goto out;
        }
        break;
    }

    memcpy(optee->dma_vaddr, (void *)&req_obj, SECURE_REQ_OBJ_SIZE);
	send_msg.type = IPC_MSG_TYPE_SIGNAL;
	send_msg.cmd = 0;
	send_msg.token = 0;
	send_msg.data = (uint32_t)optee->dma_phy_addr;
    rc = ipc_send(optee->ipc_session_id, &send_msg, -1);
    if (rc) {
        pr_err("ipc send err, rc: %d", rc);
        goto out;
    }
    rc = ipc_recv(optee->ipc_session_id, &recv_msg, 1000);
    if (rc)
        goto out;
    for (n = 0; n < msg_arg->num_params; n++) {
        if (msg_arg->params[n].attr == OPTEE_MSG_ATTR_TYPE_TMEM_OUTPUT) {
            if (ctx) {
                shm_out = (struct tee_shm *)msg_arg->params[n].u.tmem.shm_ref;
                buff_pa = (phys_addr_t)msg_arg->params[n].u.tmem.buf_ptr;
                out_size = msg_arg->params[n].u.tmem.size;
                rc = tee_shm_pa2va(shm_out, buff_pa, &buff_va);
                if (rc)
                    goto out;
                break;
            } else {
                buff_va = (void *)msg_arg->params[n].u.tmem.buf_ptr;
                out_size = msg_arg->params[n].u.tmem.size;
            }
        }
    }

    if (send_msg.data == recv_msg.data) {
        int req_stage;
        memcpy(&req_stage, optee->dma_vaddr+4, 4);
        if (req_stage != 2) {
            pr_err("req_stage err:%d", req_stage);
            rc = CALL_SEC_ERR;
            goto out;
        }
        memcpy(req_ack, optee->dma_vaddr + SECURE_REQ_OBJ_SIZE - sizeof(req_ack), sizeof(req_ack));
        if (buff_va) {
            memcpy(buff_va, (void *)&req_ack, out_size);
        }

    } else {
        pr_err("ipc recv msg error");
        rc = CALL_SEC_ERR;
    }
out:
    optee->used = 0;
    if(rc)
        return rc;
    return 0;
}


