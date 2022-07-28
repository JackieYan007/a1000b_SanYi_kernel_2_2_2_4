#ifndef IPC_COMMUNICATION_H
#define IPC_COMMUNICATION_H

#include <linux/kfifo.h>

#include "ipc_common.h"
#include <linux/ipc_interface.h>

struct ipc_mbox_client_info;

/**
 * ipc_drv_send: send a message from ARM to other cores.
 *
 * @dest: receiver's enum ipc_core_e value.
 * @buf: data to send.
 * @len: data length, now @len is ignored, can be any value.
 *
 * RETURN: 0 means send success, negative integaer means error occurs.
 */
// int32_t ipc_drv_send(enum ipc_core_e src, enum ipc_core_e dest, void* buf, uint32_t len);
int32_t ipc_drv_send(enum ipc_core_e src, enum ipc_core_e dest, int32_t session_id, void *buf, uint32_t len);
int32_t ipc_drv_recv(enum ipc_core_e src, enum ipc_core_e dest, void* buf, uint32_t len);

int32_t ipc_communication_create(void);
int32_t ipc_communication_close(void);

bool is_cmd_valid(enum ipc_core_e core, uint32_t cmd);
int32_t subscribe_cmd_of_core(enum ipc_core_e core, uint32_t cmd);
int32_t init_all_core(bool user);

//msg size increase to 128 bit
struct ipc_fill_register_msg
{
    #ifdef MSG_SIZE_EXTENSION
    uint64_t long_data:64;
    #endif
    uint32_t long_param:32;
    uint32_t short_param:16;
    uint32_t cmd:8;
    uint32_t ack:5;
    uint32_t wakeup:1;
    uint32_t type:2;
};

struct ipc_aligned_msg
{
    struct ipc_fill_register_msg msg;
    #ifdef MSG_SIZE_EXTENSION
    uint64_t reserved[6];
    #else
    uint64_t reserved[7];
    #endif
};

struct ipc_all_cores_register_addr  // place in one page : 4096Byte
{
    struct ipc_aligned_msg addr[IPC_CORE_MAX];
 
    char dsp_cv1_log[128];
    char dsp_cv2_log[128];
    char dsp_cv3_log[128];
    char dsp_cv4_log[128];
    char vsp_log[128];
    char dsp_log[128];
    char isp_log[128];
    char r5_log[128];
};

struct recv_data
{
	enum ipc_core_e core;
	uint64_t payload;
};

#endif
