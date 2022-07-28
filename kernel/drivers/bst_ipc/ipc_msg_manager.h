#ifndef IPC_MSG_MANAGER_H
#define IPC_MSG_MANAGER_H

#include <linux/list.h>
#include <linux/kfifo.h>
#include <linux/types.h>

#include "ipc_communication_manager.h"


/**
 * struct ipc_drv_msg - struct msg description send in ipc service
 *
 * this struct would describe message in detail and transferred in whole ipc service
 *
 * @this_phy_addr:		message storage physical address
 * @phy_addr_offset:	physical address offset
 * @session_id:		    message session id, every thread need create a session and fill this parameter in message type automatically.
 * @userdata:		    user application data.
 *                      if send message is synchronous, it will save wait semophare;
 *                      when if send message is asynchronous, it will save callback function address
 *
 * @cmd:	            cmd type, determined by commucicating parties, max count is 256, filled by user
 * @src:                message source
 * @dst:                message destination, filled by user
 * @long_param:         message content, determined by communicating parties, message max size is 4 bytes, filled by user
 * @token:              message sequence number, assigned by ipc service in kernel
 * @type:               message type, refer to enum msg_type_e
 * @client_info:        used in ipc service
 * @sync_call:          flag of synchronous message or asynchronous message
 */
struct ipc_drv_msg{
	uint64_t this_phy_addr;
	uint64_t phy_addr_offset;
    uint64_t userdata;
	#ifdef MSG_SIZE_EXTENSION
	uint64_t long_data;
	#endif
	
	int32_t session_id;
	uint32_t cmd;  //cmd type param:
	uint32_t src;
	uint32_t dst;
	uint32_t long_param;
	uint32_t token;
	ipc_msg_type type;  //ipc msg type
    void* client_info;
	bool sync_call;
	struct hlist_node node;
	uint32_t tx_timeout;
	int64_t timestamp;
};

int32_t send_msg_init(void);
int32_t send_msg_destroy(void);
int32_t send_msg_in(struct ipc_drv_msg* msg);
int32_t send_msg_out(uint32_t token, struct ipc_drv_msg** msg);
struct ipc_drv_msg * find_ipcmsg_by_token(uint32_t token);
uint32_t ipc_msg_get_an_available_token(void);
int32_t show_all_msgs(void);
#endif
