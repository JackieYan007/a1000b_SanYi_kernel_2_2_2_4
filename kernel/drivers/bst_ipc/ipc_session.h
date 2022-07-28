#ifndef IPC_SESSION_H
#define IPC_SESSION_H

#include <linux/list.h>
#include <linux/kfifo.h>

#include "ipc_communication_manager.h"
#include "user_head.h"

#define SESSION_NUM     64 		// must be times of 64(bitmap is used)

struct ipc_drv_msg;
struct ipc_mbox_client_info;

typedef enum ipc_session_status {
	SESSION_STATE_NULL   = 0x0,
	SESSION_INIT         = 0x1,
	SESSION_READY        = 0x2,
	SESSION_SENDING      = 0x4,
	SESSION_RECEIVING    = 0x10,
	SESSION_WAIT_RECEIVE = 0x20,
	SESSION_WAIT_SEND    = 0x30,
	SESSION_SENT		 = 0x31,
	SESSION_RECEIVED	 = 0x32,
	SESSION_DESTROY      = 0x40,
	SESSION_STATUS_MAX   = INT_MAX
} IPC_SESSION_STATUS;

struct ipc_session{
	int32_t id; //id of this session
	struct fasync_struct * ipc_fasync;
	pid_t pid_num; // for user space
	IPC_SESSION_STATUS status;
	struct kfifo recv_msg_fifo;
	enum ipc_core_e src; // bound to specified cpu core, that is the src core
	enum ipc_core_e dest;
	struct completion rx_reply_complete;
	struct completion rx_signal_complete;
	struct completion rx_method_complete;
	struct completion tx_complete;
	int32_t waiting_reply_msg_token;
	// void __iomem* tx_reg;
	// void __iomem* sem_reg;
	struct ipc_mbox_client_info* cl_info;
};

int32_t ipc_register_session(enum ipc_core_e src, struct ipc_session ** pp_session,
					const struct ipc_mbox_client_info* cl_info);
struct ipc_session * ipc_node_search(pid_t pid);
bool ipc_session_valid(int32_t id);
struct ipc_session * get_session_by_id(int32_t id);
struct ipc_session * ipc_session_by_pid(pid_t pid);
int32_t ipc_session_msg_in(int32_t session_id, struct ipc_drv_msg msg_addr);
struct ipc_drv_msg  ipc_session_msg_out(struct ipc_session *session);
struct ipc_drv_msg * read_msg_by_pid(pid_t pid);
int32_t ipc_session_destroy(pid_t pid);
int32_t ipc_session_destroy_by_id(int32_t session_id);
void list_init(void);
struct ipc_session *find_session_by_coreid(enum ipc_core_e dest);
int32_t request_a_session_id(void);
IPC_SESSION_STATUS get_session_status(int32_t session_id);
int32_t set_session_status(int32_t session_id, IPC_SESSION_STATUS status);

#endif
