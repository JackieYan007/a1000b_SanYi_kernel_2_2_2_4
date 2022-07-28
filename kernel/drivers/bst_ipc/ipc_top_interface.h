#ifndef IPC_TOP_INTERFACE
#define IPC_TOP_INTERFACE

#include <linux/ipc_interface.h>
#include "user_head.h"


int32_t ipc_sync_send_msg(struct _usr_sync_msg *sync_send_msg);

int32_t ipc_async_send_msg(struct _usr_async_msg *async_send_msg);


#endif