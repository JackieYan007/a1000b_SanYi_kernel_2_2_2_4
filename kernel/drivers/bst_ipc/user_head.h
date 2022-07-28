#ifndef __USER_HEAD_H_
#define __USER_HEAD_H_

// TODO: copyrights
#include <stdbool.h>
#include <linux/ipc_interface.h>

#define MAX_CMD_ID 256


/**
 * struct ipc_buffer - struct description of alloc memory in kernel
 * @size:		alloc memory size
 * @align:		align size of memory
 * @uaddr:		memory virtual address in kernel
 * @handle:		handle of memory
 * @phy_addr:	memory physical address in DMA
 */
struct ipc_buffer {
    uint32_t size;
    uint32_t align;
    uint64_t uaddr;
    uint64_t handle;
    union {
        struct {
            uint32_t low;
            uint32_t high;
        };
        uint64_t ptr_64;
    }phy_addr;
};

/**
 * struct msg_type - struct description of send message type
 *
 * this struct is used for user to fill up send message
 *
 * @short_param:    cmd type, determined by commucicating parties, max count is 256
 * @long_param:		message content, determined by communicating parties, message max size is 4 bytes
 */
struct msg_type{
	uint32_t cmd;  //cmd type param:
	uint32_t long_param; //valid address list number
};


//user space definition
//init
struct _usr_init {
	enum ipc_core_e src;
	enum ipc_core_e dst;
    uint32_t session_id;
};

//send or recv msg
struct _usr_send_or_recv_msg
{
    user_ipc_msg t_msg;
    int32_t session_id;
    int32_t timeout;
};

typedef void (*async_func_calllback)(int32_t session_id, ipc_msg *re_msg);
//async call
struct _usr_async_msg
{
    async_ipc_msg send_msg;
    int32_t session_id;
    pid_t pid;
};




//sync call
struct _usr_sync_msg {
	user_ipc_msg send_msg;
	user_ipc_msg recv_msg;
    int32_t session_id;
    int32_t timeout;
};

/**
 * definition of ioctl cmd
*/
#define IPC_IO_MEM 'M'
#define IPC_IO_MEM_ALLOC        	_IO(IPC_IO_MEM, 1)
#define IPC_IO_MEM_FREE	            _IO(IPC_IO_MEM, 2)
#define IPC_IO_MSG_SEND_SYNC	    _IO(IPC_IO_MEM, 3)
#define IPC_IO_SES_CREATE           _IO(IPC_IO_MEM, 4)
#define IPC_IO_SES_DESTROY          _IO(IPC_IO_MEM, 5)
#define IPC_IO_DRI_READY            _IO(IPC_IO_MEM, 6)
#define IPC_IO_MSG_SEND_ASYNC       _IO(IPC_IO_MEM, 7)
#define IPC_IO_MSG_SEDN             _IO(IPC_IO_MEM, 8)
#define IPC_IO_MSG_RECV             _IO(IPC_IO_MEM, 9)
 


#endif

