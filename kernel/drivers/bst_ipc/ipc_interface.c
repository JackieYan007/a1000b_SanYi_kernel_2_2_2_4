#include <linux/ipc_interface.h>

#include <linux/rwsem.h>
#include <linux/device.h>
#include <linux/of.h>
#include <asm/memory.h>

#include "ipc_communication_manager.h"
#include "ipc_session.h"
#include "user_head.h"
#include "ipc_mailbox_client.h"
#include "ipc_regs.h"
#include "ipc_mailbox_controller.h"
#include "ipc_msg_manager.h"
#include "ipc_common.h"
#include <linux/delay.h>

#define IPC_DRIVER_NAME		"ipc_interface"

#define SEND_MAX_TIMEOUT 200
/********************* extern global variables *******************/
extern struct platform_device * g_mbox_controller_platform_dev;
extern bool ipc_init_status;
static DEFINE_SPINLOCK(count_lock);

struct diag_info diagnose_info[SESSION_NUM];  //index of this array is session id

int32_t recv_msg_callback(enum ipc_core_e src, enum ipc_core_e dest, void* buf, u32 len, ktime_t timestamp)
{
	struct ipc_fill_register_msg *fill_msg = NULL;
    struct ipc_drv_msg *ipc_drv_msg = NULL;
    struct ipc_session* ipc_session = NULL;
    ipc_msg reply_msg;

	IPC_LOG_DEBUG( "enter");
    fill_msg = (struct ipc_fill_register_msg*)buf;
    if (!fill_msg)
    {
        IPC_LOG_WARNING("fill_msg is NULL");
        return -1;
    }
    
    IPC_LOG_INFO("src = %d", src);
    IPC_LOG_INFO("dest = %d", dest);
    IPC_LOG_INFO( "fill_msg.short_param(token) = %d", fill_msg->short_param);
    IPC_LOG_INFO( "fill_msg.long_param = %x", fill_msg->long_param);
    IPC_LOG_INFO("fill_msg.type = %d", fill_msg->type);

    ipc_session = find_session_by_coreid(src);
    if (ipc_session == NULL)
    {
        IPC_LOG_WARNING("session is NULL");
        return -1;
    }

#ifdef IPC_INTERFACE_MAKE_ALL_MSGs_BE_SIGNAL
    fill_msg->type = IPC_MSG_TYPE_SIGNAL; // TODO make things easy understood
#endif

    if(fill_msg->type == IPC_MSG_TYPE_SIGNAL)
    {
        struct ipc_drv_msg ipc_drv_msg;
        ipc_drv_msg.type = fill_msg->type;
        ipc_drv_msg.token = fill_msg->short_param;
        ipc_drv_msg.cmd = fill_msg->cmd;
        ipc_drv_msg.long_param = fill_msg->long_param;
        ipc_drv_msg.timestamp = timestamp;
        ipc_session_msg_in(ipc_session->id, ipc_drv_msg);
        complete(&ipc_session->rx_signal_complete);
    }
    else if(fill_msg->type == IPC_MSG_TYPE_REPLY)
    {
        struct ipc_drv_msg *ipc_drv_msg;
        uint32_t ret;
        // bool sync_call_flag;
        send_msg_out(fill_msg->short_param, &ipc_drv_msg);
        if(IS_ERR_OR_NULL(ipc_drv_msg))
        {
            IPC_LOG_WARNING("sender %d sent a message with invalid token %d", src, fill_msg->short_param);
            // ipc_drv_msg = devm_kzalloc(&g_mbox_controller_platform_dev->dev, sizeof(*ipc_drv_msg), GFP_KERNEL);
            IPC_LOG_ERR("err msg");
            return -1;   
        }
        else
        {
            ipc_session = get_session_by_id(ipc_drv_msg->session_id);
            if (ipc_session == NULL)
            {
                IPC_LOG_ERR("ipc_session is null!");
                return -1;
            }
            ipc_session->waiting_reply_msg_token = -1;
        }

        // printk("recv_msg_callback ipc_drv_msg address = 0x%px", ipc_drv_msg);
        IPC_LOG_INFO("ipc_drv_msg->sync_call = %d", ipc_drv_msg->sync_call);
        // IPC_LOG_INFO("ipc_drv_msg->userdata = 0x%x", ipc_drv_msg->userdata);
        
        ipc_drv_msg->type = fill_msg->type;
        ipc_drv_msg->token = fill_msg->short_param;
        ipc_drv_msg->cmd = fill_msg->cmd;
        ipc_drv_msg->long_param = fill_msg->long_param;
        ipc_drv_msg->timestamp = timestamp;

        
        IPC_LOG_INFO("ipc_drv_msg->long_param = %d", ipc_drv_msg->long_param);
        //TODO:alloc case
        ret = ipc_session_msg_in(ipc_session->id, *ipc_drv_msg);
        if (ret < 0)
        {
            IPC_LOG_ERR("ipc_session_msg_in failed");
        }
        //free alloced msg in ipc_send func 
        devm_kfree(&g_mbox_controller_platform_dev->dev ,ipc_drv_msg);
        set_session_status(ipc_session->id, SESSION_RECEIVED);
        complete(&ipc_session->rx_reply_complete);
        IPC_LOG_INFO("complete rx_reply_complete succeed");
        // IPC_LOG_INFO("sync_call_flag = %d", sync_call_flag);
        if (!ipc_drv_msg->sync_call)
        {
            kill_fasync(&(ipc_session->ipc_fasync), SIGIO, POLL_IN);
        }
    }
    else // invalid type
    {
        IPC_LOG_ERR("type is invalid , type = %d", fill_msg->type);
        return -4;
    }

	IPC_LOG_INFO("src = %d, session id = %d", src, ipc_session->id);
    IPC_LOG_INFO("fill_msg.type = %d", fill_msg->type);
    IPC_LOG_INFO("fill_msg.long_param = 0x%x", fill_msg->long_param);
    IPC_LOG_DEBUG( "exit");
	return 0;
}

int32_t ipc_init(enum ipc_core_e dest_core_id, enum ipc_core_e src_core_id, void *arg)
{
    int32_t session_id = -1;
    int32_t ret = -1;

    if (!ipc_init_status)
    {
        IPC_LOG_ERR("ipc is not ready");
        return IPC_INIT_ERR;
    }

    IPC_LOG_INFO("enter, dst = %d, src = %d", dest_core_id, src_core_id);
    if (dest_core_id < IPC_CORE_ARM0 || dest_core_id >= IPC_CORE_MAX) {
        IPC_LOG_ERR("dest_core_id is invalid");
        return IPC_INIT_ERR_INVALID_PARAM;
    }
    if (src_core_id < IPC_CORE_ARM0 || src_core_id > IPC_CORE_ARM7) {
        IPC_LOG_ERR("src_core_id is invalid");
        return IPC_INIT_ERR_INVALID_PARAM;
    }

    #ifndef ENABLE_SRC_DST_SAME
    if(dest_core_id == src_core_id)
    {
        IPC_LOG_ERR("dest_core_id == src_core_id == %d", src_core_id);
        return IPC_INIT_ERR_INVALID_PARAM;
    }
    #endif

    session_id = ipc_node_create_session(dest_core_id, src_core_id);
    if (session_id < 0) {
	    IPC_LOG_ERR("ipc_node_create_session return : 0x%d", session_id);
	    ret = IPC_INIT_ERR_SESSION;
	    return ret;
    }
    IPC_TRACE_PRINTK("ipc_node_create_session, session = %d", session_id);

    struct diag_info _info;
    _info.session_id = session_id;
    _info.src = src_core_id;
    _info.dst = dest_core_id;
    _info.num_of_send_msg = 0;
    _info.num_of_recv_msg = 0;
    _info.send_err.ipc_no_ready = 0;
    _info.send_err.session_invalid = 0;
    _info.send_err.no_ACK = 0;
    _info.send_err.queue_full = 0;
    _info.recv_err.ipc_no_ready = 0;
    _info.recv_err.session_invalid = 0;
    _info.recv_err.queue_empty = 0;
    _info.recv_err.timeout = 0;
    diagnose_info[_info.session_id] = _info;

    IPC_LOG_INFO("ipc_node_create_session, session = %d", session_id);

    IPC_LOG_DEBUG("exit");
    return session_id;
}
EXPORT_SYMBOL(ipc_init);

int32_t ipc_async_send(int32_t session_id, async_ipc_msg *msg, int32_t timeout)
{
	int32_t ret = 0;
    uint64_t wait = 0;
	struct ipc_session *session = NULL;
	struct ipc_drv_msg *ipc_drv_msg = NULL;

	IPC_LOG_DEBUG("enter");

    if (!ipc_init_status)
    {
        IPC_LOG_ERR("ipc is not ready");
        return IPC_SEND_ERR;
    }

	session = get_session_by_id(session_id);
    if (session == NULL)
    {
        diagnose_info[session_id].send_err.session_invalid += 1;
        IPC_LOG_WARNING("session is invalid");
        return IPC_SEND_ERR_INVALID_PARAM;
    }

	if (msg->type == IPC_MSG_TYPE_METHOD || msg->type == IPC_MSG_TYPE_SIGNAL) {
		msg->token = ipc_msg_get_an_available_token();
	}

	IPC_LOG_INFO("token = %d", msg->token);

    #ifdef MSG_SIZE_EXTENSION
	struct ipc_fill_register_msg fill_msg = {
        .long_data = msg->long_data,
		.long_param = msg->data,
		.short_param = msg->token,
		.cmd = msg->cmd,
		.ack = 1,
		.wakeup = 0,
		.type = msg->type,
	};
    #else
    struct ipc_fill_register_msg fill_msg = {
		.long_param = msg->data,
		.short_param = msg->token,
		.cmd = msg->cmd,
		.ack = 1,
		.wakeup = 0,
		.type = msg->type,
	};
    #endif

    IPC_LOG_INFO("send_mag->type = %d", fill_msg.type);
    //NOTE:send msg queue reserve
	if (msg->type == IPC_MSG_TYPE_METHOD)
    {
        ipc_drv_msg = devm_kzalloc(&g_mbox_controller_platform_dev->dev,
                    sizeof(*ipc_drv_msg), GFP_KERNEL);
        // printk("ipc_send ipc_drv_msg address = 0x%px", ipc_drv_msg);
        if (!ipc_drv_msg) {
            IPC_LOG_ERR("alloc mem error");
            return -EFAULT;
        }
        ipc_drv_msg->type = msg->type;
        ipc_drv_msg->cmd = msg->cmd;
        ipc_drv_msg->session_id = session_id;
        ipc_drv_msg->dst = session->dest;
        ipc_drv_msg->token = msg->token;
        ipc_drv_msg->src = session->src;
        ipc_drv_msg->long_param = msg->data;
        ipc_drv_msg->tx_timeout = timeout;
        ipc_drv_msg->sync_call = false;
        ipc_drv_msg->userdata = msg->userdata;

        // IPC_LOG_INFO("ipc_drv_msg->sync_call = %d", ipc_drv_msg->sync_call);
        // IPC_LOG_INFO("ipc_drv_msg->userdata = 0x%x", ipc_drv_msg->userdata);

        #ifdef MSG_SIZE_EXTENSION
        ipc_drv_msg->long_data = msg->long_data;
        #endif
        // put sent msg in hashtable for disptach recv msg to session later
        send_msg_in(ipc_drv_msg);
    }

    set_session_status(session_id, SESSION_SENDING);
    ret = ipc_drv_send(session->src, session->dest, session_id, (void*)&fill_msg, sizeof(fill_msg));
	if (ret) {
		IPC_LOG_ERR("ipc_drv_send fail, ret = %d", ret);
		return ret;
	}

    wait = msecs_to_jiffies(SEND_MAX_TIMEOUT);
    set_session_status(session_id, SESSION_WAIT_SEND);
    ret = wait_for_completion_timeout(&session->tx_complete, wait);
    if (ret == 0) { // timeout
        diagnose_info[session_id].send_err.no_ACK += 1;
        ret = IPC_SEND_ERR_TIMEOUT;
        IPC_LOG_ERR("NO ACK send with in 200 milliseconds");
        return ret;
    }

    if (get_session_status(session_id) == SESSION_DESTROY || get_session_status(session_id) == SESSION_STATE_NULL)
    {
        diagnose_info[session_id].send_err.session_invalid += 1;
        IPC_LOG_WARNING("session is destroy");
        return IPC_RECV_ERR_INVALID_PARAM;
    }

    //bugfix:need add lock to ensure counts correct
    uint64_t flags;
    spin_lock_irqsave(&count_lock, flags);
    diagnose_info[session_id].num_of_send_msg += 1;
    spin_unlock_irqrestore(&count_lock, flags);

	IPC_LOG_DEBUG("exit");
	return 0;
}
EXPORT_SYMBOL(ipc_async_send);

int32_t ipc_send(int32_t session_id, ipc_msg *msg, int32_t timeout)
{
	int32_t ret = 0;
    uint64_t wait = 0;
	struct ipc_session *session = NULL;
	struct ipc_drv_msg *ipc_drv_msg = NULL;

	IPC_LOG_DEBUG("enter");

    // if (!msg->sync_call)
    // {   
    //     IPC_LOG_INFO("msg->userdata = 0x%x", msg->userdata);
    // }

    if (!ipc_init_status)
    {
        IPC_LOG_ERR("ipc is not ready");
        return IPC_SEND_ERR;
    }

#ifdef IPC_INTERFACE_MAKE_ALL_MSGs_BE_SIGNAL
     // TODO make things easy understood
    msg->type = IPC_MSG_TYPE_SIGNAL;
#endif
    if (msg->type != IPC_MSG_TYPE_METHOD)
    {
        msg->type = IPC_MSG_TYPE_SIGNAL;
    }

	session = get_session_by_id(session_id);
    if (session == NULL)
    {
        diagnose_info[session_id].send_err.session_invalid += 1;
        IPC_LOG_WARNING("session is invalid");
        return IPC_SEND_ERR_INVALID_PARAM;
    }

	if (msg->type == IPC_MSG_TYPE_METHOD || msg->type == IPC_MSG_TYPE_SIGNAL) {
		msg->token = ipc_msg_get_an_available_token();
	}

	IPC_TRACE_PRINTK("token = %d", msg->token);

    #ifdef MSG_SIZE_EXTENSION
	struct ipc_fill_register_msg fill_msg = {
        .long_data = msg->long_data,
		.long_param = msg->data,
		.short_param = msg->token,
		.cmd = msg->cmd,
		.ack = 1,
		.wakeup = 0,
		.type = msg->type,
	};
    #else
    struct ipc_fill_register_msg fill_msg = {
		.long_param = msg->data,
		.short_param = msg->token,
		.cmd = msg->cmd,
		.ack = 1,
		.wakeup = 0,
		.type = msg->type,
	};
    #endif

    IPC_LOG_INFO("send_mag->type = %d", fill_msg.type);
    //NOTE:send msg queue reserve
	if (msg->type == IPC_MSG_TYPE_METHOD)
    {
        ipc_drv_msg = devm_kzalloc(&g_mbox_controller_platform_dev->dev,
                    sizeof(*ipc_drv_msg), GFP_KERNEL);
        // printk("ipc_send ipc_drv_msg address = 0x%px", ipc_drv_msg);
        if (!ipc_drv_msg) {
            IPC_LOG_ERR("alloc mem error");
            return -EFAULT;
        }
        ipc_drv_msg->type = msg->type;
        ipc_drv_msg->cmd = msg->cmd;
        ipc_drv_msg->session_id = session_id;
        ipc_drv_msg->dst = session->dest;
        ipc_drv_msg->token = msg->token;
        ipc_drv_msg->src = session->src;
        ipc_drv_msg->long_param = msg->data;
        ipc_drv_msg->tx_timeout = timeout;
        ipc_drv_msg->sync_call = true;
        IPC_LOG_INFO("ipc_drv_msg->sync_call = %d", ipc_drv_msg->sync_call);
        
        #ifdef MSG_SIZE_EXTENSION
        ipc_drv_msg->long_data = msg->long_data;
        #endif
        // put sent msg in hashtable for disptach recv msg to session later
        send_msg_in(ipc_drv_msg);
    }

    set_session_status(session_id, SESSION_SENDING);
    ret = ipc_drv_send(session->src, session->dest, session_id, (void*)&fill_msg, sizeof(fill_msg));
	if (ret) {
		IPC_LOG_ERR("ipc_drv_send fail, ret = %d", ret);
		return ret;
	}

    wait = msecs_to_jiffies(SEND_MAX_TIMEOUT);
    set_session_status(session_id, SESSION_WAIT_SEND);
    ret = wait_for_completion_timeout(&session->tx_complete, wait);
    if (ret == 0) { // timeout
        diagnose_info[session_id].send_err.no_ACK += 1;
        ret = IPC_SEND_ERR_TIMEOUT;
        IPC_LOG_ERR("NO ACK send with in 200 milliseconds");
        return ret;
    }

    if (get_session_status(session_id) == SESSION_DESTROY || get_session_status(session_id) == SESSION_STATE_NULL)
    {
        diagnose_info[session_id].send_err.session_invalid += 1;
        IPC_LOG_WARNING("session is destroy");
        return IPC_RECV_ERR_INVALID_PARAM;
    }

    //bugfix:need add lock to ensure counts correct
    uint64_t flags;
    spin_lock_irqsave(&count_lock, flags);
    diagnose_info[session_id].num_of_send_msg += 1;
    spin_unlock_irqrestore(&count_lock, flags);

	IPC_LOG_DEBUG("exit");
	return 0;
}
EXPORT_SYMBOL(ipc_send);

int32_t ipc_recv(int32_t session_id,  ipc_msg* msg, int32_t timeout)
{
    int32_t ret = 0;
    uint64_t wait;
    struct ipc_session* session;
    struct ipc_drv_msg ipc_drv_msg;
    struct completion* completion;

    if (!ipc_init_status)
    {
        diagnose_info[session_id].recv_err.ipc_no_ready += 1;
        IPC_LOG_ERR("ipc is not ready");
        return IPC_RECV_ERR;
    }

    session = get_session_by_id(session_id);
    if (session == NULL)
    {
        diagnose_info[session_id].recv_err.session_invalid += 1;
		IPC_LOG_WARNING("session is invalid(%d)", session_id);
        return IPC_RECV_ERR_INVALID_PARAM;
    }

    #ifdef IPC_INTERFACE_MAKE_ALL_MSGs_BE_SIGNAL
    // TODO make things easy understood
    msg->type = IPC_MSG_TYPE_SIGNAL;
    #endif

    IPC_LOG_INFO("enter, session = 0x%px, dest = %d",session, session->dest);

    wait = msecs_to_jiffies(timeout);

    switch (msg->type)
    {
    case IPC_MSG_TYPE_METHOD:
        completion = &session->rx_method_complete;
        IPC_LOG_INFO("recv waiting IPC_MSG_TYPE_METHOD");
        break;
    case IPC_MSG_TYPE_SIGNAL:
        completion = &session->rx_signal_complete;
        IPC_LOG_INFO("recv waiting IPC_MSG_TYPE_SIGNAL");
        break;
    case IPC_MSG_TYPE_REPLY:
        completion = &session->rx_reply_complete;
        IPC_LOG_INFO("recv waiting IPC_MSG_TYPE_REPLY");
        break;

    default:
        completion = &session->rx_signal_complete;
        IPC_LOG_INFO("recv waiting %d", msg->type);
        break;
    }
    
    set_session_status(session_id, SESSION_WAIT_RECEIVE);
    ret = wait_for_completion_timeout(completion, wait);
    if (ret == 0) { // timeout
        diagnose_info[session_id].recv_err.timeout += 1;
        ret = IPC_RECV_ERR_TIMEOUT;
        IPC_LOG_INFO("NO received with in %d milliseconds", timeout);
        return ret;
    }

    //session status check
    if (get_session_status(session_id) == SESSION_DESTROY || get_session_status(session_id) == SESSION_STATE_NULL)
    {
        diagnose_info[session_id].recv_err.session_invalid += 1;
        IPC_LOG_WARNING("session is destroy");
        return IPC_RECV_ERR_INVALID_PARAM;
    }

    ipc_drv_msg = ipc_session_msg_out(session);
    msg->type  = ipc_drv_msg.type;
    msg->cmd   = ipc_drv_msg.cmd;
    msg->token = ipc_drv_msg.token;
    msg->data  = ipc_drv_msg.long_param;
    msg->timestamp = ipc_drv_msg.timestamp;
    //printk("-------timestamp = %ld, current = %ld\n", msg->timestamp, ktime_get());

    IPC_LOG_DEBUG("exit");
    diagnose_info[session_id].num_of_recv_msg += 1;
    return 0;
}
EXPORT_SYMBOL(ipc_recv);

int32_t ipc_close(int32_t session_id)
{
    IPC_LOG_DEBUG("enter ( no exit)");

    if (!ipc_init_status)
    {
        IPC_LOG_ERR("ipc is not ready");
        return IPC_CLOSE_ERR;
    }

    // TODO send offline signal
    // ipc_msg msg = {
    //     .cmd =IPC_CMD_OFFLINE,
    // };
    // ipc_send_signal(session_id, &msg, 0);
    return ipc_session_destroy_by_id(session_id);
}
EXPORT_SYMBOL(ipc_close);
