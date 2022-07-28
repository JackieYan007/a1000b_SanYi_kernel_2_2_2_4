#include <linux/ipc_interface.h>

#include "ipc_top_interface.h"
#include <linux/ipc_interface.h>
#include "ipc_common.h"
// #include "ipc_interface.h"

#define IPC_DRIVER_NAME "ipc_top_interface"

int32_t ipc_sync_send_msg(struct _usr_sync_msg *sync_msg)
{
    IPC_LOG_DEBUG("enter");
    int32_t session_id = sync_msg->session_id;
    int32_t ret;
    ipc_msg send_msg, recv_msg;
    // sync_msg->send_msg.sync_call = true;
    
    // memcpy(&send_msg, &sync_msg->send_msg,sizeof(user_ipc_msg));
    send_msg.type = sync_msg->send_msg.type;
    send_msg.cmd = sync_msg->send_msg.cmd;
    send_msg.data = sync_msg->send_msg.data;
    ret = ipc_send(session_id, &send_msg, -1);
    if (ret < 0)
    {
        IPC_LOG_ERR("send message failed!");
        return -1;
    }
   
    recv_msg.token = send_msg.token;
    IPC_LOG_INFO("send_msg.token = %d", send_msg.token);

    ret = ipc_recv_reply(session_id, &recv_msg, sync_msg->timeout);
    if (ret < 0)
    {
        IPC_LOG_ERR("recv message failed!");
        return -1;
    }
    // memcpy(&sync_msg->recv_msg, &recv_msg, sizeof(user_ipc_msg));
    sync_msg->recv_msg.type = recv_msg.type;
    sync_msg->recv_msg.cmd = recv_msg.cmd;
    sync_msg->recv_msg.token = recv_msg.token;
    sync_msg->recv_msg.data = recv_msg.data;

    IPC_LOG_INFO("ret= %d", ret);
	IPC_LOG_INFO("recv data= %d", sync_msg->recv_msg.data);
    IPC_LOG_INFO("cmd= %d", sync_msg->recv_msg.cmd);
    IPC_LOG_INFO("type= %d", sync_msg->recv_msg.type);

    IPC_LOG_DEBUG("%s", "exit");
    return 0;
}

int32_t ipc_async_send_msg(struct _usr_async_msg *async_send_msg)
{
    IPC_LOG_DEBUG("enter");
    int32_t session_id = async_send_msg->session_id;
    int32_t ret;

    // async_send_msg->send_msg.sync_call = false;
    // IPC_LOG_INFO("async_send_msg.send_msg.userdata = 0x%x", async_send_msg->send_msg.userdata);
    ret = ipc_async_send(session_id, &async_send_msg->send_msg, -1);
    if (ret < 0)
    {
        IPC_LOG_ERR("send message failed!");
        return -1;
    }

    IPC_LOG_DEBUG("%s", "exit");
    return 0;
}