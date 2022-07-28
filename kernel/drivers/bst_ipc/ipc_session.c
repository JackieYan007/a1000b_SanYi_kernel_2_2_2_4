#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/rwsem.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <linux/ipc_interface.h>
#include "ipc_mailbox_client.h"
#include "ipc_common.h"
#include "ipc_msg_manager.h"
#include "ipc_nodemanager.h"
#include "ipc_regs.h"
#include "ipc_mailbox_controller.h"
#include "ipc_session.h"


#define IPC_DRIVER_NAME "ipc_session"

/********************* extern global variables *******************/
extern struct platform_device * g_mbox_controller_platform_dev;
extern struct diag_info diagnose_info[SESSION_NUM];

/********************* local variables ***************************/
static DECLARE_RWSEM(list_lock);
static struct ipc_session *valid_session_list;
static uint64_t queue_buff[EACH_SESSION_RECV_MSG_FIFO_SIZE];
static atomic_t atomic_session_id = ATOMIC_INIT(0);
static struct ipc_session *session_map[SESSION_NUM];     // index of this array is session id
static uint64_t bitmap = 0; // session_map occupation status
static DEFINE_SPINLOCK(bitmap_lock);

int32_t request_a_session_id(void)    // bitmap algrithem
{
    //lock it
    int32_t ret;
    uint64_t flags;

    spin_lock_irqsave(&bitmap_lock, flags);
    ret = find_bit_zero(bitmap, 64, 0);
    if(ret>=0)
        bitmap |= (1<<ret);
    spin_unlock_irqrestore(&bitmap_lock, flags);
    return ret;
}

IPC_SESSION_STATUS get_session_status(int32_t session_id)
{
    if (session_id < 0 || session_id >= SESSION_NUM)
    {
        IPC_LOG_ERR("session_id is invalid! session_id = %d", session_id);
        return SESSION_STATE_NULL;
    }
    struct ipc_session* session = session_map[session_id];
    if (session == NULL)
    {
        IPC_LOG_WARNING("session invalid");
        return SESSION_STATE_NULL; 
    }

    return session->status;
}

int32_t set_session_status(int32_t session_id, IPC_SESSION_STATUS status)
{
    if (session_id < 0 || session_id >= SESSION_NUM)
    {
        IPC_LOG_ERR("session_id is invalid! session_id = %d", session_id);
        return -1;
    }
    struct ipc_session* session = session_map[session_id];
    if (session == NULL)
    {
        IPC_LOG_WARNING("session invalid");
        return -1; 
    }

    session->status = status;
    return 0;

}

int32_t ipc_register_session(enum ipc_core_e src, struct ipc_session ** pp_session, const struct ipc_mbox_client_info* cl_info)
{
    struct ipc_session *session;
    struct kfifo recv_msg_fifo;
    int32_t valid_session_id;

    IPC_LOG_DEBUG("enter!");
    valid_session_id = request_a_session_id();
    IPC_LOG_INFO("valid_session_id = %d", valid_session_id);
    if(valid_session_id<0 || valid_session_id>=SESSION_NUM)
    {
        IPC_LOG_ERR("All session ids are consumed, please close 1 firstly");
        return -1;
    }

    //alloc session mem
    session = devm_kzalloc(&g_mbox_controller_platform_dev->dev, sizeof(*session), GFP_KERNEL);
    if (!session) {
	    IPC_LOG_ERR("session = devm_kzalloc failed");
	    return -ENOMEM;
	}

   //alloc msg fifo mem
    int ret = kfifo_alloc(&recv_msg_fifo, sizeof(struct ipc_drv_msg) * EACH_SESSION_RECV_MSG_FIFO_SIZE, GFP_KERNEL);

    session->id = valid_session_id;
    session->status = SESSION_INIT;
    session->recv_msg_fifo = recv_msg_fifo;
    session->src = src;
    session->dest = cl_info->core_id;
    session->cl_info = cl_info;
    session->waiting_reply_msg_token = -1;
    IPC_LOG_INFO("current->pid = %d", current->pid);
    session->pid_num = current->pid;

    init_completion(&session->rx_reply_complete);
    init_completion(&session->rx_signal_complete);
    init_completion(&session->rx_method_complete);
    init_completion(&session->tx_complete);

    session_map[session->id] = session;
    *pp_session = session;

    IPC_LOG_DEBUG("exit!");
    return 0;
}

bool ipc_session_valid(int32_t id)
{
    IPC_LOG_INFO(" enter!, id = %x", id);
    if(id >= SESSION_NUM || id < 0)
    {
        return false;
    }
    return session_map[id] != NULL;
}

struct ipc_session *find_session_by_coreid(enum ipc_core_e dest)
{
    int32_t i;

    IPC_LOG_INFO("enter!, dest = %d", dest);
    for(i=0; i<SESSION_NUM; i++)
    {
        if (session_map[i] != NULL)
        {    
            if(session_map[i]->dest == dest)
            {
                return session_map[i];
            }
        }
    }

    IPC_LOG_DEBUG("exit");
    return NULL;
}


struct ipc_session * get_session_by_id(int32_t id)
{
    IPC_LOG_INFO(" enter!, id = %x", id);
    if (id >= SESSION_NUM || id < 0)
    {
        IPC_LOG_ERR("session_id %d is invalid", id);
        return NULL;
    }

    return session_map[id];
}

struct ipc_session * ipc_session_by_pid(pid_t pid)
{
    int32_t i;

    IPC_LOG_INFO("enter!, pid = %d", pid);
    for(i=0; i<SESSION_NUM; i++)
    {
        if (session_map[i] != NULL)
        {    
            if(session_map[i]->pid_num == pid)
            {
                return session_map[i];
            }
        }
    }

    IPC_LOG_DEBUG("exit");
    return NULL;
}

int32_t ipc_session_msg_in(int32_t session_id, struct ipc_drv_msg msg_addr)
{
    int32_t ret;
    IPC_LOG_INFO(" enter!, session_id = %d", session_id);
    if (session_id < 0 || session_id >= SESSION_NUM)
    {
        IPC_LOG_ERR("session_id is invalid! session_id = %d", session_id);
        return -1;
    }
    struct ipc_session* session = session_map[session_id];
    if (session == NULL)
    {
        IPC_LOG_WARNING("session invalid");
        return -1; 
    }

    if (session->status == SESSION_DESTROY)
    {
        IPC_LOG_WARNING("session destroy");
        return -1; 
    }

    if (!kfifo_is_full(&session->recv_msg_fifo))
    {
        // IPC_LOG_INFO("kfifo in %d", kfifo_avail(&session->recv_msg_fifo));
        ret = kfifo_in(&session->recv_msg_fifo, &msg_addr, sizeof(struct ipc_drv_msg));
    }
    else
    {
        IPC_LOG_WARNING("fifo is overrun");    // TODO  overrun should block
        return -1;
    }
    return 0;
}

struct ipc_drv_msg ipc_session_msg_out(struct ipc_session *session)
{
    uint64_t msg_id;
    struct ipc_drv_msg msg;
    if (session->status == SESSION_DESTROY)
    {
        IPC_LOG_INFO("session destroy");
        // return NULL;
    }

    kfifo_out(&session->recv_msg_fifo, &msg, sizeof(struct ipc_drv_msg));
    if (sizeof(msg) == 0) {
        diagnose_info[session->id].recv_err.queue_empty += 1;
    }
    //IPC_LOG_INFO("msg_id = %px", msg_id);
    //msg = (struct ipc_drv_msg*)msg_id;
    IPC_LOG_INFO("session_id = %px, msg_addr = %px", session, &msg);

    return msg;
}

struct ipc_drv_msg * read_msg_by_pid(pid_t pid)
{
    return NULL;
}

int32_t ipc_session_destroy(pid_t pid)
{
    return 0;
}

int32_t ipc_session_destroy_by_id(int32_t session_id)
{
    IPC_LOG_DEBUG(" enter!");
    if (session_id < 0 || session_id >= SESSION_NUM)
    {
        IPC_LOG_ERR("session_id is invalid! session_id = %d", session_id);
        return -1;
    }
    struct ipc_session *p_sess;
    uint64_t flags;

    p_sess = session_map[session_id];
    if (p_sess == NULL)
    {
        IPC_LOG_INFO(" session is null!");
        return -1;
    }

    IPC_LOG_INFO(" session = 0x%px", p_sess);
    //clear bitmap
    spin_lock_irqsave(&bitmap_lock, flags);
    bitmap &= ~(1ull<<session_id);
    spin_unlock_irqrestore(&bitmap_lock, flags);

    //clear completion to release wait
    complete(&p_sess->tx_complete);
    complete(&p_sess->rx_signal_complete);
    complete(&p_sess->rx_method_complete);

    //session status update
    p_sess->status = SESSION_DESTROY;

    if (&p_sess->recv_msg_fifo)
    {
        //devm_kfree(&g_mbox_controller_platform_dev->dev ,p_sess->recv_msg_fifo);
        kfifo_free(&p_sess->recv_msg_fifo);
    }

    if (p_sess)
    {
        devm_kfree(&g_mbox_controller_platform_dev->dev,p_sess);
        IPC_LOG_INFO(" free session!");
    }

    session_map[session_id] = NULL;

    return 0;
}
