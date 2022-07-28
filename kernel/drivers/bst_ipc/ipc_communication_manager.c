#include "ipc_communication_manager.h"
#include <linux/module.h>
#include <linux/init.h>

#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/timekeeping.h>

#include "ipc_mempool.h"
#include "ipc_host_server.h"
#include "ipc_nodemanager.h"
#include "ipc_session.h"
#include "ipc_mailbox_client.h"
#include "ipc_mailbox_controller.h"
#include "ipc_msg_manager.h"
#include "user_head.h"
#include "ipc.h"
#include "ipc_common.h"

// macro
#define IPC_DRIVER_NAME "ipc_communication_mgr"
#define IPC_CMD_MAX
#define IPC_CHANNEL_MAX 8

struct work_data {
    struct ipc_fill_register_msg data;  
    int32_t data_len;
    enum ipc_core_e src;
    enum ipc_core_e dest;
    ktime_t timestamp;
};

struct send_work_data {
    struct work_struct work;
    struct ipc_fill_register_msg data;
    int32_t session_id;
    int32_t data_len;
    enum ipc_core_e src;
    enum ipc_core_e dest;
};

/********************* extern global variables *******************/
extern struct platform_device *g_ipc_main_pdev;
extern int32_t register_a55_all_cores(void);
extern struct ipc_mbox_client_info * client_map[IPC_CORE_MAX];
extern int32_t recv_msg_callback(enum ipc_core_e src, enum ipc_core_e dest, void* buf, u32 len, ktime_t timestamp);
extern enum ipc_core_e ipc_channel[IPC_CORE_MAX];
extern struct diag_info diagnose_info[SESSION_NUM];

/********************* function declaration ***************************/
int32_t core_addr_init(void);
int32_t core_addr_exit(void);

/********************* local variables ***************************/
static struct workqueue_struct *ipc_send_wq[IPC_CHANNEL_MAX]; //1.定义工作队列

static struct workqueue_struct *ipc_recv_wq; //1.定义工作队列
static struct work_struct ipc_recv_work;    //2定义work结构体

//work queue msg fifo
spinlock_t in_lock;
spinlock_t out_lock;

static struct kfifo pfifo_out;

static struct ipc_session* subscription_map[IPC_CORE_MAX][MAX_CMD_ID] = {{NULL}};
static DECLARE_RWSEM(list_lock);

static uint64_t g_ipc_all_cores_register_addr_phy = 0;

/********************* global variables ***************************/
struct ipc_all_cores_register_addr* g_ipc_all_cores_register_addr = NULL;
uint64_t g_ipc_all_cores_register_addr_uaddr = 0;

static void send_func(struct work_struct *work) //实现处理函数
{
    IPC_LOG_DEBUG("enter!");
    struct send_work_data *send_msg = (struct send_work_data *)work;
    struct ipc_mbox_client_info *cl = NULL;
    int ret = 0;

    IPC_LOG_DEBUG("work_data.src = %d, dst = %d", send_msg->src, send_msg->dest);
    ret = ipc_node_msgparse(send_msg->dest, &cl);
    if (ret<0)
    {
        IPC_LOG_WARNING("send failed, Can not find core %d client!", send_msg->dest);
        kfree(send_msg);
        return;
    }
    else
    {
        IPC_LOG_DEBUG("send to client 0x%px!", cl);
        ret =  ipc_send_data(cl, send_msg->src, &send_msg->data);
        if (!ret)
        {
            cl->channel_status = CHANNEL_READY;
            //get session and complete
            struct ipc_session *session = get_session_by_id(send_msg->session_id);
            if (session == NULL)
            {
                IPC_LOG_WARNING("session is NULL");
                kfree(send_msg);
                return;
            }
            set_session_status(send_msg->session_id, SESSION_WAIT_SEND);
            IPC_LOG_DEBUG("send complete to session");
            complete(&session->tx_complete);

        }
        else
        {
            IPC_LOG_DEBUG("msg send error");
        }
    }

    kfree(send_msg);
    IPC_LOG_DEBUG("exit!");
}

void bst_print(const char* fmt, ...)
{
#ifdef IPC_HOST_SERVER_PRINT
    va_list ap;
    va_start(ap, fmt);
	(void)vprintk(fmt, ap);
	va_end(ap);
#endif
}

static void msg_dispatch(enum ipc_core_e core, enum ipc_core_e dest, struct ipc_fill_register_msg *fill_msg, int32_t len, ktime_t timestamp)
{
    IPC_LOG_DEBUG("core = %d, dest = %d", core, dest);
    IPC_LOG_DEBUG("fill_msg->type = %d", fill_msg->type);
    IPC_LOG_DEBUG("fill_msg->long_param = %d", fill_msg->long_param);
    IPC_LOG_DEBUG("fill_msg->short_param = %d", fill_msg->short_param);

    if(recv_msg_callback)
    {
        recv_msg_callback(core, dest, (void*)fill_msg, len, timestamp);
    }
}

 static void recv_func(struct work_struct *work)
 {
    IPC_LOG_DEBUG("enter");

    struct work_data work_data;
    while (!kfifo_is_empty(&pfifo_out))
    {
        kfifo_out(&pfifo_out, &work_data, sizeof(work_data));
        msg_dispatch(work_data.src, work_data.dest, &work_data.data, work_data.data_len, work_data.timestamp);
    }

    IPC_LOG_DEBUG("exit");
 }

 int32_t init_all_core(bool user)
 {

     static atomic_t init_done = ATOMIC_INIT(0);
     if (1 == atomic_cmpxchg(&init_done, 0, 1))
     {
         return 0;
     }

     struct ipc_buffer ipc_buffer = {
         .size = 4096,
         .align = 64,
     };

     struct ipc_memblock *memblock = NULL;
     //todo: need add bst_free
     if (alloc_mem(&ipc_buffer) < 0)
     {
         return -1;
     }

     memblock = (struct ipc_memblock *)ipc_buffer.handle;
     g_ipc_all_cores_register_addr = memblock->k_addr;
     g_ipc_all_cores_register_addr_phy = ipc_buffer.phy_addr.low;
     g_ipc_all_cores_register_addr_uaddr = ipc_buffer.uaddr;

     IPC_LOG_INFO("g_ipc_all_cores_register_addr = 0x%px", g_ipc_all_cores_register_addr);

     return 0;
 }

 int32_t ipc_communication_create()
 {
     int32_t ret = 0;
     int32_t i = 0;

     IPC_LOG_DEBUG("enter");

     for(i = 0; i < IPC_CHANNEL_MAX; i++)
     { 
        ipc_send_wq[i] = create_singlethread_workqueue("ipc_send_wq"); //1初始化工作对列
        if (!ipc_send_wq[i])
        {
            IPC_LOG_ERR("ERROR: ipc_send_wq %d ==NULL", i);
            return -1;
        }
     }

     ipc_recv_wq = create_singlethread_workqueue("ipc_recv_wq"); //1初始化工作对列

     if (!ipc_recv_wq)
     {
         IPC_LOG_ERR("ERROR: ipc_recv_wq ==NULL");
         return -1;
     }

    //init thread one time and use shared fifo to manage msg in and out
    spin_lock_init(&in_lock);
    spin_lock_init(&out_lock);

    ret = kfifo_alloc(&pfifo_out, sizeof(struct work_data) * Rx_Tx_WORK_QUEUE_FIFO_SIZE, GFP_KERNEL);

    //init workqueue
    // INIT_WORK(&ipc_send_work, send_func);
    INIT_WORK(&ipc_recv_work, recv_func);
    init_all_core(false);
    core_addr_init();
    return ret;
 }

 int32_t ipc_communication_close()
 {
     int i;
     for (i = 0; i < IPC_CHANNEL_MAX; i++)
     {
        flush_workqueue(ipc_send_wq[i]);   //刷新工作队列
        destroy_workqueue(ipc_send_wq[i]); //注销工作队列
     }

     flush_workqueue(ipc_recv_wq);   //刷新工作队列
     destroy_workqueue(ipc_recv_wq); //注销工作队列

     //release fifo
    if (&pfifo_out) {
        kfifo_free(&pfifo_out);
        }
    core_addr_exit();
    return 0;
 }

 int32_t ipc_drv_send(enum ipc_core_e src, enum ipc_core_e dest, int32_t session_id, void *buf, uint32_t len)
 {
    bool queue_status;
    struct send_work_data *send_msg;
    if (dest > IPC_CORE_MAX)
    {
        diagnose_info[session_id].send_err.session_invalid += 1;
        IPC_LOG_WARNING("dest is invalid");
        return IPC_SEND_ERR_INVALID_PARAM;
    }

    int32_t channel_id = ipc_channel[dest];
    IPC_LOG_INFO("channel id = %d", channel_id);
    if (channel_id > IPC_CHANNEL_MAX)
    {
        diagnose_info[session_id].send_err.session_invalid += 1;
        IPC_LOG_WARNING("channel id is invalid");
        return IPC_SEND_ERR_INVALID_PARAM;
    }

    send_msg = kmalloc(sizeof(struct send_work_data), GFP_KERNEL);
    send_msg->data = *(struct ipc_fill_register_msg*)buf;
    send_msg->data_len = len;
    send_msg->src = src;
    send_msg->dest = dest;
    send_msg->session_id = session_id;

    INIT_WORK(&send_msg->work, send_func);
    queue_status = queue_work(ipc_send_wq[channel_id], &send_msg->work);
    if (!queue_status) {
        diagnose_info[session_id].send_err.queue_full += 1;
    }
    
    return queue_status ? 0 : -1;
}

int32_t ipc_drv_recv(enum ipc_core_e src, enum ipc_core_e dest, void *buf, uint32_t len)
{
    struct work_data work_data;
    int32_t ret;
    uint64_t flag;

     if (ipc_recv_wq == NULL)
         return 0;

    work_data.data = *(struct ipc_fill_register_msg*)buf;
    work_data.data_len = len;
    work_data.src = src;
    work_data.dest = dest;
    work_data.timestamp = ktime_get();

    //fifo in work_data
    spin_lock_irqsave(&out_lock, flag);
    if (kfifo_avail(&pfifo_out))
    {
        IPC_TRACE_PRINTK("kfifo in %d", kfifo_avail(&pfifo_out));
        ret = kfifo_in(&pfifo_out, &work_data, sizeof(work_data));   
        ret = queue_work(ipc_recv_wq, &ipc_recv_work);  
    }
    else
    {
        IPC_TRACE_PRINTK("fifo in is overrun");
        spin_unlock_irqrestore(&out_lock, flag);
        return -1;
    }

    spin_unlock_irqrestore(&out_lock, flag);
    return 0;
}

bool is_cmd_valid(enum ipc_core_e core, uint32_t cmd)
{
	// if(subscription_map[core][cmd] == NULL)
	// {
	// 	IPC_LOG_ERR("cmd : %d of dst %d is not supported", cmd, core);
	// 	return false;
	// }

    return true;
}

int32_t subscribe_cmd_of_core(enum ipc_core_e core, uint32_t cmd)
{
    /*
    struct ipc_session*  session;
    if(!is_cmd_valid(core, cmd))
    {
        return -ENOMSG;
    }

    session = devm_kzalloc(&g_ipc_main_pdev->dev, sizeof(*session), GFP_KERNEL);
    if (!session)
    {
        IPC_TRACE_PRINTK("devm_kzalloc session list fail");
        return -ENOMEM;
    }

    list_add_tail(&session->list, &subscription_map[core][cmd]->list);
    */
    return 0;
}
