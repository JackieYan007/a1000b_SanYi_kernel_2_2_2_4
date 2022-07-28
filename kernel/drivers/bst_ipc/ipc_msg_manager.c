#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/hashtable.h>

#include "ipc_common.h"
#include "ipc_msg_manager.h"

#define IPC_DRIVER_NAME "ipc_msg_manager"

/********************* extern global variables *******************/
extern struct platform_device * g_mbox_controller_platform_dev;

/********************* local variables ***************************/
static DECLARE_RWSEM(sent_msg_hash_rwsem);
static atomic_t token_count = ATOMIC_INIT(1);
static DECLARE_HASHTABLE(msg_hash, MSG_BIT_NUM);
static DEFINE_SPINLOCK(token_lock);

int32_t send_msg_init(void)
{
    IPC_LOG_DEBUG("enter");
    hash_init(msg_hash);
    IPC_LOG_DEBUG("exit");
    return 0;
}

int32_t send_msg_destroy(void)
{
    IPC_LOG_DEBUG("enter");
    // struct msg_node *p, *q;
    // if(!sent_msg_list)
    // {
    //     return -1;
    // }
    // down_write(&sent_msg_hash_rwsem);
    // list_for_each_entry_safe(p, q, &(sent_msg_list->list), list);
    // {
    //     list_del(&(p->list));
    //     devm_kfree(&g_mbox_controller_platform_dev->dev, p);
    // }

    // list_del(&(sent_msg_list->list));
    // devm_kfree(&g_mbox_controller_platform_dev->dev, sent_msg_list);
    // up_write(&sent_msg_hash_rwsem);
    return 0;
}

int32_t send_msg_in(struct ipc_drv_msg* msg)
{
    if (!msg)
        return -1;

    IPC_LOG_INFO("enter!, msg token = %d", msg->token);
    down_write(&sent_msg_hash_rwsem);
    hash_add(msg_hash, &msg->node, msg->token);
    up_write(&sent_msg_hash_rwsem);
#ifdef DUMP_MSG_HASH_TABLE
    show_all_msgs();
#endif // DUMP_MSG_HASH_TABLE
    IPC_LOG_DEBUG("exit");
    return 0;
}

int32_t send_msg_out(uint32_t token, struct ipc_drv_msg** msg)
{
    IPC_TRACE_PRINTK("enter!, token = %d", token);
#ifdef DUMP_MSG_HASH_TABLE
    show_all_msgs();
#endif // DUMP_MSG_HASH_TABLE
    struct ipc_drv_msg* obj = NULL;

    if (msg && token > 0)
    {
        down_write(&sent_msg_hash_rwsem);
        // TODO hash_for_each_possible_safe
        hash_for_each_possible(msg_hash, obj, node, token) {
            if(obj->token == token) {
                *msg = obj;
                hash_del(&obj->node);
                up_write(&sent_msg_hash_rwsem);
                IPC_LOG_INFO("exit 1");
                return 0;
            }
        }
        up_write(&sent_msg_hash_rwsem);
    }
    if (msg)
    {
        *msg = NULL;
    }
    IPC_LOG_INFO("exit 2");
    return -1;
}

int32_t show_all_msgs(void)
{
    int32_t i=0;
    down_read(&sent_msg_hash_rwsem);
    for (i = 0; i < HASH_SIZE(msg_hash); ++i)
    {
        if (!hlist_empty(&msg_hash[i]))
        {
            IPC_LOG_INFO("bucket[%d]=> ", i);

            struct ipc_drv_msg* obj = NULL;
            hlist_for_each_entry(obj, &msg_hash[i], node) {
                IPC_LOG_INFO("token : %d, ", obj->token);
            }
        }
        IPC_LOG_INFO("-----------------bucket %d end--------------------", i);
    }
    up_read(&sent_msg_hash_rwsem);
    return 0;
}

struct ipc_drv_msg * find_ipcmsg_by_token(uint32_t token)
{
    struct ipc_drv_msg* obj = NULL;
#ifdef DUMP_MSG_HASH_TABLE
    show_all_msgs();
#endif // DUMP_MSG_HASH_TABLE
    if (token == 0)
        return NULL;
    
    down_read(&sent_msg_hash_rwsem);
    hash_for_each_possible(msg_hash, obj, node, token) {
        if(obj->token == token) {
            up_read(&sent_msg_hash_rwsem);
            return obj;
        }
    }
    up_read(&sent_msg_hash_rwsem);
    return NULL;
}

uint32_t ipc_msg_get_an_available_token(void)
{
    uint64_t flags;
    int32_t token;
    spin_lock_irqsave(&token_lock, flags);
    token = atomic_read(&token_count);
	atomic_inc(&token_count);
	if (atomic_read(&token_count) > MSG_TOKEN_MAX) {
		atomic_set(&token_count, 1);
	}
    spin_unlock_irqrestore(&token_lock, flags);
    return token;
}