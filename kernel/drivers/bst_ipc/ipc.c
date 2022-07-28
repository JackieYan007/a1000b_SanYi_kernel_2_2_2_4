#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/of_reserved_mem.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <asm/mman.h>
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <asm/memory.h>

#include "ipc.h"
#include "ipc_mempool.h"
#include "ipc_communication_manager.h"
#include "ipc_host_server.h"
#include "ipc_session.h"
#include "ipc_nodemanager.h"
#include "ipc_msg_manager.h"
#include <linux/ipc_interface.h>
#include "user_head.h"
#include "ipc_top_interface.h"
// #include "ipc_interface.h"

/********************* macros *******************/
#define IPC_INDEX_SIZE 50
#define MAX_DEVICE_ID 255
#define IPC_DRIVER_NAME "ipc"

/********************* local variables ***************************/
static uint32_t used_max_device_id = 0;
static uint32_t assigned_mem_size = 0;

/********************* global variables ***************************/
struct platform_device *g_ipc_main_pdev;

/********************* function declaration ***************************/
int32_t init_all_core(bool user);

static int32_t ipc_mempool_init(struct bstipc *bstipc)
{
	int32_t ret = 0;
	ret = of_reserved_mem_device_init(bstipc->dev);
	if (ret < 0) {
		IPC_DEV_ERR(bstipc->dev, "of_reserved_mem_device_init fail, ret: %d\n", ret);
		return -ENODEV;
	}
	return ipc_init_cma_mempool(&bstipc->pool, bstipc->dev);
}

static int32_t ipc_ioctl_alloc(struct file *filp, struct ipc_buffer __user *p)
{
	struct bstipc* bstipc = NULL;
	struct ipc_buffer ipc_buffer;
	int32_t ret = 0;
    struct ipc_memblock * memblock = NULL;

	IPC_LOG_INFO("%s, user ptr: %px", "enter", p);

	bstipc = container_of(filp->private_data, struct bstipc, miscdev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!\n");
		return -EFAULT;
	}

	if (copy_from_user(&ipc_buffer, p, sizeof(*p))) {
		IPC_DEV_ERR(bstipc->dev, "copy_from_user fail!\n");
		return -EFAULT;
	}
	IPC_LOG_INFO("ipc_buffer: size = 0x%x, align = 0x%x",
		 ipc_buffer.size, ipc_buffer.align);

	ret = bst_alloc(&ipc_buffer, true);
	if (ret) {
		IPC_DEV_ERR(bstipc->dev, "bst_alloc fail!\n");
		return ret;
	}

	if (copy_to_user(p, &ipc_buffer, sizeof(*p))) {
		bst_free(ipc_buffer.phy_addr.ptr_64, true);
		IPC_DEV_ERR(bstipc->dev, "copy_to_user fail!\n");
		return -EFAULT;
	}

	IPC_LOG_DEBUG("%s", "exit");
	return 0;
}

static int32_t ipc_ioctl_free(struct file *filp, struct ipc_buffer __user *p)
{
	struct ipc_buffer ipc_buffer;
	struct ipc_memblock *memblock = NULL;
	struct bstipc* bstipc = NULL;
	int32_t ret;

	IPC_LOG_INFO("%s, user ptr: %px", "enter", p);

	if (copy_from_user(&ipc_buffer, p, sizeof(*p)))
		return -EFAULT;

	ret = bst_free(ipc_buffer.phy_addr.ptr_64, true);
	if(ret<0)
	{
		return ret;
	}

	IPC_LOG_DEBUG("%s", "exit");
	return ret;
}

static int32_t ipc_ioctl_bind_cpu(struct file *filp, uint32_t __user *p)
{
	IPC_LOG_DEBUG("enter");

	IPC_LOG_DEBUG("exit");
	return 0;
}

//sync call function
static int32_t ipc_ioctl_send_msg(struct file *filp, struct _usr_sync_msg __user *p)
{
	IPC_LOG_DEBUG("enter");
	struct bstipc* bstipc = NULL;
	int32_t ret;
	struct ipc_drv_msg* ipc_drv_msg = NULL;
	struct _usr_sync_msg sync_msg;

	bstipc = container_of(filp->private_data, struct bstipc, miscdev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!\n");
		return -EFAULT;
	}

	//copy msg
	if (copy_from_user(&sync_msg, p, sizeof(*p)))
	{
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}
	IPC_LOG_INFO("session_id = %d, timeout = %d",sync_msg.session_id, sync_msg.timeout);
    IPC_LOG_INFO("send data= %d", sync_msg.send_msg.data);
    IPC_LOG_INFO("cmd= %d", sync_msg.send_msg.cmd);
    IPC_LOG_INFO("type= %d", sync_msg.send_msg.type);

	// ipc_send(sync_msg.session_id, &sync_msg.send_msg, 200);
	// sync_msg.recv_msg.token = sync_msg.send_msg.token;
	// sync_msg.recv_msg.type = IPC_MSG_TYPE_SIGNAL;
	// ret = ipc_recv_reply(sync_msg.session_id, &sync_msg.recv_msg, sync_msg.timeout);

	ret = ipc_sync_send_msg(&sync_msg);

	IPC_LOG_INFO("ret= %d", ret);
	IPC_LOG_INFO("recv data= %d", sync_msg.recv_msg.data);
    IPC_LOG_INFO("cmd= %d", sync_msg.recv_msg.cmd);
    IPC_LOG_INFO("type= %d", sync_msg.recv_msg.type);

	if (ret == 0)
	{
		IPC_LOG_INFO("return to user");
		if(copy_to_user(p,&sync_msg,sizeof(*p)) == 0)
		{
			return 0;
		}
	}
	IPC_LOG_DEBUG("%s", "exit");
	return -1;
}

//async call function
static int32_t ipc_ioctl_send_async_msg(struct file *filp, struct _usr_async_msg __user *p)
{
	IPC_LOG_DEBUG("enter");
	struct bstipc* bstipc = NULL;
	int32_t ret;
	struct ipc_drv_msg* ipc_drv_msg = NULL;
	struct _usr_async_msg async_msg;
	struct ipc_session *session = NULL;

	bstipc = container_of(filp->private_data, struct bstipc, miscdev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!\n");
		return -EFAULT;
	}

	//copy msg
	if (copy_from_user(&async_msg, p, sizeof(*p)))
	{
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}
	IPC_LOG_INFO("session_id = %d", async_msg.session_id);
	IPC_LOG_INFO("pid = %d", async_msg.pid);
	IPC_LOG_INFO("send data= %d", async_msg.send_msg.data);
    IPC_LOG_INFO("cmd= %d", async_msg.send_msg.cmd);
    IPC_LOG_INFO("type= %d", async_msg.send_msg.type);
	// IPC_LOG_INFO("virt userdata = 0x%x", async_msg.send_msg.userdata);
	// async_msg.send_msg.userdata = __virt_to_phys(async_msg.send_msg.userdata);
	// async_msg.send_msg.userdata = __pa(async_msg.send_msg.userdata);
	// IPC_LOG_INFO("phys userdata =0x%x", async_msg.send_msg.userdata);

	session = get_session_by_id(async_msg.session_id);
	if (session == NULL)
	{
		IPC_LOG_ERR("session is null!");
		return -1;
	}
	// session->pid_num = async_msg.pid;

	ret = ipc_async_send_msg(&async_msg);
	if (ret != 0)
	{
		IPC_LOG_ERR("send msg failed!");
		return -1;
	}

	IPC_LOG_DEBUG("%s", "exit");
	return 0;
}


static int32_t ipc_ioctl_session_create(struct file *filp, struct _usr_init* init_msg)
{
	uint64_t id;
	struct _usr_init init;
	int32_t session_id = -1;
	IPC_LOG_DEBUG("enter");

	struct bstipc* bstipc = container_of(filp->private_data, struct bstipc, miscdev);

	if (copy_from_user(&init, init_msg, sizeof(init_msg)))
	{
		IPC_DEV_ERR(bstipc->dev, "copy_to_user fail!\n");
		return -EFAULT;
	}
	IPC_LOG_INFO("init src = %d, dst = %d", init.src, init.dst);
	struct device *dev = ipc_get_dev_by_coreid(init.dst);
	//dev of dst
	session_id = ipc_init(init.dst, init.src, &g_ipc_main_pdev->dev);
	IPC_LOG_INFO("session id %d", session_id);

	if (session_id < 0)
	{
		IPC_DEV_ERR(bstipc->dev,"ipc_init(IPC_CORE_DSP) failed, ret = %d",session_id);
	}

	IPC_LOG_DEBUG("%s", "exit");
	return session_id;
}

static int32_t ipc_ioctl_session_destroy(struct file *filp, uint64_t __user *session_id)
{
	int32_t ret = 0;
	uint64_t sess_id;

	IPC_LOG_DEBUG("%s", "enter");
	struct bstipc* bstipc = container_of(filp->private_data, struct bstipc, miscdev);

	if (copy_from_user(&sess_id, session_id, sizeof(*session_id)))
	{
		IPC_DEV_ERR(bstipc->dev, "copy_to_user fail!\n");
		return -EFAULT;
	}
	//todo::delete msg in msg_in list
	if (!ipc_session_valid(sess_id))
	{
		IPC_LOG_ERR("session is invalid!");
		return -1;
	}

	// ret = ipc_session_destroy(current->pid);
	// if (ret){
	// 	IPC_LOG_INFO("destroy session success!");
	// }
	ret = ipc_close(sess_id);
	if (ret == 0)
	{
		IPC_LOG_INFO("close session success!");
	}

	IPC_LOG_DEBUG("%s", "exit");
	return ret;
}

static int32_t ipc_ioctl_send_msg_func(struct file *filp, struct _usr_send_or_recv_msg __user *p)
{
	IPC_LOG_DEBUG("enter");
	struct bstipc* bstipc = NULL;
	int32_t ret;
	struct _usr_send_or_recv_msg _msg;

	bstipc = container_of(filp->private_data, struct bstipc, miscdev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!\n");
		return -EFAULT;
	}

	//copy msg
	if (copy_from_user(&_msg, p, sizeof(*p)))
	{
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}
	IPC_LOG_INFO("session_id = %d, timeout = %d",
					_msg.session_id, _msg.timeout);
	IPC_LOG_INFO("send data= %d", _msg.t_msg.data);
    IPC_LOG_INFO("cmd= %d", _msg.t_msg.cmd);
    IPC_LOG_INFO("type= %d", _msg.t_msg.type);

	if (!ipc_session_valid(_msg.session_id))
	{
		IPC_LOG_ERR("session is invalid!");
		return -1;
	}
	ipc_msg send_msg;
	send_msg.type = _msg.t_msg.type;
	send_msg.cmd = _msg.t_msg.cmd;
	send_msg.data = _msg.t_msg.data;
	ret = ipc_send(_msg.session_id, &send_msg, _msg.timeout);
	if (ret != 0)
	{
		IPC_LOG_ERR("ipc_send(IPC_CORE_DSP_0) failed, ret = %d", ret);
		return ret;
	}
	IPC_LOG_DEBUG("%s", "exit");
	return 0;
}

static int32_t ipc_ioctl_recv_msg_func(struct file *filp, struct _usr_send_or_recv_msg __user *p)
{
	IPC_LOG_DEBUG("enter");
	struct bstipc* bstipc = NULL;
	int32_t ret;
	struct _usr_send_or_recv_msg _msg;
	// ipc_msg re_msg;

	bstipc = container_of(filp->private_data, struct bstipc, miscdev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!\n");
		return -EFAULT;
	}

	//copy msg
	if (copy_from_user(&_msg, p, sizeof(*p)))
	{
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}
	IPC_LOG_INFO("session_id = %d, timeout = %d",
					_msg.session_id, _msg.timeout);

	if (!ipc_session_valid(_msg.session_id))
	{
		IPC_LOG_ERR("session is invalid!");
		return -1;
	}
	ipc_msg recv_msg;
	recv_msg.type = _msg.t_msg.type;
	ret = ipc_recv(_msg.session_id, &recv_msg, _msg.timeout);
	_msg.t_msg.type = recv_msg.type;
	_msg.t_msg.cmd = recv_msg.cmd;
	_msg.t_msg.data = recv_msg.data;
	
	IPC_LOG_INFO("ret= %d", ret);
	IPC_LOG_INFO("recv data= %d", _msg.t_msg.data);
    IPC_LOG_INFO("cmd= %d", _msg.t_msg.cmd);
    IPC_LOG_INFO("type= %d", _msg.t_msg.type);

	// _msg.t_msg.type = re_msg.type;
	// _msg.t_msg.cmd = re_msg.cmd;
	// _msg.t_msg.token = re_msg.token;
	// _msg.t_msg.data = re_msg.data;

	if (ret == 0)
	{
		IPC_LOG_INFO("return to user");
		if(copy_to_user(p, &_msg, sizeof(*p)) == 0)
		{
			return 0;
		}
	}
	IPC_LOG_DEBUG("%s", "exit");
	return -1;
}


static int32_t ipc_ioctl_driver_ready(struct file *filp, uint32_t __user *driver_id)
{
	int32_t ret = 0;
	uint32_t id;

	IPC_LOG_DEBUG("enter");
	struct bstipc* bstipc = container_of(filp->private_data, struct bstipc, miscdev);

	if (copy_from_user(&id, driver_id, sizeof(*driver_id)))
	{
		IPC_DEV_ERR(bstipc->dev, "copy_to_user fail!\n");
		return -EFAULT;
	}
	//-1: id invalid; -2: driver not ready; 0: driver ready
	ret = ipc_node_valid(id);

	return ret;
}

static long ipc_ioctl(struct file *filp, uint32_t cmd, unsigned long arg)
{
	int32_t ret = 0;

	IPC_LOG_INFO("%s, cmd: 0x%x", "enter", cmd);
	switch(cmd){
		case IPC_IO_MEM_ALLOC:
			ret = ipc_ioctl_alloc(filp, (struct ipc_buffer __user *)arg);
			break;
		case IPC_IO_MEM_FREE:
			ret = ipc_ioctl_free(filp, (struct ipc_buffer __user *)arg);
			break;
		case IPC_IO_SES_CREATE:
			ret = ipc_ioctl_session_create(filp, (struct _usr_init *)arg);
			break;
		case IPC_IO_SES_DESTROY:
			ret = ipc_ioctl_session_destroy(filp, (uint64_t __user *)arg);
			break;
		case IPC_IO_MSG_SEND_SYNC:
			ret = ipc_ioctl_send_msg(filp, (struct _usr_sync_msg *)arg);
			break;
		case IPC_IO_MSG_SEDN:
			ret = ipc_ioctl_send_msg_func(filp, (struct _usr_send_or_recv_msg *)arg);
			break;
		case IPC_IO_MSG_RECV:
			ret = ipc_ioctl_recv_msg_func(filp, (struct _usr_send_or_recv_msg *)arg);
			break;
		case IPC_IO_MSG_SEND_ASYNC:
			ret = ipc_ioctl_send_async_msg(filp, (struct _usr_async_msg __user *)arg);
			break;
		// case IPC_IO_BIND_CPU:
		// 	ret = ipc_ioctl_bind_cpu(filp, (uint32_t __user *)arg);
		// 	break;
		default:
			ret = -EINVAL;
			break;
	}

	IPC_LOG_INFO("%s, ret: %d", "exit", ret);
	return ret;
}


static int32_t ipc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int32_t ret = 0;

	IPC_LOG_INFO("%s, vm_start: 0x%lx, vm_end: 0x%lx, vm_pgoff: 0x%lx", "enter",
		vma->vm_start, vma->vm_end, vma->vm_pgoff);
	ret = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			      vma->vm_end - vma->vm_start,
			      pgprot_writecombine(vma->vm_page_prot)); //pgprot_writecombine():MT_NORMAL_NC,no_cache

	IPC_LOG_INFO("%s, ret: %d", "exit", ret);
	return ret;
}

static int32_t ipc_open(struct inode *inode, struct file *filp)
{
	int32_t ret = 0;
	IPC_LOG_DEBUG("%s", "enter");

	struct bstipc* bstipc = platform_get_drvdata(g_ipc_main_pdev);
	bstipc->private_data = filp;

    ret = init_all_core(true);
    if(ret < 0)
    {
        IPC_LOG_ERR("ERROR: init_all_core fail, ret = %d", ret);
        return ret;
    }
	IPC_LOG_DEBUG("%s", "exit");
	return 0;
}

static int32_t ipc_drv_close(struct inode *inode, struct file *filp)
{
	IPC_LOG_DEBUG("%s", "enter");
	int32_t ret = -1;
	//todo: destroy msg buffer
	ret = ipc_session_destroy(current->pid);

	return ret;
}

static int32_t ipc_fasync (int32_t fd, struct file *file, int32_t on)
{
	//create session for each register service and return session id
	IPC_LOG_DEBUG("enter!");
	struct ipc_session *session;
	IPC_LOG_INFO("pid = %d", current->pid);
	session = ipc_session_by_pid(current->pid);
	if (!session)
	{
		// IPC_LOG_ERR("session invalid");
		return -EIO;
	}
    fasync_helper(fd, file, on, &(session->ipc_fasync));
	return 0;
}

//ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
static ssize_t ipc_read(struct file *filp,  char __user *buf, size_t count, loff_t * ppos)
{
	IPC_LOG_DEBUG("enter!");
	struct ipc_drv_msg *pmsg;
	pmsg = read_msg_by_pid(current->pid);
	IPC_LOG_INFO("ipc_read token %d", pmsg->token);

	if (pmsg)
	{
		if(copy_to_user(buf,pmsg,sizeof(*pmsg)))
		{
			devm_kfree(&g_ipc_main_pdev->dev, pmsg);
			pmsg = NULL;
			return 0;
		}
	}

  	return -EIO;
}

static const struct file_operations ipc_fops = {
	.owner  = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = ipc_ioctl,
	.read = ipc_read,
	.mmap = ipc_mmap,
	.open = ipc_open,
	.release = ipc_drv_close,
	.fasync= ipc_fasync,

};


static const struct miscdevice ipc_misc_base = {
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &ipc_fops,
};


static int32_t ipc_probe(struct platform_device *pdev)
{
	int32_t ret = 0;
	uint32_t id = 0;
	char dev_name[sizeof(IPC_DRIVER_NAME) + IPC_INDEX_SIZE];
	struct bstipc *bstipc = NULL;
	int32_t ioresource_mem_idx = 0;

	IPC_LOG_DEBUG("%s", "enter");
	g_ipc_main_pdev = pdev;

	memset(dev_name, 0, sizeof(IPC_DRIVER_NAME) + IPC_INDEX_SIZE);

	//create bstipc device
	bstipc = devm_kzalloc(&pdev->dev, sizeof(struct bstipc), GFP_KERNEL);
	if (!bstipc) {
		IPC_DEV_ERR(&pdev->dev, "no enough memory!\n");
		return -ENOMEM;
	}

	//init bstipc device
	bstipc->dev = &pdev->dev;
	bstipc->status = IPC_INIT;
	platform_set_drvdata(pdev, bstipc);

	//init bstn share mempool
	ret = dma_set_coherent_mask(bstipc->dev, DMA_BIT_MASK(36));
	if (ret < 0) {
		IPC_DEV_ERR(&pdev->dev, "dma_set_coherent_mask fail, ret %d\n", ret);
	}
	ret = ipc_mempool_init(bstipc);
	IPC_LOG_INFO("ipc mempool init result: %d\n", ret);
	if (ret < 0) {
		IPC_DEV_ERR(&pdev->dev, "ipc_mempool_init fail, ret %d\n", ret);
		return ret;
	}

	//init ipc miscdev
	sprintf(dev_name, "bstipc%ud", bstipc->id);

	IPC_LOG_INFO("bstipc struct ptr: %px", bstipc);
	IPC_LOG_INFO("dev_name: %s", dev_name);

	memcpy(&bstipc->miscdev, &ipc_misc_base, sizeof(struct miscdevice));
	bstipc->miscdev.name = devm_kstrdup(&pdev->dev, dev_name, GFP_KERNEL);
	bstipc->miscdev.nodename = devm_kstrdup(&pdev->dev, dev_name, GFP_KERNEL);

	//regist miscdev
	ret = misc_register(&bstipc->miscdev);
	if (ret < 0) {
		goto err_destory_pool;
	}

	dev_info(bstipc->dev, "device[%px] %s is registed.\n", bstipc->dev, dev_name);
	// init_all_core(false);
	IPC_LOG_DEBUG("%s", "exit");
	return ret;

err_destory_pool:
	ipc_destroy_cma_mempool(bstipc->pool);
	return ret;
}

static int32_t ipc_remove(struct platform_device *pdev)
{
	int32_t ret;
	struct bstipc *bstipc = platform_get_drvdata(pdev);

	IPC_LOG_DEBUG("%s","enter");

	misc_deregister(&bstipc->miscdev);
	ret = ipc_communication_close();
	//todo:destroy session list and msg list
	ipc_session_destroy(current->pid);

	if (ret < 0)
	{
		IPC_DEV_ERR(&pdev->dev, "ipc_communication_close fail, ret %d\n", ret);
		return ret;
	}

	IPC_LOG_DEBUG("%s", "exit");
	return 0;
}

static const struct of_device_id ipc_of_match[] = {
	{
		.compatible = "bst,ipc",
	},
};

static struct platform_driver ipc_driver = {
	.probe   = ipc_probe,
	.remove  = ipc_remove,
	.driver  = {
		.name = IPC_DRIVER_NAME,
		.of_match_table = of_match_ptr(ipc_of_match),
		//.pm = &bstn_pm_ops,
	},
};

static int32_t __init ipc_driver_init(void)
{
	return platform_driver_register(&ipc_driver);
}

device_initcall_sync(ipc_driver_init);

MODULE_AUTHOR("Renfeng Li, Wenjian Gou, Linen");
MODULE_DESCRIPTION("IPC: Linux device driver for Black Sesame Technologies Inter Proccesso");
