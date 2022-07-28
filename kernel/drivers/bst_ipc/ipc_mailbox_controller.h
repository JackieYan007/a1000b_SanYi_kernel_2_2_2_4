#ifndef IPC_MAILBOX_CONTROLLER_H
#define IPC_MAILBOX_CONTROLLER_H

#include <linux/mailbox_controller.h>
#include "ipc_common.h"
#include <linux/ipc_interface.h>

struct ipc_mbox_client_info;

typedef enum rx_mode {
	IRQ_MODE = 0,
	POLL_MODE
} RX_MODE;

struct ipc_mbox {
	struct device *dev;
	void __iomem *event_base;
	void __iomem *sem_base;
	struct ipc_mempool *pool;
#ifdef ON_FPGA
	void __iomem *fpga_reset;
	void __iomem *fpga_status;
#endif
};

int32_t request_mbox_chan(struct ipc_mbox_client_info* cl_info);
int32_t ipc_send_data(struct ipc_mbox_client_info *client_info, enum ipc_core_e src, void *data);
int32_t alloc_mem(struct ipc_buffer * ipc_buffer);

#endif
