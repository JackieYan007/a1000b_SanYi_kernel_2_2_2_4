#ifndef IPC_MEMPOOL_H
#define IPC_MEMPOOL_H

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
struct ipc_memblock {
	struct ipc_mempool *pool;
	phys_addr_t phy_addr;
	void *k_addr;
	void* u_addr;
	u32 size;
	struct list_head memblock_list;
};
struct ipc_mempool_ops {
	 int32_t (*alloc)(struct ipc_mempool *mempool, u32 size, u32 align, struct ipc_memblock **memblock);
	 void (*free)(struct ipc_memblock *memblock);
	 phys_addr_t (*phyaddr)(const struct ipc_memblock *memblock);
	 void *(*kaddr)(const struct ipc_memblock *memblock);
};

struct ipc_mempool {
	const struct ipc_mempool_ops *ops;
	struct device *dev;
	struct list_head memblock_list_head;
};

static inline phys_addr_t ipc_memblock_phyaddr(const struct ipc_memblock *memblock)
{
	return memblock->phy_addr;
}

static inline uint32_t ipc_memblock_size(const struct ipc_memblock *memblock)
{
	return memblock->size;
}

static inline void* ipc_memblock_kaddr(const struct ipc_memblock *memblock)
{
	return memblock->k_addr;
}


int32_t ipc_init_cma_mempool(struct ipc_mempool **ppool, struct device *dev);
void ipc_destroy_cma_mempool(struct ipc_mempool *pool);

#endif