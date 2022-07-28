#ifndef IPC_NODEMANAGER_H
#define IPC_NODEMANAGER_H

#include "ipc_common.h"
#include "user_head.h"

struct ipc_mbox_client_info;

struct device* ipc_get_dev_by_coreid(enum ipc_core_e core);
int32_t ipc_node_create(struct ipc_mbox_client_info *cl_info);
int32_t ipc_node_destroy(enum ipc_core_e core_id);
int32_t ipc_node_msgparse(enum ipc_core_e core, struct ipc_mbox_client_info **cl_info);
int32_t ipc_node_valid(enum ipc_core_e core_id);
int32_t ipc_node_create_session(enum ipc_core_e dest_core_id,enum ipc_core_e src_core_id);
int32_t ipc_node_get_node_of_coreid(enum ipc_core_e core_id, struct ipc_mbox_client_info **cl_info);
struct ipc_session* ipc_node_get_session_by_coreid(struct ipc_mbox_client_info* cl_info,enum ipc_core_e core_id);
uint64_t get_tx_reg_by_intr(int32_t intr);

#endif
