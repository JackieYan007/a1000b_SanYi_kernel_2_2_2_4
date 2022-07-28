/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * author: Shichen Lin (shichen.lin@bst.ai)
 *
 * @file    bst_cv_fw_manager.h
 * @brief   This file is the header file of the firmware manager part of the
 *          bst_cv driver. It contains related constants and structure
 *          definitions and function declarations.
 */

#ifndef BST_CV_FW_MANAGER_H
#define BST_CV_FW_MANAGER_H

#define BST_CV_HANDSHAKE_TIMEOUT                    100
#define BST_CV_HANDSHAKE_SLEEP_INTERVAL             10
#define BST_CV_HANDSHAKE_RETRY_NUM                  (BST_CV_HANDSHAKE_TIMEOUT / BST_CV_HANDSHAKE_SLEEP_INTERVAL)

#define BST_CV_REG_WIDTH                            4

#define BST_CV_FIRMWARE_DUMP_SIZE                   64
#define BST_CV_FIRMWARE_DUMP_LINE_SIZE              8

#define LB_CV_REG_R_CV_SYS_CTRL_OFFSET              0x0
#define BST_CV_SOFT_RESET_BIT                       28
#define BST_CV_CLK_EN_BIT                           24
#define BST_CV_RUNSTALL_BIT                         20
#define LB_CV_REG_R_CV_SYS_CTRL_DEFAULT             0x0f000000

#define LB_CV_REG_R_CV_DSP_ALT_RESET_VEC_OFFSET     0x4

#define CACHE_LINE_SIZE                             64

struct bst_cv_rt_setup_info {
    dsp_ptr assigned_mem;
    uint32_t assigned_mem_size;
};

struct bst_cv_dsp_fw_ctl {
    uint8_t init;
    uint8_t boot;
    char *name;
    //memory reserved for firmware codes
    void __iomem *fwmem_base;
    resource_size_t fwmem_size;
    phys_addr_t fwmem_phys_addr;
    struct bst_cv_memblock *assigned_mem; //memory assigned to firmwarelb_cv_reg_base
};

struct bst_cv_fw_manager {
    //the net registers
    void __iomem *lb_cv_reg_base;
    struct bst_cv_dsp_fw_ctl dsps[BST_CV_DSP_NUM];
};

int bst_cv_fw_manager_init(struct bst_cv *pbst_cv);
int bst_cv_fw_rt_setup(struct bst_cv *pbst_cv);
void bst_cv_fw_manager_cleanup(struct bst_cv *pbst_cv);
void bst_cv_fw_rt_cleanup(struct bst_cv *pbst_cv);
void bst_cv_fw_manager_exit(struct bst_cv *pbst_cv);
void bst_cv_fw_rt_exit(struct bst_cv *pbst_cv);

#endif
