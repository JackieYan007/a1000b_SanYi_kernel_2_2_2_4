/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * author: Shichen Lin (shichen.lin@bst.ai)
 *
 * @file    bst_cv_fw_manager.c
 * @brief   This file is the source code file of the firmware manager part of
 *          the bst_cv driver. It contains function definitions of
 *          initializaiton, cleanup and exit of the firmware manager and the
 *          runtime firmware setup.
 */

#include "bst_cv.h"

#define SYS_CTRL_R_CORENOC_PARITY_ENABLE (0x33000084)
#define CV_PARITY_EN			 (0x2)

// the hard coded constants for firmware initialization
static const uint32_t ipc_dsp_irq_clear_addrs[BST_CV_DSP_NUM] = {0x33102a28};
static const uint32_t ipc_dsp_irq_trigger_addr = 0x33102a0c;
static const uint32_t ipc_dsp_ireq_enable_addrs[BST_CV_DSP_NUM] = {0x33102aa8};
static const uint32_t ipc_host_irq_clear_addr = 0x3310230c;

static const uint32_t ipc_device_irq_indices[BST_CV_DSP_NUM] = {4};

static const uint32_t ipc_host_to_dsp_addrs[BST_CV_DSP_NUM] = {0x8ff000c0};
static const uint32_t ipc_dsp_to_host_addrs[BST_CV_DSP_NUM] = {0x8ff00280};

/*******************************************************************************
 * bst_cv FIRMWARE
 ******************************************************************************/
#ifdef BST_CV_DEBUG
/*
 * @func    _dump_firmware
 * @brief   This is a debug function to dump the firmware buffer.
 * @params  fw_buf - the pointer to the firmware buffer
 *          size - the dumped size
 * @return  void
 */
static void _dump_firmware(char *fw_buf, int size)
{
    int i;
    for(i = 0; i < size/BST_CV_FIRMWARE_DUMP_LINE_SIZE; i++) {
        BST_CV_TRACE_PRINTK("%02x %02x %02x %02x %02x %02x %02x %02x",
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 1),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 2),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 3),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 4),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 5),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 6),
            *(fw_buf + i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 7));
    }
    return;
}
#endif

/*
 * @func    _load_firmware
 * @brief   This function loads the firmware.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
static int _load_firmware(struct bst_cv *pbst_cv, int i, struct firmware *fw)
{
    if (pbst_cv->fw_manager.dsps[i].fwmem_size < fw->size) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "firmware too large for DSP %d", i);
        return -EINVAL;
    }

    BST_CV_TRACE_PRINTK("firmware mem base: 0x%px, size: %lld", pbst_cv->fw_manager.dsps[i].fwmem_base,
        pbst_cv->fw_manager.dsps[i].fwmem_size);
    BST_CV_TRACE_PRINTK("firmware data: 0x%px, size: %ld", fw->data, fw->size);

    memcpy_toio(pbst_cv->fw_manager.dsps[i].fwmem_base, fw->data, fw->size);

#ifdef BST_CV_DEBUG
    _dump_firmware(pbst_cv->fw_manager.dsps[i].fwmem_base, BST_CV_FIRMWARE_DUMP_SIZE);
#endif

    return 0;
}

/*
 * @func    _boot_firmware
 * @brief   This fucntion resets the firmware to boot it up.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
static void _boot_firmware(struct bst_cv *pbst_cv, int i)
{
    uint32_t reg;
    void __iomem *crm_base = NULL;
    void __iomem *corenoc_parity_enable = NULL;
    uint32_t cv_parity_en;
    
    crm_base = ioremap(0x33002000, 0x200); // SYS_CRM base address
    // set up CRM register for CV block, or else OS hangs after vector reset
    reg = readl_relaxed(crm_base + 0x4); // TOP_CRM_REG_R_TOP_CRM_BLOCK_SW_RST0
    reg |= (1UL << 7); // cv block soft reset
    writel_relaxed(reg, crm_base + 0x4);
    
    // set CV freq to 800MHz
    reg = readl_relaxed(crm_base + 0x15c); // CLKMUX_SEL2
    // CLK_800_CV_CORE_CLK_SEL, bit[3:2] = 01: 800MHz
    reg &= ~(1UL << 3);
    reg |= (1UL << 2);
    writel_relaxed(reg, crm_base + 0x15c);

    corenoc_parity_enable =
        devm_ioremap(&pbst_cv->pdev->dev, SYS_CTRL_R_CORENOC_PARITY_ENABLE, 0x4);
    if (corenoc_parity_enable == NULL) {
        dev_err(&pbst_cv->pdev->dev,
            "IOREMAP SYS_CTRL_R_CORENOC_PARITY_ENABLE 0x%8X FAILED\n",
            SYS_CTRL_R_CORENOC_PARITY_ENABLE);
        return;
    }
    cv_parity_en = readl_relaxed(corenoc_parity_enable) & CV_PARITY_EN;
    devm_iounmap(&pbst_cv->pdev->dev, corenoc_parity_enable);

    reg = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base + 0x50);
    if (cv_parity_en) {
        reg |= 0x3; // cv block ecc and parity enable
        writel_relaxed(reg, pbst_cv->fw_manager.lb_cv_reg_base + 0x50);
    }

    //setup reset vector
    BST_CV_STAGE_PRINTK("set vector: 0x%x, addr: 0x%px",
        addr_truncate(phys_to_bus(pbst_cv, pbst_cv->fw_manager.dsps[i].fwmem_phys_addr)),
        pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_DSP_ALT_RESET_VEC_OFFSET + i * BST_CV_REG_WIDTH);
    writel_relaxed(addr_truncate(phys_to_bus(pbst_cv, pbst_cv->fw_manager.dsps[i].fwmem_phys_addr)),
        pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_DSP_ALT_RESET_VEC_OFFSET + i * BST_CV_REG_WIDTH);

    //start dsp
    reg = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    reg |= (1 << (BST_CV_SOFT_RESET_BIT + i));
    reg |= (1 << (BST_CV_CLK_EN_BIT + i));
    BST_CV_STAGE_PRINTK("set ctrl value: 0x%x, addr: 0x%px", reg,
        pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    writel_relaxed(reg, pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    return;
}

/*
 * @func    bst_cv_boot_firmware
 * @brief   This function loads the runtime firmware and boots it up.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
static int bst_cv_boot_firmware(struct bst_cv *pbst_cv)
{
    int ret;
    int i;
    struct firmware *fw;

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (pbst_cv->dsp_online[i]) {
            ret = request_firmware((const struct firmware **)&fw,
                pbst_cv->fw_manager.dsps[i].name, &pbst_cv->pdev->dev);
            if (ret < 0) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to request firmware for DSP %d: %d", i, ret);
                pbst_cv->dsp_online[i] = 0;
                continue;
            }
            BST_CV_STAGE_PRINTK("bst_cv firmware requested for DSP %d", i);

            ret = _load_firmware(pbst_cv, i, fw);
            release_firmware(fw);
            if (ret < 0) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to load firmware for DSP %d: %d", i, ret);
                pbst_cv->dsp_online[i] = 0;
                
                continue;
            }
            BST_CV_STAGE_PRINTK("bst_cv firmware loaded for DSP %d", i);

            _boot_firmware(pbst_cv, i);
            pbst_cv->fw_manager.dsps[i].boot = 1;
            BST_CV_STAGE_PRINTK("bst_cv firmware booted for DSP %d", i);
        }
    }

    return bst_cv_check_online(pbst_cv) ? 0 : -EFAULT;
}

/*
 * @func    bst_cv_fw_manager_init
 * @brief   This function initializes the firmware manager.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
int bst_cv_fw_manager_init(struct bst_cv *pbst_cv)
{
    int i;
    int ret;
    struct resource *res;
    char *fw_names[BST_CV_DSP_NUM];
    u32 assigned_mem_sizes[BST_CV_DSP_NUM];
    u64 reg[2];

    //map registers
    ret = device_property_read_u64_array(&pbst_cv->pdev->dev, "reg", reg, 2);
    if (ret == -EINVAL || ret == -ENODATA) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no reg property");
        return ret;
    } else if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid reg property");
        return ret;
    }
    pbst_cv->fw_manager.lb_cv_reg_base = devm_ioremap(&pbst_cv->pdev->dev,
        reg[0], reg[1]);
    if (IS_ERR(pbst_cv->fw_manager.lb_cv_reg_base)) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to map cv regs");
        return PTR_ERR(pbst_cv->fw_manager.lb_cv_reg_base);
    }

    //get firmware names
    ret = device_property_read_string_array(&pbst_cv->pdev->dev, "firmware", 
        (const char **)fw_names, BST_CV_DSP_NUM);
    if (ret == -EINVAL || ret == -ENODATA){
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no firmware property found");
        return ret;
    }
    else if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid firmware property");
        return ret;
    }

    //get assigned memory sizes
    ret = device_property_read_u32_array(&pbst_cv->pdev->dev, "assigned-mem-size",
        assigned_mem_sizes, BST_CV_DSP_NUM);
    if (ret == -EINVAL || ret == -ENODATA) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no assigned-mem-size property");
        return ret;
    } else if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid assigned-mem-size");
        return ret;
    }

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        //map firmware memory
        res = platform_get_resource(pbst_cv->pdev, IORESOURCE_MEM, i + 1);
        if (!res) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "could not get firmware resource for DSP %d", i);
            pbst_cv->dsp_online[i] = 0;
            pbst_cv->fw_manager.dsps[i].init = 0;
            continue;
        }
        BST_CV_TRACE_PRINTK("bst_cv firmware mem of DSP %d start: 0x%llx, end: 0x%llx",
            i, res->start, res->end);

        pbst_cv->fw_manager.dsps[i].fwmem_base = devm_ioremap_resource(&pbst_cv->pdev->dev,
            res);
        if (IS_ERR(pbst_cv->fw_manager.dsps[i].fwmem_base)) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to remap firmware mem: %d",
                ret);
            pbst_cv->dsp_online[i] = 0;
            pbst_cv->fw_manager.dsps[i].init = 0;
            continue;
        }
        pbst_cv->fw_manager.dsps[i].fwmem_size = res->end - res->start + 1;
        pbst_cv->fw_manager.dsps[i].fwmem_phys_addr = res->start;

        pbst_cv->fw_manager.dsps[i].name = fw_names[i];
        BST_CV_STAGE_PRINTK("firmware of dsp %d: %s", i, pbst_cv->fw_manager.dsps[i].name);

        //assign memory for firmware
        pbst_cv->fw_manager.dsps[i].assigned_mem = pbst_cv->mem_manager.ops->alloc(pbst_cv,
            assigned_mem_sizes[i], 0);
        if (pbst_cv->fw_manager.dsps[i].assigned_mem == NULL) {
            BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "could not allocate assigned-mem for DSP %d", i);
            pbst_cv->dsp_online[i] = 0;
            pbst_cv->fw_manager.dsps[i].init = 0;
            continue;
        }
        BST_CV_STAGE_PRINTK("assigned mem of DSP %d: 0x%px, 0x%llx, size: %d",
            i,
            pbst_cv->fw_manager.dsps[i].assigned_mem->kern_addr,
            pbst_cv->fw_manager.dsps[i].assigned_mem->phys_addr,
            assigned_mem_sizes[i]);

        pbst_cv->dsp_online[i] = 1;
        pbst_cv->fw_manager.dsps[i].init = 1;
    }
    return 0;
}

/*
 * @func    _dump_xrp_dsp_sync
 * @brief   This function dumps the xrp_dsp_sync structure used in the handshake
 *          with the firmware
 * @params  pbst_cv - the pointer to the bst_cv device
 *          xrp_dsp_sync_base - the base address of the xrp_dsp_sync structure
 * @return  void
 */
static inline void _dump_xrp_dsp_sync(struct bst_cv *pbst_cv, struct xrp_dsp_sync *xrp_dsp_sync_base){
    int j;
    
    for(j=0; j<(sizeof(*xrp_dsp_sync_base)+15)/16; j++) {
        BST_CV_TRACE_PRINTK("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x",
        kern_to_bus(pbst_cv, xrp_dsp_sync_base) + j * 16,
        *((uint32_t *)xrp_dsp_sync_base + j * 4),
        *((uint32_t *)xrp_dsp_sync_base + j * 4 + 1),
        *((uint32_t *)xrp_dsp_sync_base + j * 4 + 2),
        *((uint32_t *)xrp_dsp_sync_base + j * 4 + 3));
    }
    return;
}

/*
 * @func    bst_cv_fw_rt_setup
 * @brief   This function initializes the runtime firmware. Because it relies on
 *          messaging and the message manager can only be initialized after
 *          getting some required firmware information in bst_cv_fw_manager_init,
 *          this part is separated from the firmware manager initialization.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
int bst_cv_fw_rt_setup(struct bst_cv *pbst_cv)
{
    int i, count;
    int ret = 0;
    struct xrp_dsp_sync *xrp_dsp_sync_base;
    uint32_t data;

    //boot firmware
    ret = bst_cv_boot_firmware(pbst_cv);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "bst_cv_boot_firmware failed for all DSPs");
        return ret;
    }
    BST_CV_STAGE_PRINTK("bst_cv_boot_firmware OK");

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (pbst_cv->dsp_online[i]) {
            //sync setup(really stupid and unnecessary step from firmware side)
            xrp_dsp_sync_base = pbst_cv->fw_manager.dsps[i].assigned_mem->kern_addr;
            xrp_dsp_sync_base->device_mmio_base = 0;
            xrp_dsp_sync_base->host_irq_mode = XRP_DSP_SYNC_IRQ_MODE_BST_IPC;
            xrp_dsp_sync_base->host_irq_offset = 0;
            xrp_dsp_sync_base->host_irq_bit = IPC_CORE_DSP_0 + i; 
            xrp_dsp_sync_base->device_irq_mode = XRP_DSP_SYNC_IRQ_MODE_BST_IPC;
            xrp_dsp_sync_base->device_irq_offset = 0;
            xrp_dsp_sync_base->device_irq_bit = IPC_CORE_ARM3;
            xrp_dsp_sync_base->device_irq = ipc_device_irq_indices[i];
            xrp_dsp_sync_base->host_ipc_irq_clear = ipc_host_irq_clear_addr;
            xrp_dsp_sync_base->host_irq = 0x91;
            xrp_dsp_sync_base->device_ipc_irq_clear = ipc_dsp_irq_clear_addrs[i];
            xrp_dsp_sync_base->host_ipc_irq_trig = ipc_dsp_irq_trigger_addr;
            xrp_dsp_sync_base->device_ipc_irq_trig = 0x33102328;
            xrp_dsp_sync_base->debug_buffer_base = bst_cv_firmware_log_buffer;
            xrp_dsp_sync_base->debug_buffer_length = bst_cv_firmware_log_length;
            xrp_dsp_sync_base->debug_level = bst_cv_firmware_log_level;
            xrp_dsp_sync_base->ipc_host_to_dsp_addr = ipc_host_to_dsp_addrs[i];
            xrp_dsp_sync_base->ipc_dsp_to_host_addr = ipc_dsp_to_host_addrs[i];
            xrp_dsp_sync_base->device_ipc_irq_enable = ipc_dsp_ireq_enable_addrs[i];
            xrp_dsp_sync_base->device_ipc_irq_enable_mask = (1 << 30) | (1 << IPC_CORE_ARM3); 

#ifdef BST_CV_DEBUG
            _dump_xrp_dsp_sync(pbst_cv, xrp_dsp_sync_base);
#endif
            //start handshaking
            xrp_dsp_sync_base->sync = XRP_DSP_SYNC_START;
            count = 0;
            while (xrp_dsp_sync_base->sync != XRP_DSP_SYNC_DSP_READY
                && count < BST_CV_HANDSHAKE_RETRY_NUM) {
                msleep(BST_CV_HANDSHAKE_SLEEP_INTERVAL);
                count++;
            }
            if (count == BST_CV_HANDSHAKE_RETRY_NUM) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to wait for handshake ready from DSP %d", i);
                pbst_cv->dsp_online[i] = 0;
                continue;
            }
            xrp_dsp_sync_base->sync = XRP_DSP_SYNC_HOST_TO_DSP;
            count = 0;
            while (xrp_dsp_sync_base->sync != XRP_DSP_SYNC_DSP_TO_HOST
                 && count < BST_CV_HANDSHAKE_RETRY_NUM) {
                msleep(BST_CV_HANDSHAKE_SLEEP_INTERVAL);
                count++;
            }
            if (count == BST_CV_HANDSHAKE_RETRY_NUM) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to wait for handshake sync from DSP %d", i);
                pbst_cv->dsp_online[i] = 0;
                continue;
            }
            if (bst_cv_msg_send(pbst_cv, i, 0) < 0) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to send interrupt handshake to DSP %d", i);
                pbst_cv->dsp_online[i] = 0;
                continue;
            }
            if (bst_cv_msg_recv(pbst_cv, i, &data, BST_CV_HANDSHAKE_TIMEOUT) < 0) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to receive interrupt handshake from DSP %d", i);
                pbst_cv->dsp_online[i] = 0;
                continue;
            }
        }
    }

    bst_cv_fw_rt_cleanup(pbst_cv);
    return bst_cv_check_online(pbst_cv) ? 0 : -EFAULT;
}

/*
 * @func    bst_cv_fw_manager_cleanup
 * @brief   This function cleans up the resources allocated in firmware
 *          manager initialization for subsequential failure of DSPs or even
 *          the driver during the entire initialization process.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bst_cv_fw_manager_cleanup(struct bst_cv *pbst_cv) {
    int i;

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (!pbst_cv->dsp_online[i] && pbst_cv->fw_manager.dsps[i].init) {
            pbst_cv->mem_manager.ops->free(pbst_cv->fw_manager.dsps[i].assigned_mem);
        }
    }
    return;
}

/*
 * @func    _release_rt_firmware
 * @brief   This function resets the specified DSP.
 * @params  pbst_cv - the pointer to the bst_cv device
 *          dsp - the CV DSP number
 * @return  void
 */
static inline void _release_rt_fw(struct bst_cv *pbst_cv, int dsp)
{
    uint32_t reg;

    //stall the DSP first
    reg = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    reg |= (1 << (BST_CV_RUNSTALL_BIT + dsp));
    writel_relaxed(reg, pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    //then reset the DSP
    writel_relaxed(LB_CV_REG_R_CV_SYS_CTRL_DEFAULT, pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
    return;

}

/*
 * @func    bst_cv_fw_rt_cleanup
 * @brief   This function resets the DSPs for subsequential failure of DSPs or
 *          even the driver during the entire initialization process.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bst_cv_fw_rt_cleanup(struct bst_cv *pbst_cv) {
    int i;

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (!pbst_cv->dsp_online[i] && pbst_cv->fw_manager.dsps[i].boot) {
            _release_rt_fw(pbst_cv, i);
        }
    }
    return;
}

/*
 * @func    bst_cv_fw_manager_exit
 * @brief   This is the exit function of the firmware manager.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bst_cv_fw_manager_exit(struct bst_cv *pbst_cv)
{
    int i;

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (pbst_cv->dsp_online[i]) {
            pbst_cv->mem_manager.ops->free(pbst_cv->fw_manager.dsps[i].assigned_mem);
        }
    }
    return;
}

/*
 * @func    bst_cv_fw_rt_exit
 * @brief   This is the exit function of the runtime firmware.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bst_cv_fw_rt_exit(struct bst_cv *pbst_cv)
{
    int i;

    for (i=0; i<BST_CV_DSP_NUM; i++) {
        if (pbst_cv->dsp_online[i]) {
            _release_rt_fw(pbst_cv, i);
        }
    }
    return;
}
