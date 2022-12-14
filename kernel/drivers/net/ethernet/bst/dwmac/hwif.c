// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Copyright (c) 2018 Synopsys, Inc. and/or its affiliates.
 * bstgmac HW Interface Handling
 */

#include "common.h"
#include "bstgmac.h"
#include "dwmac_ptp.h"

static u32 bstgmac_get_id(struct bstgmac_priv *priv, u32 id_reg)
{
	u32 reg = readl(priv->ioaddr + id_reg);

	if (!reg) {
		dev_info(priv->device, "Version ID not available\n");
		return 0x0;
	}

	dev_info(priv->device, "User ID: 0x%x, Synopsys ID: 0x%x\n",
			(unsigned int)(reg & GENMASK(15, 8)) >> 8,
			(unsigned int)(reg & GENMASK(7, 0)));
	return reg & GENMASK(7, 0);
}

static void bstgmac_dwmac_mode_quirk(struct bstgmac_priv *priv)
{
	struct mac_device_info *mac = priv->hw;

	if (priv->chain_mode) {
		dev_info(priv->device, "Chain mode enabled\n");
		priv->mode = BSTGMAC_CHAIN_MODE;
		mac->mode = &chain_mode_ops;
	} else {
		dev_info(priv->device, "Ring mode enabled\n");
		priv->mode = BSTGMAC_RING_MODE;
		mac->mode = &ring_mode_ops;
	}
}

static int bstgmac_dwmac1_quirks(struct bstgmac_priv *priv)
{
	struct mac_device_info *mac = priv->hw;

	if (priv->plat->enh_desc) {
		dev_info(priv->device, "Enhanced/Alternate descriptors\n");

		/* GMAC older than 3.50 has no extended descriptors */
		if (priv->synopsys_id >= DWMAC_CORE_3_50) {
			dev_info(priv->device, "Enabled extended descriptors\n");
			priv->extend_desc = 1;
		} else {
			dev_warn(priv->device, "Extended descriptors not supported\n");
		}

		mac->desc = &enh_desc_ops;
	} else {
		dev_info(priv->device, "Normal descriptors\n");
		mac->desc = &ndesc_ops;
	}

	bstgmac_dwmac_mode_quirk(priv);
	return 0;
}

static int bstgmac_dwmac4_quirks(struct bstgmac_priv *priv)
{
	bstgmac_dwmac_mode_quirk(priv);
	return 0;
}

static const struct bstgmac_hwif_entry {
	bool gmac;
	bool gmac4;
	bool xgmac;
	u32 min_id;
	const struct bstgmac_regs_off regs;
	const void *desc;
	const void *dma;
	const void *mac;
	const void *hwtimestamp;
	const void *mode;
	const void *tc;
	int (*setup)(struct bstgmac_priv *priv);
	int (*quirks)(struct bstgmac_priv *priv);
} bstgmac_hw[] = {
	/* NOTE: New HW versions shall go to the end of this table */
	{
		.gmac = false,
		.gmac4 = true,
		.xgmac = false,
		.min_id = DWMAC_CORE_5_10,
		.regs = {
			.ptp_off = PTP_GMAC4_OFFSET,
			.mmc_off = MMC_GMAC4_OFFSET,
		},
		.desc = &dwmac4_desc_ops,
		.dma = &dwmac410_dma_ops,
		.mac = &dwmac510_ops,
		.hwtimestamp = &bstgmac_ptp,
		.mode = &dwmac4_ring_mode_ops,
		.tc = &dwmac510_tc_ops,
		.setup = dwmac4_setup,
		.quirks = NULL,
	},
};

int bstgmac_hwif_init(struct bstgmac_priv *priv)
{
	bool needs_xgmac = priv->plat->has_xgmac;
	bool needs_gmac4 = priv->plat->has_gmac4;
	bool needs_gmac = priv->plat->has_gmac;
	const struct bstgmac_hwif_entry *entry;
	struct mac_device_info *mac;
	bool needs_setup = true;
	int i, ret;
	u32 id;

	if (needs_gmac) {
		id = bstgmac_get_id(priv, GMAC_VERSION);
	} else if (needs_gmac4 || needs_xgmac) {
		id = bstgmac_get_id(priv, GMAC4_VERSION);
	} else {
		id = 0;
	}

	/* Save ID for later use */
	priv->synopsys_id = id;

	/* Lets assume some safe values first */
	priv->ptpaddr = priv->ioaddr +
		(needs_gmac4 ? PTP_GMAC4_OFFSET : PTP_GMAC3_X_OFFSET);
	priv->mmcaddr = priv->ioaddr +
		(needs_gmac4 ? MMC_GMAC4_OFFSET : MMC_GMAC3_X_OFFSET);

	/* Check for HW specific setup first */
	if (priv->plat->setup) {
		mac = priv->plat->setup(priv);
		needs_setup = false;
	} else {
		mac = devm_kzalloc(priv->device, sizeof(*mac), GFP_KERNEL);
	}

	if (!mac)
		return -ENOMEM;
	/* Fallback to generic HW */
	for (i = ARRAY_SIZE(bstgmac_hw) - 1; i >= 0; i--) {
		entry = &bstgmac_hw[i];

		if (needs_gmac ^ entry->gmac)
			continue;
		if (needs_gmac4 ^ entry->gmac4)
			continue;
		if (needs_xgmac ^ entry->xgmac)
			continue;
		/* Use synopsys_id var because some setups can override this */
		if (priv->synopsys_id < entry->min_id)
			continue;

		/* Only use generic HW helpers if needed */
		mac->desc = mac->desc ? : entry->desc;
		mac->dma = mac->dma ? : entry->dma;
		mac->mac = mac->mac ? : entry->mac;
		mac->ptp = mac->ptp ? : entry->hwtimestamp;
		mac->mode = mac->mode ? : entry->mode;
		mac->tc = mac->tc ? : entry->tc;

		priv->hw = mac;
		priv->ptpaddr = priv->ioaddr + entry->regs.ptp_off;
		priv->mmcaddr = priv->ioaddr + entry->regs.mmc_off;

		/* Entry found */
		if (needs_setup) {
			ret = entry->setup(priv);
			if (ret)
				return ret;
		}

		/* Save quirks, if needed for posterior use */
		priv->hwif_quirks = entry->quirks;
		return 0;
	}

	dev_err(priv->device, "Failed to find HW IF (id=0x%x, gmac=%d/%d)\n",
			id, needs_gmac, needs_gmac4);
	return -EINVAL;
}
