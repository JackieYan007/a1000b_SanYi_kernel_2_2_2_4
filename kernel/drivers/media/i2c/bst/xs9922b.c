// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * xs9922b yuv camera for BST Camera Driver
 *
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <linux/of_gpio.h>
#include <media/media-entity.h>
#include <media/media-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>
#include "common_deser_hub.h"
#include "xs9922.h"

#define MODULE_NAME "bst,xs9922b"

#define DESER_MISC_MINOR_BASE 150
static struct deser_hub_dev* hubs[4];


#define NI_ID0		    (0x22)
#define NI_ID1		    (0x99)
#define NI_40F0_DEVICE_ID_1 (0x40F0)
#define NI_40F1_DEVICE_ID_0 (0x40F1)

#define VIDEO_STATUS_REG_CH0	(0x0000)
#define VIDEO_STATUS_REG_CH1	(0x1000)
#define VIDEO_STATUS_REG_CH2	(0x2000)
#define VIDEO_STATUS_REG_CH3	(0x3000)
#define CDT_STATUS_CH0			(0x4345)
#define CDT_STATUS_CH1			(0x4346)
#define CDT_STATUS_CH2			(0x4347)
#define CDT_STATUS_CH3			(0x4348)

#define VIDEO_LOCK_MASK		(0x0e)
#define CDT_CABLE_MASK		(0x01)

#define REG_NULL	       0xFFFF
#define REG_DELAY	       0xFFFE
#define XS9922_REG_VALUE_08BIT 1
#define XS9922_REG_VALUE_16BIT 2
#define XS9922_REG_VALUE_24BIT 3

#define XS9922_RESET   0
#define CAMERA_POWERON 1

static int xs9922_write_reg(struct i2c_client *client, u16 reg, u32 len,
			    u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2) {
		dev_err(&client->dev, "xs9922 write reg(0x%x) failed !\n", reg);
		return -EIO;
	}

	return 0;
}

static int xs9922_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			   u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	// printk("%p ---- 0x%04x %d %d\n", client, reg, len, *val);

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	// dev_dbg(&client->dev, "xs9922 i2c addr (0x%x) !\n", client->addr);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "xs9922 read reg(0x%x) failed !\n", reg);
		return -EIO;
	}

	// printk("---- 0x%08x\n", data_be);

	*val = be32_to_cpu(data_be);
	return 0;
}

static int xs9922_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;
	int retry = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret |= xs9922_write_reg(client, regs[i].addr,
					XS9922_REG_VALUE_08BIT, regs[i].val);

		if (regs[i].nDelay != 0)
			usleep_range(regs[i].nDelay * 1000,
				     (regs[i].nDelay + 1) * 1000);
		dev_info(&client->dev, "xs9922 write reg(0x%04x) : 0x%02x!\n",
			 regs[i].addr, regs[i].val);

		while (ret) {
			usleep_range(5000, 6000);
			ret |= xs9922_write_reg(client, regs[i].addr,
						XS9922_REG_VALUE_08BIT,
						regs[i].val);
			retry++;
			if (retry == 10)
				break;

			dev_err(&client->dev,
				"xs9922 write reg(0x%04x) : happen err! value:(0x%04x)\n",
				regs[i].addr, regs[i].val);
		}
	}
	return ret;
}

static void xs9922b_switch_mode(struct deser_hub_dev *hub , int mode)
{
	int ret = 0;

	if(mode == 1)
	{
		ret = xs9922_write_array(hub->i2c_client,
					xs9922_720p_4lanes_25fps_1500M);
	}
	else if( mode == 2)
	{
		ret = xs9922_write_array(hub->i2c_client,
					xs9922_1080p_4lanes_25fps_1500M);
	}
	else
	{
		dev_err(hub->dev, "%s get mode err\n", __func__);
		return;
	}
	dev_err(hub->dev, "%s RL_DEBUG\n", __func__);

	if (ret) {
		dev_err(hub->dev, "%s write xs9922 register array error\n",
			__func__);
	}
}

static int xs9922b_global_config(struct deser_hub_dev *hub)
{
	int ret = 0;

	ret = xs9922_write_array(hub->i2c_client, xs9922_init_cfg);
	dev_err(hub->dev, "%s RL_DEBUG\n", __func__);
	if (ret)
		dev_err(hub->dev, "%s write xs9922 init error\n", __func__);
	return 0;
}

static int xs9922_start_stream(struct deser_hub_dev *hub,int on)
{
	struct i2c_client *client = hub->i2c_client;
	int enable;

	enable = (on & 0XF0) >> 4;

	if (enable == 0 || enable == 4)
	{
		xs9922_write_reg(client, 0x0e08, XS9922_REG_VALUE_08BIT, 0x01);
		dev_info(&client->dev, "xs9922 write reg(0x%04x) : 0x%02x!\n",
				 0x0e08, 0x01);
	}
	else if (enable == 1 || enable == 5)
	{
		xs9922_write_reg(client, 0x1e08, XS9922_REG_VALUE_08BIT, 0x01);
		dev_info(&client->dev, "xs9922 write reg(0x%04x) : 0x%02x!\n",
				 0x1e08, 0x01);
	}
	else if (enable == 2 || enable == 6)
	{
		xs9922_write_reg(client, 0x2e08, XS9922_REG_VALUE_08BIT, 0x01);
		dev_info(&client->dev, "xs9922 write reg(0x%04x) : 0x%02x!\n",
				 0x2e08, 0x01);
	}
	else if (enable == 3 || enable == 7)
	{
		xs9922_write_reg(client, 0x3e08, XS9922_REG_VALUE_08BIT, 0x01);
		dev_info(&client->dev, "xs9922 write reg(0x%04x) : 0x%02x!\n",
				 0x3e08, 0x01);
	}

	xs9922_write_array(client, xs9922_mipi_reset);

	usleep_range(200 * 1000, 400 * 1000);

	return 0;
}

static int xs9922_stop_stream(struct deser_hub_dev *hub)
{
	struct i2c_client *client = hub->i2c_client;

	xs9922_write_reg(client, 0x0e08, XS9922_REG_VALUE_08BIT, 0x00);
	xs9922_write_reg(client, 0x1e08, XS9922_REG_VALUE_08BIT, 0x00);
	xs9922_write_reg(client, 0x2e08, XS9922_REG_VALUE_08BIT, 0x00);
	xs9922_write_reg(client, 0x3e08, XS9922_REG_VALUE_08BIT, 0x00);

	return 0;
}

static int xs9922b_s_stream(struct v4l2_subdev *subdev, int on)
{
	struct deser_hub_dev *hub =
		container_of(subdev, struct deser_hub_dev, subdev);

	if (on)
	{
		dev_err(hub->dev, "RL_DEBUG printf on : %x ", on);
		xs9922_start_stream(hub,on);
	}
	else
		xs9922_stop_stream(hub);

	return 0;
}

#if 0 //For hotlog
/* -----------------------------------------------------------------------------
 * Deser misc perations
 */
static int deser_misc_open(struct inode *inode, struct file *filp) {
	int index = MINOR(inode->i_rdev) - DESER_MISC_MINOR_BASE;
	if (index > 3 || index < 0 )
		pr_err("%s() %d, Error index: %d", __func__, __LINE__, index);
	else
		pr_info("%s() %d, index: %d", __func__, __LINE__, index);

	filp->private_data = hubs[index];
	return 0;
}

static int deser_fasync(int32_t fd, struct file *filp, int32_t mode) {
	struct deser_hub_dev *hub = filp->private_data;
	return fasync_helper(fd, filp, mode, &(hub->async_queue));
}

int is_camera_locked_xs9922b(struct camera_dev *cam_dev)
{
	int index = cam_dev->index_in_serdes;
	struct deser_hub_dev *hub = cam_dev->deser_parent;
	int cdt_status_reg = CDT_STATUS_CH0;
	int video_status_reg = VIDEO_STATUS_REG_CH0;
	int cable_insert = 0, video_lock = 0;
	int ret = 0;
	u32 value = 0;

	/* TODO: read camera lock status here */
	switch(index)
	{
		case 0:
			cdt_status_reg = CDT_STATUS_CH0;
			video_status_reg = VIDEO_STATUS_REG_CH0;
			break;
		case 1:
			cdt_status_reg = CDT_STATUS_CH1;
			video_status_reg = VIDEO_STATUS_REG_CH1;
			break;
		case 2:
			cdt_status_reg = CDT_STATUS_CH2;
			video_status_reg = VIDEO_STATUS_REG_CH2;
			break;
		case 3:
			cdt_status_reg = CDT_STATUS_CH3;
			video_status_reg = VIDEO_STATUS_REG_CH3;
			break;
	}
	xs9922_read_reg(hub->i2c_client, cdt_status_reg, XS9922_REG_VALUE_08BIT, &value);
	pr_info("%s, reg 0x%x, reg value: 0x%x\n", __func__, cdt_status_reg, value);
	if(CDT_CABLE_MASK == (value & CDT_CABLE_MASK))
	{
		cable_insert = 1;
	}
	xs9922_read_reg(hub->i2c_client, video_status_reg, XS9922_REG_VALUE_08BIT, &value);
	pr_info("%s, reg 0x%x, reg value: 0x%x\n", __func__, video_status_reg, value);
	if(VIDEO_LOCK_MASK == (value & VIDEO_LOCK_MASK))
	{
		video_lock = 1;
	}

	ret = (video_lock & cable_insert) != 0 ? 1 : 0;

	pr_info("%s, ret: %d, cable_insert: %d, video_lock: %d \n", __func__, ret, cable_insert, video_lock);
	return ret;
}

static long deser_misc_ioctl(struct file *pfile, unsigned int cmd, unsigned long args) {
	int camera_status[MAX_CAMERAS_PER_SERDES] = {0};
	struct deser_hub_dev *temp_deser_hub = pfile->private_data;
	void __user *argp = (void __user *)args;
	int i, id;

	switch(cmd){
		case IOC_GET_ALL_CAM_CONNECT_STATUS:
			for(i = 0; i < MAX_CAMERAS_PER_SERDES; i++) {
				if(NULL != temp_deser_hub->chn[i].cam_dev)
				{
					camera_status[i] = is_camera_locked_xs9922b(temp_deser_hub->chn[i].cam_dev);
				}
			}
			pr_info("%s() %d", __func__, __LINE__);
			kill_fasync(&(temp_deser_hub->async_queue), SIGIO, POLL_IN);
			return copy_to_user(argp, &camera_status, sizeof(camera_status)) ? -EFAULT : 0;
		case IOC_GET_SIG_CAM_CONNECT_STATUS:
			id = 0;
			if (copy_from_user(&id, argp, sizeof(id))){
				pr_err("%s() %d, copy from user failed\n", __func__, __LINE__);
				return -EFAULT;
			}
			pr_info("%s() %d", __func__, __LINE__);
			camera_status[id] = is_camera_locked_xs9922b(temp_deser_hub->chn[id].cam_dev);
			return copy_to_user(argp, &camera_status, sizeof(camera_status)) ? -EFAULT : 0;
		default:
			pr_info("%s() %d , Not Support\n", __func__, __LINE__);
			return -EINVAL;
	}
}

static const struct file_operations deser_misc_fops = 
{
	.owner          = THIS_MODULE,
	.open			= deser_misc_open,
	.unlocked_ioctl = deser_misc_ioctl,
	.fasync			= deser_fasync,
};

static void register_misc_devices(struct deser_hub_dev *hub)
{
	int ret;
	
	hub->deser_misc.minor = DESER_MISC_MINOR_BASE + hub->id;
	hub->deser_misc.name = hub->name;
	hub->deser_misc.fops = &deser_misc_fops;

	dev_err(hub->dev, "device_name : %s\n",hub->deser_misc.name);

	ret = misc_register(&hub->deser_misc);
	if (ret < 0) {
		dev_err(hub->dev, "%s() %d, deser misc register failed\n", __func__, __LINE__);
	}else {
		dev_err(hub->dev, "%s() %d, deser misc register Success!\n", __func__, __LINE__);
	}
}
#endif 
static int xs9922b_s_power(struct v4l2_subdev *sd, int enable)
{
	struct deser_hub_dev *hub =
		container_of(sd, struct deser_hub_dev, subdev);

	hub->deser_boot_flag = true;

	//register misc device
	#if 0 //for hotlog
	register_misc_devices(hub);
	#endif
	return 0;
}

static struct v4l2_subdev_video_ops xs9922b_video_ops = {
	.s_stream = xs9922b_s_stream,
};

static struct v4l2_subdev_core_ops xs9922b_core_ops = {
	.s_power = xs9922b_s_power,
};

static struct v4l2_subdev_ops n4_ops = {
	.core = &xs9922b_core_ops,
	.video = &xs9922b_video_ops,
};

static int deser_notify_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *sd,
			      struct v4l2_async_subdev *asd)
{
	struct camera_dev *cam_dev;
	struct deser_channel *deser_chn;

	cam_dev = container_of(sd, struct camera_dev, subdev);
	deser_chn = container_of(asd, struct deser_channel, async_dev);

	cam_dev->sd_state = BST_SUBDEV_STATE_BOUND;
	cam_dev->deser_parent = deser_chn->deser_dev;
	cam_dev->index_in_serdes = deser_chn->index;
	deser_chn->cam_dev = cam_dev;
	deser_chn->camera_bound = true;

	return 0;
}

static void deser_notify_unbind(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
}

static const struct v4l2_async_notifier_operations deser_async_ops = {
	.bound = deser_notify_bound,
	.unbind = deser_notify_unbind,
};

static int parse_device_dt(struct deser_hub_dev *hub, struct device_node *node)
{
	// struct device_node *node = hub->dev->of_node;
	const char *type_name = NULL;
#if 0 // for hotlog
	if (of_property_read_u32(node, "id", &hub->id)) {
		dev_err(hub->dev, " Failed to find <deserx/id>\n");
		return -EINVAL;
	}
	if (!of_property_read_string(node, "type", &type_name)) {
		if (type_name != NULL) {
			strncpy(hub->name, type_name, MAX_DESER_NAME_LEN);
			snprintf(hub->name, sizeof(hub->name), "deser%d", hub->id);
			hub->src_mask = 0x0f;
			hub->max_port = 4;
			hub->type = DESER_TYPE_XS9922B;
		}
	}
#endif 

	return 0;
}

static int parse_input_dt(struct deser_hub_dev *hub, struct device_node *node)
{
	int i;

	for (i = 0; i < 4 /* hardcode */; i++) {
		struct device_node *port;
		struct device_node *remote;

		port = of_graph_get_port_by_id(node, i);
		if (!port) {
			dev_err(hub->dev, "%s: input port%d not found\n ",
				__func__, i);
			break;
		}

		remote = of_graph_get_remote_node(node, i, 0);
		if (!remote) {
			dev_err(hub->dev, "%s: input device%d not found\n",
				__func__, i);
			break;
		}

		hub->chn[i].index = i;
		hub->chn[i].camera_node = remote;
		hub->chn[i].camera_fwnode = of_fwnode_handle(remote);
		hub->num_cameras++;
	}

	return 0;
}

static int parse_output_dt(struct deser_hub_dev *hub, struct device_node *node)
{
	struct device_node *csi2 = of_get_child_by_name(node, "csi-link");

	if (!csi2) {
		dev_err(hub->dev, "csi-link not found\n");
		return -EINVAL;
	}

	hub->subdev.fwnode = of_fwnode_handle(csi2);

	return 0;
}

static int parse_dt(struct deser_hub_dev *hub)
{
	struct device_node *node = hub->dev->of_node;

	if (!node)
		return -EINVAL;

	if (parse_device_dt(hub, node)) {
		dev_err(hub->dev, "parse device dt failed\n");
		return -1;
	}

	if (parse_input_dt(hub, node)) {
		dev_err(hub->dev, "parse input dt failed\n");
		return -1;
	}

	if (parse_output_dt(hub, node)) {
		dev_err(hub->dev, "parse output dt failed\n");
		return -1;
	}

	return 0;
}

static int register_subdev(struct deser_hub_dev *hub)
{
	int ret;
	struct v4l2_subdev *sd;

	sd = &hub->subdev;
	v4l2_subdev_init(sd, &n4_ops);
	v4l2_set_subdevdata(sd, hub);

	sd->dev = hub->dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "%s", dev_name(hub->dev));

	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err(hub->dev, "register subdev failed\n");
		return -1;
	}

	return 0;
}

static int register_subdev_notifier(struct deser_hub_dev *hub)
{
	int ret;
	int i;
	int index;

	if (!hub->num_cameras) {
		dev_err(hub->dev, "%s: no input device found\n", __func__);
		return -1;
	}

	v4l2_async_notifier_init(&hub->notifier);
	hub->notifier.ops = &deser_async_ops;

	index = 0;
	for (i = 0; i < 4 /* hardcode */; i++) {
		if (!hub->chn[i].camera_fwnode)
			continue;

		hub->chn[i].deser_dev = hub;
		hub->chn[i].async_dev.match_type = V4L2_ASYNC_MATCH_FWNODE;
		hub->chn[i].async_dev.match.fwnode = hub->chn[i].camera_fwnode;
		v4l2_async_notifier_add_subdev(&hub->notifier,
					       &(hub->chn[i].async_dev));
		index++;
	}

	ret = v4l2_async_subdev_notifier_register(&hub->subdev, &hub->notifier);
	if (ret) {
		dev_err(hub->dev, "%s: register subdev notifier failed\n",
			__func__);
		return -1;
	}

	return 0;
}

static int gpio_pinctrl(struct deser_hub_dev *hub, int gpio)
{
	int ret;

	ret = gpio_is_valid(gpio);

	if (!ret) {
		dev_err(hub->dev, "gpio%d invalid\n", gpio);
		return -1;
	}

	ret = devm_gpio_request(hub->dev, gpio, dev_name(hub->dev));
	if (ret) {
		dev_err(hub->dev, "gpio%d request failed\n", gpio);
		return -1;
	}

	ret = gpio_direction_output(gpio, 0);
	if (ret) {
		dev_err(hub->dev, "gpio%d output setting failed\n", gpio);
		return -1;
	}
	if (gpio == 20 || gpio == 19)
		usleep_range(100 * 1000, 150 * 1000);
	else
		usleep_range(10 * 1000, 15 * 1000);

	ret = gpio_direction_output(gpio, 1);

	if (ret) {
		dev_err(hub->dev, "gpio%d output setting failed\n", gpio);
		return -1;
	}

	if (gpio == 20 || gpio == 19)
		usleep_range(100 * 1000, 150 * 1000);
	else
		usleep_range(10 * 1000, 15 * 1000);

	return 0;
}

static int enable_dev(struct deser_hub_dev *hub, int mode)
{
	int gpio = 0;
	// enable vin vout
	if (mode == XS9922_RESET) {
		gpio = of_get_named_gpio(hub->dev->of_node, "pdb-gpios", 0);

		dev_err(hub->dev, "RL_DEBUG_gpio %d", gpio);

		if (gpio > 0)
			gpio_pinctrl(hub, gpio);

		gpio = of_get_named_gpio(hub->dev->of_node, "reset-gpio", 0);

		dev_err(hub->dev, "RL_DEBUG_gpio %d", gpio);

		gpio_pinctrl(hub, gpio);
	} else if (mode == CAMERA_POWERON) {

		gpio = of_get_named_gpio(hub->dev->of_node, "dms-gpio", 0);
		dev_err(hub->dev, "RL_DEBUG_gpio %d", gpio);

		gpio_pinctrl(hub, gpio);

		gpio = of_get_named_gpio(hub->dev->of_node, "power0-gpio", 0);
		dev_err(hub->dev, "RL_DEBUG_gpio %d", gpio);

		gpio_pinctrl(hub, gpio);

		gpio = of_get_named_gpio(hub->dev->of_node, "power1-gpio", 0);
		dev_err(hub->dev, "RL_DEBUG_gpio %d", gpio);

		gpio_pinctrl(hub, gpio);
	}
	return 0;
}

static int check_chip_id(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	u32 chip_id0 = 0;
	u32 chip_id1 = 0;

	xs9922_read_reg(client, NI_40F0_DEVICE_ID_1, XS9922_REG_VALUE_08BIT,
			&chip_id1);
	xs9922_read_reg(client, NI_40F1_DEVICE_ID_0, XS9922_REG_VALUE_08BIT,
			&chip_id0);
	dev_err(dev, "xs9922 chip_id: 0x%04x\n", chip_id1 << 8 | chip_id0);

	if ((chip_id1 != NI_ID1) || (chip_id0 != NI_ID0)) {
		dev_err(dev, "the id of the xs9922 don't match\n");
		dev_err(dev, "chip_id1 = %02x should be 0x99\n", chip_id1);
		dev_err(dev, "chip_id0 = %02x should be 0x22\n", chip_id0);
		return -1;
	}

	return 0;
}

static int xs9922b_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	struct deser_hub_dev *hub;
	u32 mode = 0;

	hub = devm_kzalloc(&client->dev, sizeof(struct deser_hub_dev),
			   GFP_KERNEL);
	if (!hub)
		return -ENOMEM;

	hub->i2c_client = client;
	hub->dev = &client->dev;
	hub->deser_boot_flag = false;

	ret = register_subdev(hub);
	if (ret) {
		dev_err(hub->dev, "%s: register subdev failed\n", __func__);
		return -EINVAL;
	}

	ret = parse_dt(hub);
	if (ret) {
		dev_err(hub->dev, "%s: parse dt failed\n", __func__);
		return -EINVAL;
	}
	dev_err(&client->dev, "RL_DEBUG!!!!");

	ret = register_subdev_notifier(hub);
	if (ret) {
		dev_err(hub->dev, "%s: register subdev notifier failed\n",
			__func__);
		return -EINVAL;
	}

	ret = enable_dev(hub, XS9922_RESET);
	if (ret) {
		dev_err(hub->dev, "%s: enable device failed\n", __func__);
		return -EINVAL;
	}

	ret = enable_dev(hub, CAMERA_POWERON);
	if (ret) {
		dev_err(hub->dev, "%s: enable camera failed\n", __func__);
		return -EINVAL;
	}
	xs9922b_global_config(hub);

	of_property_read_u32(hub->dev->of_node, "mode", &mode);
	xs9922b_switch_mode(hub, mode);

#if 0
	/*Save hub pointer for misc*/
	hubs[hub->id] = hub;
#endif
	return 0;
}

static int xs9922b_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id xs9922b_id[] = {
	{ MODULE_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, xs9922b_id);

static const struct of_device_id xs9922b_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, xs9922b_of_match);

static struct i2c_driver xs9922b_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(xs9922b_of_match),
	},
	.probe		= xs9922b_probe,
	.remove		= xs9922b_remove,
	.id_table	= xs9922b_id,
};

module_i2c_driver(xs9922b_driver);

MODULE_DESCRIPTION("GMSL driver for XS9922");
MODULE_LICENSE("GPL");
