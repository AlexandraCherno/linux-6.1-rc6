/*
 * drivers/media/platform/sunxi-tvd/tvd/tvd.h
 *
 * Copyright (c) 2007-2018 Allwinnertech Co., Ltd.
 * Author: zhengxiaobin <zhengxiaobin@allwinnertech.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SUN8I_TVD_H__
#define __SUN8I_TVD_H__

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include "bsp_tvd.h"

#define TVD_MAX 4

#define tvd_info(tvd, fmt, arg...) \
	do { dev_info(&tvd->top->pdev->dev, "%s: " fmt, __func__, ## arg); } while (0)

#define tvd_err(tvd, fmt, arg...) \
	do { dev_err(&tvd->top->pdev->dev, "%s: " fmt, __func__, ## arg); } while (0)

#define top_info(top, fmt, arg...) \
	do { dev_info(&top->pdev->dev, "%s: " fmt, __func__, ## arg); } while (0)

#define top_err(top, fmt, arg...) \
	do { dev_err(&top->pdev->dev, "%s: " fmt, __func__, ## arg); } while (0)

struct tvd_buf {
	struct vb2_v4l2_buffer vb2_v4l2;
	struct list_head list;
	dma_addr_t paddr_y;
	dma_addr_t paddr_c;
};

struct tvd_dev {
	struct top_dev *top;
	struct video_device vfd;
	struct mutex mlock;
	struct vb2_queue queue;
	struct list_head buf_list;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_pix_format format;
	struct tvd_buf *buf;
	v4l2_std_id std;
	spinlock_t slock;
	unsigned int sequence;
	unsigned int fps;
	unsigned int id;
	char name[16];
	int irq;

	void __iomem *regs;
	struct clk *clk;
	struct clk *clk_bus;
	struct reset_control *rst_bus;
};

struct top_dev {
	struct platform_device *pdev;
	struct v4l2_device v4l2_dev;
	struct tvd_dev tvd[TVD_MAX];
	unsigned int tvd_max;

	void __iomem *regs;
	struct clk *clk_ram;
	struct clk *clk_bus;
	struct reset_control *rst_bus;
	struct regulator *vcc;
};

#endif /* __SUN8I_TVD_H__ */
