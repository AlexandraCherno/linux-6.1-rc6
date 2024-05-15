// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus VPU driver
 *
 * Copyright (C) 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright (C) 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright (C) 2018 Bootlin
 * Copyright (C) 2023 starterkit.ru <info@starterkit.ru>
 */

#include <media/videobuf2-dma-contig.h>

#include "cedrus.h"
#include "cedrus_hw.h"
#include "cedrus_regs.h"
#include "cedrus_jpeg_header.h"

#define VE_ISP_COLOR_FORMAT_NV12    0
#define VE_AVC_TRIGGER_JPEG         (1 << 16)

static void cedrus_put_bits(struct cedrus_dev *dev, u32 nbits, u32 data)
{
	u32 trigger = VE_AVC_TRIGGER_JPEG | ((nbits & 0x3f) << 8) | 0x1;
	cedrus_write(dev, VE_AVC_BASIC_BITS, data);
	cedrus_write(dev, VE_AVC_TRIGGER, trigger);
}

static enum cedrus_irq_status cedrus_jpeg_irq_status(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;
	u32 status, length;
	u8 *eoi;

	status = cedrus_read(dev, VE_AVC_STATUS) & 0xf;
	if (!status)
		return CEDRUS_IRQ_NONE;

	if (status == 0x2)
		return CEDRUS_IRQ_ERROR;

	length = cedrus_read(dev, VE_AVC_VLE_LENGTH) / 8;

	eoi = vb2_plane_vaddr(ctx->vb, 0) + length;
	eoi[0] = 0xff;
	eoi[1] = 0xd9;

	vb2_set_plane_payload(ctx->vb, 0, length + 2);

	return CEDRUS_IRQ_OK;
}

static void cedrus_jpeg_irq_clear(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;

	cedrus_write(dev, VE_AVC_STATUS, 0xf);
}

static void cedrus_jpeg_irq_disable(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;

	u32 reg = cedrus_read(dev, VE_AVC_CTRL);
	cedrus_write(dev, VE_AVC_CTRL, reg & ~0xf);
}

static void cedrus_set_jpeg_quantization(struct cedrus_dev *dev,
		struct cedrus_jpeg_ctx *jpeg_ctx)
{
	unsigned char *table_y = jpeg_ctx->hw_luma_qtable;
	unsigned char *table_c = jpeg_ctx->hw_chroma_qtable;
	u32 reg, i;

	cedrus_write(dev, VE_AVC_SRAM_INDEX, 0);

	for(i = 0; i < JPEG_QUANT_SIZE; i++) {
		reg = 0x0000ffff & (0xffff / table_y[i]);
		reg |= 0x00ff0000 & (((table_y[i] + 1) / 2) << 16);
		cedrus_write(dev, VE_AVC_SRAM_DATA, reg);
	}

	for(i = 0; i < JPEG_QUANT_SIZE; i++) {
		reg = 0x0000ffff & (0xffff / table_c[i]);
		reg |= 0x00ff0000 & (((table_c[i] + 1) / 2) << 16);
		cedrus_write(dev, VE_AVC_SRAM_DATA, reg);
	}
}

static int cedrus_jpeg_setup(struct cedrus_ctx *ctx, struct cedrus_run *run)
{
	struct cedrus_dev *dev = ctx->dev;
	dma_addr_t addr_y, addr_c, addr_j;
	u32 i, reg, width, height, size, stride, maxbits, jpeg_max;
	struct cedrus_jpeg_ctx jpeg_ctx;
	u8 header[JPEG_HEADER_SIZE];

	/* Activate JPEG engine. */
	cedrus_engine_enable(ctx);

	cedrus_write(dev, VE_RESET, 0x1);
	cedrus_write(dev, VE_RESET, 0x0);

	addr_y = vb2_dma_contig_plane_dma_addr(&run->src->vb2_buf, 0);
	cedrus_write(dev, VE_ISP_INPUT_LUMA, addr_y);

	addr_c = addr_y + ctx->src_fmt.bytesperline * ctx->src_fmt.height;
	cedrus_write(dev, VE_ISP_INPUT_CHROMA, addr_c);

	width = (ctx->src_fmt.width + 15) / 16;
	height = (ctx->src_fmt.height + 15) / 16;
	size = ((width & 0x3ff) << 16) | (height & 0x3ff);
	stride = (width & 0x3ff) << 16;

	cedrus_write(dev, VE_ISP_INPUT_SIZE, size);
	cedrus_write(dev, VE_ISP_INPUT_STRIDE, stride);

	reg = (VE_ISP_COLOR_FORMAT_NV12 & 0xf) << 28; // << 29 ??
	cedrus_write(dev, VE_ISP_CTRL, reg);

	ctx->vb = &run->dst->vb2_buf;

	addr_j = vb2_dma_contig_plane_dma_addr(ctx->vb, 0);
	cedrus_write(dev, VE_AVC_VLE_ADDR, addr_j);
	cedrus_write(dev, VE_AVC_VLE_END, addr_j + vb2_plane_size(ctx->vb, 0) - 1);

	jpeg_max = vb2_plane_size(ctx->vb, 0) - JPEG_HEADER_SIZE;
	maxbits = (jpeg_max * 8 + 0xffff) & ~0xffff;
	maxbits = maxbits > 0x0fff0000 ? 0x0fff0000 : maxbits;

	cedrus_write(dev, VE_AVC_VLE_OFFSET, 0);
	cedrus_write(dev, VE_AVC_VLE_MAX, maxbits);

	cedrus_write(dev, VE_AVC_CTRL, 0xf); // enable irq
	cedrus_write(dev, VE_AVC_TRIGGER, VE_AVC_TRIGGER_JPEG);

	reg = cedrus_read(dev, VE_AVC_STATUS);
	cedrus_write(dev, VE_AVC_STATUS, reg | 0xf);

	memset(&jpeg_ctx, 0, sizeof(jpeg_ctx));
	jpeg_ctx.buffer = header;
	jpeg_ctx.width = ctx->src_fmt.width;
	jpeg_ctx.height = ctx->src_fmt.height;
	jpeg_ctx.quality = ctx->jpeg_quality;

	cedrus_jpeg_header_assemble(&jpeg_ctx);

	cedrus_set_jpeg_quantization(dev, &jpeg_ctx);

	cedrus_write(dev, VE_AVC_PARAM, 1 << 31);

	for (i = 0; i < JPEG_HEADER_SIZE; i++)
		cedrus_put_bits(dev, 8, header[i]);

	reg = (1 << 31) | (1 << 30) |
		(((0x400 / jpeg_ctx.hw_chroma_qtable[0]) & 0x7ff) << 16) |
		(((0x400 / jpeg_ctx.hw_luma_qtable[0]) & 0x7ff) << 0);
	cedrus_write(dev, VE_AVC_PARAM, reg);

	return 0;
}

static void cedrus_jpeg_trigger(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;
	/* 0x8 - launch encoding */
	cedrus_write(dev, VE_AVC_TRIGGER, VE_AVC_TRIGGER_JPEG | 0x8);
}

struct cedrus_dec_ops cedrus_enc_ops_jpeg = {
	.irq_clear	= cedrus_jpeg_irq_clear,
	.irq_disable	= cedrus_jpeg_irq_disable,
	.irq_status	= cedrus_jpeg_irq_status,
	.setup		= cedrus_jpeg_setup,
	.trigger	= cedrus_jpeg_trigger,
};
