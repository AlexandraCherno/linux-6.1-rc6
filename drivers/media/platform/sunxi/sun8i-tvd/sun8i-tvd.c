// SPDX-License-Identifier: GPL-2.0
/*
 * Allwinner sun8i TV decoder driver
 *
 * Copyright (C) 2022 starterkit.ru <info@starterkit.ru>
 *
 */

#include "sun8i-tvd.h"

static int tvd_start(struct tvd_dev *tvd)
{
	tvd_init(tvd->id, 0);
	tvd_config(tvd->id, 0, (tvd->std & V4L2_STD_625_50) ? 1 : 0);
	tvd_set_wb_width(tvd->id, tvd->format.width);
	tvd_set_wb_width_jump(tvd->id, tvd->format.width);
	tvd_set_wb_height(tvd->id, tvd->format.height / 2);
	tvd_set_wb_fmt(tvd->id, TVD_PL_YUV420);
	tvd_set_wb_uv_swap(tvd->id, 0);
	tvd_set_wb_addr(tvd->id, tvd->buf->paddr_y, tvd->buf->paddr_c);
	tvd_irq_status_clear(tvd->id, TVD_IRQ_FRAME_END);
	tvd_irq_enable(tvd->id, TVD_IRQ_FRAME_END);
	tvd_capture_on(tvd->id);

	return 0;
}

static int tvd_stop(struct tvd_dev *tvd)
{
	tvd_capture_off(tvd->id);
	tvd_irq_disable(tvd->id, TVD_IRQ_FRAME_END);
	tvd_irq_status_clear(tvd->id, TVD_IRQ_FRAME_END);
	tvd_deinit(tvd->id, 0);

	return 0;
}

static int top_clk_disable(struct top_dev *top)
{
	clk_disable_unprepare(top->clk_bus);
	reset_control_assert(top->rst_bus);
	clk_disable_unprepare(top->clk_ram);

	return 0;
}

static int top_clk_enable(struct top_dev *top)
{
	int ret;

	ret = reset_control_deassert(top->rst_bus);
	if (ret)
		return ret;

	ret = clk_prepare_enable(top->clk_bus);
	if (ret) {
		reset_control_assert(top->rst_bus);
		return ret;
	}

	ret = clk_prepare_enable(top->clk_ram);
	if (ret) {
		clk_disable_unprepare(top->clk_bus);
		reset_control_assert(top->rst_bus);
		return ret;
	}

	return 0;
}

static int tvd_clk_disable(struct tvd_dev *tvd)
{
	clk_disable_unprepare(tvd->clk);
	clk_disable_unprepare(tvd->clk_bus);
	reset_control_assert(tvd->rst_bus);

	return 0;
}

static int tvd_clk_enable(struct tvd_dev *tvd)
{
	int ret;

	ret = clk_set_rate(tvd->clk, 27000000);
	if (ret)
		return ret;

	ret = reset_control_deassert(tvd->rst_bus);
	if (ret)
		return ret;

	ret = clk_prepare_enable(tvd->clk_bus);
	if (ret) {
		reset_control_assert(tvd->rst_bus);
		return ret;
	}

	ret = clk_prepare_enable(tvd->clk);
	if (ret) {
		clk_disable_unprepare(tvd->clk_bus);
		reset_control_assert(tvd->rst_bus);
		return ret;
	}

	return 0;
}

static int tvd_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tvd_dev *tvd =
		container_of(ctrl->handler, struct tvd_dev, hdl);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		tvd_set_luma(tvd->id, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		tvd_set_contrast(tvd->id, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		tvd_set_saturation(tvd->id, ctrl->val);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const struct v4l2_ctrl_ops tvd_ctrl_ops = {
	.s_ctrl = tvd_s_ctrl,
};

static void tvd_fill_pix_format(struct tvd_dev *tvd,
		struct v4l2_pix_format *pix)
{
	pix->width = 720;
	pix->height = (tvd->std & V4L2_STD_625_50) ? 576 : 480;
	pix->pixelformat = V4L2_PIX_FMT_NV12;
	pix->field = V4L2_FIELD_INTERLACED;
	pix->bytesperline = pix->width;
	pix->sizeimage = pix->bytesperline * pix->height * 3 / 2;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
	pix->priv = 0;
}

/**
 * Videobuf2 Operations
 */
static int queue_setup(struct vb2_queue *vq,
		unsigned int *nbuffers, unsigned int *nplanes,
		unsigned int sizes[], struct device *alloc_devs[])
{
	struct tvd_dev *tvd = vb2_get_drv_priv(vq);

	if (vq->num_buffers + *nbuffers < 5)
		*nbuffers = 5 - vq->num_buffers;

	if (*nplanes)
		return sizes[0] < tvd->format.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = tvd->format.sizeimage;

	return 0;
}

static int buffer_init(struct vb2_buffer *vb)
{
	struct tvd_dev *tvd = vb2_get_drv_priv(vb->vb2_queue);
	struct tvd_buf *buf = container_of(to_vb2_v4l2_buffer(vb),
			struct tvd_buf, vb2_v4l2);

	buf->paddr_y = vb2_dma_contig_plane_dma_addr(vb, 0);
	buf->paddr_c = buf->paddr_y + tvd->format.width * tvd->format.height;
	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
	struct tvd_dev *tvd = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = tvd->format.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		tvd_err(tvd, "buffer too small (%lu < %lu)\n",
			vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct tvd_dev *tvd = vb2_get_drv_priv(vb->vb2_queue);
	struct tvd_buf *buf = container_of(to_vb2_v4l2_buffer(vb),
			struct tvd_buf, vb2_v4l2);
	unsigned long flags;

	spin_lock_irqsave(&tvd->slock, flags);
	list_add_tail(&buf->list, &tvd->buf_list);
	spin_unlock_irqrestore(&tvd->slock, flags);
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct tvd_dev *tvd = vb2_get_drv_priv(vq);
	unsigned long flags;

	if (count < 5)
		return -ENOBUFS;

	spin_lock_irqsave(&tvd->slock, flags);
	tvd->buf = list_first_entry(&tvd->buf_list,
			struct tvd_buf, list);
	list_del(&tvd->buf->list);
	spin_unlock_irqrestore(&tvd->slock, flags);

	tvd->sequence = 0;
	tvd_clk_enable(tvd);
	tvd_start(tvd);

	return 0;
}

static void stop_streaming(struct vb2_queue *vq)
{
	struct tvd_dev *tvd = vb2_get_drv_priv(vq);
	struct tvd_buf *buf, *node;
	unsigned long flags;

	tvd_stop(tvd);
	tvd_clk_disable(tvd);

	spin_lock_irqsave(&tvd->slock, flags);
	if (tvd->buf) {
		vb2_buffer_done(&tvd->buf->vb2_v4l2.vb2_buf, VB2_BUF_STATE_ERROR);
		tvd->buf = NULL;
	}

	list_for_each_entry_safe(buf, node, &tvd->buf_list, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb2_v4l2.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&tvd->slock, flags);
}

static struct vb2_ops tvd_qops = {
	.queue_setup		= queue_setup,
	.buf_init		= buffer_init,
	.buf_prepare		= buffer_prepare,
	.buf_queue		= buffer_queue,
	.start_streaming	= start_streaming,
	.stop_streaming		= stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static const struct v4l2_file_operations tvd_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

/**
 * Video ioctls
 */
static int tvd_querycap(struct file *file, void *priv,
		struct v4l2_capability *cap)
{
	struct tvd_dev *tvd = video_drvdata(file);

	strlcpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	snprintf(cap->card, sizeof(cap->card), "tvd%u", tvd->id);
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:tvd%u", tvd->id);
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int tvd_try_fmt_vid_cap(struct file *file, void *priv,
		struct v4l2_format *f)
{
	struct tvd_dev *tvd = video_drvdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (pix->pixelformat != V4L2_PIX_FMT_NV12)
		return -EINVAL;

	tvd_fill_pix_format(tvd, pix);

	return 0;
}

static int tvd_s_fmt_vid_cap(struct file *file, void *priv,
		struct v4l2_format *f)
{
	struct tvd_dev *tvd = video_drvdata(file);

	if (tvd_try_fmt_vid_cap(file, priv, f) < 0)
		return -EINVAL;

	if (vb2_is_busy(&tvd->queue))
		return -EBUSY;

	tvd->format = f->fmt.pix;

	return 0;
}

static int tvd_g_fmt_vid_cap(struct file *file, void *priv,
		struct v4l2_format *f)
{
	struct tvd_dev *tvd = video_drvdata(file);

	f->fmt.pix = tvd->format;

	return 0;
}

static int tvd_enum_fmt_vid_cap(struct file *file, void *priv,
		struct v4l2_fmtdesc *f)
{
	if (f->index != 0)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_NV12;
	f->flags = 0;

	return 0;
}

static int tvd_querystd(struct file *file, void *priv, v4l2_std_id *std)
{
	struct tvd_dev *tvd = video_drvdata(file);
	u32 locked, system;

	if (vb2_is_streaming(&tvd->queue))
		return -EBUSY;

	tvd_clk_enable(tvd);
	tvd_init(tvd->id, 0);

	msleep(200);

	tvd_get_status(tvd->id, &locked, &system);
	tvd_info(tvd, "tvd%d locked:%d system:%d\n", tvd->id, locked, system);

	tvd_deinit(tvd->id, 0);
	tvd_clk_disable(tvd);

	if (locked)
		*std &= system ? V4L2_STD_PAL : V4L2_STD_NTSC;
	else
		*std = V4L2_STD_UNKNOWN;

	return 0;
}

static int tvd_s_std(struct file *file, void *priv, v4l2_std_id std)
{
	struct tvd_dev *tvd = video_drvdata(file);

	if (std == tvd->std)
		return 0;

	if (vb2_is_busy(&tvd->queue))
		return -EBUSY;

	tvd->std = std;
	tvd->fps = (std & V4L2_STD_625_50) ? 25 : 30;
	tvd_fill_pix_format(tvd, &tvd->format);

	return 0;
}

static int tvd_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct tvd_dev *tvd = video_drvdata(file);
	v4l2_std_id new_std = V4L2_STD_ALL;

	tvd_querystd(file, priv, &new_std);
	if (new_std && new_std != V4L2_STD_ALL)
		tvd_s_std(file, priv, new_std);

	*std = tvd->std;

	return 0;
}

static int tvd_enum_input(struct file *file, void *priv,
		struct v4l2_input *i)
{
	if (i->index != 0)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;
	i->std = V4L2_STD_ALL;
	i->capabilities = V4L2_IN_CAP_STD;
	snprintf(i->name, sizeof(i->name), "Composite%d", i->index);

	return 0;
}

static int tvd_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i != 0)
		return -EINVAL;

	return 0;
}

static int tvd_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;

	return 0;
}

static void tvd_frame_period(struct tvd_dev *tvd,
		struct v4l2_fract *frameperiod)
{
	if (tvd->std & V4L2_STD_625_50) {
		frameperiod->numerator = 1;
		frameperiod->denominator = 25;
	} else {
		frameperiod->numerator = 1001;
		frameperiod->denominator = 30000;
	}
}

static int tvd_g_parm(struct file *file, void *priv,
		struct v4l2_streamparm *sp)
{
	struct tvd_dev *tvd = video_drvdata(file);
	struct v4l2_captureparm *cp = &sp->parm.capture;

	if (sp->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(*cp));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	tvd_frame_period(tvd, &cp->timeperframe);

	return 0;
}

static int tvd_s_parm(struct file *file, void *priv,
		struct v4l2_streamparm *sp)
{
	struct tvd_dev *tvd = video_drvdata(file);

	if (sp->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (vb2_is_streaming(&tvd->queue))
		return -EBUSY;

	return tvd_g_parm(file, priv, sp);
}

static int tvd_enum_framesizes(struct file *file, void *priv,
		struct v4l2_frmsizeenum *fsize)
{
	struct tvd_dev *tvd = video_drvdata(file);
	unsigned int height = (tvd->std & V4L2_STD_625_50) ? 576 : 480;

	if (fsize->index != 0 || fsize->pixel_format != V4L2_PIX_FMT_NV12)
		return -EINVAL;

	fsize->discrete.width = 720;
	fsize->discrete.height = height;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	return 0;
}

static int tvd_enum_frameintervals(struct file *file, void *priv,
		struct v4l2_frmivalenum *fival)
{
	struct tvd_dev *tvd = video_drvdata(file);

	if (fival->index != 0 || fival->pixel_format != V4L2_PIX_FMT_NV12)
		return -EINVAL;

	tvd_frame_period(tvd, &fival->discrete);
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;

	return 0;
}

static const struct v4l2_ioctl_ops tvd_ioctl_ops = {
	.vidioc_querycap = tvd_querycap,
	.vidioc_try_fmt_vid_cap = tvd_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = tvd_s_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = tvd_g_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = tvd_enum_fmt_vid_cap,

	.vidioc_querystd = tvd_querystd,
	.vidioc_s_std = tvd_s_std,
	.vidioc_g_std = tvd_g_std,

	.vidioc_enum_input = tvd_enum_input,
	.vidioc_s_input = tvd_s_input,
	.vidioc_g_input = tvd_g_input,

	.vidioc_g_parm = tvd_g_parm,
	.vidioc_s_parm = tvd_s_parm,

	.vidioc_enum_framesizes = tvd_enum_framesizes,
	.vidioc_enum_frameintervals = tvd_enum_frameintervals,

	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_log_status = v4l2_ctrl_log_status,
};

static int tvd_video_register(struct tvd_dev *tvd)
{
	struct v4l2_ctrl_handler *hdl = &tvd->hdl;
	struct vb2_queue *q = &tvd->queue;
	struct video_device *vfd = &tvd->vfd;
	int ret;

	/* Add the controls */
	v4l2_ctrl_handler_init(hdl, 4);
	v4l2_ctrl_new_std(hdl, &tvd_ctrl_ops, V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(hdl, &tvd_ctrl_ops, V4L2_CID_CONTRAST, 0, 128, 1, 0);
	v4l2_ctrl_new_std(hdl, &tvd_ctrl_ops, V4L2_CID_SATURATION, -4, 4, 1, 0);
	if (hdl->error)
		return hdl->error;

	/* Default settings */
	tvd->std = V4L2_STD_PAL;
	tvd->fps = (tvd->std & V4L2_STD_625_50) ? 25 : 30;
	tvd_fill_pix_format(tvd, &tvd->format);
	mutex_init(&tvd->mlock);
	spin_lock_init(&tvd->slock);
	INIT_LIST_HEAD(&tvd->buf_list);

	/* Initialize the vb2 queue */
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = tvd;
	q->buf_struct_size = sizeof(struct tvd_buf);
	q->ops = &tvd_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->min_buffers_needed = 3;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->gfp_flags = GFP_DMA32;
	q->dev = &tvd->top->pdev->dev;
	q->lock = &tvd->mlock;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_queue;

	/* Initialize the video_device structure */
	strlcpy(vfd->name, tvd->name, sizeof(vfd->name));
	vfd->release = video_device_release_empty;
	vfd->fops = &tvd_fops,
	vfd->ioctl_ops = &tvd_ioctl_ops,
	vfd->lock = &tvd->mlock;
	vfd->queue = q;
	vfd->v4l2_dev = &tvd->top->v4l2_dev;
	vfd->ctrl_handler = hdl;
	vfd->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	vfd->tvnorms = V4L2_STD_ALL;
	video_set_drvdata(vfd, tvd);
	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (!ret)
		return 0;

err_queue:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static irqreturn_t tvd_irq_handler(int irq, void *dev_id)
{
	struct tvd_dev *tvd = dev_id;
	struct tvd_buf *done = NULL;
	struct tvd_buf *next = NULL;
	u32 irq_status = 0;
	u32 err = (1 << TVD_IRQ_FIFO_C_O) | (1 << TVD_IRQ_FIFO_Y_O) |
		  (1 << TVD_IRQ_FIFO_C_U) | (1 << TVD_IRQ_FIFO_Y_U) |
		  (1 << TVD_IRQ_WB_ADDR_CHANGE_ERR);
	unsigned long flags;

	tvd_dma_irq_status_get(tvd->id, &irq_status);
	if ((irq_status & err) != 0)
		tvd_dma_irq_status_clear_err_flag(tvd->id, err);

	spin_lock_irqsave(&tvd->slock, flags);
	if (!list_empty(&tvd->buf_list)) {
		next = list_first_entry(&tvd->buf_list, struct tvd_buf, list);
		list_del(&next->list);
		done = tvd->buf;
		tvd->buf = next;
	}
	spin_unlock_irqrestore(&tvd->slock, flags);

	if (done && next) {
		tvd_set_wb_addr(tvd->id, next->paddr_y, next->paddr_c);
		done->vb2_v4l2.vb2_buf.timestamp = ktime_get_ns();
		done->vb2_v4l2.sequence = tvd->sequence++;
		done->vb2_v4l2.field = V4L2_FIELD_INTERLACED;
		vb2_buffer_done(&done->vb2_v4l2.vb2_buf, VB2_BUF_STATE_DONE);
	} else {
		tvd_err(tvd, "tvd%u NOBUF seq=%u\n", tvd->id, tvd->sequence);
	}

	tvd_irq_status_clear(tvd->id, TVD_IRQ_FRAME_END);

	return IRQ_HANDLED;
}

static int tvd_dev_init(struct tvd_dev *tvd)
{
	struct platform_device *pdev = tvd->top->pdev;
	char name[32];
	int ret;

	snprintf(name, sizeof(name), "irq_tvd%d", tvd->id);
	tvd->irq = platform_get_irq_byname(pdev, name);
	if (tvd->irq < 0)
		return tvd->irq;

	snprintf(tvd->name, sizeof(tvd->name), "tvd%d", tvd->id);
	ret = devm_request_irq(&pdev->dev, tvd->irq, tvd_irq_handler,
			       0, tvd->name, tvd);
	if (ret)
		return ret;

	snprintf(name, sizeof(name), "regs_tvd%d", tvd->id);
	tvd->regs = devm_platform_ioremap_resource_byname(pdev, name);
	if (IS_ERR(tvd->regs))
		return PTR_ERR(tvd->regs);

	tvd_set_reg_base(tvd->id, (unsigned long)tvd->regs);

	snprintf(name, sizeof(name), "clk_tvd%d", tvd->id);
	tvd->clk = devm_clk_get(&pdev->dev, name);
	if (IS_ERR(tvd->clk))
		return PTR_ERR(tvd->clk);

	snprintf(name, sizeof(name), "clk_bus_tvd%d", tvd->id);
	tvd->clk_bus = devm_clk_get(&pdev->dev, name);
	if (IS_ERR(tvd->clk_bus))
		return PTR_ERR(tvd->clk_bus);

	snprintf(name, sizeof(name), "rst_bus_tvd%d", tvd->id);
	tvd->rst_bus = devm_reset_control_get(&pdev->dev, name);
	if (IS_ERR(tvd->rst_bus))
		return PTR_ERR(tvd->rst_bus);

	ret = tvd_video_register(tvd);
	if (ret)
		return ret;

	return 0;
}

static int tvd_probe(struct platform_device *pdev)
{
	struct top_dev *top;
	int i, ret;

	top = devm_kzalloc(&pdev->dev, sizeof(*top), GFP_KERNEL);
	if (!top)
		return -ENOMEM;

	top->pdev = pdev;

	top->regs = devm_platform_ioremap_resource_byname(pdev, "regs_top");
	if (IS_ERR(top->regs))
		return PTR_ERR(top->regs);

	tvd_top_set_reg_base((unsigned long)top->regs);

	top->clk_ram = devm_clk_get(&pdev->dev, "clk_dram_tvd");
	if (IS_ERR(top->clk_ram))
		return PTR_ERR(top->clk_ram);

	top->clk_bus = devm_clk_get(&pdev->dev, "clk_bus_tvd_top");
	if (IS_ERR(top->clk_bus))
		return PTR_ERR(top->clk_bus);

	top->rst_bus = devm_reset_control_get(&pdev->dev, "rst_bus_tvd_top");
	if (IS_ERR(top->rst_bus))
		return PTR_ERR(top->rst_bus);

	top->vcc = devm_regulator_get(&pdev->dev, "tvd");
	if (IS_ERR(top->vcc))
		return PTR_ERR(top->vcc);

	ret = v4l2_device_register(&pdev->dev, &top->v4l2_dev);
	if (ret)
		return ret;

	top->tvd_max = TVD_MAX;

	for (i = 0; i < TVD_MAX; i++) {
		struct tvd_dev *tvd = &top->tvd[i];

		tvd->top = top;
		tvd->id = i;
		ret = tvd_dev_init(tvd);
		if (ret) {
			dev_err(&pdev->dev, "Failed to init tvd%d\n", i);
			top->tvd_max = i;
			break;
		}
	}

	if (!top->tvd_max) {
		v4l2_device_unregister(&top->v4l2_dev);
		return -ENODEV;
	}

	ret = regulator_enable(top->vcc);
	if (ret)
		return ret;

	top_clk_enable(top);

	platform_set_drvdata(pdev, top);
	pm_runtime_enable(&pdev->dev);

	return 0;
}

static int tvd_remove(struct platform_device *pdev)
{
	struct top_dev *top = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < top->tvd_max; i++) {
		struct tvd_dev *tvd = &top->tvd[i];

		tvd_stop(tvd);
		tvd_clk_disable(tvd);
		video_unregister_device(&tvd->vfd);
		v4l2_ctrl_handler_free(&tvd->hdl);
	}

	v4l2_device_unregister(&top->v4l2_dev);
	top_clk_disable(top);
	regulator_disable(top->vcc);
	pm_runtime_force_suspend(&pdev->dev);

	return 0;
}

static int tvd_resume(struct device *device)
{
	struct top_dev *top = dev_get_drvdata(device);
	unsigned int i;

	top_clk_enable(top);

	for (i = 0; i < top->tvd_max; i++) {
		struct tvd_dev *tvd = &top->tvd[i];

		if (vb2_is_streaming(&tvd->queue)) {
			u32 locked, system;

			tvd_clk_enable(tvd);
			tvd_init(tvd->id, 0);

			msleep(200);

			tvd_get_status(tvd->id, &locked, &system);
			tvd_info(tvd, "tvd%d locked:%d system:%d\n", tvd->id, locked, system);

			tvd_deinit(tvd->id, 0);
			tvd_clk_disable(tvd);

			tvd_clk_enable(tvd);
			tvd_start(tvd);
		}
	}

	return 0;
}

static int tvd_suspend(struct device *device)
{
	struct top_dev *top = dev_get_drvdata(device);
	unsigned int i;

	for (i = 0; i < top->tvd_max; i++) {
		struct tvd_dev *tvd = &top->tvd[i];

		if (vb2_is_streaming(&tvd->queue)) {
			tvd_stop(tvd);
			tvd_clk_disable(tvd);
		}
	}

	top_clk_disable(top);

	return 0;
}

static const struct of_device_id tvd_dt_match[] = {
	{ .compatible = "allwinner,sun8i-tvd" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tvd_dt_match);

static const struct dev_pm_ops tvd_pm_ops = {
	.resume		= tvd_resume,
	.suspend	= tvd_suspend,
};

static struct platform_driver tvd_driver = {
	.probe		= tvd_probe,
	.remove		= tvd_remove,
	.driver		= {
		.name		= KBUILD_MODNAME,
		.of_match_table	= tvd_dt_match,
		.pm		= &tvd_pm_ops,
	},
};
module_platform_driver(tvd_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("starterkit.ru <info@starterkit.ru>");
MODULE_DESCRIPTION("Allwinner tvd driver");
