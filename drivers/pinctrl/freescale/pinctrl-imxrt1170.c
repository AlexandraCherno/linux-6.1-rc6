// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022
 * Author(s): Jesse Taube <Mr.Bossman075@gmail.com>
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/platform_device.h>

#include "pinctrl-imx.h"

enum imxrt1170_pads {
	IMXRT1170_PAD_RESERVE0,
	IMXRT1170_PAD_RESERVE1,
	IMXRT1170_PAD_RESERVE2,
	IMXRT1170_PAD_RESERVE3,
	IMXRT1170_PAD_EMC_B1_00,
	IMXRT1170_PAD_EMC_B1_01,
	IMXRT1170_PAD_EMC_B1_02,
	IMXRT1170_PAD_EMC_B1_03,
	IMXRT1170_PAD_EMC_B1_04,
	IMXRT1170_PAD_EMC_B1_05,
	IMXRT1170_PAD_EMC_B1_06,
	IMXRT1170_PAD_EMC_B1_07,
	IMXRT1170_PAD_EMC_B1_08,
	IMXRT1170_PAD_EMC_B1_09,
	IMXRT1170_PAD_EMC_B1_10,
	IMXRT1170_PAD_EMC_B1_11,
	IMXRT1170_PAD_EMC_B1_12,
	IMXRT1170_PAD_EMC_B1_13,
	IMXRT1170_PAD_EMC_B1_14,
	IMXRT1170_PAD_EMC_B1_15,
	IMXRT1170_PAD_EMC_B1_16,
	IMXRT1170_PAD_EMC_B1_17,
	IMXRT1170_PAD_EMC_B1_18,
	IMXRT1170_PAD_EMC_B1_19,
	IMXRT1170_PAD_EMC_B1_20,
	IMXRT1170_PAD_EMC_B1_21,
	IMXRT1170_PAD_EMC_B1_22,
	IMXRT1170_PAD_EMC_B1_23,
	IMXRT1170_PAD_EMC_B1_24,
	IMXRT1170_PAD_EMC_B1_25,
	IMXRT1170_PAD_EMC_B1_26,
	IMXRT1170_PAD_EMC_B1_27,
	IMXRT1170_PAD_EMC_B1_28,
	IMXRT1170_PAD_EMC_B1_29,
	IMXRT1170_PAD_EMC_B1_30,
	IMXRT1170_PAD_EMC_B1_31,
	IMXRT1170_PAD_EMC_B1_32,
	IMXRT1170_PAD_EMC_B1_33,
	IMXRT1170_PAD_EMC_B1_34,
	IMXRT1170_PAD_EMC_B1_35,
	IMXRT1170_PAD_EMC_B1_36,
	IMXRT1170_PAD_EMC_B1_37,
	IMXRT1170_PAD_EMC_B1_38,
	IMXRT1170_PAD_EMC_B1_39,
	IMXRT1170_PAD_EMC_B1_40,
	IMXRT1170_PAD_EMC_B1_41,
	IMXRT1170_PAD_EMC_B2_00,
	IMXRT1170_PAD_EMC_B2_01,
	IMXRT1170_PAD_EMC_B2_02,
	IMXRT1170_PAD_EMC_B2_03,
	IMXRT1170_PAD_EMC_B2_04,
	IMXRT1170_PAD_EMC_B2_05,
	IMXRT1170_PAD_EMC_B2_06,
	IMXRT1170_PAD_EMC_B2_07,
	IMXRT1170_PAD_EMC_B2_08,
	IMXRT1170_PAD_EMC_B2_09,
	IMXRT1170_PAD_EMC_B2_10,
	IMXRT1170_PAD_EMC_B2_11,
	IMXRT1170_PAD_EMC_B2_12,
	IMXRT1170_PAD_EMC_B2_13,
	IMXRT1170_PAD_EMC_B2_14,
	IMXRT1170_PAD_EMC_B2_15,
	IMXRT1170_PAD_EMC_B2_16,
	IMXRT1170_PAD_EMC_B2_17,
	IMXRT1170_PAD_EMC_B2_18,
	IMXRT1170_PAD_EMC_B2_19,
	IMXRT1170_PAD_EMC_B2_20,
	IMXRT1170_PAD_AD_00,
	IMXRT1170_PAD_AD_01,
	IMXRT1170_PAD_AD_02,
	IMXRT1170_PAD_AD_03,
	IMXRT1170_PAD_AD_04,
	IMXRT1170_PAD_AD_05,
	IMXRT1170_PAD_AD_06,
	IMXRT1170_PAD_AD_07,
	IMXRT1170_PAD_AD_08,
	IMXRT1170_PAD_AD_09,
	IMXRT1170_PAD_AD_10,
	IMXRT1170_PAD_AD_11,
	IMXRT1170_PAD_AD_12,
	IMXRT1170_PAD_AD_13,
	IMXRT1170_PAD_AD_14,
	IMXRT1170_PAD_AD_15,
	IMXRT1170_PAD_AD_16,
	IMXRT1170_PAD_AD_17,
	IMXRT1170_PAD_AD_18,
	IMXRT1170_PAD_AD_19,
	IMXRT1170_PAD_AD_20,
	IMXRT1170_PAD_AD_21,
	IMXRT1170_PAD_AD_22,
	IMXRT1170_PAD_AD_23,
	IMXRT1170_PAD_AD_24,
	IMXRT1170_PAD_AD_25,
	IMXRT1170_PAD_AD_26,
	IMXRT1170_PAD_AD_27,
	IMXRT1170_PAD_AD_28,
	IMXRT1170_PAD_AD_29,
	IMXRT1170_PAD_AD_30,
	IMXRT1170_PAD_AD_31,
	IMXRT1170_PAD_AD_32,
	IMXRT1170_PAD_AD_33,
	IMXRT1170_PAD_AD_34,
	IMXRT1170_PAD_AD_35,
	IMXRT1170_PAD_SD_B1_00,
	IMXRT1170_PAD_SD_B1_01,
	IMXRT1170_PAD_SD_B1_02,
	IMXRT1170_PAD_SD_B1_03,
	IMXRT1170_PAD_SD_B1_04,
	IMXRT1170_PAD_SD_B1_05,
	IMXRT1170_PAD_SD_B2_00,
	IMXRT1170_PAD_SD_B2_01,
	IMXRT1170_PAD_SD_B2_02,
	IMXRT1170_PAD_SD_B2_03,
	IMXRT1170_PAD_SD_B2_04,
	IMXRT1170_PAD_SD_B2_05,
	IMXRT1170_PAD_SD_B2_06,
	IMXRT1170_PAD_SD_B2_07,
	IMXRT1170_PAD_SD_B2_08,
	IMXRT1170_PAD_SD_B2_09,
	IMXRT1170_PAD_SD_B2_10,
	IMXRT1170_PAD_SD_B2_11,
	IMXRT1170_PAD_DISP_B1_00,
	IMXRT1170_PAD_DISP_B1_01,
	IMXRT1170_PAD_DISP_B1_02,
	IMXRT1170_PAD_DISP_B1_03,
	IMXRT1170_PAD_DISP_B1_04,
	IMXRT1170_PAD_DISP_B1_05,
	IMXRT1170_PAD_DISP_B1_06,
	IMXRT1170_PAD_DISP_B1_07,
	IMXRT1170_PAD_DISP_B1_08,
	IMXRT1170_PAD_DISP_B1_09,
	IMXRT1170_PAD_DISP_B1_10,
	IMXRT1170_PAD_DISP_B1_11,
	IMXRT1170_PAD_DISP_B2_00,
	IMXRT1170_PAD_DISP_B2_01,
	IMXRT1170_PAD_DISP_B2_02,
	IMXRT1170_PAD_DISP_B2_03,
	IMXRT1170_PAD_DISP_B2_04,
	IMXRT1170_PAD_DISP_B2_05,
	IMXRT1170_PAD_DISP_B2_06,
	IMXRT1170_PAD_DISP_B2_07,
	IMXRT1170_PAD_DISP_B2_08,
	IMXRT1170_PAD_DISP_B2_09,
	IMXRT1170_PAD_DISP_B2_10,
	IMXRT1170_PAD_DISP_B2_11,
	IMXRT1170_PAD_DISP_B2_12,
	IMXRT1170_PAD_DISP_B2_13,
	IMXRT1170_PAD_DISP_B2_14,
	IMXRT1170_PAD_DISP_B2_15,
};

/* Pad names for the pinmux subsystem */
static const struct pinctrl_pin_desc imxrt1170_pinctrl_pads[] = {
	IMX_PINCTRL_PIN(IMXRT1170_PAD_RESERVE0),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_RESERVE1),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_RESERVE2),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_RESERVE3),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_00),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_01),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_02),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_03),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_04),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_05),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_06),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_07),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_08),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_09),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_10),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_11),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_12),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_13),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_14),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_15),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_16),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_17),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_18),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_19),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_20),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_21),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_22),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_23),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_24),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_25),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_26),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_27),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_28),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_29),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_30),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_31),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_32),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_33),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_34),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_35),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_36),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_37),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_38),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_39),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_40),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B1_41),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_00),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_01),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_02),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_03),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_04),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_05),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_06),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_07),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_08),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_09),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_10),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_11),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_12),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_13),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_14),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_15),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_16),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_17),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_18),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_19),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_EMC_B2_20),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_00),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_01),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_02),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_03),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_04),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_05),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_06),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_07),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_08),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_09),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_10),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_11),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_12),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_13),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_14),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_15),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_16),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_17),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_18),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_19),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_20),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_21),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_22),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_23),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_24),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_25),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_26),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_27),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_28),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_29),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_30),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_31),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_32),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_33),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_34),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_AD_35),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B1_00),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B1_01),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B1_02),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B1_03),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B1_04),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B1_05),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B2_00),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B2_01),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B2_02),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B2_03),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B2_04),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B2_05),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B2_06),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B2_07),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B2_08),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B2_09),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B2_10),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_SD_B2_11),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B1_00),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B1_01),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B1_02),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B1_03),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B1_04),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B1_05),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B1_06),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B1_07),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B1_08),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B1_09),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B1_10),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B1_11),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_00),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_01),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_02),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_03),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_04),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_05),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_06),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_07),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_08),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_09),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_10),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_11),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_12),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_13),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_14),
	IMX_PINCTRL_PIN(IMXRT1170_PAD_DISP_B2_15),
};

static const struct imx_pinctrl_soc_info imxrt1170_pinctrl_info = {
	.pins = imxrt1170_pinctrl_pads,
	.npins = ARRAY_SIZE(imxrt1170_pinctrl_pads),
	.gpr_compatible = "fsl,imxrt1170-iomuxc-gpr",
};

static const struct of_device_id imxrt1170_pinctrl_of_match[] = {
	{ .compatible = "fsl,imxrt1170-iomuxc", .data = &imxrt1170_pinctrl_info, },
	{ /* sentinel */ }
};

static int imxrt1170_pinctrl_probe(struct platform_device *pdev)
{
	return imx_pinctrl_probe(pdev, &imxrt1170_pinctrl_info);
}

static struct platform_driver imxrt1170_pinctrl_driver = {
	.driver = {
		.name = "imxrt1170-pinctrl",
		.of_match_table = of_match_ptr(imxrt1170_pinctrl_of_match),
		.suppress_bind_attrs = true,
	},
	.probe = imxrt1170_pinctrl_probe,
};

static int __init imxrt1170_pinctrl_init(void)
{
	return platform_driver_register(&imxrt1170_pinctrl_driver);
}
arch_initcall(imxrt1170_pinctrl_init);
