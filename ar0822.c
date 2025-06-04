// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the OnSemi AR0822 CMOS Image Sensor.
 *
 * Copyright (C) 2025 Kurokesu UAB.
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define AR0822_PIXEL_RATE 160000000
#define AR0822_REG_ADDRESS_BITS 16

#define AR0822_EMBEDDED_LINE_WIDTH 16384
#define AR0822_NUM_EMBEDDED_LINES 0

#define AR0822_VBLANK_MIN 24
#define AR0822_VBLANK_STEP 8

#define AR0822_VTS_MAX 0xFFFF

#define AR0822_RESET_MIN_DELAY_US 7000
#define AR0822_RESET_MAX_DELAY_US (AR0822_RESET_MIN_DELAY_US + 1000)

#define AR0822_PIXEL_NATIVE_WIDTH 3840
#define AR0822_PIXEL_NATIVE_HEIGHT 2160
#define AR0822_PIXEL_ARRAY_WIDTH 3840
#define AR0822_PIXEL_ARRAY_HEIGHT 2160
#define AR0822_PIXEL_ARRAY_TOP 8
#define AR0822_PIXEL_ARRAY_LEFT 8

#define AR0822_EXPOSURE_MIN 4 // ?
#define AR0822_EXPOSURE_STEP 1

#define AR0822_ANA_GAIN_MIN 0
#define AR0822_ANA_GAIN_MAX 232
#define AR0822_ANA_GAIN_STEP 1
#define AR0822_ANA_GAIN_DEFAULT 0

#define AR0822_MODEL_ID 0x0F56
#define AR0822_MODE_LOW_POWER 0x0018
#define AR0822_MODE_STREAM_ON (AR0822_MODE_LOW_POWER | BIT(2))

#define AR0822_MODE_SELECT_STREAM_OFF 0x00
#define AR0822_MODE_SELECT_STREAM_ON BIT(0)

#define AR0822_IMAGE_ORIENTATION_HFLIP_BIT 0
#define AR0822_IMAGE_ORIENTATION_VFLIP_BIT 1

#define AR0822_TEST_PATTERN_DISABLED 0
#define AR0822_TEST_PATTERN_SOLID_COLOR 1
#define AR0822_TEST_PATTERN_VERTICAL_COLOR_BARS 2
#define AR0822_TEST_PATTERN_FADE_TO_GREY 3
#define AR0822_TEST_PATTERN_PN9 4
#define AR0822_TEST_PATTERN_WALKING_1S 256

#define AR0822_TEST_PATTERN_COLOR_MIN 0
#define AR0822_TEST_PATTERN_COLOR_MAX 0xFFF
#define AR0822_TEST_PATTERN_COLOR_STEP 1

#define AR0822_REG_CHIP_VERSION CCI_REG16(0x3000)
#define AR0822_REG_FRAME_LENGTH_LINES CCI_REG16(0x300A)
#define AR0822_REG_COARSE_INTEGRATION_TIME CCI_REG16(0x3012)
#define AR0822_REG_RESET CCI_REG16(0x301A)
#define AR0822_REG_MODE_SELECT CCI_REG8(0x301C)
#define AR0822_REG_IMAGE_ORIENTATION CCI_REG8(0x301D)
#define AR0822_REG_SENSOR_GAIN CCI_REG8(0x5900)
#define AR0822_REG_TEST_PATTERN_MODE CCI_REG16(0x3070)
#define AR0822_REG_TEST_DATA_RED CCI_REG16(0x3072)
#define AR0822_REG_TEST_DATA_GREENR CCI_REG16(0x3074)
#define AR0822_REG_TEST_DATA_BLUE CCI_REG16(0x3076)
#define AR0822_REG_TEST_DATA_GREENB CCI_REG16(0x3078)

#define AR0822_REG_VT_PIX_CLK_DIV CCI_REG16(0x302A)
#define AR0822_REG_VT_SYS_CLK_DIV CCI_REG16(0x302C)
#define AR0822_REG_PRE_PLL_CLK_DIV CCI_REG16(0x302E)
#define AR0822_REG_PLL_MULTIPLIER CCI_REG16(0x3030)
#define AR0822_REG_OP_WORD_CLK_DIV CCI_REG16(0x3036)
#define AR0822_REG_OP_SYS_CLK_DIV CCI_REG16(0x3038)
#define AR0822_REG_PLL_CONTROL CCI_REG16(0x31DC)
#define AR0822_REG_DIGITAL_TEST CCI_REG16(0x30B0)
#define AR0822_REG_X_ADDR_START CCI_REG16(0x3004)
#define AR0822_REG_X_ADDR_END CCI_REG16(0x3008)
#define AR0822_REG_Y_ADDR_START CCI_REG16(0x3002)
#define AR0822_REG_Y_ADDR_END CCI_REG16(0x3006)
#define AR0822_REG_X_ODD_INC CCI_REG16(0x30A2)
#define AR0822_REG_Y_ODD_INC CCI_REG16(0x30A6)
#define AR0822_REG_X_OUTPUT_CONTROL CCI_REG16(0x3402)
#define AR0822_REG_Y_OUTPUT_CONTROL CCI_REG16(0x3404)
#define AR0822_REG_READ_MODE CCI_REG16(0x3040)
#define AR0822_REG_DARK_CONTROL CCI_REG16(0x3044)
#define AR0822_REG_OPERATION_MODE_CTRL CCI_REG16(0x3082)
#define AR0822_REG_DIGITAL_CTRL CCI_REG16(0x30BA)
#define AR0822_REG_COMPANDING CCI_REG16(0x31D0)
#define AR0822_REG_SERIAL_FORMAT CCI_REG16(0x31AE)
#define AR0822_REG_DATA_FORMAT_BITS CCI_REG16(0x31AC)
#define AR0822_REG_LINE_LENGTH_PCK CCI_REG16(0x300C)
#define AR0822_REG_COARSE_INTEGRATION_TIME2 CCI_REG16(0x3212)
#define AR0822_REG_COARSE_INTEGRATION_TIME3 CCI_REG16(0x3216)
#define AR0822_REG_EXPOSURE_RATIO CCI_REG16(0x3238)
#define AR0822_REG_SHUT_CTRL2 CCI_REG16(0x32EC)
#define AR0822_REG_MEC_CTRL2 CCI_REG16(0x3D02)
#define AR0822_REG_FRAME_PREAMBLE CCI_REG16(0x31B0)
#define AR0822_REG_LINE_PREAMBLE CCI_REG16(0x31B2)
#define AR0822_REG_MIPI_TIMING_0 CCI_REG16(0x31B4)
#define AR0822_REG_MIPI_TIMING_1 CCI_REG16(0x31B6)
#define AR0822_REG_MIPI_TIMING_2 CCI_REG16(0x31B8)
#define AR0822_REG_MIPI_TIMING_3 CCI_REG16(0x31BA)
#define AR0822_REG_MIPI_TIMING_4 CCI_REG16(0x31BC)
#define AR0822_REG_HISPI_CONTROL CCI_REG16(0x31C6)
#define AR0822_REG_MIPI_DESKEW_PAT_WIDTH CCI_REG16(0x31C8)
#define AR0822_REG_MIPI_PER_DESKEW_PAT_WIDTH CCI_REG16(0x5930)
#define AR0822_REG_MIPI_HISPI_TRIM CCI_REG16(0x31DE)
#define AR0822_REG_MIPI_CONFIG_2 CCI_REG16(0x31F8)
#define AR0822_REG_MIPI_F1_PDT CCI_REG16(0x3342)
#define AR0822_REG_MIPI_F1_VC CCI_REG16(0x3344)
#define AR0822_REG_MIPI_F2_PDT CCI_REG16(0x3346)
#define AR0822_REG_MIPI_F2_VC CCI_REG16(0x3348)
#define AR0822_REG_MIPI_F3_PDT CCI_REG16(0x334A)
#define AR0822_REG_MIPI_F3_VC CCI_REG16(0x334C)
#define AR0822_REG_MIPI_F4_PDT CCI_REG16(0x334E)
#define AR0822_REG_MIPI_F4_VC CCI_REG16(0x3350)

enum pad_types { IMAGE_PAD, NUM_PADS };

static const char *const ar0822_supply_names[] = {
	"vana", /* Analog (2.8V) supply */
	"vdig", /* Digital Core (1.8V) supply */
	"vddl", /* IF (1.2V) supply */
};

#define AR0822_SUPPLY_AMOUNT ARRAY_SIZE(ar0822_supply_names)

enum ar0822_extclk_link_id {
	AR0822_EXTCLK_LINK_ID_24_480 = 0,
	AR0822_EXTCLK_LINK_ID_24_960,
};

static const u64 ar0822_extclk_frequencies[] = {
	[AR0822_EXTCLK_LINK_ID_24_480] = 24000000,
	[AR0822_EXTCLK_LINK_ID_24_960] = 24000000,
};

static const s64 ar0822_link_frequencies[] = {
	[AR0822_EXTCLK_LINK_ID_24_480] = 480000000,
	[AR0822_EXTCLK_LINK_ID_24_960] = 960000000,
};

enum ar0822_bit_depth {
	AR0822_BIT_DEPTH_10BIT = 0,
	AR0822_BIT_DEPTH_12BIT,
	AR0822_BIT_DEPTH_AMOUNT,
};

enum ar0822_lane_mode {
	AR0822_LANE_MODE_2 = 0,
	AR0822_LANE_MODE_4,
	AR0822_LANE_MODE_AMOUNT,
};

struct ar0822_timing {
	unsigned int line_length_pck_min;
	unsigned int frame_length_lines_min;
};

/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 */
static const u32 ar0822_format_codes_10bit[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10, /* 0: no flip */
	MEDIA_BUS_FMT_SRGGB10_1X10, /* 1: horizontal flip */
	MEDIA_BUS_FMT_SBGGR10_1X10, /* 2: vertical flip */
	MEDIA_BUS_FMT_SGBRG10_1X10, /* 3: horizontal and vertical flip */
};

static const u32 ar0822_format_codes_12bit[] = {
	MEDIA_BUS_FMT_SGRBG12_1X12, /* 0: no flip */
	MEDIA_BUS_FMT_SRGGB12_1X12, /* 1: horizontal flip */
	MEDIA_BUS_FMT_SBGGR12_1X12, /* 2: vertical flip */
	MEDIA_BUS_FMT_SGBRG12_1X12, /* 3: horizontal and vertical flip */
};

struct ar0822_mode_lut {
	unsigned int width;
	unsigned int height;
	struct v4l2_rect crop;
	struct ar0822_timing timing[AR0822_LANE_MODE_AMOUNT]
				   [AR0822_BIT_DEPTH_AMOUNT];
	struct cci_reg_sequence const *regs;
	unsigned int regs_amount;
};

struct ar0822_pll_config {
	s64 const *link_freq;
	u64 const *extclk_freq;
	unsigned long pixel_rate;
	unsigned int regs_amount;
	struct cci_reg_sequence const *regs;
	struct ar0822_mode_lut const *supported_modes;
	u32 modes_amount;
};

static const struct cci_reg_sequence ar0822_pll_config_24_480[] = {
	{ AR0822_REG_PLL_MULTIPLIER, 0x0050 },
	{ AR0822_REG_PRE_PLL_CLK_DIV, 0x0001 },
	{ AR0822_REG_VT_SYS_CLK_DIV, 0x0002 },
	{ AR0822_REG_VT_PIX_CLK_DIV, 0x0006 },
	{ AR0822_REG_OP_SYS_CLK_DIV, 0x0004 },
	{ AR0822_REG_OP_WORD_CLK_DIV, 0x0006 },
	{ AR0822_REG_FRAME_PREAMBLE, 0x006C },
	{ AR0822_REG_LINE_PREAMBLE, 0x004A },
	{ AR0822_REG_MIPI_TIMING_0, 0x51C8 },
	{ AR0822_REG_MIPI_TIMING_1, 0x5248 },
	{ AR0822_REG_MIPI_TIMING_2, 0x70CA },
	{ AR0822_REG_MIPI_TIMING_3, 0x028A },
	{ AR0822_REG_MIPI_TIMING_4, 0x0C08 },
};

static const struct cci_reg_sequence ar0822_pll_config_24_960[] = {
	{ AR0822_REG_PLL_MULTIPLIER, 0x0050 },
	{ AR0822_REG_PRE_PLL_CLK_DIV, 0x0001 },
	{ AR0822_REG_VT_SYS_CLK_DIV, 0x0002 },
	{ AR0822_REG_VT_PIX_CLK_DIV, 0x0006 },
	{ AR0822_REG_OP_SYS_CLK_DIV, 0x0002 },
	{ AR0822_REG_OP_WORD_CLK_DIV, 0x0006 },
	{ AR0822_REG_FRAME_PREAMBLE, 0x00B8 },
	{ AR0822_REG_LINE_PREAMBLE, 0x0079 },
	{ AR0822_REG_MIPI_TIMING_0, 0x830E },
	{ AR0822_REG_MIPI_TIMING_1, 0x8451 },
	{ AR0822_REG_MIPI_TIMING_2, 0xD0CE },
	{ AR0822_REG_MIPI_TIMING_3, 0x0494 },
	{ AR0822_REG_MIPI_TIMING_4, 0x1810 },
};

static const struct cci_reg_sequence ar0822_1080p_config[] = {
	{ AR0822_REG_X_ADDR_START, 968 },
	{ AR0822_REG_X_ADDR_END, 2887 },
	{ AR0822_REG_Y_ADDR_START, 548 },
	{ AR0822_REG_Y_ADDR_END, 1627 },
	{ AR0822_REG_X_ODD_INC, 0x0001 }, // default no skip
	{ AR0822_REG_Y_ODD_INC, 0x0001 }, // default no skip
	{ AR0822_REG_X_OUTPUT_CONTROL, 1920 },
	{ AR0822_REG_Y_OUTPUT_CONTROL, 1080 },
};

static const struct cci_reg_sequence ar0822_4k_config[] = {
	{ AR0822_REG_X_ADDR_START, 8 },
	{ AR0822_REG_X_ADDR_END, 3847 },
	{ AR0822_REG_Y_ADDR_START, 8 },
	{ AR0822_REG_Y_ADDR_END, 2167 },
	{ AR0822_REG_X_ODD_INC, 0x0001 }, // default no skip
	{ AR0822_REG_Y_ODD_INC, 0x0001 }, // default no skip
	{ AR0822_REG_X_OUTPUT_CONTROL, 3840 },
	{ AR0822_REG_Y_OUTPUT_CONTROL, 2160 },
};

#define AR0822_FLL_1080P_MIN 1104
#define AR0822_FLL_4K_MIN 2184

static const struct ar0822_mode_lut ar0822_modes_24_480[] = {
	// {
	// 	.width = 1920,
	// 	.height = 1080,
	// 	.crop = {
	// 		.left = 0,
	// 		.top = 0,
	// 		.width = 1920,
	// 		.height = 1080,
	// 	},
	// 	.regs = ar0822_1080p_config,
	// 	.regs_amount = ARRAY_SIZE(ar0822_1080p_config),
	// 	.timing = {
	// 		[AR0822_LANE_MODE_2][AR0822_BIT_DEPTH_10BIT] = {
	// 			.line_length_pck_min = 1812,
	// 			.frame_length_lines_min = AR0822_FLL_1080P_MIN,
	// 		},
	// 		[AR0822_LANE_MODE_2][AR0822_BIT_DEPTH_12BIT] = {
	// 			.line_length_pck_min = 2142,
	// 			.frame_length_lines_min = AR0822_FLL_1080P_MIN,
	// 		},
	// 		[AR0822_LANE_MODE_4][AR0822_BIT_DEPTH_10BIT] = {
	// 			.line_length_pck_min = 1012,
	// 			.frame_length_lines_min = AR0822_FLL_1080P_MIN,
	// 		},
	// 		[AR0822_LANE_MODE_4][AR0822_BIT_DEPTH_12BIT] = {
	// 			.line_length_pck_min = 1180,
	// 			.frame_length_lines_min = AR0822_FLL_1080P_MIN,
	// 		},
	// 	},
	// },
	{
		.width = 3840,
		.height = 2160,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 3840,
			.height = 2160,
		},
		.regs = ar0822_4k_config,
		.regs_amount = ARRAY_SIZE(ar0822_4k_config),
		.timing = {
			[AR0822_LANE_MODE_2][AR0822_BIT_DEPTH_10BIT] = {
				.line_length_pck_min = 3412,
				.frame_length_lines_min = AR0822_FLL_4K_MIN,
			},
			[AR0822_LANE_MODE_2][AR0822_BIT_DEPTH_12BIT] = {
				.line_length_pck_min = 4062,
				.frame_length_lines_min = AR0822_FLL_4K_MIN,
			},
			[AR0822_LANE_MODE_4][AR0822_BIT_DEPTH_10BIT] = {
				.line_length_pck_min = 1812,
				.frame_length_lines_min = AR0822_FLL_4K_MIN,
			},
			[AR0822_LANE_MODE_4][AR0822_BIT_DEPTH_12BIT] = {
				.line_length_pck_min = 2140,
				.frame_length_lines_min = AR0822_FLL_4K_MIN,
			},
		},
	},
};

static const struct ar0822_mode_lut ar0822_modes_24_960[] = {
	// {
	// 	.width = 1920,
	// 	.height = 1080,
	// 	.crop = {
	// 		.left = 0,
	// 		.top = 0,
	// 		.width = 1920,
	// 		.height = 1080,
	// 	},
	// 	.timing = {
	// 		[AR0822_LANE_MODE_2][AR0822_BIT_DEPTH_10BIT] = {
	// 			.line_length_pck_min = 982,
	// 			.frame_length_lines_min = AR0822_FLL_1080P_MIN,
	// 		},
	// 		[AR0822_LANE_MODE_2][AR0822_BIT_DEPTH_12BIT] = {
	// 			.line_length_pck_min = 2414,
	// 			.frame_length_lines_min = 1104,
	// 		},
	// 		[AR0822_LANE_MODE_4][AR0822_BIT_DEPTH_10BIT] = {
	// 			.line_length_pck_min = 792,
	// 			.frame_length_lines_min = AR0822_FLL_1080P_MIN,
	// 		},
	// 		[AR0822_LANE_MODE_4][AR0822_BIT_DEPTH_12BIT] = {
	// 			.line_length_pck_min = 792,
	// 			.frame_length_lines_min = AR0822_FLL_1080P_MIN,
	// 		},
	// 	},
	// },
	{
		.width = 3840,
		.height = 2160,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 3840,
			.height = 2160,
		},
		.timing = {
			[AR0822_LANE_MODE_2][AR0822_BIT_DEPTH_10BIT] = {
				.line_length_pck_min = 1782,
				.frame_length_lines_min = AR0822_FLL_4K_MIN,
			},
			[AR0822_LANE_MODE_2][AR0822_BIT_DEPTH_12BIT] = {
				.line_length_pck_min = 2106,
				.frame_length_lines_min = AR0822_FLL_4K_MIN,
			},
			[AR0822_LANE_MODE_4][AR0822_BIT_DEPTH_10BIT] = {
				.line_length_pck_min = 982,
				.frame_length_lines_min = AR0822_FLL_4K_MIN,
			},
			[AR0822_LANE_MODE_4][AR0822_BIT_DEPTH_12BIT] = {
				.line_length_pck_min = 1146,
				.frame_length_lines_min = AR0822_FLL_4K_MIN,
			},
		},
	},
};

static const struct ar0822_pll_config ar0822_pll_configs[] = {
	{
		.link_freq =
			&ar0822_link_frequencies[AR0822_EXTCLK_LINK_ID_24_480],
		.extclk_freq =
			&ar0822_extclk_frequencies[AR0822_EXTCLK_LINK_ID_24_480],
		.pixel_rate = AR0822_PIXEL_RATE,
		.regs_amount = ARRAY_SIZE(ar0822_pll_config_24_480),
		.regs = ar0822_pll_config_24_480,
		.supported_modes = ar0822_modes_24_480,
		.modes_amount = ARRAY_SIZE(ar0822_modes_24_480),
	},
	{
		.link_freq =
			&ar0822_link_frequencies[AR0822_EXTCLK_LINK_ID_24_960],
		.extclk_freq =
			&ar0822_extclk_frequencies[AR0822_EXTCLK_LINK_ID_24_960],
		.pixel_rate = AR0822_PIXEL_RATE,
		.regs_amount = ARRAY_SIZE(ar0822_pll_config_24_960),
		.regs = ar0822_pll_config_24_960,
		.supported_modes = ar0822_modes_24_960,
		.modes_amount = ARRAY_SIZE(ar0822_modes_24_960),
	},
};

static const char *const ar0822_test_pattern_menu[] = {
	"Disabled",
	"Solid Color",
	"Vertical Color Bars",
	"Fade to Grey Vertical Color Bars",
	"PN9",
	"Walking 1s",
};

static const unsigned int ar0822_test_pattern_val[] = {
	AR0822_TEST_PATTERN_DISABLED,
	AR0822_TEST_PATTERN_SOLID_COLOR,
	AR0822_TEST_PATTERN_VERTICAL_COLOR_BARS,
	AR0822_TEST_PATTERN_FADE_TO_GREY,
	AR0822_TEST_PATTERN_PN9,
	AR0822_TEST_PATTERN_WALKING_1S,
};

struct ar0822_mode {
	struct ar0822_mode_lut const *info;
	enum ar0822_bit_depth bit_depth;
};

struct ar0822_hw_config {
	struct clk *extclk;
	struct regulator_bulk_data supplies[AR0822_SUPPLY_AMOUNT];
	struct gpio_desc *gpio_reset;
	struct ar0822_pll_config const *pll_config;
	unsigned int num_data_lanes;
	enum ar0822_lane_mode lane_mode;
};

struct ar0822 {
	struct device *dev;
	struct ar0822_hw_config hw_config;

	struct regmap *regmap;

	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_hdlr;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *exposure;

	struct mutex mutex;
	bool streaming;
	struct ar0822_mode mode;
	unsigned int fmt_code;
};

static const struct cci_reg_sequence ar0822_init_table[] = {
	{ AR0822_REG_MIPI_F1_PDT, 0x122C },
	{ AR0822_REG_T1_PIX_DEF_ID2, 0x37C3 },
	{ AR0822_REG_SERIAL_FORMAT, 0x0202 }, // 2 lane MIPI
	{ AR0822_REG_DATA_FORMAT_BITS, 0x0C0C }, // 12bit / 12bit raw
	{ AR0822_REG_OPERATION_MODE_CTRL, 0x0001 },
	{ AR0822_REG_DIGITAL_CTRL, 0x0024 },

	// { AR0822_REG_PLL_CONTROL, 0x0030 },
	// // { AR0822_REG_RESET, AR0822_MODE_STREAM_ON },
	// { AR0822_REG_DIGITAL_TEST, 0x0800 }, // default
	// { AR0822_REG_HISPI_CONTROL, 0x8000 }, // default
	// { AR0822_REG_MIPI_DESKEW_PAT_WIDTH, 0x0ABF },
	// { AR0822_REG_MIPI_PER_DESKEW_PAT_WIDTH, 0x006E },
	// { AR0822_REG_MIPI_HISPI_TRIM, 0x0000 }, // default
	// { AR0822_REG_MIPI_CONFIG_2, 0x0408 },
	// { AR0822_REG_MIPI_F1_VC, 0x0011 },
	// { AR0822_REG_MIPI_F2_PDT, 0x122C },
	// { AR0822_REG_MIPI_F2_VC, 0x0011 },
	// { AR0822_REG_MIPI_F3_PDT, 0x122C },
	// { AR0822_REG_MIPI_F3_VC, 0x0011 },
	// { AR0822_REG_MIPI_F4_PDT, 0x122C },
	// { AR0822_REG_MIPI_F4_VC, 0x0011 },

	// // /* Timing settings */
	// { AR0822_REG_READ_MODE, 0x0000 },
	// { AR0822_REG_DARK_CONTROL, 0x0400 },
	// { AR0822_REG_COMPANDING, 0x0000 }, // default
	// { AR0822_REG_COARSE_INTEGRATION_TIME2, 0x0000 },
	// { AR0822_REG_COARSE_INTEGRATION_TIME3, 0x0000 },
	// { AR0822_REG_EXPOSURE_RATIO, 0x0022 }, // default
	// { AR0822_REG_SHUT_CTRL2, 0x72A0 }, // unknown
	// { AR0822_REG_MEC_CTRL2, 0x0003 }, // unknown
};

static inline struct ar0822 *to_ar0822(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar0822, subdev);
}

// static int ar0822_set_testpattern(struct ar0822 *sensor, int val)
// {
// 	// int ret = 0;

// 	return -1; // TODO: implement

// if (val) {
// 	cci_write(sensor->regmap, IMX415_BLKLEVEL, 0x00, &ret);
// 	cci_write(sensor->regmap, IMX415_TPG_EN_DUOUT, 0x01, &ret);
// 	cci_write(sensor->regmap, IMX415_TPG_PATSEL_DUOUT, val - 1,
// 		  &ret);
// 	cci_write(sensor->regmap, IMX415_TPG_COLORWIDTH, 0x01, &ret);
// 	cci_write(sensor->regmap, IMX415_TESTCLKEN_MIPI, 0x20, &ret);
// 	cci_write(sensor->regmap, IMX415_DIG_CLP_MODE, 0x00, &ret);
// 	cci_write(sensor->regmap, IMX415_WRJ_OPEN, 0x00, &ret);
// } else {
// 	cci_write(sensor->regmap, IMX415_BLKLEVEL,
// 		  IMX415_BLKLEVEL_DEFAULT, &ret);
// 	cci_write(sensor->regmap, IMX415_TPG_EN_DUOUT, 0x00, &ret);
// 	cci_write(sensor->regmap, IMX415_TESTCLKEN_MIPI, 0x00, &ret);
// 	cci_write(sensor->regmap, IMX415_DIG_CLP_MODE, 0x01, &ret);
// 	cci_write(sensor->regmap, IMX415_WRJ_OPEN, 0x01, &ret);
// }
// return 0;
// }

static void ar0822_adjust_exposure_range(struct ar0822 *sensor)
{
	int exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure. */
	exposure_max = sensor->mode.info->height + sensor->vblank->val -
		       AR0822_EXPOSURE_MIN;
	exposure_def = min(exposure_max, sensor->exposure->val);
	__v4l2_ctrl_modify_range(sensor->exposure, sensor->exposure->minimum,
				 exposure_max, sensor->exposure->step,
				 exposure_def);
}

static int ar0822_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0822 *sensor =
		container_of(ctrl->handler, struct ar0822, ctrl_hdlr);
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	int ret = 0;

	dev_dbg(sensor->dev, "ar0822_set_ctrl: %d %d\n", ctrl->id, ctrl->val);

	if (ctrl->id == V4L2_CID_VBLANK)
		ar0822_adjust_exposure_range(sensor);

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		dev_dbg(sensor->dev,
			"ar0822_set_ctrl: AR0822_REG_FRAME_LENGTH_LINES %d\n",
			sensor->mode.info->height + ctrl->val);
		ret = cci_write(sensor->regmap, AR0822_REG_FRAME_LENGTH_LINES,
				sensor->mode.info->height + ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(sensor->dev,
			"ar0822_set_ctrl: AR0822_REG_COARSE_INTEGRATION_TIME %d\n",
			ctrl->val);
		ret = cci_write(sensor->regmap,
				AR0822_REG_COARSE_INTEGRATION_TIME, ctrl->val,
				NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(sensor->dev,
			"ar0822_set_ctrl: AR0822_REG_SENSOR_GAIN %d\n",
			ctrl->val);
		ret = cci_write(sensor->regmap, AR0822_REG_SENSOR_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:

		dev_dbg(sensor->dev,
			"ar0822_set_ctrl: AR0822_REG_IMAGE_ORIENTATION %d\n",
			sensor->hflip->val | sensor->vflip->val << 1);
		ret = cci_write(sensor->regmap, AR0822_REG_IMAGE_ORIENTATION,
				sensor->hflip->val | sensor->vflip->val << 1,
				NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		dev_dbg(sensor->dev, "AR0822_REG_TEST_PATTERN_MODE %d\n",
			ar0822_test_pattern_val[ctrl->val]);

		ret = cci_write(sensor->regmap, AR0822_REG_TEST_PATTERN_MODE,
				ar0822_test_pattern_val[ctrl->val], NULL);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = cci_write(sensor->regmap, AR0822_REG_TEST_DATA_RED,
				ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = cci_write(sensor->regmap, AR0822_REG_TEST_DATA_GREENR,
				ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = cci_write(sensor->regmap, AR0822_REG_TEST_DATA_BLUE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = cci_write(sensor->regmap, AR0822_REG_TEST_DATA_GREENB,
				ctrl->val, NULL);
		break;
	default:
		dev_err(sensor->dev, "unhandled control %d\n", ctrl->id);
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ar0822_ctrl_ops = {
	.s_ctrl = ar0822_set_ctrl,
};

static struct ar0822_timing const *ar0822_get_timing(struct ar0822 *sensor)
{
	return &sensor->mode.info->timing[sensor->hw_config.lane_mode]
					 [sensor->mode.bit_depth];
}

static void ar0822_set_framing_limits(struct ar0822 *sensor)
{
	int hblank;
	const struct ar0822_mode_lut *mode_info = sensor->mode.info;
	struct ar0822_timing const *timing = ar0822_get_timing(sensor);

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(
		sensor->vblank, AR0822_VBLANK_MIN,
		AR0822_VTS_MAX - mode_info->height, sensor->vblank->step,
		timing->frame_length_lines_min - mode_info->height);

	/* Setting this will adjust the exposure limits as well */
	__v4l2_ctrl_s_ctrl(sensor->vblank,
			   timing->frame_length_lines_min - mode_info->height);

	hblank = timing->line_length_pck_min - mode_info->width;
	__v4l2_ctrl_modify_range(sensor->hblank, hblank, hblank, 1, hblank);
	__v4l2_ctrl_s_ctrl(sensor->hblank, hblank);
}

static int ar0822_ctrls_init(struct ar0822 *sensor)
{
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	struct ar0822_mode_lut const *mode_info = sensor->mode.info;
	struct ar0822_timing const *timing = ar0822_get_timing(sensor);
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	u8 link_freq_id = sensor->hw_config.pll_config->link_freq -
			  ar0822_link_frequencies;
	u32 exposure_max, exposure_def, hblank;
	int ret;

	ret = v4l2_ctrl_handler_init(&sensor->ctrl_hdlr, 10);
	if (ret)
		return ret;

	mutex_init(&sensor->mutex);
	sensor->ctrl_hdlr.lock = &sensor->mutex;

	/* Link frequency control */
	ctrl = v4l2_ctrl_new_int_menu(&sensor->ctrl_hdlr, &ar0822_ctrl_ops,
				      V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(ar0822_link_frequencies) - 1,
				      link_freq_id, ar0822_link_frequencies);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Exposure control */
	exposure_max = timing->frame_length_lines_min - AR0822_EXPOSURE_MIN;
	exposure_def = (exposure_max < AR0822_EXPOSURE_DEFAULT) ?
			       exposure_max :
			       AR0822_EXPOSURE_DEFAULT;
	sensor->exposure = v4l2_ctrl_new_std(
		&sensor->ctrl_hdlr, &ar0822_ctrl_ops, V4L2_CID_EXPOSURE,
		AR0822_EXPOSURE_MIN, exposure_max, AR0822_EXPOSURE_STEP,
		exposure_def);

	/* Analogue gain control */
	v4l2_ctrl_new_std(&sensor->ctrl_hdlr, &ar0822_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, AR0822_ANA_GAIN_MIN,
			  AR0822_ANA_GAIN_MAX, AR0822_ANA_GAIN_STEP,
			  AR0822_ANA_GAIN_MIN);

	/* Horizontal blanking control */
	hblank = timing->line_length_pck_min - mode_info->width;
	sensor->hblank = v4l2_ctrl_new_std(&sensor->ctrl_hdlr, &ar0822_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (sensor->hblank)
		sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Vertical blanking control */
	sensor->vblank = v4l2_ctrl_new_std(
		&sensor->ctrl_hdlr, &ar0822_ctrl_ops, V4L2_CID_VBLANK,
		AR0822_VBLANK_MIN, AR0822_VTS_MAX - mode_info->height,
		AR0822_VBLANK_STEP,
		timing->frame_length_lines_min - mode_info->height);

	/* Pixel rate control */
	ctrl = v4l2_ctrl_new_std(&sensor->ctrl_hdlr, NULL, V4L2_CID_PIXEL_RATE,
				 sensor->hw_config.pll_config->pixel_rate,
				 sensor->hw_config.pll_config->pixel_rate, 1,
				 sensor->hw_config.pll_config->pixel_rate);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Horizontal flip control */
	sensor->hflip = v4l2_ctrl_new_std(&sensor->ctrl_hdlr, &ar0822_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (sensor->hflip)
		sensor->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	/* Vertical flip control */
	sensor->vflip = v4l2_ctrl_new_std(&sensor->ctrl_hdlr, &ar0822_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (sensor->vflip)
		sensor->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	/* Test pattern control */
	v4l2_ctrl_new_std_menu_items(&sensor->ctrl_hdlr, &ar0822_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ar0822_test_pattern_menu) - 1,
				     0, 0, ar0822_test_pattern_menu);

	for (unsigned int i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(&sensor->ctrl_hdlr, &ar0822_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  AR0822_TEST_PATTERN_COLOR_MIN,
				  AR0822_TEST_PATTERN_COLOR_MAX,
				  AR0822_TEST_PATTERN_COLOR_STEP,
				  AR0822_TEST_PATTERN_COLOR_MAX);
		/* The "Solid color" pattern is white by default */
	}

	if (sensor->ctrl_hdlr.error) {
		ret = sensor->ctrl_hdlr.error;
		dev_err(&client->dev, "failed to init controls %d\n", ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(&sensor->ctrl_hdlr,
					      &ar0822_ctrl_ops, &props);
	if (ret)
		goto error;

	sensor->subdev.ctrl_handler = &sensor->ctrl_hdlr;

	mutex_lock(&sensor->mutex);

	ar0822_set_framing_limits(sensor);

	mutex_unlock(&sensor->mutex);

	return 0;

error:
	v4l2_ctrl_handler_free(&sensor->ctrl_hdlr);
	mutex_destroy(&sensor->mutex);
	return ret;
}

static int ar0822_mode_stream_on(struct ar0822 *sensor)
{
	dev_dbg(sensor->dev, "%s\n", __func__);

	return cci_write(sensor->regmap, AR0822_REG_MODE_SELECT,
			 AR0822_MODE_SELECT_STREAM_ON, NULL);
}

static int ar0822_mode_stream_off(struct ar0822 *sensor)
{
	dev_dbg(sensor->dev, "%s\n", __func__);

	return cci_write(sensor->regmap, AR0822_REG_MODE_SELECT,
			 AR0822_MODE_SELECT_STREAM_OFF, NULL);
}

static int ar0822_start_streaming(struct ar0822 *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	struct ar0822_timing const *timing = ar0822_get_timing(sensor);
	int ret;

	ret = pm_runtime_resume_and_get(&client->dev);
	if (ret < 0)
		return ret;

	dev_dbg(sensor->dev, "%s: setting up sensor\n", __func__);

	ret = cci_write(sensor->regmap, AR0822_REG_RESET, AR0822_MODE_STREAM_ON,
			NULL);
	if (ret < 0) {
		dev_err(sensor->dev, "Failed to set stream on: %d\n", ret);
		return ret;
	}

	/* Datasheet states that stream ON should be toggled ON for minimum 2ms */
	usleep_range(2000, 2100);

	/* Set the sensor back to low power mode */
	ret = cci_write(sensor->regmap, AR0822_REG_RESET, AR0822_MODE_LOW_POWER,
			NULL);
	if (ret < 0) {
		dev_err(sensor->dev, "Failed to set low power mode: %d\n", ret);
		return ret;
	}

	/* wait 160000 EXTCLKs for software standdby */
	usleep_range(7000, 8000);

	/* PLL config */
	ret = cci_multi_reg_write(sensor->regmap,
				  sensor->hw_config.pll_config->regs,
				  sensor->hw_config.pll_config->regs_amount,
				  NULL);
	if (ret) {
		dev_err(sensor->dev, "Failed to write PLL config: %d\n", ret);
		return ret;
	}

	/* Common init */
	ret = cci_multi_reg_write(sensor->regmap, ar0822_init_table,
				  ARRAY_SIZE(ar0822_init_table), NULL);
	if (ret) {
		dev_err(sensor->dev, "Failed to write init table: %d\n", ret);
		return ret;
	}

	/* Mode init */
	ret = cci_multi_reg_write(sensor->regmap, sensor->mode.info->regs,
				  sensor->mode.info->regs_amount, NULL);
	if (ret) {
		dev_err(sensor->dev, "Failed to write mode config: %d\n", ret);
		return ret;
	}

	ret = cci_write(sensor->regmap, AR0822_REG_LINE_LENGTH_PCK,
			timing->line_length_pck_min, NULL);
	if (ret) {
		dev_err(sensor->dev, "Failed to set line length: %d\n", ret);
		return ret;
	}

	ret = cci_write(sensor->regmap, AR0822_REG_FRAME_LENGTH_LINES,
			timing->frame_length_lines_min, NULL);
	if (ret) {
		dev_err(sensor->dev, "Failed to set frame length: %d\n", ret);
		return ret;
	}

	dev_dbg(sensor->dev, "%s: w %d h %d fmt_code %x llp %d fll %d\n",
		__func__, sensor->mode.info->width, sensor->mode.info->height,
		sensor->fmt_code, timing->line_length_pck_min,
		timing->frame_length_lines_min);
	// { AR0822_REG_COARSE_INTEGRATION_TIME, 0x017C },

	usleep_range(1000, 1100);

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(sensor->subdev.ctrl_handler);
	if (ret) {
		dev_err(sensor->dev, "Failed to setup controls: %d\n", ret);
		return ret;
	}

	ret = ar0822_mode_stream_on(sensor);
	return ret;
}

/* Stop streaming */
static void ar0822_stop_streaming(struct ar0822 *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	int ret;

	ret = ar0822_mode_stream_off(sensor);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);
}

static int ar0822_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0822 *sensor = to_ar0822(sd);
	int ret = 0;

	mutex_lock(&sensor->mutex);
	if (sensor->streaming == enable) {
		mutex_unlock(&sensor->mutex);
		return 0;
	}

	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = ar0822_start_streaming(sensor);
		if (ret)
			goto err_start_streaming;
	} else {
		ar0822_stop_streaming(sensor);
	}

	sensor->streaming = enable;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(sensor->vflip, enable);
	__v4l2_ctrl_grab(sensor->hflip, enable);

	mutex_unlock(&sensor->mutex);

	return ret;

err_start_streaming:
	mutex_unlock(&sensor->mutex);

	return ret;
}

/* Get bayer order based on flip setting. */
static u32 ar0822_get_format_code(struct ar0822 *sensor,
				  enum ar0822_bit_depth bit_depth)
{
	u32 i, format_code;

	lockdep_assert_held(&sensor->mutex);

	i = (sensor->vflip->val ? 2 : 0) | (sensor->hflip->val ? 1 : 0);

	switch (bit_depth) {
	case AR0822_BIT_DEPTH_10BIT:
		format_code = ar0822_format_codes_10bit[i];
		break;
	case AR0822_BIT_DEPTH_12BIT:
		format_code = ar0822_format_codes_12bit[i];
		break;
	default:
		dev_err(sensor->dev, "Unsupported bit depth: %d\n", bit_depth);
		return ar0822_format_codes_10bit[i]; // default to 10-bit
	}

	dev_dbg(sensor->dev, "%s: format code %x\n", __func__, format_code);

	return format_code;
}

static int ar0822_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	pr_info("%s %d\n", __func__, code->index);

	struct ar0822 *sensor = to_ar0822(sd);

	if (code->index >= AR0822_BIT_DEPTH_AMOUNT)
		return -EINVAL;

	code->code = ar0822_get_format_code(sensor, code->index);
	pr_info("%s: index %d code %x\n", __func__, code->index, code->code);

	return 0;
}

static enum ar0822_bit_depth ar0822_fmt_code_to_bit_depth(u32 code)
{
	enum ar0822_bit_depth bit_depth;

	switch (code) {
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		bit_depth = AR0822_BIT_DEPTH_10BIT;
		break;
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		bit_depth = AR0822_BIT_DEPTH_12BIT;
		break;
	default:
		bit_depth = AR0822_BIT_DEPTH_AMOUNT;
		break;
	}

	return bit_depth;
}

static int ar0822_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ar0822 *sensor = to_ar0822(sd);
	struct ar0822_mode_lut const *supported_modes =
		sensor->hw_config.pll_config->supported_modes;

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad != IMAGE_PAD)
		return -EINVAL;

	if (fse->index >= sensor->hw_config.pll_config->modes_amount)
		return -EINVAL;

	enum ar0822_bit_depth bit_depth =
		ar0822_fmt_code_to_bit_depth(fse->code);

	if (fse->code != ar0822_get_format_code(sensor, bit_depth)) {
		dev_err(sensor->dev, "Unsupported format code %x\n", fse->code);
		return -EINVAL;
	}

	dev_dbg(sensor->dev, "%s: index %d code %x\n", __func__, fse->index,
		fse->code);

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void ar0822_set_default_format(struct ar0822 *sensor)
{
	/* Set default mode to max resolution */
	sensor->mode.info = &sensor->hw_config.pll_config->supported_modes[0];
	sensor->mode.bit_depth = AR0822_BIT_DEPTH_10BIT;
	sensor->fmt_code = ar0822_format_codes_10bit[0];
}

static void ar0822_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true, fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void ar0822_update_image_pad_format(struct ar0822 *sensor,
					   const struct ar0822_mode_lut *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	ar0822_reset_colorspace(&fmt->format);
}

static void ar0822_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = AR0822_EMBEDDED_LINE_WIDTH;
	fmt->format.height = AR0822_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int ar0822_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct ar0822 *sensor = to_ar0822(sd);
	int ret = 0;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	dev_dbg(sensor->dev, "%s: pad %d which %d\n", __func__, fmt->pad,
		fmt->which);

	mutex_lock(&sensor->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_state_get_format(sd_state, fmt->pad);
		/* update the code which could change due to vflip or hflip: */

		if (fmt->pad != IMAGE_PAD) {
			ret = -EINVAL;
			goto mutex_release;
		}

		dev_dbg(sensor->dev, "%s: try fmt w %d h %d code %x\n",
			__func__, try_fmt->width, try_fmt->height,
			try_fmt->code);

		enum ar0822_bit_depth bit_depth =
			ar0822_fmt_code_to_bit_depth(try_fmt->code);

		try_fmt->code = ar0822_get_format_code(sensor, bit_depth);
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad != IMAGE_PAD) {
			ret = -EINVAL;
			goto mutex_release;
		}

		ar0822_update_image_pad_format(sensor, sensor->mode.info, fmt);
		fmt->format.code =
			ar0822_get_format_code(sensor, sensor->mode.bit_depth);
		dev_dbg(sensor->dev, "%s: active fmt w %d h %d code %x\n",
			__func__, fmt->format.width, fmt->format.height,
			fmt->format.code);
	}

mutex_release:
	mutex_unlock(&sensor->mutex);
	return ret;
}

static int ar0822_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct ar0822 *sensor = to_ar0822(sd);
	struct ar0822_mode_lut const *mode;
	struct v4l2_mbus_framefmt *framefmt;
	enum ar0822_bit_depth bit_depth;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	dev_dbg(sensor->dev, "%s: req w %d h %d format code %x\n", __func__,
		fmt->format.width, fmt->format.height, fmt->format.code);

	bit_depth = ar0822_fmt_code_to_bit_depth(fmt->format.code);
	if (bit_depth == AR0822_BIT_DEPTH_AMOUNT) {
		dev_err(sensor->dev, "Unsupported format code: %x\n",
			fmt->format.code);
		return -EINVAL;
	}

	mutex_lock(&sensor->mutex);

	fmt->format.code = ar0822_get_format_code(sensor, bit_depth);

	mode = v4l2_find_nearest_size(
		sensor->hw_config.pll_config->supported_modes,
		sensor->hw_config.pll_config->modes_amount, width, height,
		fmt->format.width, fmt->format.height);

	dev_dbg(sensor->dev, "%s: found mode w %d h %d\n", __func__,
		mode->width, mode->height);

	ar0822_update_image_pad_format(sensor, mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_state_get_format(state, fmt->pad);
		*framefmt = fmt->format;
	} else if ((sensor->mode.info != mode) ||
		   (sensor->mode.bit_depth != bit_depth) ||
		   (sensor->fmt_code != fmt->format.code)) {
		sensor->mode.info = mode;
		sensor->mode.bit_depth = bit_depth;
		sensor->fmt_code = fmt->format.code;
		dev_dbg(sensor->dev,
			"%s: set mode %dx%d bit_depth %d code %x\n", __func__,
			sensor->mode.info->width, sensor->mode.info->height,
			sensor->mode.bit_depth, sensor->fmt_code);
		ar0822_set_framing_limits(sensor);
	}

	mutex_unlock(&sensor->mutex);

	return 0;
}

static const struct v4l2_rect *
__ar0822_get_pad_crop(struct ar0822 *sensor, struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->mode.info->crop;
	}

	return NULL;
}

static int ar0822_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct ar0822 *sensor = to_ar0822(sd);

		mutex_lock(&sensor->mutex);
		sel->r = *__ar0822_get_pad_crop(sensor, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&sensor->mutex);

		return 0;
	}
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = AR0822_PIXEL_NATIVE_WIDTH; // TODO: check
		sel->r.height = AR0822_PIXEL_NATIVE_HEIGHT;

		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = AR0822_PIXEL_ARRAY_WIDTH;
		sel->r.height = AR0822_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_subdev_core_ops ar0822_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops ar0822_subdev_video_ops = {
	.s_stream = ar0822_set_stream,
};

static const struct v4l2_subdev_pad_ops ar0822_subdev_pad_ops = {
	.enum_mbus_code = ar0822_enum_mbus_code,
	.enum_frame_size = ar0822_enum_frame_size,
	.get_fmt = ar0822_get_pad_format,
	.set_fmt = ar0822_set_pad_format,
	.get_selection = ar0822_get_selection,
};

static const struct v4l2_subdev_ops ar0822_subdev_ops = {
	.core = &ar0822_core_ops,
	.video = &ar0822_subdev_video_ops,
	.pad = &ar0822_subdev_pad_ops,
};

static void ar0822_free_controls(struct ar0822 *sensor)
{
	v4l2_ctrl_handler_free(&sensor->ctrl_hdlr);
	mutex_destroy(&sensor->mutex);
}

static int ar0822_subdev_init(struct ar0822 *sensor)
{
	int ret;

	dev_dbg(sensor->dev, "%s\n", __func__);

	ret = ar0822_ctrls_init(sensor);
	if (ret)
		return ret;

	sensor->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->subdev.entity, 1, &sensor->pad);
	if (ret < 0) {
		dev_err(sensor->dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&sensor->subdev);
	if (ret < 0) {
		dev_err(sensor->dev,
			"failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&sensor->subdev.entity);

error_handler_free:
	ar0822_free_controls(sensor);

	return ret;
}

static int ar0822_power_on(struct ar0822 *sensor)
{
	int ret;
	struct ar0822_hw_config *hw_config = &sensor->hw_config;

	dev_dbg(sensor->dev, "%s\n", __func__);

	ret = regulator_bulk_enable(AR0822_SUPPLY_AMOUNT, hw_config->supplies);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(hw_config->extclk);
	if (ret < 0)
		goto err_reset;

	gpiod_set_value_cansleep(hw_config->gpio_reset, 1);

	usleep_range(AR0822_RESET_MIN_DELAY_US,
		     AR0822_RESET_MAX_DELAY_US); // TODO this can be reduced

	return 0;

err_reset:
	gpiod_set_value_cansleep(hw_config->gpio_reset, 0);
	regulator_bulk_disable(AR0822_SUPPLY_AMOUNT, hw_config->supplies);
	return ret;
}

static void ar0822_power_off(struct ar0822 *sensor)
{
	dev_dbg(sensor->dev, "%s\n", __func__);
	clk_disable_unprepare(sensor->hw_config.extclk);
	gpiod_set_value_cansleep(sensor->hw_config.gpio_reset, 0);
	regulator_bulk_disable(AR0822_SUPPLY_AMOUNT,
			       sensor->hw_config.supplies);
}

static int ar0822_identify_model(struct ar0822 *sensor)
{
	int ret;
	u64 model_id;

	ret = cci_read(sensor->regmap, AR0822_REG_CHIP_VERSION, &model_id,
		       NULL);
	if (ret < 0) {
		dev_err_probe(sensor->dev, ret,
			      "failed to read sensor information\n");
		return ret;
	}

	if (model_id != AR0822_MODEL_ID) {
		dev_err(sensor->dev, "invalid model id 0x%04llx\n", model_id);
		return -ENODEV;
	}

	dev_info(sensor->dev, "Detected AR0822 image sensor\n");

	return ret;
}

static int ar0822_parse_hw_config(struct ar0822 *sensor)
{
	struct v4l2_fwnode_endpoint endpoint_config = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *endpoint;
	struct ar0822_hw_config *hw_config = &sensor->hw_config;
	unsigned long extclk_frequency;
	unsigned int i;
	int ret;

	dev_dbg(sensor->dev, "parsing hardware configuration\n");

	// Get the regulators
	for (i = 0; i < AR0822_SUPPLY_AMOUNT; i++)
		hw_config->supplies[i].supply = ar0822_supply_names[i];

	ret = devm_regulator_bulk_get(sensor->dev, AR0822_SUPPLY_AMOUNT,
				      hw_config->supplies);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to get supplies\n");

	// Get the reset GPIO
	hw_config->gpio_reset =
		devm_gpiod_get_optional(sensor->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(hw_config->gpio_reset))
		return dev_err_probe(sensor->dev,
				     PTR_ERR(hw_config->gpio_reset),
				     "failed to get reset GPIO\n");

	// Get EXTCLK
	hw_config->extclk = devm_clk_get(sensor->dev, "extclk");
	if (IS_ERR(hw_config->extclk))
		return dev_err_probe(sensor->dev, PTR_ERR(hw_config->extclk),
				     "failed to get EXTCLK\n");

	endpoint =
		fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);
	if (!endpoint) {
		dev_err(sensor->dev, "endpoint node not found\n");
		return -ENXIO;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &endpoint_config);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(sensor->dev, "failed to parse endpoint\n");
		return ret;
	}

	// TODO: check
	switch (endpoint_config.bus.mipi_csi2.num_data_lanes) {
	case 2:
		hw_config->lane_mode = AR0822_LANE_MODE_2;
		hw_config->num_data_lanes =
			endpoint_config.bus.mipi_csi2.num_data_lanes;
		break;
	case 4:
		hw_config->lane_mode = AR0822_LANE_MODE_4;
		hw_config->num_data_lanes =
			endpoint_config.bus.mipi_csi2.num_data_lanes;
		break;
	default:
		ret = dev_err_probe(
			sensor->dev, -EINVAL,
			"invalid number of CSI2 data lanes %d\n",
			endpoint_config.bus.mipi_csi2.num_data_lanes);
		goto done_endpoint_free;
	}

	if (!endpoint_config.nr_of_link_frequencies) {
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "no link frequencies defined");
		goto done_endpoint_free;
	}

	/*
	 * Check if there exists a sensor mode defined for current EXTCLK,
	 * number of lanes and given lane rate.
	 */
	extclk_frequency = clk_get_rate(hw_config->extclk);

	for (i = 0; i < ARRAY_SIZE(ar0822_pll_configs); i++) {
		if ((*ar0822_pll_configs[i].extclk_freq == extclk_frequency) &&
		    (*ar0822_pll_configs[i].link_freq ==
		     endpoint_config.link_frequencies[0]))
			break;
	}

	if (i == ARRAY_SIZE(ar0822_pll_configs)) {
		ret = dev_err_probe(
			sensor->dev, -EINVAL,
			"no valid sensor mode defined for EXTCLK %lu Hz and link frequency %llu bps\n",
			extclk_frequency, endpoint_config.link_frequencies[0]);
		goto done_endpoint_free;
	}

	hw_config->pll_config = &ar0822_pll_configs[i];

	ret = 0;
	dev_dbg(sensor->dev,
		"clock: %lu Hz, link_frequency: %llu bps, lanes: %d\n",
		extclk_frequency, *hw_config->pll_config->link_freq,
		hw_config->num_data_lanes);

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&endpoint_config);

	return ret;
}

static int ar0822_probe(struct i2c_client *client)
{
	struct ar0822 *sensor;
	int ret;

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = &client->dev;

	dev_dbg(sensor->dev, "probing AR0822 sensor\n");

	v4l2_i2c_subdev_init(&sensor->subdev, client, &ar0822_subdev_ops);

	ret = ar0822_parse_hw_config(sensor);
	if (ret)
		return ret;

	sensor->regmap =
		devm_cci_regmap_init_i2c(client, AR0822_REG_ADDRESS_BITS);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	/*
	 * Enable power management. The driver supports runtime PM, but needs to
	 * work when runtime PM is disabled in the kernel. To that end, power
	 * the sensor on manually here, identify it, and fully initialize it.
	 */
	ret = ar0822_power_on(sensor);
	if (ret)
		return ret;

	pm_runtime_set_active(sensor->dev);
	pm_runtime_get_noresume(sensor->dev);
	pm_runtime_enable(sensor->dev);
	pm_runtime_set_autosuspend_delay(sensor->dev, 1000);
	pm_runtime_use_autosuspend(sensor->dev);

	ret = ar0822_identify_model(sensor);
	if (ret)
		goto err_power_off;

	/* Initialize default format */
	ar0822_set_default_format(sensor);

	ret = ar0822_subdev_init(sensor);
	if (ret)
		goto err_power_off;

	/*
	 * Finally, enable autosuspend and decrease the usage count. The device
	 * will get suspended after the autosuspend delay, turning the power
	 * off.
	 */
	pm_runtime_mark_last_busy(sensor->dev);
	pm_runtime_put_autosuspend(sensor->dev);

	dev_dbg(sensor->dev, "AR0822 sensor probed successfully\n");

	return 0;

err_power_off:
	pm_runtime_disable(sensor->dev);
	pm_runtime_put_noidle(sensor->dev);
	ar0822_power_off(sensor);

	return ret;
}

static void ar0822_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ar0822 *sensor = to_ar0822(subdev);

	v4l2_async_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);
	ar0822_free_controls(sensor);

	/*
	 * Disable runtime PM. In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	 */
	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		ar0822_power_off(sensor);
	pm_runtime_set_suspended(sensor->dev);
}

static int ar0822_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ar0822 *sensor = to_ar0822(subdev);

	return ar0822_power_on(sensor);
}

static int ar0822_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ar0822 *sensor = to_ar0822(subdev);

	ar0822_power_off(sensor);

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(ar0822_pm_ops, ar0822_runtime_suspend,
				 ar0822_runtime_resume, NULL);

static const struct of_device_id ar0822_of_match[] = {
	{ .compatible = "onnn,ar0822" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ar0822_of_match);

static struct i2c_driver ar0822_driver = {
	.probe = ar0822_probe,
	.remove = ar0822_remove,
	.driver = {
		.name = "ar0822",
		.of_match_table = ar0822_of_match,
		.pm = pm_ptr(&ar0822_pm_ops),
	},
};

module_i2c_driver(ar0822_driver);

MODULE_DESCRIPTION("OnSemi AR0822 image sensor driver");
MODULE_AUTHOR("Danius Kalvaitis <danius@kurokesu.com>");
MODULE_LICENSE("GPL v2");
