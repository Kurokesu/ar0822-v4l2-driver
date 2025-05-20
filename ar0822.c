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
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define AR0822_PIXEL_RATE 320000000
#define AR0822_REG_ADDRESS_BITS 16

#define AR0822_EMBEDDED_LINE_WIDTH 16384
#define AR0822_NUM_EMBEDDED_LINES 2

#define AR0822_VBLANK_MIN 16
#define AR0822_VTS_MAX 0xFFFF
#define AR0822_VTS_30FPS 0x4C4
#define AR0822_EXPOSURE_DEFAULT 0x0640
#define AR0822_PPL_DEFAULT 3840

#define AR0822_RESET_MIN_DELAY_US 7000
#define AR0822_RESET_MAX_DELAY_US (AR0822_RESET_MIN_DELAY_US + 1000)

#define AR0822_PIXEL_NATIVE_WIDTH 3840
#define AR0822_PIXEL_NATIVE_HEIGHT 2160
#define AR0822_PIXEL_ARRAY_WIDTH 3840
#define AR0822_PIXEL_ARRAY_HEIGHT 2160
#define AR0822_PIXEL_ARRAY_TOP 0
#define AR0822_PIXEL_ARRAY_LEFT 0

#define AR0822_PIXEL_ARRAY_VBLANK 2184 // TODO check

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

#define AR0822_REG_CHIP_VERSION CCI_REG16(0x3000)
#define AR0822_REG_FRAME_LENGTH_LINES CCI_REG16(0x300A)
#define AR0822_REG_COARSE_INTEGRATION_TIME CCI_REG16(0x3012)
#define AR0822_REG_RESET CCI_REG16(0x301A)
#define AR0822_REG_MODE_SELECT CCI_REG8(0x301C)
#define AR0822_REG_IMAGE_ORIENTATION CCI_REG8(0x301D)
#define AR0822_REG_SENSOR_GAIN CCI_REG8(0x5900)

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

#define AR0822_REG_DARK_CONTROL CCI_REG16(0x3044)
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
#define AR0822_REG_FRAME_PREAMBLE CCI_REG8(0x31B0)
#define AR0822_REG_LINE_PREAMBLE CCI_REG8(0x31B2)
#define AR0822_REG_MIPI_TIMING_0 CCI_REG16(0x31B4)
#define AR0822_REG_MIPI_TIMING_1 CCI_REG16(0x31B6)
#define AR0822_REG_MIPI_TIMING_2 CCI_REG16(0x31B8)
#define AR0822_REG_MIPI_TIMING_3 CCI_REG16(0x31BA)
#define AR0822_REG_MIPI_TIMING_4 CCI_REG16(0x31BC)
#define AR0822_REG_HISPI_CONTROL_MODE CCI_REG16(0x31C6)
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

enum pad_types { IMAGE_PAD, METADATA_PAD, NUM_PADS };

static const char *const ar0822_supply_names[] = {
	"vana", /* Analog (2.8V) supply */
	"vdig", /* Digital Core (1.8V) supply */
	"vddl", /* IF (1.2V) supply */
};

#define AR0822_SUPPLY_AMOUNT ARRAY_SIZE(ar0822_supply_names)

static const s64 link_freq_menu_items[] = {
	480000000,
};

struct ar0822_clk_params {
	u64 link_frequency;
	u64 extclk_frequency;
};

/* EXTCLK Settings - includes all lane rate and EXTCLK dependent registers */
static const struct ar0822_clk_params ar0822_clk_params[] = {
	{
		.link_frequency = 480000000UL,
		.extclk_frequency = 24000000,
	},
};

/* Example mode register list (replace with actual values for your mode) */
static const struct cci_reg_sequence ar0822_link_480mbps[] = {
	// TODO: Fill with actual mode registers for 1920x1080@30fps, 2-lane, 480Mbps
	// { AR0822_REG_X_ADDR_START, 0x0000 },
	// { AR0822_REG_X_ADDR_END,   0x077F }, // 1920 width
	// { AR0822_REG_Y_ADDR_START, 0x0000 },
	// { AR0822_REG_Y_ADDR_END,   0x0437 }, // 1080 height
};

struct ar0822_mode_reg_list {
	u32 num_of_regs;
	const struct cci_reg_sequence *regs;
};

struct ar0822_mode {
	u64 link_frequency;
	unsigned int width;
	unsigned int height;
	struct v4l2_rect crop;
	/* V-timing */
	unsigned int vts_def;
	struct ar0822_mode_reg_list reg_list;
};

static const struct ar0822_mode ar0822_supported_modes[] = {
	{
		.link_frequency = 480000000,
		.width = 1920,
		.height = 1080,
		.crop = {
			.left = AR0822_PIXEL_ARRAY_LEFT,
			.top = AR0822_PIXEL_ARRAY_TOP,
			.width = 1920,
			.height = 1080,
		},
		.vts_def = AR0822_VTS_30FPS,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(ar0822_link_480mbps),
			.regs = ar0822_link_480mbps,
		},
	},
};

static const char *const ar0822_test_pattern_menu[] = {
	"disabled",
	"solid black",
	"solid white",
	"solid dark gray",
	"solid light gray",
	"stripes light/dark grey",
	"stripes dark/light grey",
	"stripes black/dark grey",
	"stripes dark grey/black",
	"stripes black/white",
	"stripes white/black",
	"horizontal color bar",
	"vertical color bar",
};

struct ar0822 {
	struct device *dev;
	struct clk *extclk;
	unsigned long pixel_rate;
	struct regulator_bulk_data supplies[AR0822_SUPPLY_AMOUNT];
	struct gpio_desc *reset;
	struct regmap *regmap;

	unsigned int fmt_code;

	const struct ar0822_clk_params *clk_params;

	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *exposure;

	unsigned int cur_mode;
	unsigned int num_data_lanes;

	struct mutex mutex;
	const struct ar0822_mode *mode;
};

/*
 * This table includes fixed register settings and a bunch of undocumented
 * registers that have to be set to another value than default.
 */
static const struct cci_reg_sequence ar0822_init_table[] = {
	/* PLL setup */
	{ AR0822_REG_VT_PIX_CLK_DIV, 0x0003 },
	{ AR0822_REG_VT_SYS_CLK_DIV, 0x0001 },
	{ AR0822_REG_PRE_PLL_CLK_DIV, 0x0001 },
	{ AR0822_REG_PLL_MULTIPLIER, 0x0014 },
	{ AR0822_REG_OP_WORD_CLK_DIV, 0x0006 },
	{ AR0822_REG_OP_SYS_CLK_DIV, 0x0001 },
	{ AR0822_REG_PLL_CONTROL, 0x0003 },

	{ AR0822_REG_DIGITAL_TEST, 0x0800 }, // default
	{ AR0822_REG_X_ADDR_START, 0x03C8 }, // 968
	{ AR0822_REG_X_ADDR_END, 0x0B47 }, // 2887
	{ AR0822_REG_X_ODD_INC, 0x0001 }, // default no skip
	{ AR0822_REG_Y_ODD_INC, 0x0001 }, //default no skip
	{ AR0822_REG_X_OUTPUT_CONTROL, 0x0780 }, // 1920 disabled
	{ AR0822_REG_Y_OUTPUT_CONTROL, 0x0438 }, // 1080 disabled
	/* read mode */
	{ AR0822_REG_DARK_CONTROL, 0x0000 },
	/* operation mode */
	{ AR0822_REG_DIGITAL_CTRL, 0x0003 },
	{ AR0822_REG_COMPANDING, 0x0000 }, // default
	{ AR0822_REG_SERIAL_FORMAT, 0x0202 }, // 2 lane MIPI
	{ AR0822_REG_DATA_FORMAT_BITS, 0x0C0C }, // 12bit / 12bit raw
	{ AR0822_REG_FRAME_LENGTH_LINES, 0x0450 }, // 1104
	{ AR0822_REG_LINE_LENGTH_PCK, 0x096E }, // 2414
	{ AR0822_REG_COARSE_INTEGRATION_TIME, 0x0176 },
	{ AR0822_REG_COARSE_INTEGRATION_TIME2, 0x0000 },
	{ AR0822_REG_COARSE_INTEGRATION_TIME3, 0x0000 },
	{ AR0822_REG_EXPOSURE_RATIO, 0x0022 }, // default
	{ AR0822_REG_SHUT_CTRL2, 0x72A0 }, // unknown
	{ AR0822_REG_MEC_CTRL2, 0x0000 }, // unknown
	/* MIPI timings */
	{ AR0822_REG_FRAME_PREAMBLE, 0x006C },
	{ AR0822_REG_LINE_PREAMBLE, 0x004A },
	{ AR0822_REG_MIPI_TIMING_0, 0x51C8 },
	{ AR0822_REG_MIPI_TIMING_1, 0x5248 },
	{ AR0822_REG_MIPI_TIMING_2, 0x70CA },
	{ AR0822_REG_MIPI_TIMING_3, 0x028A },
	{ AR0822_REG_MIPI_TIMING_4, 0x0C08 },
	{ AR0822_REG_HISPI_CONTROL_MODE, 0x8000 }, // default
	{ AR0822_REG_MIPI_DESKEW_PAT_WIDTH, 0x0AEC },
	{ AR0822_REG_MIPI_PER_DESKEW_PAT_WIDTH, 0x00A7 },
	{ AR0822_REG_MIPI_HISPI_TRIM, 0x0000 }, // default
	{ AR0822_REG_MIPI_CONFIG_2, 0x0000 },
	{ AR0822_REG_MIPI_F1_PDT, 0x002C },
	{ AR0822_REG_MIPI_F1_VC, 0x0000 },
	{ AR0822_REG_MIPI_F2_PDT, 0x002C },
	{ AR0822_REG_MIPI_F2_VC, 0x0000 },
	{ AR0822_REG_MIPI_F3_PDT, 0x002C },
	{ AR0822_REG_MIPI_F3_VC, 0x0000 },
	{ AR0822_REG_MIPI_F4_PDT, 0x002C },
	{ AR0822_REG_MIPI_F4_VC, 0x0000 },
	// /* use all-pixel readout mode, no flip */
	// { IMX415_WINMODE, 0x00 },
	// { IMX415_ADDMODE, 0x00 },
	// { IMX415_REVERSE, 0x00 },
	// /* use RAW 10-bit mode */
	// { IMX415_ADBIT, 0x00 },
	// { IMX415_MDBIT, 0x00 },
	// /* output VSYNC on XVS and low on XHS */
	// { IMX415_OUTSEL, 0x22 },
	// { IMX415_DRV, 0x00 },

	// /* SONY magic registers */
	// { CCI_REG8(0x32D4), 0x21 },
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

static int ar0822_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0822 *sensor =
		container_of(ctrl->handler, struct ar0822, ctrls);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	u32 exposure_max;
	unsigned int vmax;
	int ret;

	dev_dbg(sensor->dev, "ar0822_s_ctrl: %d %d\n", ctrl->id, ctrl->val);

	state = v4l2_subdev_get_locked_active_state(&sensor->subdev);
	format = v4l2_subdev_state_get_format(state, 0);

	if (ctrl->id == V4L2_CID_VBLANK) {
		dev_dbg(sensor->dev, "ar0822_s_ctrl: VBLANK %d\n", ctrl->val);
		dev_dbg(sensor->dev, "ar0822_s_ctrl: format height %d\n",
			format->height);
		exposure_max = format->height + ctrl->val - AR0822_EXPOSURE_MIN;
		dev_dbg(sensor->dev, "ar0822_s_ctrl: exposure_max %d\n",
			exposure_max);
		__v4l2_ctrl_modify_range(sensor->exposure,
					 sensor->exposure->minimum,
					 exposure_max, sensor->exposure->step,
					 sensor->exposure->default_value);
	}

	if (!pm_runtime_get_if_in_use(sensor->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		dev_dbg(sensor->dev, "ar0822_s_ctrl: VBLANK %d\n",
			format->height + ctrl->val);
		ret = cci_write(sensor->regmap, AR0822_REG_FRAME_LENGTH_LINES,
				format->height + ctrl->val, NULL);
		if (ret)
			return ret;
		/*
		 * Deliberately fall through as exposure is set based on VMAX
		 * which has just changed.
		 */
		fallthrough;
	case V4L2_CID_EXPOSURE:
		/* clamp the exposure value to VMAX. */
		vmax = format->height + sensor->vblank->cur.val;
		ctrl->val = min_t(int, ctrl->val, vmax);
		dev_dbg(sensor->dev,
			"ar0822_s_ctrl: AR0822_REG_COARSE_INTEGRATION_TIME %d\n",
			vmax - ctrl->val);
		ret = cci_write(sensor->regmap,
				AR0822_REG_COARSE_INTEGRATION_TIME,
				vmax - ctrl->val, NULL);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(sensor->dev,
			"ar0822_s_ctrl: AR0822_REG_SENSOR_GAIN %d\n",
			ctrl->val);
		ret = cci_write(sensor->regmap, AR0822_REG_SENSOR_GAIN,
				ctrl->val, NULL);
		break;

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP: {
		u32 flip = (sensor->hflip->val
			    << AR0822_IMAGE_ORIENTATION_HFLIP_BIT) |
			   (sensor->vflip->val
			    << AR0822_IMAGE_ORIENTATION_VFLIP_BIT);
		dev_dbg(sensor->dev,
			"ar0822_s_ctrl: AR0822_REG_IMAGE_ORIENTATION %d\n",
			flip);
		ret = cci_write(sensor->regmap, AR0822_REG_IMAGE_ORIENTATION,
				flip, NULL);
		break;
	}

		// case V4L2_CID_TEST_PATTERN:
		// 	ret = ar0822_set_testpattern(sensor, ctrl->val);
		// 	break;

	default:
		dev_err(sensor->dev, "unhandled control %d\n", ctrl->id);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(sensor->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ar0822_ctrl_ops = {
	.s_ctrl = ar0822_s_ctrl,
};

static int ar0822_ctrls_init(struct ar0822 *sensor)
{
	dev_dbg(sensor->dev, "initializing controls\n");
	dev_dbg(sensor->dev, "current mode: %d\n", sensor->cur_mode);

	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	u64 link_frequency =
		ar0822_supported_modes[sensor->cur_mode].link_frequency;
	unsigned int height = sensor->mode->height;
	u32 exposure_max = AR0822_PIXEL_ARRAY_HEIGHT +
			   AR0822_PIXEL_ARRAY_VBLANK - AR0822_EXPOSURE_MIN;
	u32 hblank, exposure_def;
	unsigned int i;
	int ret;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 10);

	for (i = 0; i < ARRAY_SIZE(link_freq_menu_items); i++) {
		if (link_frequency == link_freq_menu_items[i])
			break;
	}

	if (i == ARRAY_SIZE(link_freq_menu_items)) {
		return dev_err_probe(sensor->dev, -EINVAL,
				     "link frequency %llu not supported\n",
				     link_frequency);
	}

	dev_dbg(sensor->dev, "link frequency: %llu\n", link_frequency);

	ctrl = v4l2_ctrl_new_int_menu(&sensor->ctrls, &ar0822_ctrl_ops,
				      V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(link_freq_menu_items) - 1, i,
				      link_freq_menu_items);

	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	exposure_max = sensor->mode->vts_def - 4;
	exposure_def = (exposure_max < AR0822_EXPOSURE_DEFAULT) ?
			       exposure_max :
			       AR0822_EXPOSURE_DEFAULT;
	sensor->exposure = v4l2_ctrl_new_std(&sensor->ctrls, &ar0822_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     AR0822_EXPOSURE_MIN, exposure_max,
					     AR0822_EXPOSURE_STEP,
					     exposure_def);

	v4l2_ctrl_new_std(&sensor->ctrls, &ar0822_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, AR0822_ANA_GAIN_MIN,
			  AR0822_ANA_GAIN_MAX, AR0822_ANA_GAIN_STEP,
			  AR0822_ANA_GAIN_MIN);

	hblank = AR0822_PPL_DEFAULT - sensor->mode->width;
	ctrl = v4l2_ctrl_new_std(&sensor->ctrls, &ar0822_ctrl_ops,
				 V4L2_CID_HBLANK, hblank, hblank, 1, hblank);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->vblank = v4l2_ctrl_new_std(&sensor->ctrls, &ar0822_ctrl_ops,
					   V4L2_CID_VBLANK, AR0822_VBLANK_MIN,
					   AR0822_VTS_MAX - height, 1,
					   sensor->mode->vts_def - height);

	ctrl = v4l2_ctrl_new_std(&sensor->ctrls, NULL, V4L2_CID_PIXEL_RATE,
				 sensor->pixel_rate, sensor->pixel_rate, 1,
				 sensor->pixel_rate);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->hflip = v4l2_ctrl_new_std(&sensor->ctrls, &ar0822_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	sensor->vflip = v4l2_ctrl_new_std(&sensor->ctrls, &ar0822_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	// v4l2_ctrl_new_std_menu_items(&sensor->ctrls, &ar0822_ctrl_ops,
	// 			     V4L2_CID_TEST_PATTERN,
	// 			     ARRAY_SIZE(ar0822_test_pattern_menu) - 1,
	// 			     0, 0, ar0822_test_pattern_menu);

	v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &ar0822_ctrl_ops,
					&props);

	if (sensor->ctrls.error) {
		dev_err_probe(sensor->dev, sensor->ctrls.error,
			      "failed to add controls\n");
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return sensor->ctrls.error;
	}
	sensor->subdev.ctrl_handler = &sensor->ctrls;

	return 0;
}

static int ar0822_set_mode(struct ar0822 *sensor, int mode)
{
	int ret = 0;

	dev_dbg(sensor->dev, "%s: setting mode %d\n", __func__, mode);

	if (mode >= ARRAY_SIZE(ar0822_supported_modes)) {
		dev_err(sensor->dev, "Mode %d not supported\n", mode);
		return -EINVAL;
	}

	if (ar0822_supported_modes[mode].reg_list.num_of_regs) {
		ret = cci_multi_reg_write(
			sensor->regmap,
			ar0822_supported_modes[mode].reg_list.regs,
			ar0822_supported_modes[mode].reg_list.num_of_regs,
			NULL);
		if (ret)
			return ret;
	}

	return ret;
}

static int ar0822_setup(struct ar0822 *sensor, struct v4l2_subdev_state *state)
{
	int ret;

	dev_dbg(sensor->dev, "%s: setting up sensor\n", __func__);

	ret = cci_multi_reg_write(sensor->regmap, ar0822_init_table,
				  ARRAY_SIZE(ar0822_init_table), NULL);
	if (ret)
		return ret;

	return ar0822_set_mode(sensor, sensor->cur_mode);
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

static int ar0822_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0822 *sensor = to_ar0822(sd);
	struct v4l2_subdev_state *state;
	int ret;

	pr_info("%s enable %d\n", __func__, enable);

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (!enable) {
		ret = ar0822_mode_stream_off(sensor);

		pm_runtime_mark_last_busy(sensor->dev);
		pm_runtime_put_autosuspend(sensor->dev);

		goto unlock;
	}

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		goto unlock;

	ret = ar0822_setup(sensor, state);
	if (ret)
		goto err_pm;

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret < 0)
		goto err_pm;

	ret = ar0822_mode_stream_on(sensor);
	if (ret)
		goto err_pm;

	ret = 0;

unlock:
	v4l2_subdev_unlock_state(state);

	return ret;

err_pm:
	/*
	 * In case of error, turn the power off synchronously as the device
	 * likely has no other chance to recover.
	 */
	pm_runtime_put_sync(sensor->dev);

	goto unlock;
}

static int ar0822_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	pr_info("%s\n", __func__);

	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG12_1X12;

	return 0;
}

static int ar0822_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct v4l2_mbus_framefmt *format;

	pr_info("%s\n", __func__);

	format = v4l2_subdev_state_get_format(state, fse->pad);

	if (fse->index > 0 || fse->code != format->code)
		return -EINVAL;

	fse->min_width = AR0822_PIXEL_ARRAY_WIDTH;
	fse->max_width = fse->min_width;
	fse->min_height = AR0822_PIXEL_ARRAY_HEIGHT;
	fse->max_height = fse->min_height;
	return 0;
}

/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	/* 12-bit modes. */
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12, // TODO: check
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
};

/* Get bayer order based on flip setting. */
static u32 ar0822_get_format_code(struct ar0822 *sensor, u32 code)
{
	unsigned int i;

	lockdep_assert_held(&sensor->mutex);

	i = (sensor->vflip->val ? 2 : 0) | (sensor->hflip->val ? 1 : 0);

	return codes[i];
}

static void ar0822_set_default_format(struct ar0822 *sensor)
{
	/* Set default mode to max resolution */
	sensor->mode = &ar0822_supported_modes[0];
	sensor->fmt_code = MEDIA_BUS_FMT_SGRBG12_1X12;
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
					   const struct ar0822_mode *mode,
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
	int ret;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&sensor->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_state_get_format(sd_state, fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code =
			fmt->pad == IMAGE_PAD ?
				ar0822_get_format_code(sensor, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			ar0822_update_image_pad_format(sensor, sensor->mode,
						       fmt);
			fmt->format.code = ar0822_get_format_code(
				sensor, sensor->fmt_code);
		} else {
			ar0822_update_metadata_pad_format(fmt);
		}
	}
	mutex_unlock(&sensor->mutex);

	return ret;
}

static void ar0822_set_framing_limits(struct ar0822 *sensor)
{
	const struct ar0822_mode *mode = sensor->mode;
	int exposure_max, exposure_def, hblank;

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(sensor->vblank, AR0822_VBLANK_MIN,
				 AR0822_VTS_MAX - mode->height, 1,
				 mode->vts_def - mode->height);
	__v4l2_ctrl_s_ctrl(sensor->vblank, mode->vts_def - mode->height);
	/*
			* Update max exposure while meeting
			* expected vblanking
			*/
	exposure_max = mode->vts_def - 4;
	exposure_def = (exposure_max < AR0822_EXPOSURE_DEFAULT) ?
			       exposure_max :
			       AR0822_EXPOSURE_DEFAULT;
	__v4l2_ctrl_modify_range(sensor->exposure, sensor->exposure->minimum,
				 exposure_max, sensor->exposure->step,
				 exposure_def);
	/*
			* Currently PPL is fixed to AR0234_PPL_DEFAULT, so
			* hblank depends on mode->width only, and is not
			* changeble in any way other than changing the mode.
			*/
	hblank = AR0822_PPL_DEFAULT - mode->width;
	__v4l2_ctrl_modify_range(sensor->hblank, hblank, hblank, 1, hblank);
}

static int ar0822_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct ar0822 *sensor = to_ar0822(sd);
	const struct ar0822_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&sensor->mutex);

	if (fmt->pad == IMAGE_PAD) {
		fmt->format.code =
			ar0822_get_format_code(sensor, fmt->format.code);

		mode = v4l2_find_nearest_size(
			ar0822_supported_modes,
			ARRAY_SIZE(ar0822_supported_modes), width, height,
			fmt->format.width, fmt->format.height);
		ar0822_update_image_pad_format(sensor, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt =
				v4l2_subdev_state_get_format(state, fmt->pad);
			*framefmt = fmt->format;
		} else if (sensor->mode != mode ||
			   sensor->fmt_code != fmt->format.code) {
			sensor->mode = mode;
			sensor->fmt_code = fmt->format.code;
			ar0822_set_framing_limits(sensor);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt =
				v4l2_subdev_state_get_format(state, fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			ar0822_update_metadata_pad_format(fmt);
		}
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
		return &sensor->mode->crop;
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
		sel->r.top = AR0822_PIXEL_ARRAY_TOP;
		sel->r.left = AR0822_PIXEL_ARRAY_LEFT;
		sel->r.width = AR0822_PIXEL_ARRAY_WIDTH;
		sel->r.height = AR0822_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_subdev_video_ops ar0822_subdev_video_ops = {
	.s_stream = ar0822_s_stream,
};

static const struct v4l2_subdev_pad_ops ar0822_subdev_pad_ops = {
	.enum_mbus_code = ar0822_enum_mbus_code,
	.enum_frame_size = ar0822_enum_frame_size,
	.get_fmt = ar0822_get_pad_format,
	.set_fmt = ar0822_set_pad_format,
	.get_selection = ar0822_get_selection,
};

static const struct v4l2_subdev_ops ar0822_subdev_ops = {
	.video = &ar0822_subdev_video_ops,
	.pad = &ar0822_subdev_pad_ops,
};

static int ar0822_subdev_init(struct ar0822 *sensor)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int ret;

	dev_dbg(sensor->dev, "%s\n", __func__);

	v4l2_i2c_subdev_init(&sensor->subdev, client, &ar0822_subdev_ops);

	ret = ar0822_ctrls_init(sensor);
	if (ret)
		return ret;

	sensor->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->subdev.entity, 1, &sensor->pad);
	if (ret < 0) {
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return ret;
	}

	sensor->subdev.state_lock = sensor->subdev.ctrl_handler->lock;
	v4l2_subdev_init_finalize(&sensor->subdev);

	return 0;
}

static void ar0822_subdev_cleanup(struct ar0822 *sensor)
{
	media_entity_cleanup(&sensor->subdev.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
}

static int ar0822_power_on(struct ar0822 *sensor)
{
	int ret;

	dev_dbg(sensor->dev, "%s\n", __func__);

	ret = regulator_bulk_enable(AR0822_SUPPLY_AMOUNT, sensor->supplies);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(sensor->extclk);
	if (ret < 0)
		goto err_reset;

	gpiod_set_value_cansleep(sensor->reset, 1);

	usleep_range(AR0822_RESET_MIN_DELAY_US,
		     AR0822_RESET_MAX_DELAY_US); // TODO this can be reduced

	return 0;

err_reset:
	gpiod_set_value_cansleep(sensor->reset, 0);
	regulator_bulk_disable(AR0822_SUPPLY_AMOUNT, sensor->supplies);
	return ret;
}

static void ar0822_power_off(struct ar0822 *sensor)
{
	dev_dbg(sensor->dev, "%s\n", __func__);
	clk_disable_unprepare(sensor->extclk);
	gpiod_set_value_cansleep(sensor->reset, 0);
	regulator_bulk_disable(AR0822_SUPPLY_AMOUNT, sensor->supplies);
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

static int ar0822_check_extclk(unsigned long extclk_frequency,
			       u64 link_frequency)
{
	unsigned int i;

	// TODO: optimize
	for (i = 0; i < ARRAY_SIZE(ar0822_clk_params); i++) {
		if ((ar0822_clk_params[i].link_frequency == link_frequency) &&
		    ar0822_clk_params[i].extclk_frequency == extclk_frequency)
			break;
	}

	if (i == ARRAY_SIZE(ar0822_clk_params))
		return -EINVAL;

	return 0;
}

static int ar0822_parse_hw_config(struct ar0822 *sensor)
{
	struct v4l2_fwnode_endpoint endpoint_config = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *endpoint;
	u64 link_frequency;
	unsigned long extclk_frequency;
	unsigned int i, j;
	int ret;

	dev_dbg(sensor->dev, "parsing hardware configuration\n");

	// Get the regulators
	for (i = 0; i < AR0822_SUPPLY_AMOUNT; i++)
		sensor->supplies[i].supply = ar0822_supply_names[i];

	ret = devm_regulator_bulk_get(sensor->dev, AR0822_SUPPLY_AMOUNT,
				      sensor->supplies);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to get supplies\n");

	// Get the reset GPIO
	sensor->reset =
		devm_gpiod_get_optional(sensor->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->reset),
				     "failed to get reset GPIO\n");

	// Get EXTCLK
	sensor->extclk = devm_clk_get(sensor->dev, "extclk");
	if (IS_ERR(sensor->extclk))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->extclk),
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
	case 4:
		sensor->num_data_lanes =
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
	 * number of lanes and given lane rates.
	 */
	extclk_frequency = clk_get_rate(sensor->extclk);
	dev_dbg(sensor->dev, "EXTCLK frequency: %lu Hz, number of lanes: %d\n",
		extclk_frequency, sensor->num_data_lanes);

	for (i = 0; i < endpoint_config.nr_of_link_frequencies; i++) {
		if (ar0822_check_extclk(extclk_frequency,
					endpoint_config.link_frequencies[i])) {
			dev_dbg(sensor->dev,
				"EXTCLK %lu Hz not supported for this link freq",
				extclk_frequency);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(ar0822_supported_modes); j++) {
			if (endpoint_config.link_frequencies[i] !=
			    ar0822_supported_modes[j].link_frequency)
				continue;
			sensor->cur_mode = j;
			dev_dbg(sensor->dev, "set sensor mode %d\n", j);
			break;
		}

		if (j < ARRAY_SIZE(ar0822_supported_modes))
			break;
	}

	if (i == endpoint_config.nr_of_link_frequencies) {
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "no valid sensor mode defined\n");
		goto done_endpoint_free;
	}

	switch (extclk_frequency) {
	// case 27000000:
	// case 37125000:
	// case 74250000:
	// 	sensor->pixel_rate = IMX415_PIXEL_RATE_74_25MHZ;
	// 	break;
	case 24000000:
		// case 72000000:
		sensor->pixel_rate = AR0822_PIXEL_RATE;
		break;
	}

	link_frequency =
		ar0822_supported_modes[sensor->cur_mode].link_frequency;
	for (i = 0; i < ARRAY_SIZE(ar0822_clk_params); i++) {
		if (link_frequency == ar0822_clk_params[i].link_frequency &&
		    extclk_frequency == ar0822_clk_params[i].extclk_frequency) {
			sensor->clk_params = &ar0822_clk_params[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(ar0822_clk_params)) {
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "Mode %d not supported\n",
				    sensor->cur_mode);
		goto done_endpoint_free;
	}

	ret = 0;
	dev_dbg(sensor->dev,
		"clock: %lu Hz, link_frequency: %llu bps, lanes: %d\n",
		extclk_frequency, link_frequency, sensor->num_data_lanes);

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

	ret = ar0822_identify_model(sensor);
	if (ret)
		goto err_power_off;

	/* Initialize default format */
	ar0822_set_default_format(sensor);

	ret = ar0822_subdev_init(sensor);
	if (ret)
		goto err_power_off;

	/* sensor doesn't enter LP-11 state upon power up until and unless
	* streaming is started, so upon power up switch the modes to:
	* streaming -> standby
	*/
	ret = cci_write(sensor->regmap, AR0822_REG_RESET, AR0822_MODE_STREAM_ON,
			NULL);
	if (ret < 0)
		goto err_power_off;

	/* Datasheet states that stream ON should be toggled ON for minimum 2ms */
	usleep_range(2000, 2100);

	/* Set the sensor back to low power mode */
	ret = cci_write(sensor->regmap, AR0822_REG_RESET, AR0822_MODE_LOW_POWER,
			NULL);
	if (ret < 0)
		goto err_power_off;

	/*
	 * Enable runtime PM. As the device has been powered manually, mark it
	 * as active, and increase the usage count without resuming the device.
	 */
	pm_runtime_set_active(sensor->dev);
	pm_runtime_get_noresume(sensor->dev);
	pm_runtime_enable(sensor->dev);

	ret = v4l2_async_register_subdev_sensor(&sensor->subdev);
	if (ret < 0)
		goto err_pm;

	/*
	 * Finally, enable autosuspend and decrease the usage count. The device
	 * will get suspended after the autosuspend delay, turning the power
	 * off.
	 */
	pm_runtime_set_autosuspend_delay(sensor->dev, 1000);
	pm_runtime_use_autosuspend(sensor->dev);
	pm_runtime_put_autosuspend(sensor->dev);

	dev_dbg(sensor->dev, "AR0822 sensor probed successfully\n");

	return 0;

err_pm:
	pm_runtime_disable(sensor->dev);
	pm_runtime_put_noidle(sensor->dev);
	ar0822_subdev_cleanup(sensor);
err_power_off:
	ar0822_power_off(sensor);
	return ret;
}

static void ar0822_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ar0822 *sensor = to_ar0822(subdev);

	v4l2_async_unregister_subdev(subdev);

	ar0822_subdev_cleanup(sensor);

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
