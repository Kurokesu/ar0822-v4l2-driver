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

static const char *const ar0822_supply_names[] = {
	"vana", /* Analog (2.8V) supply */
	"vdig", /* Digital Core (1.8V) supply */
	"vddl", /* IF (1.2V) supply */
};

struct ar0822_clk_params {
	u64 lane_rate;
	u64 extclk;
	// struct cci_reg_sequence regs[IMX415_NUM_CLK_PARAM_REGS];
};

/* EXTCLK Settings - includes all lane rate and EXTCLK dependent registers */
static const struct ar0822_clk_params ar0822_clk_params[] = {
	{
		.lane_rate = 594000000UL,
		.extclk = 27000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x05D },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x042 },
		.regs[2] = { IMX415_SYS_MODE, 0x7 },
		.regs[3] = { IMX415_EXTCLKSEL1, 0x00 },
		.regs[4] = { IMX415_EXTCLKSEL2, 0x23 },
		.regs[5] = { IMX415_EXTCLKSEL3, 0x084 },
		.regs[6] = { IMX415_EXTCLKSEL4, 0x0E7 },
		.regs[7] = { IMX415_EXTCLKSEL5, 0x23 },
		.regs[8] = { IMX415_EXTCLKSEL6, 0x0 },
		.regs[9] = { IMX415_EXTCLKSEL7, 0x1 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x06C0 },
	},
};

/* 720 Mbps CSI configuration */
static const struct cci_reg_sequence ar0822_linkrate_720mbps[] = {
	{ IMX415_TCLKPOST, 0x006F },   { IMX415_TCLKPREPARE, 0x002F },
	{ IMX415_TCLKTRAIL, 0x002F },  { IMX415_TCLKZERO, 0x00BF },
	{ IMX415_THSPREPARE, 0x002F }, { IMX415_THSZERO, 0x0057 },
	{ IMX415_THSTRAIL, 0x002F },   { IMX415_THSEXIT, 0x004F },
	{ IMX415_TLPX, 0x0027 },
};

struct ar0822_mode_reg_list {
	u32 num_of_regs;
	const struct cci_reg_sequence *regs;
};

struct ar0822_mode {
	u64 lane_rate;
	u32 hmax_min[2];
	struct ar0822_mode_reg_list reg_list;
};

/* mode configs */
static const struct ar0822_mode supported_modes[] = {
	{
		.lane_rate = 720000000,
		.hmax_min = { 2032, 1066 },
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(ar0822_linkrate_720mbps),
			.regs = ar0822_linkrate_720mbps,
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
	struct clk *clk;
	unsigned long pixel_rate;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ar0822_supply_names)];
	struct gpio_desc *reset;
	struct regmap *regmap;

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
};

/*
 * This table includes fixed register settings and a bunch of undocumented
 * registers that have to be set to another value than default.
 */
static const struct cci_reg_sequence ar0822_init_table[] = {
	/* use all-pixel readout mode, no flip */
	{ IMX415_WINMODE, 0x00 },
	{ IMX415_ADDMODE, 0x00 },
	{ IMX415_REVERSE, 0x00 },
	/* use RAW 10-bit mode */
	{ IMX415_ADBIT, 0x00 },
	{ IMX415_MDBIT, 0x00 },
	/* output VSYNC on XVS and low on XHS */
	{ IMX415_OUTSEL, 0x22 },
	{ IMX415_DRV, 0x00 },

	/* SONY magic registers */
	{ CCI_REG8(0x32D4), 0x21 },
};

static inline struct ar0822 *to_ar0822(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar0822, subdev);
}

static int ar0822_set_testpattern(struct ar0822 *sensor, int val)
{
	int ret = 0;

	if (val) {
		cci_write(sensor->regmap, IMX415_BLKLEVEL, 0x00, &ret);
		cci_write(sensor->regmap, IMX415_TPG_EN_DUOUT, 0x01, &ret);
		cci_write(sensor->regmap, IMX415_TPG_PATSEL_DUOUT, val - 1,
			  &ret);
		cci_write(sensor->regmap, IMX415_TPG_COLORWIDTH, 0x01, &ret);
		cci_write(sensor->regmap, IMX415_TESTCLKEN_MIPI, 0x20, &ret);
		cci_write(sensor->regmap, IMX415_DIG_CLP_MODE, 0x00, &ret);
		cci_write(sensor->regmap, IMX415_WRJ_OPEN, 0x00, &ret);
	} else {
		cci_write(sensor->regmap, IMX415_BLKLEVEL,
			  IMX415_BLKLEVEL_DEFAULT, &ret);
		cci_write(sensor->regmap, IMX415_TPG_EN_DUOUT, 0x00, &ret);
		cci_write(sensor->regmap, IMX415_TESTCLKEN_MIPI, 0x00, &ret);
		cci_write(sensor->regmap, IMX415_DIG_CLP_MODE, 0x01, &ret);
		cci_write(sensor->regmap, IMX415_WRJ_OPEN, 0x01, &ret);
	}
	return 0;
}

static int ar0822_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0822 *sensor =
		container_of(ctrl->handler, struct imx415, ctrls);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	u32 exposure_max;
	unsigned int vmax;
	unsigned int flip;
	int ret;

	state = v4l2_subdev_get_locked_active_state(&sensor->subdev);
	format = v4l2_subdev_state_get_format(state, 0);

	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max =
			format->height + ctrl->val - IMX415_EXPOSURE_OFFSET;
		__v4l2_ctrl_modify_range(sensor->exposure,
					 sensor->exposure->minimum,
					 exposure_max, sensor->exposure->step,
					 sensor->exposure->default_value);
	}

	if (!pm_runtime_get_if_in_use(sensor->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ret = cci_write(sensor->regmap, IMX415_VMAX,
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
		ret = cci_write(sensor->regmap, IMX415_SHR0, vmax - ctrl->val,
				NULL);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		/* analogue gain in 0.3 dB step size */
		ret = cci_write(sensor->regmap, IMX415_GAIN_PCG_0, ctrl->val,
				NULL);
		break;

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		flip = (sensor->hflip->val << IMX415_HREVERSE_SHIFT) |
		       (sensor->vflip->val << IMX415_VREVERSE_SHIFT);
		ret = cci_write(sensor->regmap, IMX415_REVERSE, flip, NULL);
		break;

	case V4L2_CID_TEST_PATTERN:
		ret = ar0822_set_testpattern(sensor, ctrl->val);
		break;

	case V4L2_CID_HBLANK:
		return cci_write(sensor->regmap, IMX415_HMAX,
				 (format->width + ctrl->val) /
					 IMX415_HMAX_MULTIPLIER,
				 NULL);

	default:
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
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	u64 lane_rate = supported_modes[sensor->cur_mode].lane_rate;
	u32 exposure_max = IMX415_PIXEL_ARRAY_HEIGHT +
			   IMX415_PIXEL_ARRAY_VBLANK - IMX415_EXPOSURE_OFFSET;
	u32 hblank_min, hblank_max;
	unsigned int i;
	int ret;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 10);

	for (i = 0; i < ARRAY_SIZE(link_freq_menu_items); ++i) {
		if (lane_rate == link_freq_menu_items[i] * 2)
			break;
	}
	if (i == ARRAY_SIZE(link_freq_menu_items)) {
		return dev_err_probe(sensor->dev, -EINVAL,
				     "lane rate %llu not supported\n",
				     lane_rate);
	}

	ctrl = v4l2_ctrl_new_int_menu(&sensor->ctrls, &ar0822_ctrl_ops,
				      V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(link_freq_menu_items) - 1, i,
				      link_freq_menu_items);

	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->exposure = v4l2_ctrl_new_std(&sensor->ctrls, &ar0822_ctrl_ops,
					     V4L2_CID_EXPOSURE, 4, exposure_max,
					     1, exposure_max);

	v4l2_ctrl_new_std(&sensor->ctrls, &ar0822_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, IMX415_AGAIN_MIN,
			  IMX415_AGAIN_MAX, IMX415_AGAIN_STEP,
			  IMX415_AGAIN_MIN);

	hblank_min = (supported_modes[sensor->cur_mode]
			      .hmax_min[sensor->num_data_lanes == 2 ? 0 : 1] *
		      IMX415_HMAX_MULTIPLIER) -
		     IMX415_PIXEL_ARRAY_WIDTH;
	hblank_max = (IMX415_HMAX_MAX * IMX415_HMAX_MULTIPLIER) -
		     IMX415_PIXEL_ARRAY_WIDTH;
	ctrl = v4l2_ctrl_new_std(&sensor->ctrls, &ar0822_ctrl_ops,
				 V4L2_CID_HBLANK, hblank_min, hblank_max,
				 IMX415_HMAX_MULTIPLIER, hblank_min);

	sensor->vblank =
		v4l2_ctrl_new_std(&sensor->ctrls, &ar0822_ctrl_ops,
				  V4L2_CID_VBLANK, IMX415_PIXEL_ARRAY_VBLANK,
				  IMX415_VMAX_MAX - IMX415_PIXEL_ARRAY_HEIGHT,
				  1, IMX415_PIXEL_ARRAY_VBLANK);

	v4l2_ctrl_new_std(&sensor->ctrls, NULL, V4L2_CID_PIXEL_RATE,
			  sensor->pixel_rate, sensor->pixel_rate, 1,
			  sensor->pixel_rate);

	sensor->hflip = v4l2_ctrl_new_std(&sensor->ctrls, &ar0822_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	sensor->vflip = v4l2_ctrl_new_std(&sensor->ctrls, &ar0822_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(&sensor->ctrls, &ar0822_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ar0822_test_pattern_menu) - 1,
				     0, 0, ar0822_test_pattern_menu);

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

	if (mode >= ARRAY_SIZE(supported_modes)) {
		dev_err(sensor->dev, "Mode %d not supported\n", mode);
		return -EINVAL;
	}

	cci_multi_reg_write(sensor->regmap, supported_modes[mode].reg_list.regs,
			    supported_modes[mode].reg_list.num_of_regs, &ret);

	cci_multi_reg_write(sensor->regmap, sensor->clk_params->regs,
			    IMX415_NUM_CLK_PARAM_REGS, &ret);

	ret = cci_write(sensor->regmap, IMX415_LANEMODE,
			sensor->num_data_lanes == 2 ? IMX415_LANEMODE_2 :
						      IMX415_LANEMODE_4,
			NULL);

	return ret;
}

static int ar0822_setup(struct ar0822 *sensor, struct v4l2_subdev_state *state)
{
	int ret;

	ret = cci_multi_reg_write(sensor->regmap, ar0822_init_table,
				  ARRAY_SIZE(ar0822_init_table), NULL);
	if (ret)
		return ret;

	return ar0822_set_mode(sensor, sensor->cur_mode);
}

static int ar0822_wakeup(struct ar0822 *sensor)
{
	int ret;

	ret = cci_write(sensor->regmap, IMX415_MODE, IMX415_MODE_OPERATING,
			NULL);
	if (ret)
		return ret;

	/*
	 * According to the datasheet we have to wait at least 63 us after
	 * leaving standby mode. But this doesn't work even after 30 ms.
	 * So probably this should be 63 ms and therefore we wait for 80 ms.
	 */
	msleep(80);

	return 0;
}

static int ar0822_stream_on(struct ar0822 *sensor)
{
	int ret;

	ret = ar0822_wakeup(sensor);
	return cci_write(sensor->regmap, IMX415_XMSTA, IMX415_XMSTA_START,
			 &ret);
}

static int ar0822_stream_off(struct ar0822 *sensor)
{
	int ret;

	ret = cci_write(sensor->regmap, IMX415_XMSTA, IMX415_XMSTA_STOP, NULL);
	return cci_write(sensor->regmap, IMX415_MODE, IMX415_MODE_STANDBY,
			 &ret);
}

static int ar0822_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0822 *sensor = to_ar0822(sd);
	struct v4l2_subdev_state *state;
	int ret;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (!enable) {
		ret = ar0822_stream_off(sensor);

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

	ret = ar0822_stream_on(sensor);
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

	format = v4l2_subdev_state_get_format(state, fse->pad);

	if (fse->index > 0 || fse->code != format->code)
		return -EINVAL;

	fse->min_width = IMX415_PIXEL_ARRAY_WIDTH;
	fse->max_width = fse->min_width;
	fse->min_height = IMX415_PIXEL_ARRAY_HEIGHT;
	fse->max_height = fse->min_height;
	return 0;
}

static int ar0822_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_state_get_format(state, fmt->pad);

	format->width = fmt->format.width;
	format->height = fmt->format.height;
	format->code = MEDIA_BUS_FMT_SGRBG12_1X12;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_DEFAULT;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	fmt->format = *format;
	return 0;
}

static int ar0822_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = IMX415_PIXEL_ARRAY_TOP;
		sel->r.left = IMX415_PIXEL_ARRAY_LEFT;
		sel->r.width = IMX415_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX415_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int ar0822_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {
		.format = {
			.width = IMX415_PIXEL_ARRAY_WIDTH,
			.height = IMX415_PIXEL_ARRAY_HEIGHT,
		},
	};

	ar0822_set_format(sd, state, &format);

	return 0;
}

static const struct v4l2_subdev_video_ops ar0822_subdev_video_ops = {
	.s_stream = ar0822_s_stream,
};

static const struct v4l2_subdev_pad_ops ar0822_subdev_pad_ops = {
	.enum_mbus_code = ar0822_enum_mbus_code,
	.enum_frame_size = ar0822_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ar0822_set_format,
	.get_selection = ar0822_get_selection,
};

static const struct v4l2_subdev_ops ar0822_subdev_ops = {
	.video = &ar0822_subdev_video_ops,
	.pad = &ar0822_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ar0822_internal_ops = {
	.init_state = ar0822_init_state,
};

static int ar0822_subdev_init(struct ar0822 *sensor)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int ret;

	v4l2_i2c_subdev_init(&sensor->subdev, client, &ar0822_subdev_ops);
	sensor->subdev.internal_ops = &ar0822_internal_ops;

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

	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies),
				    sensor->supplies);
	if (ret < 0)
		return ret;

	gpiod_set_value_cansleep(sensor->reset, 0);

	udelay(1);

	ret = clk_prepare_enable(sensor->clk);
	if (ret < 0)
		goto err_reset;

	/*
	 * Data sheet states that 20 us are required before communication start,
	 * but this doesn't work in all cases. Use 100 us to be on the safe
	 * side.
	 */
	usleep_range(100, 200);

	return 0;

err_reset:
	gpiod_set_value_cansleep(sensor->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
	return ret;
}

static void ar0822_power_off(struct ar0822 *sensor)
{
	clk_disable_unprepare(sensor->clk);
	gpiod_set_value_cansleep(sensor->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
}

static int ar0822_identify_model(struct ar0822 *sensor)
{
	int model, ret;
	u64 chip_id;

	/*
	 * While most registers can be read when the sensor is in standby, this
	 * is not the case of the sensor info register :-(
	 */
	ret = ar0822_wakeup(sensor);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to get sensor out of standby\n");

	ret = cci_read(sensor->regmap, IMX415_SENSOR_INFO, &chip_id, NULL);
	if (ret < 0) {
		dev_err_probe(sensor->dev, ret,
			      "failed to read sensor information\n");
		goto done;
	}

	model = chip_id & IMX415_SENSOR_INFO_MASK;

	switch (model) {
	case IMX415_CHIP_ID:
		dev_info(sensor->dev, "Detected IMX415 image sensor\n");
		break;
	default:
		ret = dev_err_probe(sensor->dev, -ENODEV,
				    "invalid device model 0x%04x\n", model);
		goto done;
	}

	ret = 0;

done:
	cci_write(sensor->regmap, IMX415_MODE, IMX415_MODE_STANDBY, &ret);
	return ret;
}

static int ar0822_parse_hw_config(struct ar0822 *sensor)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	u64 lane_rate;
	unsigned long inck;
	unsigned int i, j;
	int ret;

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); ++i)
		sensor->supplies[i].supply = ar0822_supply_names[i];

	ret = devm_regulator_bulk_get(sensor->dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to get supplies\n");

	sensor->reset =
		devm_gpiod_get_optional(sensor->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->reset),
				     "failed to get reset GPIO\n");

	sensor->clk = devm_clk_get(sensor->dev, "inck");
	if (IS_ERR(sensor->clk))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->clk),
				     "failed to get clock\n");

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	switch (bus_cfg.bus.mipi_csi2.num_data_lanes) {
	case 2:
	case 4:
		sensor->num_data_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
		break;
	default:
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "invalid number of CSI2 data lanes %d\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "no link frequencies defined");
		goto done_endpoint_free;
	}

	/*
	 * Check if there exists a sensor mode defined for current EXTCLK,
	 * number of lanes and given lane rates.
	 */
	inck = clk_get_rate(sensor->clk);
	for (i = 0; i < bus_cfg.nr_of_link_frequencies; ++i) {
		if (imx415_check_inck(inck, bus_cfg.link_frequencies[i])) {
			dev_dbg(sensor->dev,
				"EXTCLK %lu Hz not supported for this link freq",
				inck);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(supported_modes); ++j) {
			if (bus_cfg.link_frequencies[i] * 2 !=
			    supported_modes[j].lane_rate)
				continue;
			sensor->cur_mode = j;
			break;
		}
		if (j < ARRAY_SIZE(supported_modes))
			break;
	}
	if (i == bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "no valid sensor mode defined\n");
		goto done_endpoint_free;
	}
	switch (inck) {
	case 27000000:
	case 37125000:
	case 74250000:
		sensor->pixel_rate = IMX415_PIXEL_RATE_74_25MHZ;
		break;
	case 24000000:
	case 72000000:
		sensor->pixel_rate = IMX415_PIXEL_RATE_72MHZ;
		break;
	}

	lane_rate = supported_modes[sensor->cur_mode].lane_rate;
	for (i = 0; i < ARRAY_SIZE(ar0822_clk_params); ++i) {
		if (lane_rate == ar0822_clk_params[i].lane_rate &&
		    inck == ar0822_clk_params[i].inck) {
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
	dev_dbg(sensor->dev, "clock: %lu Hz, lane_rate: %llu bps, lanes: %d\n",
		inck, lane_rate, sensor->num_data_lanes);

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

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

	ret = ar0822_parse_hw_config(sensor);
	if (ret)
		return ret;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
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
		goto err_power;

	ret = ar0822_subdev_init(sensor);
	if (ret)
		goto err_power;

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

	return 0;

err_pm:
	pm_runtime_disable(sensor->dev);
	pm_runtime_put_noidle(sensor->dev);
	ar0822_subdev_cleanup(sensor);
err_power:
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
MODULE_LICENSE("GPL 2");
