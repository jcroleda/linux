// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Backlight driver for Analog Devices LTC3208 Multi-Display Driver
 *
 * Copyright 2025 Analog Devices Inc.
 *
 * Author: Jan Carlo Roleda <jancarlo.roleda@analog.com>
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/leds.h>
#include <linux/backlight.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/bitfield.h>
#include <linux/regmap.h>

#define LTC3208_DEVICE_NAME "ltc3208"

#define LTC3208_HIGH_BYTE_DATA(x)   (((x) & 0xF) << 4)
#define LTC3208_LOW_BYTE_DATA(x)    ((x) & 0xF)
#define LTC3208_BYTE                (0xFF)

#define LTC3208_LED_GRPS    8

/* Registers */
#define LTC3208_REG_A_GNRD  0x1 /* Green (High half-byte) and Red (Low half-byte) current DAC*/
#define LTC3208_REG_B_AXBL  0x2 /* AUX (High half-byte) and Blue (Low half-byte) current DAC*/
#define LTC3208_REG_C_MAIN  0x3 /* Main current DAC */
#define LTC3208_REG_D_SUB   0x4 /* Sub current DAC */
#define LTC3208_REG_E_AUX   0x5 /* AUX DAC Select */
#define LTC3208_REG_F_CAM   0x6 /* CAM (High half-byte and Low half-byte) current DAC*/
#define LTC3208_REG_G_OPT   0x7 /* Device Options */

/* Device Options register */
#define LTC3208_OPT_F2X     BIT(7)
#define LTC3208_OPT_F1P5X   BIT(6)
#define LTC3208_OPT_RGBDROP BIT(3)
#define LTC3208_OPT_CAMHILO BIT(2)
#define LTC3208_OPT_ELRGBS  BIT(1)

/* Auxillary DAC select masks */
#define LTC3208_AUX1_MASK   GENMASK(1, 0)
#define LTC3208_AUX2_MASK   GENMASK(3, 2)
#define LTC3208_AUX3_MASK   GENMASK(5, 4)
#define LTC3208_AUX4_MASK   GENMASK(7, 6)

#define LTC3208_MAX_BRIGHTNESS_4BIT 0xF
#define LTC3208_MAX_BRIGHTNESS_8BIT 0xFF

#define LTC3208_AUX_DAC_MAX_OPT     (3)

enum ltc3208_aux_channel {
	LTC3208_AUX_CHAN_AUX = 0,
	LTC3208_AUX_CHAN_MAIN,
	LTC3208_AUX_CHAN_SUB,
	LTC3208_AUX_CHAN_CAM
};

enum ltc3208_channel {
	LTC3208_CHAN_MAIN = 0,
	LTC3208_CHAN_SUB,
	LTC3208_CHAN_AUX,
	LTC3208_CHAN_CAM,
	LTC3208_CHAN_CAM_HI,
	LTC3208_CHAN_RED,
	LTC3208_CHAN_BLUE,
	LTC3208_CHAN_GREEN
};

struct ltc3208_led {
	struct led_classdev cdev;
	struct i2c_client *client;
	enum ltc3208_channel channel;
};

/*
 * as device is write-only the device configs must be stored in the driver to
 * allow atomic operations in registers that control multiple channels/options
 */
struct ltc3208_chip_data {
	struct ltc3208_led *leds; /* leds[LTC3208_LED_GRPS] */
	/* device control values */
	enum ltc3208_aux_channel aux_config[4];
	bool force_cpo_1p5x;
	bool force_cpo_2x;
	bool en_rgbs;
	bool cam_hi;
	bool rgb_aux4_dropout_dis;
};

struct ltc3208_dev {
	struct i2c_client *client;
	struct regmap *map;
	struct ltc3208_chip_data chip_data;
};

static const struct regmap_config ltc3208_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int ltc3208_led_set_brightness(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct ltc3208_led *led = container_of(led_cdev, struct ltc3208_led, cdev);
	struct i2c_client *client = led->client;
	struct ltc3208_dev *dev = i2c_get_clientdata(client);
	struct regmap *map = dev->map;
	int ret;
	u8 current_level = (u8) brightness;
	u8 reg = 0;

	if (current_level > led->cdev.max_brightness) {
		dev_err(&client->dev, "Brightness set is too High\n");
		return -EINVAL;
	}

	/*
	 * For registers with 4-bit splits (CAM, AUX/BLUE, GREEN/RED), the other
	 * half of the byte will be retrieved from the stored DAC value before
	 * updating the register.
	 */
	switch (led->channel) {
	case LTC3208_CHAN_MAIN:
		reg = LTC3208_REG_C_MAIN;
		break;
	case LTC3208_CHAN_SUB:
		reg = LTC3208_REG_D_SUB;
		break;

	case LTC3208_CHAN_AUX:
		/* combine both low and high halves of byte */
		current_level = LTC3208_HIGH_BYTE_DATA(current_level);
		current_level |= dev->chip_data.leds[LTC3208_CHAN_BLUE].cdev.brightness;
		reg = LTC3208_REG_B_AXBL;
		break;

	case LTC3208_CHAN_BLUE:
		/* apply high bits stored in other led */
		current_level |= LTC3208_HIGH_BYTE_DATA(
			dev->chip_data.leds[LTC3208_CHAN_AUX].cdev.brightness);
		reg = LTC3208_REG_B_AXBL;
		break;

	case LTC3208_CHAN_CAM_HI:
		current_level = LTC3208_HIGH_BYTE_DATA(current_level);
		current_level |= dev->chip_data.leds[LTC3208_CHAN_CAM].cdev.brightness;
		reg = LTC3208_REG_F_CAM;
		break;

	case LTC3208_CHAN_CAM:
		current_level |= LTC3208_HIGH_BYTE_DATA(
			dev->chip_data.leds[LTC3208_CHAN_CAM_HI].cdev.brightness);
		reg = LTC3208_REG_F_CAM;
		break;

	case LTC3208_CHAN_GREEN:
		current_level = LTC3208_HIGH_BYTE_DATA(current_level);
		current_level |= dev->chip_data.leds[LTC3208_CHAN_RED].cdev.brightness;
		reg = LTC3208_REG_A_GNRD;
		break;
	case LTC3208_CHAN_RED:
		current_level |= LTC3208_HIGH_BYTE_DATA(
			dev->chip_data.leds[LTC3208_CHAN_GREEN].cdev.brightness);
		reg = LTC3208_REG_A_GNRD;
		break;
	}

	ret = regmap_write(map, reg, current_level);
	if (ret) {
		dev_err(&client->dev, "Error Writing brightness to register %u\n", reg);
		return ret;
	}

	return 0;
}

static enum led_brightness ltc3208_led_get_brightness(
	struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

static int ltc3208_update_options(struct ltc3208_dev *dev,
	bool is_sub, bool is_cam_hi, bool is_rgb_drop,
	bool force_cpo_1p5x, bool force_cpo_2x)
{
	struct regmap *map = dev->map;
	int ret;
	u8 val = 0;

		val |= is_sub ? LTC3208_OPT_ELRGBS : 0;
		val |= is_cam_hi ? LTC3208_OPT_CAMHILO : 0;
		val |= is_rgb_drop ? LTC3208_OPT_RGBDROP : 0;
		val |= force_cpo_1p5x ? LTC3208_OPT_F1P5X : 0;
		val |= force_cpo_2x ? LTC3208_OPT_F2X : 0;

	ret = regmap_write(map, LTC3208_REG_G_OPT, val);
	if (ret) {
		dev_err(&dev->client->dev, "Error writing options to device");
		return ret;
	}

	return 0;
}

static ssize_t ltc3208_select_rgbs_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;
	bool is_sub;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 1)
		return -EINVAL;

	is_sub = !!val;

	ret = ltc3208_update_options(data, is_sub, data->chip_data.cam_hi,
		data->chip_data.rgb_aux4_dropout_dis, data->chip_data.force_cpo_1p5x,
		data->chip_data.force_cpo_2x);
	if (ret)
		return ret;

	data->chip_data.en_rgbs = is_sub;

	return count;
}

static ssize_t ltc3208_select_rgbs_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", data->chip_data.en_rgbs);
}

static ssize_t ltc3208_select_cam_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;
	bool is_cam_hi;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 1)
		return -EINVAL;

	is_cam_hi = !!val;

	ret = ltc3208_update_options(data, data->chip_data.en_rgbs, is_cam_hi,
		data->chip_data.rgb_aux4_dropout_dis, data->chip_data.force_cpo_1p5x,
		data->chip_data.force_cpo_2x);
	if (ret)
		return ret;

	data->chip_data.cam_hi = is_cam_hi;

	return count;
}

static ssize_t ltc3208_select_cam_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", data->chip_data.cam_hi);
}

static ssize_t ltc3208_select_rgb_drop_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;
	bool is_rgb_drop;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 1)
		return -EINVAL;

	is_rgb_drop = !!val;

	ret = ltc3208_update_options(data, data->chip_data.en_rgbs,
		data->chip_data.cam_hi, is_rgb_drop, data->chip_data.force_cpo_1p5x,
		data->chip_data.force_cpo_2x);
	if (ret)
		return ret;

	data->chip_data.rgb_aux4_dropout_dis = is_rgb_drop;

	return count;
}

static ssize_t ltc3208_select_rgb_drop_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", data->chip_data.rgb_aux4_dropout_dis);
}

static ssize_t ltc3208_cpo_1p5x_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;
	bool force_cpo_1p5x;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 1)
		return -EINVAL;

	force_cpo_1p5x = !!val;

	ret = ltc3208_update_options(data, data->chip_data.en_rgbs,
		data->chip_data.cam_hi, data->chip_data.rgb_aux4_dropout_dis,
		force_cpo_1p5x, data->chip_data.force_cpo_2x);
	if (ret)
		return ret;

	data->chip_data.force_cpo_1p5x = force_cpo_1p5x;

	return count;
}

static ssize_t ltc3208_cpo_1p5x_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", data->chip_data.force_cpo_1p5x);
}

static ssize_t ltc3208_cpo_2x_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;
	bool force_cpo_2x;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 1)
		return -EINVAL;

	force_cpo_2x = !!val;

	ret = ltc3208_update_options(data, data->chip_data.en_rgbs,
		data->chip_data.cam_hi, data->chip_data.rgb_aux4_dropout_dis,
		data->chip_data.force_cpo_1p5x, force_cpo_2x);
	if (ret)
		return ret;

	data->chip_data.force_cpo_2x = force_cpo_2x;

	return count;
}


static ssize_t ltc3208_cpo_2x_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", data->chip_data.force_cpo_2x);
}

static DEVICE_ATTR(rgb_sub_enable, 0664,
	ltc3208_select_rgbs_show, ltc3208_select_rgbs_store);
static DEVICE_ATTR(cam_dac, 0664,
	ltc3208_select_cam_show, ltc3208_select_cam_store);
static DEVICE_ATTR(rgb_aux4_dropout_disable, 0664,
	ltc3208_select_rgb_drop_show, ltc3208_select_rgb_drop_store);
static DEVICE_ATTR(cpo_1p5x, 0664,
	ltc3208_cpo_1p5x_show, ltc3208_cpo_1p5x_store);
static DEVICE_ATTR(cpo_2x, 0664,
	ltc3208_cpo_2x_show, ltc3208_cpo_2x_store);

static int ltc3208_update_aux_dac(struct ltc3208_dev *dev,
	enum ltc3208_aux_channel aux_1, enum ltc3208_aux_channel aux_2,
	enum ltc3208_aux_channel aux_3, enum ltc3208_aux_channel aux_4)
{
	struct regmap *map = dev->map;
	int ret;
	u8 val = 0;

	val = FIELD_PREP(LTC3208_AUX1_MASK, aux_1) |
		  FIELD_PREP(LTC3208_AUX2_MASK, aux_2) |
		  FIELD_PREP(LTC3208_AUX3_MASK, aux_3) |
		  FIELD_PREP(LTC3208_AUX4_MASK, aux_4);

	ret = regmap_write(map, LTC3208_REG_E_AUX, val);
	if (ret) {
		dev_err(&dev->client->dev, "Error writing AUX DAC selection to device");
		return ret;
	}

	return 0;
}

static ssize_t ltc3208_select_aux1_dac_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;
	enum ltc3208_aux_channel dac1;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val > LTC3208_AUX_DAC_MAX_OPT)
		return -EINVAL;

	dac1 = (enum ltc3208_aux_channel) val;

	ret = ltc3208_update_aux_dac(data, dac1, data->chip_data.aux_config[1],
		data->chip_data.aux_config[2], data->chip_data.aux_config[3]);
	if (ret)
		return ret;

	data->chip_data.aux_config[0] = dac1;

	return count;
}

static ssize_t ltc3208_select_aux1_dac_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", data->chip_data.aux_config[0]);
}

static ssize_t ltc3208_select_aux2_dac_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;
	enum ltc3208_aux_channel dac2;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val > LTC3208_AUX_DAC_MAX_OPT)
		return -EINVAL;

	dac2 = (enum ltc3208_aux_channel) val;

	ret = ltc3208_update_aux_dac(data, data->chip_data.aux_config[0], dac2,
		data->chip_data.aux_config[2], data->chip_data.aux_config[3]);
	if (ret)
		return ret;

	data->chip_data.aux_config[1] = dac2;

	return count;
}

static ssize_t ltc3208_select_aux2_dac_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", data->chip_data.aux_config[1]);
}

static ssize_t ltc3208_select_aux3_dac_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;
	enum ltc3208_aux_channel dac3;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val > LTC3208_AUX_DAC_MAX_OPT)
		return -EINVAL;

	dac3 = (enum ltc3208_aux_channel) val;

	ret = ltc3208_update_aux_dac(data, data->chip_data.aux_config[0],
		data->chip_data.aux_config[1], dac3, data->chip_data.aux_config[3]);
	if (ret)
		return ret;

	data->chip_data.aux_config[2] = dac3;

	return count;
}

static ssize_t ltc3208_select_aux3_dac_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", data->chip_data.aux_config[2]);
}

static ssize_t ltc3208_select_aux4_dac_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret;
	enum ltc3208_aux_channel dac4;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val > LTC3208_AUX_DAC_MAX_OPT)
		return -EINVAL;

	dac4 = (enum ltc3208_aux_channel) val;

	ret = ltc3208_update_aux_dac(data, data->chip_data.aux_config[0],
		data->chip_data.aux_config[1], data->chip_data.aux_config[2], dac4);
	if (ret)
		return ret;

	data->chip_data.aux_config[3] = dac4;

	return count;
}

static ssize_t ltc3208_select_aux4_dac_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ltc3208_dev *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", data->chip_data.aux_config[3]);
}

static DEVICE_ATTR(aux1_dac, 0664,
	ltc3208_select_aux1_dac_show, ltc3208_select_aux1_dac_store);
static DEVICE_ATTR(aux2_dac, 0664,
	ltc3208_select_aux2_dac_show, ltc3208_select_aux2_dac_store);
static DEVICE_ATTR(aux3_dac, 0664,
	ltc3208_select_aux3_dac_show, ltc3208_select_aux3_dac_store);
static DEVICE_ATTR(aux4_dac, 0664,
	ltc3208_select_aux4_dac_show, ltc3208_select_aux4_dac_store);

static int ltc3208_probe(struct i2c_client *client)
{
	struct ltc3208_dev *data;
	struct ltc3208_led *leds, *led_data;
	struct device_node *root = client->dev.of_node;
	struct device_node *child;
	struct regmap *map;
	const char *label;
	int ret;
	u8 reg;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	map = devm_regmap_init_i2c(client, &ltc3208_regmap_cfg);

	if (!map) {
		dev_err(&client->dev, "SMBus");
		return -EIO;
	}

	if (!(root && of_get_child_count(root))) {
		dev_err(&client->dev, "Devicetree not found?\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	leds = devm_kcalloc(&client->dev, LTC3208_LED_GRPS,
		sizeof(*leds), GFP_KERNEL);
	if (leds == NULL)
		return -ENOMEM;

	/* clear any data before initializing */
	for (int i = 0; i < LTC3208_LED_GRPS; i++) {
		leds[i].cdev.name = NULL;
		leds[i].cdev.dev = NULL;
	}


	data->client = client;
	data->chip_data.leds = leds;
	data->map = map;

	/* set defaults to 0 */
	data->chip_data.en_rgbs = false;
	data->chip_data.cam_hi = false;
	data->chip_data.rgb_aux4_dropout_dis = false;
	data->chip_data.force_cpo_1p5x = false;
	data->chip_data.force_cpo_2x = false;
	data->chip_data.aux_config[0] = LTC3208_AUX_CHAN_AUX;
	data->chip_data.aux_config[1] = LTC3208_AUX_CHAN_AUX;
	data->chip_data.aux_config[2] = LTC3208_AUX_CHAN_AUX;
	data->chip_data.aux_config[3] = LTC3208_AUX_CHAN_AUX;

	i2c_set_clientdata(client, data);

	for_each_available_child_of_node(root, child) {

		ret = of_property_read_u8(child, "reg", &reg);
		if (ret) {
			dev_err(&client->dev, "Missing reg property for LED\n");
			of_node_put(child);
			return -EINVAL;
		}

		if (reg >= LTC3208_LED_GRPS) {
			dev_err(&client->dev, "Invalid LED channel %u\n", reg);
			of_node_put(child);
			return -EINVAL;
		}

		led_data = &leds[reg];

		led_data->client = client;
		led_data->channel = reg;
		led_data->cdev.brightness_set_blocking = ltc3208_led_set_brightness;
		led_data->cdev.brightness_get = ltc3208_led_get_brightness;
		led_data->cdev.max_brightness = LTC3208_MAX_BRIGHTNESS_4BIT;
		if (reg == LTC3208_CHAN_MAIN || reg == LTC3208_CHAN_SUB)
			led_data->cdev.max_brightness = LTC3208_MAX_BRIGHTNESS_8BIT;

		ret = of_property_read_string(child, "label", &label);
		if (ret) {
			dev_err(&client->dev, "Missing label property for LED\n");
			of_node_put(child);
			return -EINVAL;
		}

		led_data->cdev.name = devm_kstrdup(&client->dev, label, GFP_KERNEL);
		if (!led_data->cdev.name) {
			of_node_put(child);
			return -ENOMEM;
		}

		ret = led_classdev_register(&client->dev, &led_data->cdev);
		if (ret) {
			dev_err(&client->dev, "Failed to register LED %s\n", led_data->cdev.name);
			of_node_put(child);
			goto led_init_error;
		}

		ret = ltc3208_led_set_brightness(&led_data->cdev, LED_OFF);
		if (ret) {
			dev_err(&client->dev, "Failed to set initial brightness for LED %u\n", reg);
			of_node_put(child);
			goto led_init_error;
		}
	}

	ret = ltc3208_update_options(data, data->chip_data.en_rgbs, data->chip_data.cam_hi,
		data->chip_data.rgb_aux4_dropout_dis, data->chip_data.force_cpo_1p5x, data->chip_data.force_cpo_2x);
	if (ret) {
		dev_err(&client->dev, "Failed to initialize device options\n");
		goto led_init_error;
	}

	ret = ltc3208_update_aux_dac(data, data->chip_data.aux_config[0], data->chip_data.aux_config[1],
		data->chip_data.aux_config[2], data->chip_data.aux_config[3]);
	if (ret) {
		dev_err(&client->dev, "Failed to initalize AUX DACs\n");
		goto led_init_error;
	}

	return 0;

 led_init_error:
	for (int channel = 0; channel < LTC3208_LED_GRPS; channel++) {
		if (leds[channel].cdev.name && leds[channel].cdev.dev)
			led_classdev_unregister(&leds[channel].cdev);
	}

	return ret;
}

static void ltc3208_remove(struct i2c_client *client)
{
	struct ltc3208_dev *dev = i2c_get_clientdata(client);

	if (!dev || !dev->chip_data.leds)
		return;

	for (int i = 0; i < LTC3208_LED_GRPS; i++) {
		if (dev->chip_data.leds[i].cdev.name && dev->chip_data.leds[i].cdev.dev)
			led_classdev_unregister(&dev->chip_data.leds[i].cdev);
	}
}

static struct attribute *ltc3208_attrs[] = {
	&dev_attr_rgb_sub_enable.attr,
	&dev_attr_cam_dac.attr,
	&dev_attr_rgb_aux4_dropout_disable.attr,
	&dev_attr_cpo_1p5x.attr,
	&dev_attr_cpo_2x.attr,
	&dev_attr_aux1_dac.attr,
	&dev_attr_aux2_dac.attr,
	&dev_attr_aux3_dac.attr,
	&dev_attr_aux4_dac.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ltc3208);

static const struct of_device_id ltc3208_match_table[] = {
	{.compatible = "adi,ltc3208"},
	{ }
};
MODULE_DEVICE_TABLE(of, ltc3208_match_table);

static const struct i2c_device_id ltc3208_idtable[] = {
	{ "ltc3208" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltc3208_idtable);

static struct i2c_driver ltc3208_driver = {
	.driver = {
		.name = LTC3208_DEVICE_NAME,
		.of_match_table = ltc3208_match_table,
		.groups = ltc3208_groups,
	},
	.id_table = ltc3208_idtable,
	.probe = ltc3208_probe,
	.remove = ltc3208_remove,
};
module_i2c_driver(ltc3208_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jan Carlo Roleda <jancarlo.roleda@analog.com>");
MODULE_DESCRIPTION("LTC3208 Backlight Driver");
