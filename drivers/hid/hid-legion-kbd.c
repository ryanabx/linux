// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * HID driver for Lenovo Legion/Ideapad 4-zone RGB keyboard backlight
 *
 * Copyright (C) 2024 Sean Farley <sean@farley.ws>
 *
 * These keyboards use an ITE-based USB HID controller for RGB backlight
 * management. The controller is accessed via vendor-defined HID feature
 * reports (usage page 0xff89, usage 0x00cc).
 *
 * Protocol: 33-byte feature report sent via HID output report.
 *   Byte 0:  Report ID (0xcc)
 *   Byte 1:  Command (0x16)
 *   Byte 2:  Effect type (static=0x01, breath=0x03, smooth=0x06, wave=0x04)
 *   Byte 3:  Speed (1-4)
 *   Byte 4:  Brightness (1-2)
 *   Bytes 5-16: RGB values for 4 zones (R,G,B per zone)
 *   Byte 18:  Right-wave direction flag (0x0 or 0x1)
 *   Byte 19:  Left-wave direction flag (0x0 or 0x1)
 *
 * Exposes 4 multicolor LED class devices (one per zone) and custom sysfs
 * attributes for effect type, speed, and brightness control.
 */

#include <linux/hid.h>
#include <linux/led-class-multicolor.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/sysfs.h>

#include "hid-ids.h"

/* HID report constants */
#define LEGION_KBD_REPORT_ID		0xcc
#define LEGION_KBD_CMD_SET_LIGHTING	0x16
#define LEGION_KBD_REPORT_SIZE		33

/* Effect types */
#define LEGION_KBD_EFFECT_STATIC	0x01
#define LEGION_KBD_EFFECT_BREATH	0x03
#define LEGION_KBD_EFFECT_SMOOTH	0x06
#define LEGION_KBD_EFFECT_WAVE		0x04

/* Effect parameters */
#define LEGION_KBD_SPEED_MIN		1
#define LEGION_KBD_SPEED_MAX		4
#define LEGION_KBD_BRIGHTNESS_MIN	1
#define LEGION_KBD_BRIGHTNESS_MAX	2

/* Number of RGB zones */
#define LEGION_KBD_NUM_ZONES		4

/* Usage page for the vendor-specific RGB control interface */
#define LEGION_KBD_USAGE_PAGE		0xff89

struct legion_kbd_led {
	struct led_classdev_mc mc;
	struct mc_subled subleds[3];
	struct legion_kbd *kbd;	/* Back-pointer to parent */
	int zone;			/* Zone index (0-3) */
};

struct legion_kbd {
	struct hid_device *hdev;
	struct mutex lock;

	/* Current keyboard state (cached for report reconstruction) */
	u8 effect;
	u8 speed;
	u8 brightness;
	u8 rgb[LEGION_KBD_NUM_ZONES][3]; /* Per-zone RGB values */

	/* Zone colors saved across suspend */
	u8 suspend_rgb[LEGION_KBD_NUM_ZONES][3];

	/* LED class devices - one per zone */
	struct legion_kbd_led leds[LEGION_KBD_NUM_ZONES];
};

static const char *const legion_kbd_effect_text[] = {
	[LEGION_KBD_EFFECT_STATIC] = "static",
	[LEGION_KBD_EFFECT_BREATH] = "breath",
	[LEGION_KBD_EFFECT_SMOOTH] = "smooth",
	[LEGION_KBD_EFFECT_WAVE] = "wave",
};

static int legion_kbd_send_report(struct legion_kbd *kbd)
{
	struct hid_device *hdev = kbd->hdev;
	unsigned char report[LEGION_KBD_REPORT_SIZE];
	int ret;

	memset(report, 0, sizeof(report));
	report[0] = LEGION_KBD_REPORT_ID;
	report[1] = LEGION_KBD_CMD_SET_LIGHTING;
	report[2] = kbd->effect;
	report[3] = kbd->speed;
	report[4] = kbd->brightness;

	/* Copy RGB values for all zones */
	memcpy(&report[5], kbd->rgb, sizeof(kbd->rgb));

	/* Wave direction: default to left wave */
	if (kbd->effect == LEGION_KBD_EFFECT_WAVE)
		report[19] = 0x01; /* Left wave */

	ret = hid_hw_output_report(hdev, report, sizeof(report));
	if (ret < 0)
		return ret;
	if (ret != sizeof(report))
		return -EINVAL;

	return 0;
}

/*
 * LED classdev brightness_set_blocking callback.
 * Called when user writes to the 'brightness' sysfs attribute.
 * The multicolor framework updates subled intensities before calling this.
 */
static int legion_kbd_led_set_brightness(struct led_classdev *led_cdev,
					  enum led_brightness brightness)
{
	struct led_classdev_mc *mc_cdev = lcdev_to_mccdev(led_cdev);
	struct legion_kbd_led *zone_led = container_of(mc_cdev,
						struct legion_kbd_led, mc);
	struct legion_kbd *kbd = zone_led->kbd;
	int zone = zone_led->zone;
	int i, ret;

	mutex_lock(&kbd->lock);

	/* Update the RGB values for this zone from the subled intensities */
	for (i = 0; i < 3; i++)
		kbd->rgb[zone][i] = mc_cdev->subled_info[i].intensity;

	/* Send the updated state to the hardware */
	ret = legion_kbd_send_report(kbd);

	mutex_unlock(&kbd->lock);

	return ret;
}

static int legion_kbd_register_leds(struct legion_kbd *kbd)
{
	struct hid_device *hdev = kbd->hdev;
	int ret, i;

	for (i = 0; i < LEGION_KBD_NUM_ZONES; i++) {
		struct led_init_data led_init = { };
		struct legion_kbd_led *led = &kbd->leds[i];
		struct led_classdev *cdev = &led->mc.led_cdev;
		char *label;

		led->kbd = kbd;
		led->zone = i;

		label = devm_kasprintf(&hdev->dev, GFP_KERNEL,
					"legion-kbd:rgb:zone%d", i);
		if (!label)
			return -ENOMEM;

		cdev->color = LED_COLOR_ID_RGB;
		cdev->max_brightness = 255;
		cdev->brightness_set_blocking = legion_kbd_led_set_brightness;

		led->subleds[0] = (struct mc_subled){
			.color_index = LED_COLOR_ID_RED,
		};
		led->subleds[1] = (struct mc_subled){
			.color_index = LED_COLOR_ID_GREEN,
		};
		led->subleds[2] = (struct mc_subled){
			.color_index = LED_COLOR_ID_BLUE,
		};

		led->mc.num_colors = ARRAY_SIZE(led->subleds);
		led->mc.subled_info = led->subleds;

		led_init.default_label = label;
		ret = devm_led_classdev_multicolor_register_ext(&hdev->dev,
								 &led->mc, &led_init);
		if (ret) {
			dev_err(&hdev->dev,
				"Failed to register LED zone %d: %d\n", i, ret);
			return ret;
		}
	}

	return 0;
}

/* ---- Custom sysfs attributes ---- */

static ssize_t effect_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct legion_kbd *kbd = dev_get_drvdata(dev);
	int index;

	mutex_lock(&kbd->lock);
	index = kbd->effect;
	mutex_unlock(&kbd->lock);

	if (index < ARRAY_SIZE(legion_kbd_effect_text) &&
	    legion_kbd_effect_text[index])
		return sysfs_emit(buf, "%s\n", legion_kbd_effect_text[index]);

	return sysfs_emit(buf, "unknown\n");
}

static ssize_t effect_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct legion_kbd *kbd = dev_get_drvdata(dev);
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(legion_kbd_effect_text); i++) {
		if (!legion_kbd_effect_text[i])
			continue;
		if (sysfs_streq(buf, legion_kbd_effect_text[i]))
			break;
	}
	if (i >= ARRAY_SIZE(legion_kbd_effect_text) || !legion_kbd_effect_text[i])
		return -EINVAL;

	mutex_lock(&kbd->lock);
	kbd->effect = i;
	ret = legion_kbd_send_report(kbd);
	mutex_unlock(&kbd->lock);

	if (ret)
		return ret;
	return count;
}
static DEVICE_ATTR_RW(effect);

static ssize_t speed_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct legion_kbd *kbd = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", kbd->speed);
}

static ssize_t speed_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct legion_kbd *kbd = dev_get_drvdata(dev);
	unsigned int speed;
	int ret;

	ret = kstrtouint(buf, 10, &speed);
	if (ret)
		return ret;
	if (speed < LEGION_KBD_SPEED_MIN || speed > LEGION_KBD_SPEED_MAX)
		return -ERANGE;

	mutex_lock(&kbd->lock);
	kbd->speed = speed;
	ret = legion_kbd_send_report(kbd);
	mutex_unlock(&kbd->lock);

	if (ret)
		return ret;
	return count;
}
static DEVICE_ATTR_RW(speed);

static ssize_t brightness_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct legion_kbd *kbd = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", kbd->brightness);
}

static ssize_t brightness_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct legion_kbd *kbd = dev_get_drvdata(dev);
	unsigned int brightness;
	int ret;

	ret = kstrtouint(buf, 10, &brightness);
	if (ret)
		return ret;
	if (brightness < LEGION_KBD_BRIGHTNESS_MIN ||
	    brightness > LEGION_KBD_BRIGHTNESS_MAX)
		return -ERANGE;

	mutex_lock(&kbd->lock);
	kbd->brightness = brightness;
	ret = legion_kbd_send_report(kbd);
	mutex_unlock(&kbd->lock);

	if (ret)
		return ret;
	return count;
}
static DEVICE_ATTR_RW(brightness);

static struct attribute *legion_kbd_attrs[] = {
	&dev_attr_effect.attr,
	&dev_attr_speed.attr,
	&dev_attr_brightness.attr,
	NULL,
};
ATTRIBUTE_GROUPS(legion_kbd);

/* ---- HID driver callbacks ---- */

static int legion_kbd_probe(struct hid_device *hdev,
			    const struct hid_device_id *id)
{
	struct legion_kbd *kbd;
	int ret;

	/*
	 * This device has multiple USB interfaces: a standard HID keyboard
	 * interface and a vendor-specific interface for RGB control. Both
	 * interfaces share the same vendor/product ID and thus match our id
	 * table. Since hid-generic steps aside for any device another driver
	 * claims, we must bind both interfaces ourselves: the keyboard
	 * interface is simply left running in generic mode, and only the
	 * vendor-specific interface gets the RGB handling below.
	 *
	 * Note that hdev->collection is only populated by hid_parse(), so
	 * the interface can only be identified after parsing.
	 */
	ret = hid_parse(hdev);
	if (ret) {
		dev_err(&hdev->dev, "Failed to parse HID report: %d\n", ret);
		return ret;
	}

	/* Standard keyboard interface: leave it running in generic mode */
	if ((hdev->collection[0].usage >> 16) != LEGION_KBD_USAGE_PAGE) {
		ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
		if (ret)
			dev_err(&hdev->dev, "Failed to start HID: %d\n", ret);
		return ret;
	}

	/*
	 * Vendor RGB interface: no input devices, only this driver talks to
	 * it.
	 */
	ret = hid_hw_start(hdev, HID_CONNECT_DRIVER);
	if (ret) {
		dev_err(&hdev->dev, "Failed to start HID: %d\n", ret);
		return ret;
	}

	/* Vendor RGB interface: set up per-zone LED controls */
	kbd = devm_kzalloc(&hdev->dev, sizeof(*kbd), GFP_KERNEL);
	if (!kbd) {
		ret = -ENOMEM;
		goto err_stop;
	}

	kbd->hdev = hdev;
	mutex_init(&kbd->lock);

	/* Initialize default state */
	kbd->effect = LEGION_KBD_EFFECT_STATIC;
	kbd->speed = LEGION_KBD_SPEED_MIN;
	kbd->brightness = LEGION_KBD_BRIGHTNESS_MIN;
	memset(kbd->rgb, 0, sizeof(kbd->rgb));

	hid_set_drvdata(hdev, kbd);

	ret = legion_kbd_register_leds(kbd);
	if (ret) {
		dev_err(&hdev->dev, "Failed to register LEDs: %d\n", ret);
		goto err_stop;
	}

	ret = devm_device_add_group(&hdev->dev, legion_kbd_groups[0]);
	if (ret) {
		dev_err(&hdev->dev, "Failed to add sysfs group: %d\n", ret);
		goto err_stop;
	}

	/* Send initial state to turn off all LEDs */
	mutex_lock(&kbd->lock);
	ret = legion_kbd_send_report(kbd);
	mutex_unlock(&kbd->lock);
	if (ret)
		dev_warn(&hdev->dev, "Failed to send initial report: %d\n", ret);

	return 0;

err_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void legion_kbd_remove(struct hid_device *hdev)
{
	struct legion_kbd *kbd = hid_get_drvdata(hdev);

	/* Turn off LEDs on removal */
	if (kbd) {
		mutex_lock(&kbd->lock);
		legion_kbd_send_report(kbd);
		mutex_unlock(&kbd->lock);
	}

	hid_hw_stop(hdev);
}

/*
 * The transport-level suspend/resume is handled by the USB HID core, which
 * dispatches to these callbacks via hid_driver_suspend()/hid_driver_resume().
 */
static int legion_kbd_suspend(struct hid_device *hdev, pm_message_t state)
{
	struct legion_kbd *kbd = hid_get_drvdata(hdev);

	if (!kbd)
		return 0;

	/*
	 * Turn off the RGB zones while the device is suspended. The current
	 * zone colors are saved so that resume() can restore them.
	 */
	mutex_lock(&kbd->lock);
	memcpy(kbd->suspend_rgb, kbd->rgb, sizeof(kbd->suspend_rgb));
	memset(kbd->rgb, 0, sizeof(kbd->rgb));
	legion_kbd_send_report(kbd);
	mutex_unlock(&kbd->lock);

	return 0;
}

static int legion_kbd_resume(struct hid_device *hdev)
{
	struct legion_kbd *kbd = hid_get_drvdata(hdev);

	if (!kbd)
		return 0;

	/* Restore the RGB state saved at suspend */
	mutex_lock(&kbd->lock);
	memcpy(kbd->rgb, kbd->suspend_rgb, sizeof(kbd->rgb));
	legion_kbd_send_report(kbd);
	mutex_unlock(&kbd->lock);

	return 0;
}

/*
 * Device ID table.
 *
 * These keyboards have multiple USB interfaces: a standard HID keyboard
 * interface and a vendor-specific interface for RGB control. We match on
 * vendor/product and distinguish the interfaces by usage page in the probe
 * function, binding both (see legion_kbd_probe()).
 */
static const struct hid_device_id legion_kbd_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_ITE,
			 USB_DEVICE_ID_ITE_LENOVO_LEGION_KBD_2024_PRO) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_ITE,
			 USB_DEVICE_ID_ITE_LENOVO_LEGION_KBD_2024) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_ITE,
			 USB_DEVICE_ID_ITE_LENOVO_LEGION_KBD_2024_LOQ) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_ITE,
			 USB_DEVICE_ID_ITE_LENOVO_LEGION_KBD_2023_PRO) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_ITE,
			 USB_DEVICE_ID_ITE_LENOVO_LEGION_KBD_2023) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_ITE,
			 USB_DEVICE_ID_ITE_LENOVO_LEGION_KBD_2023_LOQ) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_ITE,
			 USB_DEVICE_ID_ITE_LENOVO_LEGION_KBD_2022) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_ITE,
			 USB_DEVICE_ID_ITE_LENOVO_LEGION_KBD_2022_IDEAPAD) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_ITE,
			 USB_DEVICE_ID_ITE_LENOVO_LEGION_KBD_2021) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_ITE,
			 USB_DEVICE_ID_ITE_LENOVO_LEGION_KBD_2021_IDEAPAD) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_ITE,
			 USB_DEVICE_ID_ITE_LENOVO_LEGION_KBD_2020) },
	{ },
};
MODULE_DEVICE_TABLE(hid, legion_kbd_devices);

static struct hid_driver legion_kbd_driver = {
	.name = "legion-kbd",
	.id_table = legion_kbd_devices,
	.probe = legion_kbd_probe,
	.remove = legion_kbd_remove,
	.suspend = pm_ptr(legion_kbd_suspend),
	.resume = pm_ptr(legion_kbd_resume),
	.reset_resume = pm_ptr(legion_kbd_resume),
};
module_hid_driver(legion_kbd_driver);

MODULE_DESCRIPTION("Lenovo Legion/Ideapad 4-zone RGB keyboard backlight driver");
MODULE_AUTHOR("Sean Farley <sean@farley.ws>");
MODULE_LICENSE("GPL");
