// SPDX-License-Identifier: GPL-2.0-only
/*
 * HID driver for Lenovo Legion / LOQ keyboards with RGB backlight
 *
 * Copyright (c) 2024-2025 Legion Keyboard Contributors
 *
 * This driver exposes 4 RGB backlight zones as LED class devices.
 * It communicates via USB HID Feature reports (33 bytes).
 *
 * Supported effects:
 *   - Static (0x01): Solid color for all zones
 *   - Breath (0x03): Breathing/pulsing effect
 *   - Smooth (0x06): Internal smooth color transition
 *   - Wave Left/Right (0x04): Wave traveling across zones
 */

#include <linux/hid.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include "hid-ids.h"

/* Report constants */
#define LEGIONKBD_REPORT_SIZE		33
#define LEGIONKBD_REPORT_ID		0xCC
#define LEGIONKBD_SUBCMD		0x16

/* Effect type codes */
#define LEGIONKBD_EFFECT_STATIC		0x01
#define LEGIONKBD_EFFECT_BREATH		0x03
#define LEGIONKBD_EFFECT_WAVE		0x04
#define LEGIONKBD_EFFECT_SMOOTH		0x06

/* Wave direction flags */
#define LEGIONKBD_WAVE_DIR_RIGHT	BIT(18)
#define LEGIONKBD_WAVE_DIR_LEFT		BIT(19)

/* Valid ranges */
#define LEGIONKBD_SPEED_MIN		1
#define LEGIONKBD_SPEED_MAX		4
#define LEGIONKBD_BRIGHTNESS_MIN	1
#define LEGIONKBD_BRIGHTNESS_MAX	2
#define LEGIONKBD_NUM_ZONES		4
#define LEGIONKBD_RGB_PER_ZONE		3

/* Device IDs - all share VID 0x048D (Mediatek/Lenovo) */
#define LEGIONKBD_VENDOR_ID		0x048D

/* 2024 models */
#define LEGIONKBD_PID_2024_PRO		0xC995
#define LEGIONKBD_PID_2024		0xC994
#define LEGIONKBD_PID_2024_LOQ		0xC993
/* 2023 models */
#define LEGIONKBD_PID_2023_PRO		0xC985
#define LEGIONKBD_PID_2023		0xC984
#define LEGIONKBD_PID_2023_LOQ		0xC983
/* 2022 models */
#define LEGIONKBD_PID_2022_5PRO		0xC975
#define LEGIONKBD_PID_2022_IDEAPAD	0xC973
/* 2021 models */
#define LEGIONKBD_PID_2021_5PRO		0xC965
#define LEGIONKBD_PID_2021_IDEAPAD	0xC963
/* 2020 model */
#define LEGIONKBD_PID_2020_5		0xC955

/*
 * led_zone_data - per-zone LED data
 * @led: LED class device for this zone
 * @hdev: parent HID device (for accessing transfer buffer)
 * @zone_index: zone number (0-3)
 * @color: current RGB color for this zone
 */
struct led_zone_data {
	struct led_classdev led;
	struct hid_device *hdev;
	unsigned int zone_index;
	u8 color[LEGIONKBD_RGB_PER_ZONE]; /* R, G, B */
};

/*
 * legionkbd_data - per-device driver data
 * @hdev: HID device
 * @lock: protects report transfer and state
 * @transfer_buf: DMA-safe buffer for HID reports
 * @zones: LED zone data
 * @effect: current effect type
 * @speed: current effect speed
 * @brightness: current backlight brightness (1-2)
 */
struct legionkbd_data {
	struct hid_device *hdev;
	struct mutex lock;
	u8 *transfer_buf;
	struct led_zone_data zones[LEGIONKBD_NUM_ZONES];
	u8 effect;
	u8 speed;
	u8 brightness;
};

/* Zone names for sysfs LED naming convention: legionkbd::kbd_backlight_zoneN */
static const char *const legionkbd_zone_names[] = {
	"legionkbd::kbd_backlight_zone1",
	"legionkbd::kbd_backlight_zone2",
	"legionkbd::kbd_backlight_zone3",
	"legionkbd::kbd_backlight_zone4",
};

/*
 * Build the 33-byte HID feature report.
 *
 * Report layout:
 *   Byte 0: Report ID (0xCC)
 *   Byte 1: Sub-command (0x16)
 *   Byte 2: Effect type (0x01=Static, 0x03=Breath, 0x04=Wave, 0x06=Smooth)
 *   Byte 3: Speed (1-4)
 *   Byte 4: Brightness (1-2)
 *   Bytes 5-16: RGB colors for 4 zones (R0,G0,B0, R1,G1,B1, R2,G2,B2, R3,G3,B3)
 *     - Only populated for Static and Breath effects
 *   Byte 17: Reserved
 *   Byte 18: Wave direction flag - Right (1 = wave right-to-left)
 *   Byte 19: Wave direction flag - Left (1 = wave left-to-right)
 *   Bytes 20-32: Reserved
 */
static void legionkbd_build_report(struct legionkbd_data *drv,
				   struct led_zone_data *active_zone)
{
	u8 *buf = drv->transfer_buf;
	unsigned int i;

	buf[0] = LEGIONKBD_REPORT_ID;
	buf[1] = LEGIONKBD_SUBCMD;
	buf[2] = drv->effect;
	buf[3] = drv->speed;
	buf[4] = drv->brightness;

	/* Only Static and Breath effects include RGB color data */
	if (drv->effect == LEGIONKBD_EFFECT_STATIC ||
	    drv->effect == LEGIONKBD_EFFECT_BREATH) {
		for (i = 0; i < LEGIONKBD_NUM_ZONES; i++) {
			int offset = 5 + i * LEGIONKBD_RGB_PER_ZONE;
			buf[offset + 0] = drv->zones[i].color[0]; /* R */
			buf[offset + 1] = drv->zones[i].color[1]; /* G */
			buf[offset + 2] = drv->zones[i].color[2]; /* B */
		}
	} else {
		/* Clear RGB data for other effects */
		memset(buf + 5, 0, LEGIONKBD_NUM_ZONES * LEGIONKBD_RGB_PER_ZONE);
	}

	/* Clear direction flags */
	buf[17] = 0;
	buf[18] = 0;
	buf[19] = 0;

	/* Wave direction flag */
	if (drv->effect == LEGIONKBD_EFFECT_WAVE) {
		buf[18] = 1; /* Right direction */
	}
}

static int legionkbd_send_report(struct legionkbd_data *drv)
{
	int ret;

	ret = hid_hw_raw_request(drv->hdev, LEGIONKBD_REPORT_ID,
				 drv->transfer_buf, LEGIONKBD_REPORT_SIZE,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0) {
		hid_err(drv->hdev, "hid_hw_raw_request failed: %d\n", ret);
		return ret;
	}
	if (ret != LEGIONKBD_REPORT_SIZE) {
		hid_err(drv->hdev, "Unexpected report write size: %d (expected %d)\n",
			ret, LEGIONKBD_REPORT_SIZE);
		return -EIO;
	}

	return 0;
}

static int legionkbd_zone_brightness_set_blocking(struct led_classdev *led_cdev,
						  enum led_brightness br)
{
	struct led_zone_data *zled = container_of(led_cdev, struct led_zone_data, led);
	struct legionkbd_data *drv = dev_get_drvdata(led_cdev->dev->parent);
	int ret;

	/* Ignore LED off during unregister */
	if (led_cdev->flags & LED_UNREGISTERING)
		return 0;

	/* If setting to 0, just turn off this zone (brightness 0 in RGB) */
	if (br == LED_OFF) {
		zled->color[0] = 0;
		zled->color[1] = 0;
		zled->color[2] = 0;
	} else {
		/* Convert brightness to color values - scale to full RGB range */
		u8 c = (u8)br;
		zled->color[0] = c; /* R */
		zled->color[1] = c; /* G */
		zled->color[2] = c; /* B */
	}

	mutex_lock(&drv->lock);
	legionkbd_build_report(drv, zled);
	ret = legionkbd_send_report(drv);
	mutex_unlock(&drv->lock);

	return ret;
}

static enum led_brightness legionkbd_zone_brightness_get(struct led_classdev *led_cdev)
{
	struct led_zone_data *zled = container_of(led_cdev, struct led_zone_data, led);

	return zled->color[0]; /* R, G, B are all the same for single-color zones */
}

/*
 * Effect control helpers - exposed via debugfs or ioctls in the future.
 * For now, the driver defaults to Static effect with user-settable brightness.
 *
 * Userspace tools can set effects by writing to sysfs files we create
 * under /sys/class/leds/legionkbd::kbd_backlight_zone1/
 */

static const struct hid_device_id legionkbd_devices[] = {
	/* 2024 models */
	{ HID_USB_DEVICE(LEGIONKBD_VENDOR_ID, LEGIONKBD_PID_2024_PRO) },
	{ HID_USB_DEVICE(LEGIONKBD_VENDOR_ID, LEGIONKBD_PID_2024) },
	{ HID_USB_DEVICE(LEGIONKBD_VENDOR_ID, LEGIONKBD_PID_2024_LOQ) },
	/* 2023 models */
	{ HID_USB_DEVICE(LEGIONKBD_VENDOR_ID, LEGIONKBD_PID_2023_PRO) },
	{ HID_USB_DEVICE(LEGIONKBD_VENDOR_ID, LEGIONKBD_PID_2023) },
	{ HID_USB_DEVICE(LEGIONKBD_VENDOR_ID, LEGIONKBD_PID_2023_LOQ) },
	/* 2022 models */
	{ HID_USB_DEVICE(LEGIONKBD_VENDOR_ID, LEGIONKBD_PID_2022_5PRO) },
	{ HID_USB_DEVICE(LEGIONKBD_VENDOR_ID, LEGIONKBD_PID_2022_IDEAPAD) },
	/* 2021 models */
	{ HID_USB_DEVICE(LEGIONKBD_VENDOR_ID, LEGIONKBD_PID_2021_5PRO) },
	{ HID_USB_DEVICE(LEGIONKBD_VENDOR_ID, LEGIONKBD_PID_2021_IDEAPAD) },
	/* 2020 model */
	{ HID_USB_DEVICE(LEGIONKBD_VENDOR_ID, LEGIONKBD_PID_2020_5) },
	{ }
};
MODULE_DEVICE_TABLE(hid, legionkbd_devices);

static int legionkbd_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct legionkbd_data *drv;
	int ret, i;

	drv = devm_kzalloc(&hdev->dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	drv->hdev = hdev;
	mutex_init(&drv->lock);

	drv->transfer_buf = devm_kmalloc(&hdev->dev, LEGIONKBD_REPORT_SIZE, GFP_KERNEL);
	if (!drv->transfer_buf)
		return -ENOMEM;

	drv->effect = LEGIONKBD_EFFECT_STATIC;
	drv->speed = 2;
	drv->brightness = LEGIONKBD_BRIGHTNESS_MAX;

	hid_set_drvdata(hdev, drv);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		return ret;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		return ret;
	}

	/* Initialize all zones to off */
	for (i = 0; i < LEGIONKBD_NUM_ZONES; i++) {
		struct led_zone_data *zled = &drv->zones[i];

		zled->hdev = hdev;
		zled->zone_index = i;
		zled->color[0] = 0;
		zled->color[1] = 0;
		zled->color[2] = 0;

		zled->led.name = legionkbd_zone_names[i];
		zled->led.brightness = LED_OFF;
		zled->led.max_brightness = 255;
		zled->led.brightness_set_blocking = legionkbd_zone_brightness_set_blocking;
		zled->led.brightness_get = legionkbd_zone_brightness_get;
		zled->led.flags = LED_HW_PLUGGABLE;

		ret = devm_led_classdev_register(&hdev->dev, &zled->led);
		if (ret) {
			hid_err(hdev, "failed to register zone %d LED: %d\n", i, ret);
			return ret;
		}
	}

	/* Send initial report (all zones off) */
	mutex_lock(&drv->lock);
	legionkbd_build_report(drv, NULL);
	ret = legionkbd_send_report(drv);
	mutex_unlock(&drv->lock);

	if (ret)
		hid_warn(hdev, "initial report failed: %d\n", ret);

	hid_info(hdev, "initialized - %d RGB zones registered\n", LEGIONKBD_NUM_ZONES);
	return 0;
}

static void legionkbd_remove(struct hid_device *hdev)
{
	struct legionkbd_data *drv = hid_get_drvdata(hdev);

	/* Turn off all LEDs before disconnect */
	mutex_lock(&drv->lock);
	drv->brightness = 0;
	legionkbd_build_report(drv, NULL);
	legionkbd_send_report(drv);
	drv->brightness = LEGIONKBD_BRIGHTNESS_MAX;
	mutex_unlock(&drv->lock);

	hid_hw_stop(hdev);
}

static struct hid_driver legionkbd_driver = {
	.name = "legionkbd",
	.id_table = legionkbd_devices,
	.probe = legionkbd_probe,
	.remove = legionkbd_remove,
};

module_hid_driver(legionkbd_driver);

MODULE_AUTHOR("Legion Keyboard Contributors");
MODULE_DESCRIPTION("HID driver for Lenovo Legion / LOQ RGB keyboard backlight");
MODULE_LICENSE("GPL");
