// SPDX-License-Identifier: GPL-2.0-only
/*
 * Reboot mode for the bootloader of the Samsung Galaxy Tab 2 (espresso)
 *
 * The bootloader reads the boot mode from a flag word in the OMAP4 SAR RAM,
 * but only if the word before it holds the tag that the vendor kernel writes
 * with it. At every boot the bootloader leaves its own tag ("NORM") there, so
 * the tag has to be written again together with the flag.
 */

#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot-mode.h>
#include <linux/regmap.h>

#define ESPRESSO_REBOOT_TAG_OFFSET	0xff4
#define ESPRESSO_REBOOT_FLAG_OFFSET	0xff8

/* "RESE", the first four bytes of the vendor kernel's "RESET" tag */
#define ESPRESSO_REBOOT_TAG_RESET	0x45534552

struct espresso_reboot_mode {
	struct reboot_mode_driver reboot;
	struct regmap *map;
};

static int espresso_reboot_mode_write(struct reboot_mode_driver *reboot,
				      unsigned int magic)
{
	struct espresso_reboot_mode *erm =
		container_of(reboot, struct espresso_reboot_mode, reboot);
	int ret;

	ret = regmap_write(erm->map, ESPRESSO_REBOOT_FLAG_OFFSET, magic);
	if (ret)
		return ret;

	return regmap_write(erm->map, ESPRESSO_REBOOT_TAG_OFFSET,
			    ESPRESSO_REBOOT_TAG_RESET);
}

static int espresso_reboot_mode_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct espresso_reboot_mode *erm;
	int ret;

	erm = devm_kzalloc(dev, sizeof(*erm), GFP_KERNEL);
	if (!erm)
		return -ENOMEM;

	erm->map = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(erm->map))
		return dev_err_probe(dev, PTR_ERR(erm->map),
				     "failed to get the SAR RAM regmap\n");

	erm->reboot.dev = dev;
	erm->reboot.write = espresso_reboot_mode_write;

	ret = devm_reboot_mode_register(dev, &erm->reboot);
	if (ret)
		return dev_err_probe(dev, ret, "can't register reboot mode\n");

	return 0;
}

static const struct of_device_id espresso_reboot_mode_of_match[] = {
	{ .compatible = "samsung,espresso-reboot-mode" },
	{ }
};
MODULE_DEVICE_TABLE(of, espresso_reboot_mode_of_match);

static struct platform_driver espresso_reboot_mode_driver = {
	.probe = espresso_reboot_mode_probe,
	.driver = {
		.name = "samsung-espresso-reboot-mode",
		.of_match_table = espresso_reboot_mode_of_match,
	},
};
module_platform_driver(espresso_reboot_mode_driver);

MODULE_AUTHOR("Ryan Brue <ryanbrue.dev@gmail.com>");
MODULE_DESCRIPTION("Samsung Galaxy Tab 2 bootloader reboot mode driver");
MODULE_LICENSE("GPL");
