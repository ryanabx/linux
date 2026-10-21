// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6393 PMIC regulator driver
 *
 * The MT6393 is an I2C power management IC with four bucks and an LDO.
 * Only the first buck (VPROC, the CPU supply) is adjustable; the others
 * have fixed outputs and can only be switched.
 */

#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define MT6393_PRODUCT_ID		0x00
#define MT6393_MANUFACTURE_ID		0x01
#define MT6393_REVISION_NUMBER		0x02
#define MT6393_CH1_VID_SETTING		0x06
#define MT6393_INTERNAL_ENABLE		0x08
#define MT6393_CH2_VOLTAGE_OFFSET	0x20

#define MT6393_CH1_VID_MASK		GENMASK(6, 0)
#define MT6393_CH1_EN			BIT(1)
#define MT6393_CH2_EN			BIT(2)
#define MT6393_CH3_EN			BIT(3)
#define MT6393_CH4_EN			BIT(4)
#define MT6393_CH5_EN			BIT(5)

enum mt6393_regulator_id {
	MT6393_ID_VPROC,
	MT6393_ID_VDDR,
	MT6393_ID_VIO18,
	MT6393_ID_VIO33,
	MT6393_ID_VSD,
};

static const struct regmap_config mt6393_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MT6393_CH2_VOLTAGE_OFFSET,
};

static const struct linear_range mt6393_vproc_range[] = {
	REGULATOR_LINEAR_RANGE(600000, 0, 0x7f, 6250),
};

static const struct regulator_ops mt6393_vproc_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_ops mt6393_fixed_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

#define MT6393_FIXED(_id, _match, _uV, _en)			\
	[MT6393_ID_##_id] = {					\
		.name = _match,					\
		.of_match = _match,				\
		.regulators_node = "regulators",		\
		.id = MT6393_ID_##_id,				\
		.ops = &mt6393_fixed_ops,			\
		.type = REGULATOR_VOLTAGE,			\
		.owner = THIS_MODULE,				\
		.n_voltages = 1,				\
		.fixed_uV = _uV,				\
		.enable_reg = MT6393_INTERNAL_ENABLE,		\
		.enable_mask = _en,				\
	}

static const struct regulator_desc mt6393_regulators[] = {
	[MT6393_ID_VPROC] = {
		.name = "buck-vproc",
		.of_match = "buck-vproc",
		.regulators_node = "regulators",
		.id = MT6393_ID_VPROC,
		.ops = &mt6393_vproc_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 0x80,
		.linear_ranges = mt6393_vproc_range,
		.n_linear_ranges = ARRAY_SIZE(mt6393_vproc_range),
		.vsel_reg = MT6393_CH1_VID_SETTING,
		.vsel_mask = MT6393_CH1_VID_MASK,
		.enable_reg = MT6393_INTERNAL_ENABLE,
		.enable_mask = MT6393_CH1_EN,
	},
	MT6393_FIXED(VDDR, "buck-vddr", 1100000, MT6393_CH2_EN),
	MT6393_FIXED(VIO18, "buck-vio18", 1800000, MT6393_CH3_EN),
	MT6393_FIXED(VIO33, "buck-vio33", 3300000, MT6393_CH4_EN),
	MT6393_FIXED(VSD, "ldo-vsd", 3300000, MT6393_CH5_EN),
};

static int mt6393_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct regulator_config config = { .dev = dev };
	struct regulator_dev *rdev;
	unsigned int product, rev;
	int i, ret;

	config.regmap = devm_regmap_init_i2c(i2c, &mt6393_regmap_config);
	if (IS_ERR(config.regmap))
		return dev_err_probe(dev, PTR_ERR(config.regmap),
				     "failed to init regmap\n");

	ret = regmap_read(config.regmap, MT6393_PRODUCT_ID, &product);
	if (!ret)
		ret = regmap_read(config.regmap, MT6393_REVISION_NUMBER, &rev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the chip ID\n");

	dev_dbg(dev, "product 0x%02x, revision 0x%02x\n", product, rev);

	for (i = 0; i < ARRAY_SIZE(mt6393_regulators); i++) {
		rdev = devm_regulator_register(dev, &mt6393_regulators[i],
					       &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     mt6393_regulators[i].name);
	}

	return 0;
}

static const struct of_device_id mt6393_of_match[] = {
	{ .compatible = "mediatek,mt6393-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6393_of_match);

static const struct i2c_device_id mt6393_i2c_id[] = {
	{ "mt6393" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt6393_i2c_id);

static struct i2c_driver mt6393_regulator_driver = {
	.driver = {
		.name = "mt6393-regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = mt6393_of_match,
	},
	.probe = mt6393_probe,
	.id_table = mt6393_i2c_id,
};
module_i2c_driver(mt6393_regulator_driver);

MODULE_DESCRIPTION("MediaTek MT6393 PMIC regulator driver");
MODULE_LICENSE("GPL");
