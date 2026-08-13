// SPDX-License-Identifier: GPL-2.0
/*
 * extcon-p30con.c - Samsung 30-pin ("P30") connector accessory detection
 *
 * The Samsung 30-pin dock connector found on Galaxy Tab (espresso and
 * friends) tablets identifies analog accessories by the resistance on
 * the ACCESSORY_ID pin, measured with an external ADC (STMPE811).
 * A dedicated detect line (DOCK_INT in the schematics) goes low while
 * any such accessory is attached.
 *
 * The USB OTG adapter ("Camera Connection Kit", ID pulled to ~2.2 V)
 * is reported as EXTCON_USB_HOST so that the USB PHY driver can switch
 * the MUSB controller to host mode.
 *
 * Voltage thresholds follow the vendor kernel (board-espresso-connector.c
 * and drivers/misc/30pin_con.c of the GT-P31xx/P51xx source drop).
 */

#include <linux/delay.h>
#include <linux/extcon-provider.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/limits.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

/*
 * P30 accessory ID levels, 12-bit ADC counts at 3.3 V full scale:
 *
 * Accessory		Vacc [V]		counts
 * OTG adapter		2.2 (2.1..2.3)		2731 (2606..2855)
 * Analog TV cable	1.8 (1.7..1.9)		2234 (2110..2359)
 * Car mount		1.38 (1.28..1.48)	1715 (1590..1839)
 * 3-pole earjack	0.99 (0.89..1.09)	1232 (1107..1356)
 */
#define P30_ADC_OTG_MIN		2600
#define P30_ADC_OTG_MAX		2860

/*
 * "HQRL Standard": voltage on ACCESSORY_ID is specified to be stable
 * no earlier than 400 ms after VBUS detection; the vendor kernel waits
 * 420 ms before sampling.
 */
#define P30_SETTLE_MS		420
#define P30_NUM_SAMPLES		5

struct p30con {
	struct device *dev;
	struct extcon_dev *edev;
	struct gpio_desc *detect_gpiod;
	struct iio_channel *acc_id_chan;
	struct delayed_work work;
	int irq;
	bool attached;
};

static const unsigned int p30con_cables[] = {
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static int p30con_read_acc_id(struct p30con *con)
{
	int i, ret, val;
	int lowest = INT_MAX, highest = INT_MIN, sum = 0;

	/*
	 * Average five samples after discarding the lowest and highest values,
	 * matching the vendor driver.
	 */
	for (i = 0; i < P30_NUM_SAMPLES; i++) {
		ret = iio_read_channel_raw(con->acc_id_chan, &val);
		if (ret < 0)
			return ret;

		sum += val;
		lowest = min(lowest, val);
		highest = max(highest, val);
		msleep(20);
	}

	return (sum - lowest - highest) / (P30_NUM_SAMPLES - 2);
}

static void p30con_detect_work(struct work_struct *work)
{
	struct p30con *con = container_of(work, struct p30con, work.work);
	bool attached = gpiod_get_value_cansleep(con->detect_gpiod);
	int adc;

	if (attached == con->attached)
		return;

	if (!attached) {
		con->attached = false;
		extcon_set_state_sync(con->edev, EXTCON_USB_HOST, false);
		dev_dbg(con->dev, "accessory detached\n");
		return;
	}

	msleep(P30_SETTLE_MS);

	/* Re-check: the cable may already be gone again */
	if (!gpiod_get_value_cansleep(con->detect_gpiod))
		return;

	adc = p30con_read_acc_id(con);
	if (adc < 0) {
		dev_err(con->dev, "failed to read ACCESSORY_ID: %d\n", adc);
		return;
	}

	con->attached = true;

	dev_dbg(con->dev, "accessory attached, ACCESSORY_ID = %d\n", adc);

	if (adc > P30_ADC_OTG_MIN && adc < P30_ADC_OTG_MAX)
		extcon_set_state_sync(con->edev, EXTCON_USB_HOST, true);
	else
		dev_info(con->dev, "unhandled accessory, ACCESSORY_ID = %d\n",
			 adc);
}

static irqreturn_t p30con_irq_handler(int irq, void *data)
{
	struct p30con *con = data;

	mod_delayed_work(system_wq, &con->work, msecs_to_jiffies(50));

	return IRQ_HANDLED;
}

static int p30con_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct p30con *con;
	int ret;

	con = devm_kzalloc(dev, sizeof(*con), GFP_KERNEL);
	if (!con)
		return -ENOMEM;

	con->dev = dev;
	platform_set_drvdata(pdev, con);
	INIT_DELAYED_WORK(&con->work, p30con_detect_work);

	con->detect_gpiod = devm_gpiod_get(dev, "detect", GPIOD_IN);
	if (IS_ERR(con->detect_gpiod))
		return dev_err_probe(dev, PTR_ERR(con->detect_gpiod),
				     "failed to get detect gpio\n");

	con->acc_id_chan = devm_iio_channel_get(dev, "accessory-id");
	if (IS_ERR(con->acc_id_chan))
		return dev_err_probe(dev, PTR_ERR(con->acc_id_chan),
				     "failed to get ACCESSORY_ID channel\n");

	con->edev = devm_extcon_dev_allocate(dev, p30con_cables);
	if (IS_ERR(con->edev))
		return PTR_ERR(con->edev);

	ret = devm_extcon_dev_register(dev, con->edev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register extcon\n");

	con->irq = gpiod_to_irq(con->detect_gpiod);
	if (con->irq < 0)
		return dev_err_probe(dev, con->irq,
				     "failed to get detect irq\n");

	ret = devm_request_irq(dev, con->irq, p30con_irq_handler,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       dev_name(dev), con);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	device_init_wakeup(dev, true);

	/* Pick up an accessory that is attached at boot */
	queue_delayed_work(system_wq, &con->work, msecs_to_jiffies(100));

	return 0;
}

static void p30con_remove(struct platform_device *pdev)
{
	struct p30con *con = platform_get_drvdata(pdev);

	devm_free_irq(&pdev->dev, con->irq, con);
	cancel_delayed_work_sync(&con->work);
}

static int __maybe_unused p30con_suspend(struct device *dev)
{
	struct p30con *con = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(con->irq);

	return 0;
}

static int __maybe_unused p30con_resume(struct device *dev)
{
	struct p30con *con = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(con->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(p30con_pm_ops, p30con_suspend, p30con_resume);

static const struct of_device_id p30con_of_match[] = {
	{ .compatible = "samsung,p30-connector" },
	{ },
};
MODULE_DEVICE_TABLE(of, p30con_of_match);

static struct platform_driver p30con_driver = {
	.probe = p30con_probe,
	.remove = p30con_remove,
	.driver = {
		.name = "extcon-p30con",
		.pm = &p30con_pm_ops,
		.of_match_table = p30con_of_match,
	},
};
module_platform_driver(p30con_driver);

MODULE_DESCRIPTION("Samsung 30-pin connector accessory detection driver");
MODULE_LICENSE("GPL");
