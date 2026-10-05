// SPDX-License-Identifier: BSD-3-Clause-Clear
/* Copyright (C) 2026 Ryan Brue <ryanbrue.dev@gmail.com> */

#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/module.h>

#include "mt6630.h"
#include "mac.h"

#define MT6630_WLAN_FUNC		1
#define MT6630_BT_FUNC			2

#define MT6630S_WHIER	(WHIER_TX_DONE_INT_EN | WHIER_RX0_DONE_INT_EN | \
			 WHIER_RX1_DONE_INT_EN)

static const struct sdio_device_id mt6630s_table[] = {
	{ SDIO_DEVICE(SDIO_VENDOR_ID_MEDIATEK, SDIO_DEVICE_ID_MEDIATEK_MT6630) },
	{ }	/* Terminating entry */
};
MODULE_DEVICE_TABLE(sdio, mt6630s_table);

static void mt6630s_txrx_worker(struct mt76_worker *w)
{
	struct mt76_sdio *sdio = container_of(w, struct mt76_sdio,
					      txrx_worker);

	mt76s_txrx_worker(sdio);
}

/* The chip counts every released TX page in its free-for-all queue and,
 * separately and often in a later report, a command's pages in its CPU
 * queue. mt76 reads connac1 PSE/PLE groups instead, so the counts are
 * carried over to the data and MCU groups: commands get the CPU count, and
 * data what the free-for-all count has in excess of it over time.
 *
 * The budgets the driver keeps are a fraction of the firmware's pool, which
 * is what lets them be approximate: the TX descriptor is not charged to data
 * frames, and with no PLE on this chip its count is just kept from growing
 * past the pool.
 */
static int mt6630s_parse_intr(struct mt76_dev *mdev, struct mt76s_intr *intr)
{
	struct mt6630_dev *dev = container_of(mdev, struct mt6630_dev, mt76);
	struct mt76_sdio *sdio = &mdev->sdio;
	struct mt6630_sdio_intr *irq = sdio->intr_data;
	int i, err, data;
	u16 mcu;

	sdio_claim_host(sdio->func);
	err = sdio_readsb(sdio->func, irq, MCR_WHISR, sizeof(*irq));
	sdio_release_host(sdio->func);

	if (err)
		return err;

	if (irq->isr & WHISR_FW_ASSERT)
		dev_err_ratelimited(mdev->dev,
				    "Firmware assert, mailbox %08x %08x\n",
				    irq->rec_mb[0], irq->rec_mb[1]);

	intr->isr = irq->isr;
	intr->rec_mb = irq->rec_mb;
	intr->rx.num = irq->rx_num;
	for (i = 0; i < 2; i++)
		intr->rx.len[i] = irq->rx_len[i];

	mcu = irq->tx_cnt[MT6630_HIF_TX_CPU];
	dev->tx_balance += irq->tx_cnt[MT6630_HIF_TX_FFA] - mcu;
	data = max(dev->tx_balance, 0);
	dev->tx_balance -= data;

	memset(dev->tx_quota, 0, sizeof(dev->tx_quota));
	dev->tx_quota[0] = FIELD_PREP(TXQ_CNT_L, data);
	dev->tx_quota[2] = FIELD_PREP(TXQ_CNT_L, mcu) |
			   FIELD_PREP(TXQ_CNT_H, data);
	intr->tx.wtqcr = dev->tx_quota;

	/* mt76s_refill_sched_quota() adds the counts above after this */
	sdio->sched.pse_data_quota = min(sdio->sched.pse_data_quota,
					 MT6630_TX_RING_PAGES -
					 MT6630_TX_MCU_PAGES - data);
	sdio->sched.ple_data_quota = min(sdio->sched.ple_data_quota,
					 MT6630_TX_RING_PAGES - data);

	return 0;
}

static u32 mt6630s_read_pcr(struct sdio_func *func)
{
	return sdio_readl(func, MCR_WHLPCR, NULL);
}

static int mt6630s_drv_own(struct sdio_func *func)
{
	u32 val;
	int ret;

	sdio_writel(func, WHLPCR_INT_EN_CLR | WHLPCR_FW_OWN_REQ_CLR,
		    MCR_WHLPCR, &ret);
	if (ret)
		return ret;

	return readx_poll_timeout(mt6630s_read_pcr, func, val,
				  val & WHLPCR_IS_DRIVER_OWN, 2000, 1000000);
}

static int mt6630s_hw_init(struct mt6630_dev *dev, struct sdio_func *func)
{
	u32 val;
	int ret;

	sdio_claim_host(func);

	ret = sdio_enable_func(func);
	if (ret)
		goto release;

	ret = mt6630s_drv_own(func);
	if (ret) {
		dev_err(dev->mt76.dev, "Cannot get ownership from device\n");
		goto disable;
	}

	ret = sdio_set_block_size(func, 512);
	if (ret)
		goto disable;

	val = sdio_readl(func, MCR_WCIR, &ret);
	if (ret)
		goto disable;

	if (FIELD_GET(WCIR_CHIP_ID, val) != MT6630_CHIP_ID) {
		dev_err(dev->mt76.dev, "Unexpected chip ID %#lx\n",
			FIELD_GET(WCIR_CHIP_ID, val));
		ret = -ENODEV;
		goto disable;
	}
	dev->mt76.rev = (MT6630_CHIP_ID << 16) |
			FIELD_GET(WCIR_REVISION, val);

	sdio_release_host(func);

	return 0;

disable:
	sdio_disable_func(func);
release:
	sdio_release_host(func);

	return ret;
}

static int mt6630s_irq_init(struct mt6630_dev *dev, struct sdio_func *func)
{
	u32 ctrl;
	int ret;

	sdio_claim_host(func);

	/* read-clear status, no length tail, no limit on reported lengths */
	ctrl = sdio_readl(func, MCR_WHCR, &ret);
	if (ret)
		goto out;

	ctrl &= ~(W_INT_CLR_CTRL | RECV_MAILBOX_RD_CLR_EN |
		  MAX_HIF_RX_LEN_NUM | RX_ENHANCE_MODE);
	sdio_writel(func, ctrl, MCR_WHCR, &ret);
	if (ret)
		goto out;

	sdio_writel(func, MT6630S_WHIER, MCR_WHIER, &ret);
	if (ret)
		goto out;

	ret = sdio_claim_irq(func, mt76s_sdio_irq);
	if (ret)
		goto out;

	sdio_writel(func, WHLPCR_INT_EN_SET, MCR_WHLPCR, &ret);
	if (ret)
		sdio_release_irq(func);
out:
	sdio_release_host(func);

	return ret;
}

/* The WLAN firmware cannot start until the ROM patch is in, and only the
 * Bluetooth function's WMT channel can load the patch, which btmtksdio does
 * at probe time. Hold off until that driver is bound.
 */
static int mt6630s_wait_for_patch(struct sdio_func *func)
{
	struct sdio_func *bt = func->card->sdio_func[MT6630_BT_FUNC - 1];
	struct device_link *link;

	if (!bt)
		return -ENODEV;

	/* Probed as soon as it is added, before function 2 is registered */
	if (!device_is_registered(&bt->dev))
		return dev_err_probe(&func->dev, -EPROBE_DEFER,
				     "Waiting for the Bluetooth function\n");

	link = device_link_add(&func->dev, &bt->dev,
			       DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!link)
		return -EINVAL;

	if (!device_is_bound(&bt->dev))
		return dev_err_probe(&func->dev, -EPROBE_DEFER,
				     "Waiting for the Bluetooth function\n");

	return 0;
}

static int mt6630s_probe(struct sdio_func *func,
			 const struct sdio_device_id *id)
{
	static const struct mt76_bus_ops mt6630s_ops = {
		.rr = mt76s_rr,
		.rmw = mt76s_rmw,
		.wr = mt76s_wr,
		.write_copy = mt76s_write_copy,
		.read_copy = mt76s_read_copy,
		.wr_rp = mt76s_wr_rp,
		.rd_rp = mt76s_rd_rp,
		.type = MT76_BUS_SDIO,
	};
	struct mt76_sdio *sdio;
	struct mt6630_dev *dev;
	struct mt76_dev *mdev;
	int ret;

	/* both functions carry the same IDs */
	if (func->num != MT6630_WLAN_FUNC)
		return -ENODEV;

	ret = mt6630s_wait_for_patch(func);
	if (ret)
		return ret;

	mdev = mt76_alloc_device(&func->dev, sizeof(*dev), &mt6630_ops,
				 &mt6630_drv_ops);
	if (!mdev)
		return -ENOMEM;

	dev = container_of(mdev, struct mt6630_dev, mt76);
	sdio_set_drvdata(func, dev);
	sdio = &mdev->sdio;

	ret = mt76s_init(mdev, func, &mt6630s_ops);
	if (ret)
		goto err_free;

	mdev->mcu_ops = &mt6630_mcu_ops;
	mdev->mcu.timeout = 3 * HZ;
	sdio->parse_irq = mt6630s_parse_intr;
	sdio->intr_data = devm_kmalloc(mdev->dev,
				       sizeof(struct mt6630_sdio_intr),
				       GFP_KERNEL);
	if (!sdio->intr_data) {
		ret = -ENOMEM;
		goto err_deinit;
	}

	ret = mt76s_alloc_rx_queue(mdev, MT_RXQ_MAIN);
	if (ret)
		goto err_deinit;

	/* events and frames come on port 0 and port 1 alike */
	ret = mt76s_alloc_rx_queue(mdev, MT_RXQ_MCU);
	if (ret)
		goto err_deinit;

	ret = mt76s_alloc_tx(mdev);
	if (ret)
		goto err_deinit;

	ret = mt76_worker_setup(mdev->hw, &sdio->txrx_worker,
				mt6630s_txrx_worker, "sdio-txrx");
	if (ret)
		goto err_deinit;

	sched_set_fifo_low(sdio->txrx_worker.task);

	ret = mt6630s_hw_init(dev, func);
	if (ret)
		goto err_deinit;

	ret = mt6630_mcu_init(dev);
	if (ret)
		goto err_disable;

	sdio->sched.pse_page_size = MT_PSE_PAGE_SZ;
	sdio->sched.pse_data_quota = MT6630_TX_RING_PAGES -
				     MT6630_TX_MCU_PAGES;
	sdio->sched.ple_data_quota = MT6630_TX_RING_PAGES;
	sdio->sched.pse_mcu_quota = MT6630_TX_MCU_PAGES;
	sdio->pse_mcu_quota_max = MT6630_TX_MCU_PAGES;

	set_bit(MT76_STATE_MCU_RUNNING, &mdev->phy.state);
	set_bit(MT76_STATE_INITIALIZED, &mdev->phy.state);

	ret = mt6630s_irq_init(dev, func);
	if (ret)
		goto err_stop;

	ret = mt6630_register_device(dev);
	if (ret)
		goto err_stop;

	return 0;

err_stop:
	/* a running firmware does not take a second download */
	mt6630_mcu_stop(dev);
	sdio_claim_host(func);
	sdio_writel(func, WHLPCR_FW_OWN_REQ_SET, MCR_WHLPCR, NULL);
	sdio_release_host(func);
err_disable:
	clear_bit(MT76_STATE_INITIALIZED, &mdev->phy.state);
	mt76s_deinit(mdev);
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
	goto err_free;
err_deinit:
	mt76s_deinit(mdev);
err_free:
	mt76_free_device(mdev);

	return ret;
}

static void mt6630s_remove(struct sdio_func *func)
{
	struct mt6630_dev *dev = sdio_get_drvdata(func);
	struct mt76_dev *mdev = &dev->mt76;

	mt6630_unregister_device(dev);

	if (mt6630_mcu_stop(dev))
		dev_warn(mdev->dev, "Firmware did not stop\n");

	clear_bit(MT76_STATE_INITIALIZED, &mdev->phy.state);
	mt76s_deinit(mdev);

	sdio_claim_host(func);
	sdio_writel(func, WHLPCR_FW_OWN_REQ_SET, MCR_WHLPCR, NULL);
	sdio_disable_func(func);
	sdio_release_host(func);

	mt76_free_device(mdev);
}

/* The firmware lives in the chip's RAM, and powering the card down would lose
 * it along with the ROM patch only the Bluetooth function can put back, so
 * the card stays powered.
 *
 * The stat worker is left alone: TX status comes in MCU events, there is no
 * tx_status_data callback, and mt76_worker_enable() would schedule it.
 */
static int mt6630s_suspend(struct device *dev)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct mt6630_dev *mdev = sdio_get_drvdata(func);
	struct mt76_sdio *sdio = &mdev->mt76.sdio;
	int ret;

	ret = sdio_set_host_pm_flags(func, MMC_PM_KEEP_POWER);
	if (ret)
		return ret;

	mt76_worker_disable(&sdio->txrx_worker);
	mt76_worker_disable(&sdio->status_worker);
	mt76_worker_disable(&sdio->net_worker);

	return 0;
}

static int mt6630s_resume(struct device *dev)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct mt6630_dev *mdev = sdio_get_drvdata(func);
	struct mt76_sdio *sdio = &mdev->mt76.sdio;

	/* each runs once, collecting what arrived meanwhile */
	mt76_worker_enable(&sdio->txrx_worker);
	mt76_worker_enable(&sdio->status_worker);
	mt76_worker_enable(&sdio->net_worker);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mt6630s_pm_ops, mt6630s_suspend,
				mt6630s_resume);

static struct sdio_driver mt6630s_driver = {
	.name		= KBUILD_MODNAME,
	.probe		= mt6630s_probe,
	.remove		= mt6630s_remove,
	.id_table	= mt6630s_table,
	.drv.pm		= pm_sleep_ptr(&mt6630s_pm_ops),
};
module_sdio_driver(mt6630s_driver);

MODULE_FIRMWARE(MT6630_FIRMWARE);
MODULE_DESCRIPTION("MediaTek MT6630 SDIO wireless driver");
MODULE_AUTHOR("Ryan Brue <ryanbrue.dev@gmail.com>");
MODULE_LICENSE("Dual BSD/GPL");
