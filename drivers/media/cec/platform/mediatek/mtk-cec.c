// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek HDMI CEC controller (the second-generation CEC block of MT8695)
 *
 * The SoC has two CEC blocks: the first-generation one, which is also the
 * HDMI hotplug detector on MT8173 (drivers/gpu/drm/mediatek/mtk_cec.c), and
 * a newer one with hardware message framing, three logical addresses and
 * retransmission. A bit in the first block selects which of the two drives
 * the CEC pin. This driver switches the pin to the newer block and runs it.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#include <media/cec.h>
#include <media/cec-notifier.h>

/* First-generation block ("legacy") */
#define CEC1_CKGEN			0x04
#define CEC1_CKGEN_PDN			BIT(16)
#define CEC1_TR_TEST			0xbc
#define CEC1_TR_TEST_SEL_NEW		BIT(31)
/* The rest of the value written along with the switch, as the vendor does */
#define CEC1_TR_TEST_VAL		0x40004019

/* Second-generation block */
#define CEC_TR_CONFIG			0x00
#define CEC_TR_CONFIG_RX_CHK_DST	BIT(29)
#define CEC_TR_CONFIG_BYPASS		BIT(28)
#define CEC_TR_CONFIG_LA1		GENMASK(27, 24)
#define CEC_TR_CONFIG_LA2		GENMASK(23, 20)
#define CEC_TR_CONFIG_LA3		GENMASK(19, 16)
#define CEC_TR_CONFIG_RX_ACK_ERR_BYPASS	BIT(13)
#define CEC_TR_CONFIG_TX_RESET		BIT(1)
#define CEC_TR_CONFIG_RX_RESET		BIT(0)

#define CEC_CKGEN			0x04
#define CEC_CKGEN_TX_EN			BIT(20)
#define CEC_CKGEN_32K_EN		BIT(19)
#define CEC_CKGEN_27M_EN		BIT(18)
#define CEC_CKGEN_SEL_DIV		BIT(17)
#define CEC_CKGEN_PDN			BIT(16)
#define CEC_CKGEN_DIV			GENMASK(15, 0)
#define CEC_CKGEN_DIV_100K		0x82

#define CEC_RX_TIMER_START_R		0x08
#define CEC_RX_TIMER_START_F		0x0c
#define CEC_RX_TIMER_DATA		0x10
#define CEC_RX_TIMER_ACK		0x14
#define CEC_RX_TIMER_ERROR		0x18
#define CEC_TX_TIMER_START		0x1c
#define CEC_TX_TIMER_DATA_R		0x20
#define CEC_TX_TIMER_DATA_F		0x24
#define CEC_TX_TIMER_DATA_S		0x28

#define CEC_TX_ARB			0x2c
#define CEC_TX_ARB_MAX_RETX_ARB		GENMASK(29, 24)
#define CEC_TX_ARB_MAX_RETX_COL		GENMASK(23, 20)
#define CEC_TX_ARB_MAX_RETX_NAK		GENMASK(19, 16)
#define CEC_TX_ARB_BCNT_RETX		GENMASK(11, 8)
#define CEC_TX_ARB_BCNT_NEW_MSG		GENMASK(7, 4)
#define CEC_TX_ARB_BCNT_NEW_INIT	GENMASK(3, 0)

#define CEC_TX_HEADER			0x30
#define CEC_TX_HEADER_READY		BIT(16)
#define CEC_TX_HEADER_CNT		GENMASK(15, 12)
#define CEC_TX_HEADER_EOM		BIT(8)
#define CEC_TX_HEADER_SRC		GENMASK(7, 4)
#define CEC_TX_HEADER_DST		GENMASK(3, 0)
#define CEC_TX_DATA(n)			(0x34 + 4 * (n))

#define CEC_TX_TIMER_ACK		0x44
#define CEC_TX_TIMER_ACK_MASK		GENMASK(26, 0)
#define CEC_LINE_DET			0x48

#define CEC_RX_BUF_HEADER		0x4c
#define CEC_RX_BUF_RISC_ACK		BIT(16)
#define CEC_RX_BUF_CNT			GENMASK(15, 12)
#define CEC_RX_HEADER_EOM		BIT(8)
#define CEC_RX_HEADER_SRC		GENMASK(7, 4)
#define CEC_RX_HEADER_DST		GENMASK(3, 0)
#define CEC_RX_DATA(n)			(0x50 + 4 * (n))

#define CEC_RX_STATUS			0x60
#define CEC_TX_STATUS			0x64
#define CEC_TX_STATUS_FSM		GENMASK(24, 16)
#define CEC_TX_FSM_IDLE			0x01

#define CEC_INT_CLR			0x74
#define CEC_INT_EN			0x78
#define CEC_INT_STA			0x7c
/* The same bit in all three */
#define CEC_INT_RX_BUF_FULL		BIT(20)
#define CEC_INT_RX_DATA_RCVD		BIT(18)
#define CEC_INT_RX_BUF_READY		BIT(16)
#define CEC_INT_TX_FAIL_DATA_ACK	BIT(14)
#define CEC_INT_TX_FAIL_HEADER_ACK	BIT(13)
#define CEC_INT_TX_FAIL_RETX		BIT(12)
#define CEC_INT_TX_FAIL_DATA		BIT(11)
#define CEC_INT_TX_FAIL_HEADER		BIT(10)
#define CEC_INT_TX_FAIL_SRC		BIT(9)
#define CEC_INT_TX_FINISH		BIT(8)
#define CEC_INT_LINE_LOW_LONG		BIT(7)
#define CEC_INT_RX_ALL			GENMASK(23, 16)
#define CEC_INT_TX_ALL			(GENMASK(15, 8) | BIT(7))
#define CEC_INT_ALL			GENMASK(23, 0)
#define CEC_INT_TX_EN			(CEC_INT_TX_FINISH | CEC_INT_TX_FAIL_RETX | \
					 CEC_INT_LINE_LOW_LONG)

#define CEC_NUM_LA			3

struct mtk_cec {
	struct device *dev;
	void __iomem *base;
	void __iomem *legacy;
	struct clk *clk;
	struct cec_adapter *adap;
	struct cec_notifier *notifier;
	u8 la[CEC_NUM_LA];

	bool tx_broadcast;

	/* Results handed from the hard IRQ handler to the thread */
	u32 tx_status;
	bool rx_pending;
	struct cec_msg rx_msg;
};

static void mtk_cec_mask(struct mtk_cec *cec, u32 reg, u32 mask, u32 val)
{
	u32 tmp = readl(cec->base + reg);

	writel((tmp & ~mask) | (val & mask), cec->base + reg);
}

/* Interrupt status is cleared by pulsing the matching bits of INT_CLR. */
static void mtk_cec_int_clear(struct mtk_cec *cec, u32 bits)
{
	mtk_cec_mask(cec, CEC_INT_CLR, bits, bits);
	mtk_cec_mask(cec, CEC_INT_CLR, bits, 0);
}

static void mtk_cec_reset(struct mtk_cec *cec, u32 which)
{
	mtk_cec_mask(cec, CEC_TR_CONFIG, which, 0);
	udelay(1);
	mtk_cec_mask(cec, CEC_TR_CONFIG,
		     CEC_TR_CONFIG_TX_RESET | CEC_TR_CONFIG_RX_RESET,
		     CEC_TR_CONFIG_TX_RESET | CEC_TR_CONFIG_RX_RESET);
}

static void mtk_cec_set_la(struct mtk_cec *cec)
{
	mtk_cec_mask(cec, CEC_TR_CONFIG,
		     CEC_TR_CONFIG_LA1 | CEC_TR_CONFIG_LA2 | CEC_TR_CONFIG_LA3,
		     FIELD_PREP(CEC_TR_CONFIG_LA1, cec->la[0]) |
		     FIELD_PREP(CEC_TR_CONFIG_LA2, cec->la[1]) |
		     FIELD_PREP(CEC_TR_CONFIG_LA3, cec->la[2]));
}

/*
 * Timings for the 26 MHz source divided down to the controller's sampling
 * clock, as programmed by the vendor kernel.
 */
static void mtk_cec_hw_init(struct mtk_cec *cec)
{
	u32 val;

	/* Route the CEC pin to this block and power down the old one. */
	writel(CEC1_TR_TEST_SEL_NEW | CEC1_TR_TEST_VAL, cec->legacy + CEC1_TR_TEST);
	val = readl(cec->legacy + CEC1_CKGEN);
	writel(val | CEC1_CKGEN_PDN, cec->legacy + CEC1_CKGEN);

	mtk_cec_mask(cec, CEC_CKGEN, CEC_CKGEN_PDN, 0);
	mtk_cec_mask(cec, CEC_CKGEN, CEC_CKGEN_27M_EN | CEC_CKGEN_DIV |
		     CEC_CKGEN_SEL_DIV,
		     CEC_CKGEN_27M_EN | FIELD_PREP(CEC_CKGEN_DIV, CEC_CKGEN_DIV_100K));

	/* Only accept messages for our logical addresses and broadcasts. */
	mtk_cec_mask(cec, CEC_TR_CONFIG,
		     CEC_TR_CONFIG_RX_CHK_DST | CEC_TR_CONFIG_BYPASS,
		     CEC_TR_CONFIG_RX_CHK_DST);

	writel(0x0186015e, cec->base + CEC_RX_TIMER_START_R);
	writel(0x01d601ae, cec->base + CEC_RX_TIMER_START_F);
	writel(0x011300cd, cec->base + CEC_RX_TIMER_DATA);
	writel(0x00690082, cec->base + CEC_RX_TIMER_ACK);
	writel(0x01680000, cec->base + CEC_RX_TIMER_ERROR);
	mtk_cec_mask(cec, CEC_TR_CONFIG, CEC_TR_CONFIG_RX_ACK_ERR_BYPASS,
		     CEC_TR_CONFIG_RX_ACK_ERR_BYPASS);
	mtk_cec_reset(cec, CEC_TR_CONFIG_RX_RESET);

	writel(0x01c20172, cec->base + CEC_TX_TIMER_START);
	writel(0x003c0096, cec->base + CEC_TX_TIMER_DATA_R);
	writel(0x00f000f0, cec->base + CEC_TX_TIMER_DATA_F);
	writel(0x00040069, cec->base + CEC_TX_TIMER_DATA_S);
	writel(0x1f4004b0, cec->base + CEC_LINE_DET);
	mtk_cec_mask(cec, CEC_TX_TIMER_ACK, CEC_TX_TIMER_ACK_MASK, 0x00aa0028);

	/*
	 * One hardware retry for each failure kind; signal free times of 5, 7
	 * and 3 bit periods for a new initiator, a new message and a retry.
	 */
	mtk_cec_mask(cec, CEC_TX_ARB,
		     CEC_TX_ARB_MAX_RETX_ARB | CEC_TX_ARB_MAX_RETX_COL |
		     CEC_TX_ARB_MAX_RETX_NAK | CEC_TX_ARB_BCNT_RETX |
		     CEC_TX_ARB_BCNT_NEW_MSG | CEC_TX_ARB_BCNT_NEW_INIT,
		     FIELD_PREP(CEC_TX_ARB_MAX_RETX_ARB, 1) |
		     FIELD_PREP(CEC_TX_ARB_MAX_RETX_COL, 1) |
		     FIELD_PREP(CEC_TX_ARB_MAX_RETX_NAK, 1) |
		     FIELD_PREP(CEC_TX_ARB_BCNT_RETX, 3) |
		     FIELD_PREP(CEC_TX_ARB_BCNT_NEW_MSG, 7) |
		     FIELD_PREP(CEC_TX_ARB_BCNT_NEW_INIT, 5));
	mtk_cec_mask(cec, CEC_CKGEN, CEC_CKGEN_TX_EN, CEC_CKGEN_TX_EN);
	mtk_cec_reset(cec, CEC_TR_CONFIG_TX_RESET);

	memset(cec->la, CEC_LOG_ADDR_UNREGISTERED, sizeof(cec->la));
	mtk_cec_set_la(cec);

	writel(CEC_INT_ALL, cec->base + CEC_INT_CLR);
	writel(0, cec->base + CEC_INT_CLR);
	writel(CEC_INT_RX_ALL, cec->base + CEC_INT_EN);
}

static int mtk_cec_adap_enable(struct cec_adapter *adap, bool enable)
{
	struct mtk_cec *cec = cec_get_drvdata(adap);
	int ret;

	if (enable) {
		ret = clk_prepare_enable(cec->clk);
		if (ret)
			return ret;
		mtk_cec_hw_init(cec);
	} else {
		writel(0, cec->base + CEC_INT_EN);
		mtk_cec_int_clear(cec, CEC_INT_ALL);
		mtk_cec_mask(cec, CEC_CKGEN, CEC_CKGEN_PDN, CEC_CKGEN_PDN);
		clk_disable_unprepare(cec->clk);
	}

	return 0;
}

static int mtk_cec_adap_log_addr(struct cec_adapter *adap, u8 logical_addr)
{
	struct mtk_cec *cec = cec_get_drvdata(adap);
	unsigned int i;

	if (logical_addr == CEC_LOG_ADDR_INVALID) {
		memset(cec->la, CEC_LOG_ADDR_UNREGISTERED, sizeof(cec->la));
	} else {
		for (i = 0; i < CEC_NUM_LA; i++) {
			if (cec->la[i] == CEC_LOG_ADDR_UNREGISTERED)
				break;
		}
		if (i == CEC_NUM_LA)
			return -ENXIO;
		cec->la[i] = logical_addr;
	}
	mtk_cec_set_la(cec);

	return 0;
}

static int mtk_cec_adap_transmit(struct cec_adapter *adap, u8 attempts,
				 u32 signal_free_time, struct cec_msg *msg)
{
	struct mtk_cec *cec = cec_get_drvdata(adap);
	unsigned int len = msg->len - 1;
	u32 data[4] = {};
	unsigned int i;
	u32 hdr;

	if ((readl(cec->base + CEC_TX_STATUS) & CEC_TX_STATUS_FSM) !=
	    FIELD_PREP(CEC_TX_STATUS_FSM, CEC_TX_FSM_IDLE))
		mtk_cec_reset(cec, CEC_TR_CONFIG_TX_RESET);

	for (i = 0; i < len; i++)
		data[i / 4] |= msg->msg[i + 1] << (8 * (i % 4));
	for (i = 0; i < DIV_ROUND_UP(len, 4); i++)
		writel(data[i], cec->base + CEC_TX_DATA(i));

	hdr = FIELD_PREP(CEC_TX_HEADER_SRC, cec_msg_initiator(msg)) |
	      FIELD_PREP(CEC_TX_HEADER_DST, cec_msg_destination(msg));
	if (len)
		hdr |= FIELD_PREP(CEC_TX_HEADER_CNT, len);
	else
		hdr |= CEC_TX_HEADER_EOM;

	cec->tx_broadcast = cec_msg_is_broadcast(msg);

	mtk_cec_mask(cec, CEC_INT_EN, CEC_INT_TX_ALL, 0);
	mtk_cec_int_clear(cec, CEC_INT_TX_ALL);
	mtk_cec_mask(cec, CEC_TX_HEADER, CEC_TX_HEADER_CNT | CEC_TX_HEADER_EOM |
		     CEC_TX_HEADER_SRC | CEC_TX_HEADER_DST, hdr);
	mtk_cec_mask(cec, CEC_INT_EN, CEC_INT_TX_EN, CEC_INT_TX_EN);
	mtk_cec_mask(cec, CEC_TX_HEADER, CEC_TX_HEADER_READY, CEC_TX_HEADER_READY);

	return 0;
}

static void mtk_cec_read_msg(struct mtk_cec *cec)
{
	struct cec_msg *msg = &cec->rx_msg;
	u32 hdr = readl(cec->base + CEC_RX_BUF_HEADER);
	unsigned int len, i;
	u32 data = 0;

	/* A polling message (header only) needs nothing from us. */
	if (hdr & CEC_RX_HEADER_EOM)
		goto ack;

	len = FIELD_GET(CEC_RX_BUF_CNT, hdr);
	if (!len || cec->rx_pending)
		goto ack;

	memset(msg, 0, sizeof(*msg));
	msg->len = 1 + len;
	msg->msg[0] = (FIELD_GET(CEC_RX_HEADER_SRC, hdr) << 4) |
		      FIELD_GET(CEC_RX_HEADER_DST, hdr);
	for (i = 0; i < len; i++) {
		if (!(i % 4))
			data = readl(cec->base + CEC_RX_DATA(i / 4));
		msg->msg[i + 1] = data >> (8 * (i % 4));
	}
	cec->rx_pending = true;

ack:
	mtk_cec_mask(cec, CEC_RX_BUF_HEADER, CEC_RX_BUF_RISC_ACK,
		     CEC_RX_BUF_RISC_ACK);
}

static irqreturn_t mtk_cec_irq(int irq, void *data)
{
	struct mtk_cec *cec = data;
	u32 sta = readl(cec->base + CEC_INT_STA) &
		  readl(cec->base + CEC_INT_EN);
	irqreturn_t ret = IRQ_NONE;

	if (sta & (CEC_INT_RX_BUF_READY | CEC_INT_RX_DATA_RCVD))
		mtk_cec_read_msg(cec);
	if (sta & CEC_INT_RX_BUF_FULL)
		dev_dbg(cec->dev, "receive buffer overflow\n");
	if (sta & CEC_INT_RX_ALL) {
		mtk_cec_int_clear(cec, CEC_INT_RX_ALL);
		ret = IRQ_WAKE_THREAD;
	}

	if (sta & CEC_INT_TX_EN) {
		u32 all = readl(cec->base + CEC_INT_STA);

		if (sta & CEC_INT_TX_FINISH)
			cec->tx_status = CEC_TX_STATUS_OK;
		else if (sta & CEC_INT_LINE_LOW_LONG)
			cec->tx_status = CEC_TX_STATUS_LOW_DRIVE;
		else if (all & (CEC_INT_TX_FAIL_HEADER_ACK | CEC_INT_TX_FAIL_DATA_ACK))
			/*
			 * The ACK bit is inverted for broadcasts: nobody
			 * pulling it low means nobody rejected the message.
			 */
			cec->tx_status = cec->tx_broadcast ? CEC_TX_STATUS_OK :
				CEC_TX_STATUS_NACK | CEC_TX_STATUS_MAX_RETRIES;
		else
			cec->tx_status = CEC_TX_STATUS_ERROR | CEC_TX_STATUS_MAX_RETRIES;

		mtk_cec_mask(cec, CEC_INT_EN, CEC_INT_TX_ALL, 0);
		mtk_cec_int_clear(cec, CEC_INT_TX_ALL);
		ret = IRQ_WAKE_THREAD;
	}

	return ret;
}

static irqreturn_t mtk_cec_irq_thread(int irq, void *data)
{
	struct mtk_cec *cec = data;

	if (cec->tx_status) {
		cec_transmit_attempt_done(cec->adap, cec->tx_status);
		cec->tx_status = 0;
	}
	if (cec->rx_pending) {
		cec_received_msg(cec->adap, &cec->rx_msg);
		cec->rx_pending = false;
	}

	return IRQ_HANDLED;
}

static const struct cec_adap_ops mtk_cec_adap_ops = {
	.adap_enable = mtk_cec_adap_enable,
	.adap_log_addr = mtk_cec_adap_log_addr,
	.adap_transmit = mtk_cec_adap_transmit,
};

static int mtk_cec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *hdmi_dev;
	struct mtk_cec *cec;
	int irq, ret;

	hdmi_dev = cec_notifier_parse_hdmi_phandle(dev);
	if (IS_ERR(hdmi_dev))
		return PTR_ERR(hdmi_dev);

	cec = devm_kzalloc(dev, sizeof(*cec), GFP_KERNEL);
	if (!cec)
		return -ENOMEM;
	cec->dev = dev;

	cec->base = devm_platform_ioremap_resource_byname(pdev, "cec");
	if (IS_ERR(cec->base))
		return PTR_ERR(cec->base);

	cec->legacy = devm_platform_ioremap_resource_byname(pdev, "legacy");
	if (IS_ERR(cec->legacy))
		return PTR_ERR(cec->legacy);

	cec->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(cec->clk))
		return dev_err_probe(dev, PTR_ERR(cec->clk), "failed to get clock\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	cec->adap = cec_allocate_adapter(&mtk_cec_adap_ops, cec, dev_name(dev),
					 CEC_CAP_DEFAULTS | CEC_CAP_CONNECTOR_INFO,
					 CEC_NUM_LA);
	if (IS_ERR(cec->adap))
		return PTR_ERR(cec->adap);

	ret = devm_request_threaded_irq(dev, irq, mtk_cec_irq, mtk_cec_irq_thread,
					0, dev_name(dev), cec);
	if (ret)
		goto err_delete_adapter;

	cec->notifier = cec_notifier_cec_adap_register(hdmi_dev, NULL, cec->adap);
	if (!cec->notifier) {
		ret = -ENOMEM;
		goto err_delete_adapter;
	}

	ret = cec_register_adapter(cec->adap, dev);
	if (ret)
		goto err_notifier;

	platform_set_drvdata(pdev, cec);

	return 0;

err_notifier:
	cec_notifier_cec_adap_unregister(cec->notifier, cec->adap);
err_delete_adapter:
	cec_delete_adapter(cec->adap);
	return ret;
}

static void mtk_cec_remove(struct platform_device *pdev)
{
	struct mtk_cec *cec = platform_get_drvdata(pdev);

	cec_notifier_cec_adap_unregister(cec->notifier, cec->adap);
	cec_unregister_adapter(cec->adap);
}

static const struct of_device_id mtk_cec_of_match[] = {
	{ .compatible = "mediatek,mt8695-cec" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_cec_of_match);

static struct platform_driver mtk_cec_driver = {
	.probe = mtk_cec_probe,
	.remove = mtk_cec_remove,
	.driver = {
		.name = "mtk-cec",
		.of_match_table = mtk_cec_of_match,
	},
};
module_platform_driver(mtk_cec_driver);

MODULE_DESCRIPTION("MediaTek HDMI CEC driver");
MODULE_LICENSE("GPL");
