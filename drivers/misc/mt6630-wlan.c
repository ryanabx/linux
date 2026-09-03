// SPDX-License-Identifier: GPL-2.0
/*
 * WLAN driver for the MediaTek MT6630 combo chip, SDIO function 1.
 *
 * Bring-up scope: firmware download + start, then a mac80211 device:
 * hw_scan backed by the firmware's offloaded scan (CMD_ID_SCAN_REQ_V2),
 * frame TX (long-format TXD to WTDR1), and the station-mode MLME glue
 * (bss activate / channel privilege / sta_rec / bss_info / keys) so
 * mac80211's host MLME can authenticate, associate and pass traffic.
 * The firmware runs autorate for data; management goes out at the base
 * rate like the vendor driver.
 *
 * Protocol per ~/suez-work/MT6630-WIFI-DESIGN.md (hardware-verified).
 * RX runs a polling kthread for now; the SDIO in-band interrupt +
 * enhanced mode are a later optimization.
 *
 * Depends on mt6630-wmt (function 2) having patched the chip and sent
 * FUNC_CTRL WIFI ON; probe defers until then.
 */

#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/firmware.h>
#include <linux/hex.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/of.h>
#include <linux/unaligned.h>
#include <linux/module.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#include <net/mac80211.h>

#include "mt6630-wmt.h"

#define MT6630_SDIO_DEVICE_ID	0x6630
#define MT6630_WLAN_FUNC	1
#define MT6630_SDIO_BLK_SIZE	512

#define MT6630_FW_RAM_CODE	"mediatek/mt6630/WIFI_RAM_CODE_MT6630"

/* Function-1 registers (design doc 2; == vendor mt6630_reg.h and upstream
 * mt76/sdio.h) */
#define MCR_WCIR	0x0000
#define MCR_WHLPCR	0x0004
#define MCR_WHCR	0x000c
#define MCR_WHISR	0x0010
#define MCR_WHIER	0x0014
#define MCR_WASR	0x0020
#define MCR_WTDR1	0x0034
#define MCR_WRDR0	0x0050
#define MCR_WRDR1	0x0054
#define MCR_D2HRM0R	0x0078
#define MCR_WRPLR	0x0090

#define WCIR_CHIP_ID		GENMASK(15, 0)
#define WCIR_REVISION		GENMASK(19, 16)
#define WCIR_WLAN_READY		BIT(21)

#define WHLPCR_FW_OWN_REQ_CLR	BIT(9)
#define WHLPCR_IS_DRIVER_OWN	BIT(8)
#define WHLPCR_INT_EN_CLR	BIT(1)	/* card-level DAT1 gate (byte 0) */
#define WHLPCR_INT_EN_SET	BIT(0)

#define WHCR_RX_ENHANCE_MODE_EN	BIT(16)
#define WHCR_MAX_HIF_RX_LEN_NUM	GENMASK(13, 8)
#define WHCR_RECV_MAILBOX_RD_CLR BIT(2)
#define WHCR_W_INT_CLR_CTRL	BIT(1)	/* 0 = WHISR read-to-clear */

#define WHISR_TX_DONE		BIT(0)
#define WHISR_RX0_DONE		BIT(1)
#define WHISR_RX1_DONE		BIT(2)
#define WHISR_ABNORMAL		BIT(6)
/* only bits we actually service; D2H SW bits stay off (lesson: they do
 * not clear on a plain status read and storm the line). ABNORMAL is also
 * off: block-mode reads > 512 B round up and legitimately over-read the
 * rx ring (WASR 0x100 RX0_UNDER_FLOW), which is harmless but would
 * interrupt on every large frame. */
#define WHIER_IRQ_BITS		(WHISR_TX_DONE | WHISR_RX0_DONE | \
				 WHISR_RX1_DONE)

/* Enhanced interrupt status (vendor ENHANCE_MODE_DATA_STRUCT_T): one
 * CMD53 at WHISR returns status + released TX counts + pending RX
 * lengths atomically, and clears them all (everything is read-clear). */
#define INT_STRUCT_LEN		112
#define INT_WHISR_OFF		0
#define INT_WTSR_OFF		4	/* 16 x u16 released page counts */
#define INT_RX0_NUM_OFF		36
#define INT_RX1_NUM_OFF		38
#define INT_RX0_LEN_OFF		40	/* 16 x u16 */
#define INT_RX1_LEN_OFF		72
#define INT_MAX_RX_PKTS		16
#define HIF_RX_HW_APPENDED	4	/* checksum DW after every packet */
#define IRQ_RX_BUDGET		8	/* enhanced reads per invocation */

/* Command/event framing (design doc 4/5b) */
#define PQ_ID_CMD		0x8000
#define PQ_ID_PDA		0xc000
#define PKT_TYPE_CMD		0xa0

#define INIT_CMD_DOWNLOAD_CONFIG	1
#define INIT_CMD_WIFI_START		2
#define INIT_EVT_CMD_RESULT		1

#define CMD_ID_BASIC_CONFIG	0x02
#define CMD_ID_SCAN_REQ_V2	0x03
#define CMD_ID_SCAN_CANCEL	0x04
#define CMD_ID_POWER_SAVE_MODE	0x05
#define CMD_ID_ADD_REMOVE_KEY	0x07
#define CMD_ID_BSS_ACTIVATE_CTRL 0x11
#define CMD_ID_SET_BSS_INFO	0x12
#define CMD_ID_UPDATE_STA_RECORD 0x13
#define CMD_ID_REMOVE_STA_RECORD 0x14
#define CMD_ID_PM_BSS_CONNECTED	0x16
#define CMD_ID_PM_BSS_ABORT	0x17
#define CMD_ID_CH_PRIVILEGE	0x1c
#define CMD_ID_GET_NIC_CAPABILITY 0x80

#define EVENT_ID_NIC_CAPABILITY	0x01
#define EVENT_ID_ACTIVATE_STA_REC 0x0c
#define EVENT_ID_SCAN_DONE	0x0d
#define EVENT_ID_TX_DONE	0x0f
#define EVENT_ID_CH_PRIVILEGE	0x10
#define EVENT_ID_BSS_BEACON_TIMEOUT 0x13
#define EVENT_ID_DEBUG_MSG	0x27

/* Fixed index plan (host-chosen; the fw learns them from our commands):
 * one BSS, one own-MAC entry, WTBL 0 = broadcast/multicast, WTBL 1 = the
 * AP's station entry, sta_rec slot 0.  WTBL 31 is the hardware's
 * "no station" TX index (probe requests, pre-auth frames). */
#define MT6630_BSS_IDX		0
#define MT6630_OWN_MAC_IDX	0
#define MT6630_BMC_WLAN_IDX	0
#define MT6630_AP_WLAN_IDX	1
#define MT6630_AP_STA_IDX	0
#define MT6630_NO_STA_WLAN_IDX	31

/* Long-format TX descriptor (28 bytes, vendor nic_tx.h). Written to WTDR1
 * as [TXD][frame]; the byte count in DW0 includes the TXD. */
#define MT6630_TXD_LEN		28

#define TXD_PQ_MGMT		0x88	/* port 1 (MCU), queue 1 */
#define TXD_PQ_DATA(hwq)	((hwq) << 3)	/* port 0 (LMAC), AC queue */
#define TXD_HWQ_NON_QOS		4

#define TXD_FMT_LONG		BIT(7)	/* byte 5 */
#define TXD_HF_802_11		(2 << 5)
#define TXD_NO_ACK		BIT(3)	/* byte 6 */
#define TXD_PROTECTED		BIT(7)
#define TXD_BMC			BIT(2)	/* byte 9 */
#define TXD_FIXED_RATE		BIT(7)	/* byte 11 */

#define TXD_RETRY_MGMT		30
#define TXD_RETRY_DATA		7

#define RATE_CCK_1M		0x0000	/* DW6 fixed-rate codes << 2 */
#define RATE_OFDM_6M		(0x4b << 2)

/* CMD_802_11_KEY cipher ids (vendor privacy.h CIPHER_SUITE_*) */
#define MT6630_CIPHER_WEP40	1
#define MT6630_CIPHER_TKIP	2
#define MT6630_CIPHER_CCMP	4
#define MT6630_CIPHER_WEP104	5

#define DL_CONFIG_ENCRYPTION	BIT(0)
#define DL_CONFIG_RESET		BIT(3)
#define DL_CONFIG_ACK		BIT(31)

#define MT6630_FW_CHUNK		2048
#define MT6630_FW_START_ADDR	0x00091400

#define MT6630_FW_HDR_SIZE	16
#define MT6630_FW_SECT_SIZE	16
#define MT6630_FW_SIGNATURE	0x574b544d	/* "MTKW" */

/* RX descriptor (design doc 6; vendor HW_MAC_RX_DESC_T). The 16-byte base
 * may be followed by status groups 4/1/2/3 in that order per the
 * group-valid bits, then the 802.11 frame (+2 if the offset bit is set). */
#define RXD_LEN_MASK		GENMASK(15, 0)
#define RXD_GROUP_VLD(t)	(((t) >> 9) & 0xf)
#define RXD_TYPE_SW_MASK	0xe00f
#define RXD_TYPE_SW_EVENT	0xe000
#define RXD_TYPE_SW_FRAME	0xe001
#define RXD_GRP1		BIT(0)	/* 16 bytes: PN */
#define RXD_GRP2		BIT(1)	/* 8 bytes: timestamp, CRC */
#define RXD_GRP3		BIT(2)	/* 24 bytes: RX vector */
#define RXD_GRP4		BIT(3)	/* 16 bytes: hdr translation */
#define RXD_BASE_LEN		16
#define RXD_HDR_OFFSET		BIT(6)	/* in ucHeaderLen byte */
#define RXD_HDR_TRANSLATED	BIT(7)	/* payload is 802.3, not 802.11 */

/* RXD DW2: byte 9 high nibble = cipher the hw used, bytes 10-11 = status
 * flags (vendor nic_rx.h RX_STATUS_FLAG_*) */
#define RXD_SEC_MODE(p)		((p)[9] >> 4)
#define RXD_FLAG_FCS_ERR	BIT(1)
#define RXD_FLAG_CIPHER_MISMATCH BIT(2)
#define RXD_FLAG_CIPHER_LEN_ERR	BIT(3)
#define RXD_FLAG_ICV_ERR	BIT(4)
#define RXD_FLAG_TKIP_MIC_ERR	BIT(5)

#define NIC_CAPABILITY_LEN	116

/* CMD_SCAN_REQ_V2 (design doc 5b): 8 fixed bytes, 4 x {u32 len, 32-byte
 * ssid}, 3 x u16, channel type, list num, 32 x {band, ch}, u16 ie len,
 * 600-byte ie buf */
#define SCAN_REQ_V2_LEN		(8 + 4 * 36 + 6 + 2 + 64 + 2 + 600)
#define SCAN_MAX_SSIDS		4
#define SCAN_MAX_CHANNELS	32
#define SCAN_MAX_IE		600
#define SCAN_TYPE_PASSIVE	0
#define SCAN_TYPE_ACTIVE	1
#define SCAN_SSID_WILDCARD	BIT(0)
#define SCAN_SSID_SPECIFIED	BIT(2)
#define SCAN_CHANNEL_SPECIFIED	4
#define SCAN_BAND_2G4		1
#define SCAN_BAND_5G		2

#define OWN_POLL_RETRIES	1200
#define OWN_POLL_DELAY_US	500
#define EVT_POLL_RETRIES	40000
#define EVT_POLL_DELAY_US	50
#define READY_POLL_RETRIES	512
#define CMD_RESP_TIMEOUT	(2 * HZ)

#define MT6630_XFER_BUF_SIZE	2560

struct mt6630_wlan {
	struct sdio_func *func;
	struct ieee80211_hw *hw;

	u8 *tx_buf;			/* under tx_lock */
	u8 *rx_buf;			/* rx thread (or probe phase) only */
	struct mutex tx_lock;
	u8 seq;

	struct task_struct *rx_thread;

	/* one command-with-response in flight (cmd_lock) */
	struct mutex cmd_lock;
	spinlock_t resp_lock;		/* guards the resp_* fields */
	u8 resp_eid, resp_seq;		/* what we are waiting for */
	u8 *resp_buf;
	u16 resp_max;
	int resp_len;
	struct completion resp_done;

	bool scanning;
	struct delayed_work scan_timeout;
	u8 macaddr[ETH_ALEN];

	/* single station-mode interface */
	struct ieee80211_vif *vif;
	bool sta_added;			/* AP sta_rec pushed to fw */
	bool assoc;			/* bss_info CONNECTED sent */
	u8 sta_addr[ETH_ALEN];

	struct sk_buff_head tx_skbs;	/* .tx op defers here (SDIO sleeps) */
	struct work_struct tx_work;

	u8 ch_token;			/* channel privilege request id */
	struct completion scan_done;
	struct completion ch_grant;

	/* interrupt-mode RX (runtime-toggleable) */
	bool irq_active;
	u8 *int_buf;			/* INT_STRUCT_LEN, handler only */
	struct completion tx_done;	/* signaled by the irq handler */
	struct work_struct irq_mode_work;
	/* TX page credits (128-byte pages incl. TXD; fw pool mirror).
	 * Only maintained in interrupt mode, where the enhanced read
	 * returns released counts atomically. Gating submissions on this
	 * is what prevents the fw TX engine from wedging at saturation
	 * (TX_DONE only means the HIF consumed the write). */
	atomic_t tx_pages;
};

#define MT6630_TX_PAGE_SHIFT	7	/* 128-byte pages */
#define MT6630_TX_POOL_PAGES	64	/* conservative vs fw's ~546 */
#define INT_WTSR_FFA_OFF	(INT_WTSR_OFF + 2 * 14)	/* global count */

/* Runtime-toggleable via /sys/module/mt6630_wlan/parameters/irq_rx;
 * boot-time escape hatch: mt6630_wlan.irq_rx=0 on the kernel cmdline.
 * Probe always brings the device up polled and flips afterwards, and the
 * polled fallback stays intact, so recovery is one toggle (or reboot
 * with the cmdline override). */
static bool irq_rx = true;
static struct mt6630_wlan *mt6630_wlan_dev;	/* single device */

/* ---- low-level ---- */

static int mt6630_wlan_own_clear(struct sdio_func *func)
{
	int ret = 0, i;
	u32 val;

	sdio_writeb(func, WHLPCR_FW_OWN_REQ_CLR >> 8, MCR_WHLPCR + 1, &ret);
	if (ret)
		return ret;

	for (i = 0; i < OWN_POLL_RETRIES; i++) {
		val = sdio_readl(func, MCR_WHLPCR, &ret);
		if (ret)
			return ret;
		if (val & WHLPCR_IS_DRIVER_OWN)
			return 0;
		udelay(OWN_POLL_DELAY_US);
	}
	return -ETIMEDOUT;
}

/* Write the first 'total' bytes of wl->tx_buf to WTDR1 (padded, with the
 * zero-dword aggregation terminator) and wait for TX_DONE (one frame in
 * flight). Caller holds tx_lock. */
static int mt6630_wlan_port_write(struct mt6630_wlan *wl, u32 total)
{
	struct sdio_func *func = wl->func;
	u32 xfer = ALIGN(total + 4, 4);	/* + zero-dword terminator */
	u32 whisr;
	int ret, i;

	if (xfer > MT6630_SDIO_BLK_SIZE)
		xfer = ALIGN(xfer, MT6630_SDIO_BLK_SIZE);
	if (xfer > MT6630_XFER_BUF_SIZE)
		return -EINVAL;
	memset(wl->tx_buf + total, 0, xfer - total);

	if (READ_ONCE(wl->irq_active)) {
		int pages = DIV_ROUND_UP(total, 1 << MT6630_TX_PAGE_SHIFT);
		int waits = 0;

		/* gate on page credits: TX_DONE only means the HIF consumed
		 * the write; submitting past the fw pool wedges its TX
		 * engine permanently (needs a chip power cycle) */
		while (atomic_read(&wl->tx_pages) < pages) {
			reinit_completion(&wl->tx_done);
			if (atomic_read(&wl->tx_pages) >= pages)
				break;
			if (!wait_for_completion_timeout(&wl->tx_done,
						msecs_to_jiffies(100)) &&
			    ++waits >= 3) {
				/* anomaly: resync rather than deadlock */
				dev_warn_ratelimited(&func->dev,
						     "tx credit starvation, resetting pool\n");
				atomic_set(&wl->tx_pages,
					   MT6630_TX_POOL_PAGES);
				break;
			}
		}
		atomic_sub(pages, &wl->tx_pages);

		sdio_claim_host(func);
		ret = sdio_writesb(func, MCR_WTDR1, wl->tx_buf, xfer);
		sdio_release_host(func);
		return ret;
	}

	sdio_claim_host(func);
	ret = sdio_writesb(func, MCR_WTDR1, wl->tx_buf, xfer);
	if (!ret) {
		for (i = 0; i < EVT_POLL_RETRIES; i++) {
			whisr = sdio_readl(func, MCR_WHISR, &ret);
			if (ret || (whisr & WHISR_TX_DONE))
				break;
			udelay(EVT_POLL_DELAY_US);
		}
		/* NB: never read WTQCR here -- consuming those read-clear
		 * counters mid-flow suppresses the next TX_DONE and broke
		 * the fw download outright. Polled mode touches WHISR only. */
		if (!ret && !(whisr & WHISR_TX_DONE)) {
			/* nothing consumed; WHISR bit31 = fw assert */
			dev_warn_ratelimited(&func->dev,
					     "tx_done timeout, whisr %08x\n",
					     whisr);
			ret = -ETIMEDOUT;
		}
	}
	sdio_release_host(func);
	return ret;
}

/* Send one command frame (8-byte WIFI_CMD_T header) to WTDR1. */
static int mt6630_wlan_tx(struct mt6630_wlan *wl, u16 pq_id, u8 cid, u8 set,
			  u8 seq, const void *payload, u16 len)
{
	u16 total = 8 + len;
	u8 *tx = wl->tx_buf;
	int ret;

	if (total > MT6630_XFER_BUF_SIZE)
		return -EINVAL;

	mutex_lock(&wl->tx_lock);
	put_unaligned_le16(total, &tx[0]);
	put_unaligned_le16(pq_id, &tx[2]);
	tx[4] = cid;
	tx[5] = PKT_TYPE_CMD;
	tx[6] = set;
	tx[7] = seq;
	if (payload && len)
		memcpy(&tx[8], payload, len);
	else if (len)
		memset(&tx[8], 0, len);
	ret = mt6630_wlan_port_write(wl, total);
	mutex_unlock(&wl->tx_lock);

	if (ret)
		dev_err(&wl->func->dev, "tx cid 0x%02x failed: %d\n",
			cid, ret);
	return ret;
}

/* Read one pending RX packet into wl->rx_buf; 0 = nothing pending.
 * Caller holds the host. */
static int mt6630_wlan_rx_pkt(struct mt6630_wlan *wl, int port)
{
	struct sdio_func *func = wl->func;
	u32 plr, pkt_len, xfer;
	int ret;

	plr = sdio_readl(func, MCR_WRPLR, &ret);
	if (ret)
		return ret;
	pkt_len = port == 0 ? (plr & 0xffff) : (plr >> 16);
	if (!pkt_len)
		return 0;

	xfer = ALIGN(pkt_len + 4, 4);	/* read-extra-4-bytes quirk */
	if (xfer > MT6630_SDIO_BLK_SIZE)
		xfer = ALIGN(xfer, MT6630_SDIO_BLK_SIZE);
	if (xfer > MT6630_XFER_BUF_SIZE)
		return -EPROTO;

	ret = sdio_readsb(func, wl->rx_buf,
			  port == 0 ? MCR_WRDR0 : MCR_WRDR1, xfer);
	if (ret)
		return ret;
	return pkt_len;
}

/* ---- init phase (probe context, polled, rx thread not running) ---- */

static int mt6630_wlan_init_evt_wait(struct mt6630_wlan *wl, u8 want_seq,
				     const char *what)
{
	struct device *dev = &wl->func->dev;
	int ret, i;

	for (i = 0; i < EVT_POLL_RETRIES; i++) {
		ret = mt6630_wlan_rx_pkt(wl, 0);
		if (ret)
			break;
		udelay(EVT_POLL_DELAY_US);
	}
	if (ret < 0) {
		dev_err(dev, "%s: rx failed: %d\n", what, ret);
		return ret;
	}
	if (ret == 0) {
		dev_err(dev, "%s: event timeout\n", what);
		return -ETIMEDOUT;
	}
	/* {len, 0xE000, eid, seq, rsvd[2], status} */
	if (get_unaligned_le16(&wl->rx_buf[2]) != RXD_TYPE_SW_EVENT ||
	    wl->rx_buf[4] != INIT_EVT_CMD_RESULT || wl->rx_buf[5] != want_seq ||
	    wl->rx_buf[8] != 0) {
		dev_err(dev, "%s: bad result: %*ph\n", what,
			min(ret, 12), wl->rx_buf);
		return -EPROTO;
	}
	return 0;
}

static int mt6630_wlan_load_firmware(struct mt6630_wlan *wl)
{
	struct device *dev = &wl->func->dev;
	const struct firmware *fw;
	u32 n_sections, i;
	u8 cmd[12], seq;
	int ret;

	ret = request_firmware(&fw, MT6630_FW_RAM_CODE, dev);
	if (ret)
		return ret;

	if (fw->size < MT6630_FW_HDR_SIZE ||
	    get_unaligned_le32(&fw->data[0]) != MT6630_FW_SIGNATURE) {
		dev_err(dev, "bad WIFI_RAM_CODE signature\n");
		ret = -EINVAL;
		goto out;
	}
	n_sections = get_unaligned_le32(&fw->data[8]);
	if (n_sections == 0 || n_sections > 8 ||
	    MT6630_FW_HDR_SIZE + n_sections * MT6630_FW_SECT_SIZE > fw->size) {
		ret = -EINVAL;
		goto out;
	}

	sdio_claim_host(wl->func);
	for (i = 0; i < n_sections; i++) {
		const u8 *sect = fw->data + MT6630_FW_HDR_SIZE +
				 i * MT6630_FW_SECT_SIZE;
		u32 offset = get_unaligned_le32(&sect[0]);
		u32 length = get_unaligned_le32(&sect[8]);
		u32 dest   = get_unaligned_le32(&sect[12]);
		u32 pos;

		if (offset + length > fw->size) {
			ret = -EINVAL;
			break;
		}
		dev_dbg(dev, "fw section %u: %u bytes -> 0x%08x\n",
			i, length, dest);

		put_unaligned_le32(dest, &cmd[0]);
		put_unaligned_le32(length, &cmd[4]);
		put_unaligned_le32(DL_CONFIG_ENCRYPTION | DL_CONFIG_ACK |
				   (i == 0 ? DL_CONFIG_RESET : 0), &cmd[8]);
		seq = ++wl->seq;
		ret = mt6630_wlan_tx(wl, PQ_ID_CMD, INIT_CMD_DOWNLOAD_CONFIG,
				     0, seq, cmd, sizeof(cmd));
		if (!ret)
			ret = mt6630_wlan_init_evt_wait(wl, seq,
							"download config");
		if (ret)
			break;

		for (pos = 0; pos < length; pos += MT6630_FW_CHUNK) {
			u32 n = min_t(u32, MT6630_FW_CHUNK, length - pos);

			ret = mt6630_wlan_tx(wl, PQ_ID_PDA, 0, 0, 0,
					     fw->data + offset + pos, n);
			if (ret)
				break;
		}
		if (ret)
			break;
	}

	if (!ret) {
		/* WIFI_START, no event follows (design doc 5b) */
		put_unaligned_le32(1, &cmd[0]);
		put_unaligned_le32(MT6630_FW_START_ADDR, &cmd[4]);
		ret = mt6630_wlan_tx(wl, PQ_ID_CMD, INIT_CMD_WIFI_START, 0,
				     ++wl->seq, cmd, 8);
	}
	sdio_release_host(wl->func);
out:
	release_firmware(fw);
	return ret;
}

/* ---- RX path (kthread, polled) ---- */

static void mt6630_wlan_rx_frame(struct mt6630_wlan *wl, u8 *pkt, u16 len)
{
	struct ieee80211_rx_status *status;
	struct sk_buff *skb;
	u16 pkt_type = get_unaligned_le16(&pkt[2]);
	u16 sflag = get_unaligned_le16(&pkt[10]);
	u8 groups = RXD_GROUP_VLD(pkt_type);
	u8 channel = pkt[5];
	u8 sec = RXD_SEC_MODE(pkt);
	bool translated = pkt[6] & RXD_HDR_TRANSLATED;
	u32 hdr_len = RXD_BASE_LEN;
	u8 rcpi = 0;

	/* status groups, in fill order 4, 1, 2, 3 (design doc 6) */
	if (groups & RXD_GRP4)
		hdr_len += 16;
	if (groups & RXD_GRP1)
		hdr_len += 16;
	if (groups & RXD_GRP2)
		hdr_len += 8;
	if (groups & RXD_GRP3) {
		rcpi = (get_unaligned_le32(&pkt[hdr_len + 8]) >> 8) & 0xff;
		hdr_len += 24;
	}
	if (pkt[6] & RXD_HDR_OFFSET)
		hdr_len += 2;

	if (len <= hdr_len || len > MT6630_XFER_BUF_SIZE)
		return;

	if (translated) {
		/* The fw translates data frames to 802.3 unconditionally
		 * (ucNative80211 is not honored). Rebuild the 802.11 frame:
		 * group 4 has the original FC/TA/seq-ctl/QoS, the payload is
		 * DA/SA/ethertype. STA infra: addr1=DA, addr2=TA, addr3=SA. */
		const u8 *g4 = pkt + RXD_BASE_LEN, *eth = pkt + hdr_len;
		u32 plen = len - hdr_len;
		struct ieee80211_hdr *hdr;
		__le16 fc;
		u32 wlen;

		if (!(groups & RXD_GRP4) || plen < ETH_HLEN) {
			dev_warn_ratelimited(&wl->func->dev,
					     "translated rx without group4 (groups %x len %u)\n",
					     groups, plen);
			return;
		}
		fc = cpu_to_le16(get_unaligned_le16(&g4[0]));
		if (!ieee80211_is_data(fc))
			return;
		wlen = ieee80211_hdrlen(fc);
		if (wlen < 24 || wlen > 30)
			return;

		skb = dev_alloc_skb(wlen + 8 + plen - ETH_HLEN);
		if (!skb)
			return;
		hdr = skb_put_zero(skb, wlen);
		hdr->frame_control = fc;
		memcpy(hdr->addr1, &eth[0], ETH_ALEN);	/* DA */
		memcpy(hdr->addr2, &g4[2], ETH_ALEN);	/* TA/BSSID */
		memcpy(hdr->addr3, &eth[6], ETH_ALEN);	/* SA */
		hdr->seq_ctrl = cpu_to_le16(get_unaligned_le16(&g4[8]));
		if (ieee80211_is_data_qos(fc))
			memcpy(ieee80211_get_qos_ctl(hdr), &g4[10], 2);
		/* LLC/SNAP + ethertype, then the payload */
		skb_put_data(skb, rfc1042_header, 6);
		skb_put_data(skb, &eth[12], 2);
		skb_put_data(skb, eth + ETH_HLEN, plen - ETH_HLEN);
	} else {
		skb = dev_alloc_skb(len - hdr_len);
		if (!skb)
			return;
		skb_put_data(skb, pkt + hdr_len, len - hdr_len);
	}

	status = IEEE80211_SKB_RXCB(skb);
	memset(status, 0, sizeof(*status));
	if (channel > 14) {
		status->band = NL80211_BAND_5GHZ;
		status->freq = ieee80211_channel_to_frequency(channel,
							NL80211_BAND_5GHZ);
	} else {
		status->band = NL80211_BAND_2GHZ;
		status->freq = ieee80211_channel_to_frequency(channel,
							NL80211_BAND_2GHZ);
	}
	status->signal = rcpi / 2 - 110;	/* RCPI -> dBm */

	if (sflag & RXD_FLAG_FCS_ERR)
		status->flag |= RX_FLAG_FAILED_FCS_CRC;
	if (sflag & RXD_FLAG_TKIP_MIC_ERR)
		status->flag |= RX_FLAG_MMIC_ERROR;
	/* hw decrypted and stripped IV/MIC (mt7615, the direct descendant,
	 * reports the same way) */
	if (sec && !(sflag & (RXD_FLAG_CIPHER_MISMATCH |
			      RXD_FLAG_CIPHER_LEN_ERR | RXD_FLAG_ICV_ERR)))
		status->flag |= RX_FLAG_DECRYPTED | RX_FLAG_IV_STRIPPED |
				RX_FLAG_MMIC_STRIPPED | RX_FLAG_MIC_STRIPPED;

	ieee80211_rx_irqsafe(wl->hw, skb);
}

static void mt6630_wlan_rx_event(struct mt6630_wlan *wl, u8 *pkt, u16 len)
{
	struct device *dev = &wl->func->dev;
	u8 eid = pkt[4], seq = pkt[5];
	unsigned long flags;

	/* a command waiter? */
	spin_lock_irqsave(&wl->resp_lock, flags);
	if (wl->resp_buf && eid == wl->resp_eid && seq == wl->resp_seq) {
		wl->resp_len = min_t(int, len - 8, wl->resp_max);
		memcpy(wl->resp_buf, &pkt[8], wl->resp_len);
		wl->resp_buf = NULL;
		spin_unlock_irqrestore(&wl->resp_lock, flags);
		complete(&wl->resp_done);
		return;
	}
	spin_unlock_irqrestore(&wl->resp_lock, flags);

	switch (eid) {
	case EVENT_ID_SCAN_DONE:
		if (wl->scanning) {
			struct cfg80211_scan_info info = {};

			cancel_delayed_work(&wl->scan_timeout);
			wl->scanning = false;
			ieee80211_scan_completed(wl->hw, &info);
		}
		/* Outside the flag test on purpose: the cancel path below
		 * waits on this, and it must be woken even for a scan it has
		 * already finished accounting for. */
		complete(&wl->scan_done);
		break;
	case EVENT_ID_CH_PRIVILEGE:
		/* grant: {bss, token, status, chan, ...}, status 0 = ok */
		if (len >= 8 + 3 && pkt[8 + 1] == wl->ch_token &&
		    pkt[8 + 2] == 0)
			complete(&wl->ch_grant);
		break;
	case EVENT_ID_BSS_BEACON_TIMEOUT:
		dev_info(dev, "fw reports beacon timeout\n");
		if (wl->vif && wl->assoc)
			ieee80211_connection_loss(wl->vif);
		break;
	case EVENT_ID_TX_DONE:
		break;			/* not requested (no PID set) */
	case EVENT_ID_DEBUG_MSG:
		dev_dbg(dev, "fw: %.*s\n", len - 8, &pkt[8]);
		break;
	default:
		dev_dbg(dev, "unsolicited event 0x%02x (%u)\n", eid, len);
	}
}

static void mt6630_wlan_rx_dispatch(struct mt6630_wlan *wl, u16 len)
{
	if ((get_unaligned_le16(&wl->rx_buf[2]) &
	     RXD_TYPE_SW_MASK) == RXD_TYPE_SW_EVENT)
		mt6630_wlan_rx_event(wl, wl->rx_buf, len);
	else
		mt6630_wlan_rx_frame(wl, wl->rx_buf, len);
}

static int mt6630_wlan_rx_thread(void *data)
{
	struct mt6630_wlan *wl = data;
	struct sdio_func *func = wl->func;
	int ret, port, got;

	while (!kthread_should_stop()) {
		if (kthread_should_park()) {
			kthread_parkme();	/* interrupt mode owns RX */
			continue;
		}
		got = 0;
		for (port = 0; port < 2; port++) {
			sdio_claim_host(func);
			ret = mt6630_wlan_rx_pkt(wl, port);
			sdio_release_host(func);
			if (ret <= 0)
				continue;
			got++;
			mt6630_wlan_rx_dispatch(wl, ret);
		}
		if (!got)
			usleep_range(1000, 2000);
	}
	return 0;
}

/* ---- interrupt-mode RX (chained from the wmt driver's SDIO irq) ----
 *
 * One 112-byte CMD53 at WHISR returns status + pending RX lengths
 * atomically and clears them (all read-clear). The RX lengths are
 * authoritative -- WRPLR is never consulted, which is what prevented the
 * historical interrupt storm (WHISR asserting before WRPLR updates).
 * Runs from sdio_irq_thread context with the host claimed; must stay
 * bounded (the thread is SCHED_FIFO -- an unbounded loop starves the
 * system). Level-triggered: pending work just re-fires the interrupt. */

static atomic_t mt6630_wlan_isr_calls = ATOMIC_INIT(0);

static void mt6630_wlan_isr(void *ctx)
{
	struct mt6630_wlan *wl = ctx;
	struct sdio_func *func = wl->func;
	int calls = atomic_inc_return(&mt6630_wlan_isr_calls);
	int loops, port, i, ret;

	for (loops = 0; loops < IRQ_RX_BUDGET; loops++) {
		u16 nvalid[2], ffa;
		u32 whisr;
		bool tx_credit = false;

		ret = sdio_readsb(func, wl->int_buf, MCR_WHISR,
				  INT_STRUCT_LEN);
		if (ret) {
			dev_warn_ratelimited(&func->dev,
					     "isr: enhanced read failed %d\n",
					     ret);
			return;
		}
		whisr = get_unaligned_le32(&wl->int_buf[INT_WHISR_OFF]);
		nvalid[0] = get_unaligned_le16(&wl->int_buf[INT_RX0_NUM_OFF]);
		nvalid[1] = get_unaligned_le16(&wl->int_buf[INT_RX1_NUM_OFF]);

		if (calls <= 4)	/* bring-up visibility */
			dev_info(&func->dev,
				 "isr #%d.%d whisr 0x%08x rx %u/%u\n",
				 calls, loops, whisr, nvalid[0], nvalid[1]);

		/* released TX pages (read-clear, consumed by this read).
		 * Every release mirrors into the FFA counter (verified: a
		 * cmd shows FFA=1 + CPU=1 for the same page), so FFA alone
		 * is the global total; summing all 16 would double-count. */
		ffa = get_unaligned_le16(&wl->int_buf[INT_WTSR_FFA_OFF]);
		if (ffa) {
			static atomic_t credit_logs = ATOMIC_INIT(0);

			if (atomic_add_unless(&credit_logs, 1, 8))
				dev_info(&func->dev,
					 "tx credits +%u (wtsr %*ph)\n", ffa,
					 32, &wl->int_buf[INT_WTSR_OFF]);
			atomic_add(ffa, &wl->tx_pages);
			if (atomic_read(&wl->tx_pages) > MT6630_TX_POOL_PAGES)
				atomic_set(&wl->tx_pages,
					   MT6630_TX_POOL_PAGES);
			tx_credit = true;
		}
		if ((whisr & WHISR_TX_DONE) || tx_credit)
			complete(&wl->tx_done);

		if (whisr & WHISR_ABNORMAL) {
			/* read-clear; expected: 0x100 rx0 underflow from
			 * the block-mode round-up on large frames */
			u32 wasr = sdio_readl(func, MCR_WASR, &ret);

			dev_dbg(&func->dev, "abnormal int, wasr 0x%08x\n",
				wasr);
		}

		if (!nvalid[0] && !nvalid[1])
			return;	/* status quiescent */

		for (port = 0; port < 2; port++) {
			u32 len_off = port ? INT_RX1_LEN_OFF : INT_RX0_LEN_OFF;

			if (nvalid[port] > INT_MAX_RX_PKTS) {
				dev_warn_ratelimited(&func->dev,
						     "rx%d count %u insane\n",
						     port, nvalid[port]);
				continue;
			}
			for (i = 0; i < nvalid[port]; i++) {
				u16 len = get_unaligned_le16(
					&wl->int_buf[len_off + 2 * i]);
				u32 xfer = ALIGN(len + HIF_RX_HW_APPENDED, 4);

				if (!len || xfer > MT6630_XFER_BUF_SIZE)
					break;
				ret = sdio_readsb(func, wl->rx_buf,
						  port ? MCR_WRDR1 : MCR_WRDR0,
						  xfer);
				if (ret)
					return;
				mt6630_wlan_rx_dispatch(wl, len);
			}
		}
	}
}

/* Switch between the polled kthread and interrupt RX. Runs from a
 * workqueue (the param setter cannot sleep on the sdio host lock). */
static void mt6630_wlan_irq_mode_work(struct work_struct *work)
{
	struct mt6630_wlan *wl = container_of(work, struct mt6630_wlan,
					      irq_mode_work);
	struct sdio_func *func = wl->func;
	bool want = READ_ONCE(irq_rx);
	u32 whcr;
	int ret = 0;

	if (want == wl->irq_active)
		return;

	if (want) {
		u8 ien;

		/* park the poller first: rx_buf must have a single owner */
		kthread_park(wl->rx_thread);
		atomic_set(&wl->tx_pages, MT6630_TX_POOL_PAGES);
		WRITE_ONCE(wl->irq_active, true);

		sdio_claim_host(func);
		/* CCCR IENx: sdio_claim_irq would set the per-function gate,
		 * but we chain off the wmt (func 2) claim instead -- open
		 * function 1's gate into DAT1 by hand (LENIENT_FN0 quirk) */
		ien = sdio_f0_readb(func, SDIO_CCCR_IENx, &ret);
		sdio_f0_writeb(func, ien | BIT(func->num) | BIT(0),
			       SDIO_CCCR_IENx, &ret);
		dev_info(&func->dev, "cccr ienx 0x%02x -> 0x%02x (%d)\n",
			 ien, (u8)(ien | BIT(func->num) | BIT(0)), ret);
		/* WHCR: read-to-clear WHISR, no rx-enhance tail, rx len
		 * reporting unlimited (field 0 = 16) */
		whcr = sdio_readl(func, MCR_WHCR, &ret);
		whcr &= ~(WHCR_RX_ENHANCE_MODE_EN | WHCR_MAX_HIF_RX_LEN_NUM |
			  WHCR_RECV_MAILBOX_RD_CLR | WHCR_W_INT_CLR_CTRL);
		sdio_writel(func, whcr, MCR_WHCR, &ret);
		sdio_writel(func, WHIER_IRQ_BITS, MCR_WHIER, &ret);
		mt6630_wmt_register_wlan_isr(mt6630_wlan_isr, wl);
		/* card-level interrupt output on (CMD52, byte 0) */
		sdio_writeb(func, WHLPCR_INT_EN_SET, MCR_WHLPCR, &ret);
		/* prime: drain anything already pending (and prove the
		 * enhanced read works) while we still hold the host */
		mt6630_wlan_isr(wl);
		sdio_release_host(func);
		dev_info(&func->dev, "interrupt rx enabled (%d)\n", ret);
	} else {
		u8 ien;

		mt6630_wmt_unregister_wlan_isr();
		/* taking the host lock synchronizes with a running isr */
		sdio_claim_host(func);
		sdio_writeb(func, WHLPCR_INT_EN_CLR, MCR_WHLPCR, &ret);
		ien = sdio_f0_readb(func, SDIO_CCCR_IENx, &ret);
		sdio_f0_writeb(func, ien & ~BIT(func->num), SDIO_CCCR_IENx,
			       &ret);
		sdio_release_host(func);
		WRITE_ONCE(wl->irq_active, false);
		kthread_unpark(wl->rx_thread);
		dev_info(&func->dev, "polled rx restored (%d)\n", ret);
	}
}

static int mt6630_wlan_irq_rx_set(const char *val,
				  const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (!ret && mt6630_wlan_dev)
		schedule_work(&mt6630_wlan_dev->irq_mode_work);
	return ret;
}

static const struct kernel_param_ops mt6630_wlan_irq_rx_ops = {
	.set = mt6630_wlan_irq_rx_set,
	.get = param_get_bool,
};
module_param_cb(irq_rx, &mt6630_wlan_irq_rx_ops, &irq_rx, 0644);
MODULE_PARM_DESC(irq_rx, "interrupt-driven RX + TX credits (default on; 0 = polled fallback)");

/* ---- runtime commands (rx thread delivers the response) ---- */

static int mt6630_wlan_cmd(struct mt6630_wlan *wl, u8 cid, bool set,
			   const void *payload, u16 len,
			   u8 want_eid, u8 *resp, u16 resp_max)
{
	unsigned long flags;
	u8 seq;
	int ret;

	mutex_lock(&wl->cmd_lock);
	seq = ++wl->seq;

	if (resp) {
		spin_lock_irqsave(&wl->resp_lock, flags);
		wl->resp_eid = want_eid;
		wl->resp_seq = seq;
		wl->resp_buf = resp;
		wl->resp_max = resp_max;
		wl->resp_len = 0;
		reinit_completion(&wl->resp_done);
		spin_unlock_irqrestore(&wl->resp_lock, flags);
	}

	ret = mt6630_wlan_tx(wl, PQ_ID_CMD, cid, set ? 1 : 0, seq,
			     payload, len);
	if (!ret && resp) {
		if (!wait_for_completion_timeout(&wl->resp_done,
						 CMD_RESP_TIMEOUT))
			ret = -ETIMEDOUT;
		else
			ret = wl->resp_len;
	}

	if (ret < 0 && resp) {
		spin_lock_irqsave(&wl->resp_lock, flags);
		wl->resp_buf = NULL;
		spin_unlock_irqrestore(&wl->resp_lock, flags);
	}
	mutex_unlock(&wl->cmd_lock);
	return ret;
}

static int mt6630_wlan_query_caps(struct mt6630_wlan *wl)
{
	struct device *dev = &wl->func->dev;
	u8 cap[NIC_CAPABILITY_LEN];
	int ret, tries;

	/* the first query's response has been seen to stall until the next
	 * TX reaches the fw (design doc 5b quirk) -- retry a few times */
	for (tries = 0; tries < 3; tries++) {
		ret = mt6630_wlan_cmd(wl, CMD_ID_GET_NIC_CAPABILITY, false,
				      NULL, NIC_CAPABILITY_LEN,
				      EVENT_ID_NIC_CAPABILITY,
				      cap, sizeof(cap));
		if (ret >= 20)
			break;
	}
	if (ret < 20)
		return ret < 0 ? ret : -EPROTO;

	memcpy(wl->macaddr, &cap[8], ETH_ALEN);
	dev_info(dev, "fw: product 0x%04x ver 0x%04x, MAC %pM%s\n",
		 get_unaligned_le16(&cap[0]), get_unaligned_le16(&cap[2]),
		 wl->macaddr, cap[6] ? ", 2.4 GHz only" : "");
	return 0;
}

/*
 * The bootloader injects the factory MAC as an ASCII-hex string into the
 * device tree at /idme/mac_addr (the vendor driver reads the same node,
 * gl_init.c: IDME_OF_MAC_ADDR). Prefer it over the fw's all-zero eeprom
 * address. Returns 0 and fills wl->macaddr on success.
 */
static int mt6630_wlan_idme_mac(struct mt6630_wlan *wl)
{
	struct device_node *np;
	const char *val;
	u8 mac[ETH_ALEN];
	int len, ret = -ENODEV;

	np = of_find_node_by_path("/idme/mac_addr");
	if (!np)
		return -ENODEV;

	val = of_get_property(np, "value", &len);
	if (val && len >= 12 && hex2bin(mac, val, ETH_ALEN) == 0 &&
	    is_valid_ether_addr(mac)) {
		ether_addr_copy(wl->macaddr, mac);
		ret = 0;
	}
	of_node_put(np);
	return ret;
}

/* ---- frame TX ---- */

static bool mt6630_wlan_band_5ghz(struct mt6630_wlan *wl)
{
	struct ieee80211_channel *chan = wl->hw->conf.chandef.chan;

	return chan && chan->band == NL80211_BAND_5GHZ;
}

/* Build the long-format TXD + frame in tx_buf and send it. */
static int mt6630_wlan_tx_frame(struct mt6630_wlan *wl, struct sk_buff *skb)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	/* RA decides the WTBL entry and ACK policy: in infra STA mode even
	 * broadcast-DA frames (DHCP!) are unicast to the AP */
	bool mcast = is_multicast_ether_addr(hdr->addr1);
	bool mgmt = ieee80211_is_mgmt(hdr->frame_control);
	bool fixed_rate = mgmt;
	u32 hdrlen = ieee80211_hdrlen(hdr->frame_control);
	u16 fc, total = MT6630_TXD_LEN + skb->len;
	u8 *txd, wlan_idx, tid = 0;
	int ret;

	if (total > MT6630_XFER_BUF_SIZE || hdrlen > 62)
		return -EMSGSIZE;

	/* hw crypto: the fw/WTBL inserts IV and encrypts; mark the frame */
	if (info->control.hw_key)
		hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_PROTECTED);
	fc = le16_to_cpu(hdr->frame_control);

	if (mcast)
		wlan_idx = MT6630_BMC_WLAN_IDX;
	else if (wl->sta_added && ether_addr_equal(hdr->addr1, wl->sta_addr))
		wlan_idx = MT6630_AP_WLAN_IDX;
	else
		wlan_idx = MT6630_NO_STA_WLAN_IDX;

	if (ieee80211_is_data_qos(hdr->frame_control))
		tid = ieee80211_get_tid(hdr);

	/* EAPOL at the base rate, like the vendor's security-frame path */
	if (!mgmt && skb->len >= hdrlen + 8 &&
	    get_unaligned((__be16 *)(skb->data + hdrlen + 6)) ==
							htons(ETH_P_PAE))
		fixed_rate = true;

	mutex_lock(&wl->tx_lock);
	txd = wl->tx_buf;
	memset(txd, 0, MT6630_TXD_LEN);
	put_unaligned_le16(total, &txd[0]);
	/* ether-type offset (words); only consumed by csum offload */
	txd[2] = (MT6630_TXD_LEN + hdrlen + (mgmt ? 0 : 6)) >> 1;
	if (mgmt)
		txd[3] = TXD_PQ_MGMT;
	else if (ieee80211_is_data_qos(hdr->frame_control))
		/* mac80211 queue 0..3 = VO..BK; hw AC queue 3..0 */
		txd[3] = TXD_PQ_DATA(3 - skb_get_queue_mapping(skb));
	else
		txd[3] = TXD_PQ_DATA(TXD_HWQ_NON_QOS);
	txd[4] = wlan_idx;
	txd[5] = TXD_FMT_LONG | TXD_HF_802_11 | (hdrlen >> 1);
	txd[6] = tid << 4;
	if (mcast || (info->flags & IEEE80211_TX_CTL_NO_ACK))
		txd[6] |= TXD_NO_ACK;
	if (info->control.hw_key)
		txd[6] |= TXD_PROTECTED;
	txd[7] = MT6630_OWN_MAC_IDX << 2;
	/* DW2: FC subtype/type; SN/PN stay hw-assigned (DW3/4 zero) */
	txd[8] = ((fc >> 4) & 0x0f) | (((fc >> 2) & 0x03) << 4);
	if (mcast)
		txd[9] = TXD_BMC;
	if (fixed_rate) {
		txd[11] = TXD_FIXED_RATE;
		put_unaligned_le16(mt6630_wlan_band_5ghz(wl) ? RATE_OFDM_6M
							     : RATE_CCK_1M,
				   &txd[26]);
	}
	put_unaligned_le16((mgmt ? TXD_RETRY_MGMT : TXD_RETRY_DATA) << 11,
			   &txd[12]);
	memcpy(txd + MT6630_TXD_LEN, skb->data, skb->len);
	ret = mt6630_wlan_port_write(wl, total);
	mutex_unlock(&wl->tx_lock);
	return ret;
}

static void mt6630_wlan_tx_work(struct work_struct *work)
{
	struct mt6630_wlan *wl = container_of(work, struct mt6630_wlan,
					      tx_work);
	struct sk_buff *skb;
	int ret;

	while ((skb = skb_dequeue(&wl->tx_skbs))) {
		struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);

		ret = mt6630_wlan_tx_frame(wl, skb);
		ieee80211_tx_info_clear_status(info);
		/* no hw ACK reporting; claim success so MLME proceeds */
		if (!ret && !(info->flags & IEEE80211_TX_CTL_NO_ACK))
			info->flags |= IEEE80211_TX_STAT_ACK;
		ieee80211_tx_status_ni(wl->hw, skb);
	}
}

/* ---- MLME firmware glue (layouts per the vendor gen3 driver; all
 * fire-and-forget except the state-3 sta_rec update) ---- */

/* vendor RATE_SET_BIT_* bitmap: bits 0-3 = 1/2/5.5/11M (CCK), bits 6-13 =
 * 6/9/12/18/24/36/48/54M (OFDM; bits 4-5 are the unused 22/33M) */
#define MT6630_BASIC_RATES_2G	0x000f		/* CCK basic set */
#define MT6630_BASIC_RATES_5G	0x0540		/* 6/12/24M */
#define MT6630_ALL_RATES_2G	0x3fcf
#define MT6630_ALL_RATES_5G	0x3fc0

static u16 mt6630_wlan_rate_bitmap(enum nl80211_band band, u32 supp)
{
	u16 set = 0;
	int i;

	for (i = 0; i < 12; i++) {
		if (!(supp & BIT(i)))
			continue;
		if (band == NL80211_BAND_5GHZ)
			set |= BIT(i + 6);	/* 5 GHz bitrates start at 6M */
		else
			set |= BIT(i < 4 ? i : i + 2);
	}
	return set;
}

static int mt6630_wlan_bss_activate(struct mt6630_wlan *wl, bool active)
{
	u8 cmd[12] = {};

	cmd[0] = MT6630_BSS_IDX;
	cmd[1] = active;
	/* [2] network type 0 = AIS, [3] own-MAC index 0 */
	if (wl->vif)
		memcpy(&cmd[4], wl->vif->addr, ETH_ALEN);
	cmd[10] = MT6630_BMC_WLAN_IDX;
	return mt6630_wlan_cmd(wl, CMD_ID_BSS_ACTIVATE_CTRL, true, cmd,
			       sizeof(cmd), 0, NULL, 0);
}

static int mt6630_wlan_ps_profile(struct mt6630_wlan *wl, u8 profile)
{
	u8 cmd[4] = { MT6630_BSS_IDX, profile, 0, 0 };

	return mt6630_wlan_cmd(wl, CMD_ID_POWER_SAVE_MODE, true, cmd,
			       sizeof(cmd), 0, NULL, 0);
}

/* Ask the fw to park on the join channel (vendor CH_REQ_TYPE_JOIN, 4 s
 * window). The grant arrives as an unsolicited event; a timeout is not
 * fatal, mac80211 will retry the auth anyway. */
static void mt6630_wlan_ch_abort(struct mt6630_wlan *wl)
{
	u8 cmd[16] = {};

	cmd[0] = MT6630_BSS_IDX;
	cmd[1] = wl->ch_token;
	cmd[2] = 1;			/* abort */
	mt6630_wlan_cmd(wl, CMD_ID_CH_PRIVILEGE, true, cmd, sizeof(cmd),
			0, NULL, 0);
}

static int mt6630_wlan_ch_request(struct mt6630_wlan *wl,
				  struct ieee80211_channel *chan)
{
	int tries;

	for (tries = 0; tries < 2; tries++) {
		u8 cmd[16] = {};
		int ret;

		cmd[0] = MT6630_BSS_IDX;
		cmd[1] = ++wl->ch_token;
		/* [2] action 0 = req, [4] sco, [6] 20 MHz, [9] type JOIN */
		cmd[3] = chan->hw_value;
		cmd[5] = chan->band == NL80211_BAND_5GHZ ? 2 : 1;
		put_unaligned_le32(4000, &cmd[12]);	/* ms */

		reinit_completion(&wl->ch_grant);
		ret = mt6630_wlan_cmd(wl, CMD_ID_CH_PRIVILEGE, true, cmd,
				      sizeof(cmd), 0, NULL, 0);
		if (ret)
			return ret;
		/* grants are deferred while the fw scan engine is busy */
		if (wait_for_completion_timeout(&wl->ch_grant, 5 * HZ))
			return 0;
		/* stale privilege state (e.g. after a teardown) can block
		 * the grant; abort and ask once more */
		dev_warn(&wl->func->dev, "no channel grant for ch %u%s\n",
			 chan->hw_value, tries ? "" : ", retrying");
		mt6630_wlan_ch_abort(wl);
	}
	return 0;
}

/* CMD_SET_BSS_INFO, 88 bytes with the RLM sub-struct at +68. Padding holes
 * at +49 and +61..63 are real (the vendor struct is naturally aligned). */
static int mt6630_wlan_bss_info(struct mt6630_wlan *wl, bool connected,
				u16 rates_op)
{
	struct ieee80211_bss_conf *conf = &wl->vif->bss_conf;
	bool g5 = mt6630_wlan_band_5ghz(wl);
	struct ieee80211_channel *chan = wl->hw->conf.chandef.chan;
	/* mac80211 doesn't expose the AKM; approximate from the AP's
	 * privacy capability bit (only informs fw-side heuristics) */
	bool privacy = conf->assoc_capability & WLAN_CAPABILITY_PRIVACY;
	u8 cmd[88] = {};

	cmd[0] = MT6630_BSS_IDX;
	cmd[1] = connected ? 0 : 1;	/* media state */
	/* [2] op mode 0 = infrastructure */
	if (connected) {
		cmd[3] = min_t(u8, wl->vif->cfg.ssid_len, 32);
		memcpy(&cmd[4], wl->vif->cfg.ssid, cmd[3]);
		memcpy(&cmd[36], conf->bssid, ETH_ALEN);
		cmd[42] = conf->qos;
		put_unaligned_le16(rates_op, &cmd[44]);
		put_unaligned_le16(g5 ? MT6630_BASIC_RATES_5G
				      : MT6630_BASIC_RATES_2G, &cmd[46]);
		cmd[48] = MT6630_AP_STA_IDX;
		put_unaligned_le16(g5 ? RATE_OFDM_6M >> 2 : RATE_CCK_1M,
				   &cmd[50]);
		cmd[52] = g5 ? 3 : 1;		/* non-HT phy: OFDM / ERP */
		cmd[53] = privacy ? 7 : 0;	/* auth: WPA2_PSK / open */
		cmd[54] = privacy ? 6 : 1;	/* enc: ENC3 / disabled */
		cmd[55] = g5 ? 0x08 : 0x03;	/* phy set: OFDM / DSSS|ERP */
		cmd[58] = MT6630_BMC_WLAN_IDX;
		/* [60] disconnect detect threshold 0 = fw default */
	} else {
		cmd[48] = 0xfe;		/* STA_REC_INDEX_NOT_FOUND */
	}
	/* RLM parameters (+68) */
	cmd[68] = MT6630_BSS_IDX;
	if (connected && chan) {
		cmd[69] = g5 ? 2 : 1;		/* band */
		cmd[70] = chan->hw_value;
		/* [71] sco 0, [83] vht width 0 = 20/40 */
		cmd[81] = conf->use_short_preamble;
		cmd[82] = conf->use_short_slot;
		put_unaligned_le16(g5 ? MT6630_BASIC_RATES_5G
				      : MT6630_BASIC_RATES_2G, &cmd[86]);
	}
	return mt6630_wlan_cmd(wl, CMD_ID_SET_BSS_INFO, true, cmd,
			       sizeof(cmd), 0, NULL, 0);
}

/* CMD_UPDATE_STA_RECORD, 124 bytes (padding hole at +78). state is the
 * wire value: 0 = class 1 (pre-auth, makes the hw ACK the AP), 2 = class 3
 * (associated; needs the ACTIVATE_STA_REC response before data flows). */
static int mt6630_wlan_sta_rec(struct mt6630_wlan *wl, const u8 *mac,
			       u16 aid, u16 rates, bool qos, u8 state,
			       bool need_resp)
{
	bool g5 = mt6630_wlan_band_5ghz(wl);
	u8 cmd[124] = {};
	u8 resp[8];
	int ret;

	cmd[0] = MT6630_AP_STA_IDX;
	cmd[1] = 0x21;			/* STA_TYPE_LEGACY_AP */
	memcpy(&cmd[2], mac, ETH_ALEN);
	put_unaligned_le16(aid, &cmd[8]);
	put_unaligned_le16(10, &cmd[10]);	/* listen interval */
	cmd[12] = MT6630_BSS_IDX;
	cmd[13] = g5 ? 0x08 : 0x03;	/* desired phy type set */
	put_unaligned_le16(rates, &cmd[14]);
	put_unaligned_le16(g5 ? MT6630_BASIC_RATES_5G : MT6630_BASIC_RATES_2G,
			   &cmd[16]);
	cmd[18] = qos;
	cmd[20] = state;
	cmd[50] = 100;			/* RCPI seed (~-60 dBm) for autorate */
	cmd[51] = need_resp;
	cmd[54] = MT6630_AP_WLAN_IDX;
	cmd[55] = MT6630_BMC_WLAN_IDX;
	cmd[68] = 3;			/* RTS policy: legacy */
	put_unaligned_le16(g5 ? RATE_OFDM_6M >> 2 : RATE_CCK_1M, &cmd[74]);

	if (!need_resp)
		return mt6630_wlan_cmd(wl, CMD_ID_UPDATE_STA_RECORD, true,
				       cmd, sizeof(cmd), 0, NULL, 0);
	ret = mt6630_wlan_cmd(wl, CMD_ID_UPDATE_STA_RECORD, true, cmd,
			      sizeof(cmd), EVENT_ID_ACTIVATE_STA_REC,
			      resp, sizeof(resp));
	return ret < 0 ? ret : 0;
}

static int mt6630_wlan_sta_remove(struct mt6630_wlan *wl)
{
	/* action 0 = remove this sta_rec */
	u8 cmd[4] = { 0, MT6630_AP_STA_IDX, MT6630_BSS_IDX, 0 };

	return mt6630_wlan_cmd(wl, CMD_ID_REMOVE_STA_RECORD, true, cmd,
			       sizeof(cmd), 0, NULL, 0);
}

/* PM/CNM connected indication (vendor nicPmIndicateBssConnected). Without
 * it the fw does not pin the radio to the connected BSS's channel: RX went
 * silent right as the 4 s JOIN grant expired. Deliberately no BSS_ABORT
 * counterpart at disconnect (suspected in an earlier full-fw wedge). */
static int mt6630_wlan_pm_connected(struct mt6630_wlan *wl)
{
	struct ieee80211_bss_conf *conf = &wl->vif->bss_conf;
	u8 cmd[12] = {};

	cmd[0] = MT6630_BSS_IDX;
	cmd[1] = conf->dtim_period ?: 1;
	put_unaligned_le16(wl->vif->cfg.aid, &cmd[2]);
	put_unaligned_le16(conf->beacon_int, &cmd[4]);
	return mt6630_wlan_cmd(wl, CMD_ID_PM_BSS_CONNECTED, true, cmd,
			       sizeof(cmd), 0, NULL, 0);
}

/* The counterpart, and the fw needs it: PM_BSS_CONNECTED makes the firmware
 * treat the BSS as connected, which is what holds the channel once the 4 s
 * JOIN privilege expires (see the comment in the assoc path). Nothing else
 * ever releases that, so without this a later CH_PRIVILEGE request for the
 * same channel is never granted and the radio cannot re-associate at all.
 *
 * The vendor sends it as the first action of aisFsmDisconnect() and of
 * aisFsmRoamingDisconnectPrevAP() (gen3 mgmt/ais_fsm.c), i.e. on exactly the
 * transitions handled below. Payload is CMD_INDICATE_PM_BSS_ABORT,
 * { ucBssIndex, aucReserved[3] }.
 */
static int mt6630_wlan_pm_abort(struct mt6630_wlan *wl)
{
	u8 cmd[4] = { MT6630_BSS_IDX, 0, 0, 0 };

	return mt6630_wlan_cmd(wl, CMD_ID_PM_BSS_ABORT, true, cmd,
			       sizeof(cmd), 0, NULL, 0);
}

/* Native-802.11 RX: without this the fw hands data frames up translated
 * to 802.3 (vendor default), which mac80211 cannot take. */
static int mt6630_wlan_basic_config(struct mt6630_wlan *wl)
{
	u8 cmd[8] = {};

	cmd[0] = 1;			/* ucNative80211 */
	return mt6630_wlan_cmd(wl, CMD_ID_BASIC_CONFIG, true, cmd,
			       sizeof(cmd), 0, NULL, 0);
}

/* ---- mac80211 ---- */

static void mt6630_wlan_op_tx(struct ieee80211_hw *hw,
			      struct ieee80211_tx_control *control,
			      struct sk_buff *skb)
{
	struct mt6630_wlan *wl = hw->priv;

	skb_queue_tail(&wl->tx_skbs, skb);
	schedule_work(&wl->tx_work);
}

static int mt6630_wlan_op_start(struct ieee80211_hw *hw)
{
	return 0;
}

static void mt6630_wlan_op_stop(struct ieee80211_hw *hw, bool suspend)
{
}

static int mt6630_wlan_op_add_interface(struct ieee80211_hw *hw,
					struct ieee80211_vif *vif)
{
	struct mt6630_wlan *wl = hw->priv;

	if (vif->type != NL80211_IFTYPE_STATION || wl->vif)
		return -EOPNOTSUPP;
	wl->vif = vif;
	/* PS off during bring-up: missed auth/assoc responses otherwise */
	mt6630_wlan_ps_profile(wl, 0);	/* CONTINUOUS_ACTIVE */
	return mt6630_wlan_bss_activate(wl, true);
}

static void mt6630_wlan_op_remove_interface(struct ieee80211_hw *hw,
					    struct ieee80211_vif *vif)
{
	struct mt6630_wlan *wl = hw->priv;

	cancel_work_sync(&wl->tx_work);
	ieee80211_purge_tx_queue(hw, &wl->tx_skbs);
	mt6630_wlan_bss_activate(wl, false);
	wl->vif = NULL;
	wl->sta_added = false;
	wl->assoc = false;
}

static int mt6630_wlan_op_config(struct ieee80211_hw *hw, int radio_idx,
				 u32 changed)
{
	return 0;
}

static int mt6630_wlan_op_sta_state(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif,
				    struct ieee80211_sta *sta,
				    enum ieee80211_sta_state old_state,
				    enum ieee80211_sta_state new_state)
{
	struct mt6630_wlan *wl = hw->priv;
	struct ieee80211_channel *chan = hw->conf.chandef.chan;
	int ret = 0;

	if (old_state == IEEE80211_STA_NOTEXIST &&
	    new_state == IEEE80211_STA_NONE) {
		/* vendor join order: channel privilege, then the state-1
		 * sta_rec that makes the hardware ACK the AP; mac80211
		 * sends the auth frame right after this returns */
		if (chan)
			mt6630_wlan_ch_request(wl, chan);
		memcpy(wl->sta_addr, sta->addr, ETH_ALEN);
		ret = mt6630_wlan_sta_rec(wl, sta->addr, 0, 0, false, 0,
					  false);
		if (!ret)
			wl->sta_added = true;
	} else if (old_state == IEEE80211_STA_NONE &&
		   new_state == IEEE80211_STA_NOTEXIST) {
		flush_work(&wl->tx_work);	/* let the deauth out first */
		mt6630_wlan_sta_remove(wl);
		wl->sta_added = false;
	}
	return ret;
}

static void mt6630_wlan_op_bss_info_changed(struct ieee80211_hw *hw,
					    struct ieee80211_vif *vif,
					    struct ieee80211_bss_conf *info,
					    u64 changed)
{
	struct mt6630_wlan *wl = hw->priv;

	if (!(changed & BSS_CHANGED_ASSOC))
		return;

	if (vif->cfg.assoc && !wl->assoc) {
		enum nl80211_band band = mt6630_wlan_band_5ghz(wl) ?
				NL80211_BAND_5GHZ : NL80211_BAND_2GHZ;
		struct ieee80211_sta *sta;
		u16 rates = 0;
		bool qos = false;

		rcu_read_lock();
		sta = ieee80211_find_sta(vif, info->bssid);
		if (sta) {
			rates = mt6630_wlan_rate_bitmap(band,
					sta->deflink.supp_rates[band]);
			qos = sta->wme;
		}
		rcu_read_unlock();
		if (!rates)
			rates = band == NL80211_BAND_5GHZ ?
				MT6630_ALL_RATES_5G : MT6630_ALL_RATES_2G;

		/* vendor order: bss_info CONNECTED, then sta_rec state 3
		 * (its ACTIVATE_STA_REC response gates the data path) */
		mt6630_wlan_bss_info(wl, true, rates);
		if (mt6630_wlan_sta_rec(wl, wl->sta_addr, vif->cfg.aid,
					rates, qos, 2, true))
			dev_warn(&wl->func->dev,
				 "sta_rec state-3 update failed\n");
		/* No ch_abort here: aborting the privilege right as the
		 * 4-way EAPOL starts races the fw off-channel (observed as
		 * a 2/4 -> 3/4 stall). The 4 s grant expires on its own;
		 * the connected BSS then holds the channel. */
		/* the connect path resets per-BSS PS state in the fw;
		 * re-assert always-on (dozing showed as ~1 s ping RTT) */
		mt6630_wlan_ps_profile(wl, 0);
		mt6630_wlan_pm_connected(wl);
		wl->assoc = true;
	} else if (!vif->cfg.assoc && wl->assoc) {
		/* Release the fw's connected-BSS state before tearing the
		 * BSS down, which is the order the vendor uses. Without it
		 * the fw keeps holding this channel and the next join gets
		 * "no channel grant". */
		mt6630_wlan_pm_abort(wl);
		mt6630_wlan_bss_info(wl, false, 0);
		wl->assoc = false;
	}
}

static int mt6630_wlan_op_set_key(struct ieee80211_hw *hw,
				  enum set_key_cmd cmd_op,
				  struct ieee80211_vif *vif,
				  struct ieee80211_sta *sta,
				  struct ieee80211_key_conf *key)
{
	struct mt6630_wlan *wl = hw->priv;
	bool pairwise = key->flags & IEEE80211_KEY_FLAG_PAIRWISE;
	u8 cmd[64] = {};
	u8 algo;

	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
		algo = MT6630_CIPHER_WEP40;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		algo = MT6630_CIPHER_TKIP;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		algo = MT6630_CIPHER_CCMP;
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		algo = MT6630_CIPHER_WEP104;
		break;
	default:
		/* incl. BIP: mac80211 falls back to software crypto */
		return -EOPNOTSUPP;
	}
	if (key->keylen > 32)
		return -EOPNOTSUPP;

	cmd[0] = cmd_op == SET_KEY;	/* add / remove */
	cmd[1] = pairwise;		/* tx key */
	cmd[2] = pairwise;		/* key type */
	if (sta)
		memcpy(&cmd[4], sta->addr, ETH_ALEN);
	else if (wl->sta_added)		/* GTK: vendor uses the AP's MAC */
		memcpy(&cmd[4], wl->sta_addr, ETH_ALEN);
	cmd[10] = MT6630_BSS_IDX;
	cmd[11] = algo;
	cmd[12] = key->keyidx;
	cmd[13] = key->keylen;
	cmd[14] = pairwise ? MT6630_AP_WLAN_IDX : MT6630_BMC_WLAN_IDX;
	memcpy(&cmd[16], key->key, key->keylen);
	if (key->cipher == WLAN_CIPHER_SUITE_TKIP && key->keylen == 32) {
		/* fw wants rx mic @16, tx mic @24 (mac80211: the reverse) */
		memcpy(&cmd[16 + 16], key->key + 24, 8);
		memcpy(&cmd[16 + 24], key->key + 16, 8);
	}
	return mt6630_wlan_cmd(wl, CMD_ID_ADD_REMOVE_KEY, true, cmd,
			       sizeof(cmd), 0, NULL, 0);
}

static void mt6630_wlan_op_flush(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif, u32 queues,
				 bool drop)
{
	struct mt6630_wlan *wl = hw->priv;

	flush_work(&wl->tx_work);
}

/* required by HAS_RATE_CONTROL; the fw applies the sta_rec RTS policy */
static int mt6630_wlan_op_set_rts_threshold(struct ieee80211_hw *hw,
					    int radio_idx, u32 value)
{
	return 0;
}

static void mt6630_wlan_op_configure_filter(struct ieee80211_hw *hw,
					    unsigned int changed_flags,
					    unsigned int *total_flags,
					    u64 multicast)
{
	*total_flags &= FIF_ALLMULTI | FIF_BCN_PRBRESP_PROMISC;
}

static int mt6630_wlan_op_hw_scan(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif,
				  struct ieee80211_scan_request *hw_req)
{
	struct mt6630_wlan *wl = hw->priv;
	struct cfg80211_scan_request *req = &hw_req->req;
	u8 *cmd __free(kfree) = kzalloc(SCAN_REQ_V2_LEN, GFP_KERNEL);
	u32 n_ch = min_t(u32, req->n_channels, SCAN_MAX_CHANNELS);
	u32 n_ssid = min_t(u32, req->n_ssids, SCAN_MAX_SSIDS);
	bool passive = !req->n_ssids;
	u32 i;
	int ret;

	if (!cmd)
		return -ENOMEM;

	cmd[0] = 1;				/* fw-side scan seq */
	cmd[1] = 0;				/* bss index */
	cmd[2] = passive ? SCAN_TYPE_PASSIVE : SCAN_TYPE_ACTIVE;
	cmd[3] = SCAN_SSID_WILDCARD;
	cmd[4] = 0;
	cmd[5] = 2;				/* probe reqs per SSID */
	for (i = 0; i < n_ssid; i++) {
		if (!req->ssids[i].ssid_len)
			continue;
		put_unaligned_le32(req->ssids[i].ssid_len,
				   &cmd[8 + i * 36]);
		memcpy(&cmd[8 + i * 36 + 4], req->ssids[i].ssid,
		       req->ssids[i].ssid_len);
		cmd[4]++;			/* ssid num */
	}
	if (cmd[4])
		cmd[3] = SCAN_SSID_SPECIFIED | SCAN_SSID_WILDCARD;

	/* u16 probe delay / dwell / timeout: 0 = fw defaults */
	cmd[8 + 4 * 36 + 6] = SCAN_CHANNEL_SPECIFIED;
	cmd[8 + 4 * 36 + 7] = n_ch;
	for (i = 0; i < n_ch; i++) {
		struct ieee80211_channel *ch = req->channels[i];

		cmd[8 + 4 * 36 + 8 + i * 2] =
			ch->band == NL80211_BAND_5GHZ ? SCAN_BAND_5G
						      : SCAN_BAND_2G4;
		cmd[8 + 4 * 36 + 8 + i * 2 + 1] = ch->hw_value;
	}
	if (req->ie_len && req->ie_len <= SCAN_MAX_IE) {
		put_unaligned_le16(req->ie_len, &cmd[8 + 4 * 36 + 8 + 64]);
		memcpy(&cmd[8 + 4 * 36 + 8 + 64 + 2], req->ie, req->ie_len);
	}

	wl->scanning = true;
	ret = mt6630_wlan_cmd(wl, CMD_ID_SCAN_REQ_V2, true, cmd,
			      SCAN_REQ_V2_LEN, 0, NULL, 0);
	if (ret)
		wl->scanning = false;
	else
		/* the fw sometimes swallows a scan (e.g. while connected);
		 * without SCAN_DONE mac80211 would stay -EBUSY forever */
		schedule_delayed_work(&wl->scan_timeout, 10 * HZ);
	return ret;
}

static void mt6630_wlan_scan_timeout(struct work_struct *work)
{
	struct mt6630_wlan *wl = container_of(work, struct mt6630_wlan,
					      scan_timeout.work);
	struct cfg80211_scan_info info = { .aborted = true };

	if (!wl->scanning)
		return;
	dev_warn(&wl->func->dev, "scan done never arrived, aborting scan\n");
	wl->scanning = false;
	ieee80211_scan_completed(wl->hw, &info);
}

static void mt6630_wlan_op_cancel_hw_scan(struct ieee80211_hw *hw,
					  struct ieee80211_vif *vif)
{
	struct mt6630_wlan *wl = hw->priv;

	if (!wl->scanning)
		return;
	cancel_delayed_work_sync(&wl->scan_timeout);

	/* Do NOT send CMD_ID_SCAN_CANCEL. It wedges the firmware.
	 *
	 * Isolated on one boot with NetworkManager and wpa_supplicant stopped,
	 * so nothing could scan behind the test's back:
	 *
	 *   ip link set wlan0 down; sleep 3; up  -- no scan running -> 108 BSS
	 *   the same down/up 1 s into a scan                        ->   0 BSS
	 *
	 * Once wedged, every later scan ends in "scan done never arrived", a
	 * rejoin gets "no channel grant", and only a reboot recovers the chip
	 * -- which takes Bluetooth with it, since both are the same firmware.
	 *
	 * It is the cancel itself, not the teardown that follows it. Two builds
	 * established that:
	 *
	 *  - waiting 1 s for a SCAN_DONE after the cancel logged "fw did not
	 *    acknowledge the scan cancel" every time. The vendor knows the
	 *    firmware does not answer: its scnFsmMsgAbort() (gen3
	 *    mgmt/scan_fsm.c) sends the cancel and then *fabricates* the
	 *    scan-done with scnFsmGenerateScanDoneMsg(SCAN_STATUS_CANCELLED).
	 *  - waiting 8 s for the natural SCAN_DONE instead still gave "scan
	 *    still running 8 s after the cancel", for a sweep that finishes in
	 *    3-5 s when left alone -- so the engine was already dead before any
	 *    teardown command could reach it.
	 *
	 * An explicit abort wedging this firmware is a pattern we have hit
	 * before, in the channel-privilege path.
	 *
	 * So just wait for the SCAN_DONE the firmware sends on its own, bounded
	 * under the driver's 10 s scan_timeout. The event handler above
	 * completes the scan towards mac80211 when it arrives, so the common
	 * path does not need to do it here. mac80211 asks drivers to cancel
	 * "if possible" and only requires the scan be finished with
	 * ieee80211_scan_completed(), so waiting out a sweep is within the
	 * contract; in practice it returns in ~0.2 s because the sweep is
	 * already nearly done by the time anything tears the interface down.
	 */
	reinit_completion(&wl->scan_done);
	if (!wait_for_completion_timeout(&wl->scan_done,
					 msecs_to_jiffies(8000)))
		dev_warn(&wl->func->dev,
			 "scan did not finish within 8 s\n");

	/* Belt and braces: if the firmware never answered, mac80211 is still
	 * waiting and would stay -EBUSY for every future scan. */
	if (wl->scanning) {
		struct cfg80211_scan_info info = { .aborted = true };

		wl->scanning = false;
		ieee80211_scan_completed(hw, &info);
	}
}

static const struct ieee80211_ops mt6630_wlan_ops = {
	.add_chanctx = ieee80211_emulate_add_chanctx,
	.remove_chanctx = ieee80211_emulate_remove_chanctx,
	.change_chanctx = ieee80211_emulate_change_chanctx,
	.switch_vif_chanctx = ieee80211_emulate_switch_vif_chanctx,
	.tx = mt6630_wlan_op_tx,
	.wake_tx_queue = ieee80211_handle_wake_tx_queue,
	.start = mt6630_wlan_op_start,
	.stop = mt6630_wlan_op_stop,
	.add_interface = mt6630_wlan_op_add_interface,
	.remove_interface = mt6630_wlan_op_remove_interface,
	.config = mt6630_wlan_op_config,
	.configure_filter = mt6630_wlan_op_configure_filter,
	.sta_state = mt6630_wlan_op_sta_state,
	.bss_info_changed = mt6630_wlan_op_bss_info_changed,
	.set_key = mt6630_wlan_op_set_key,
	.flush = mt6630_wlan_op_flush,
	.set_rts_threshold = mt6630_wlan_op_set_rts_threshold,
	.hw_scan = mt6630_wlan_op_hw_scan,
	.cancel_hw_scan = mt6630_wlan_op_cancel_hw_scan,
};

#define CHAN2G(_ch) {						\
	.band = NL80211_BAND_2GHZ,				\
	.center_freq = 2407 + (_ch) * 5,			\
	.hw_value = (_ch),					\
}

#define CHAN5G(_ch) {						\
	.band = NL80211_BAND_5GHZ,				\
	.center_freq = 5000 + (_ch) * 5,			\
	.hw_value = (_ch),					\
}

static struct ieee80211_channel mt6630_channels_2ghz[] = {
	CHAN2G(1), CHAN2G(2), CHAN2G(3), CHAN2G(4), CHAN2G(5),
	CHAN2G(6), CHAN2G(7), CHAN2G(8), CHAN2G(9), CHAN2G(10),
	CHAN2G(11), CHAN2G(12), CHAN2G(13),
};

static struct ieee80211_channel mt6630_channels_5ghz[] = {
	CHAN5G(36), CHAN5G(40), CHAN5G(44), CHAN5G(48),
	CHAN5G(52), CHAN5G(56), CHAN5G(60), CHAN5G(64),
	CHAN5G(100), CHAN5G(104), CHAN5G(108), CHAN5G(112),
	CHAN5G(116), CHAN5G(120), CHAN5G(124), CHAN5G(128),
	CHAN5G(132), CHAN5G(136), CHAN5G(140),
	CHAN5G(149), CHAN5G(153), CHAN5G(157), CHAN5G(161),
	CHAN5G(165),
};

#define RATE(_rate, _hw_value) {			\
	.bitrate = (_rate),				\
	.hw_value = (_hw_value),			\
}

static struct ieee80211_rate mt6630_rates[] = {
	RATE(10, 0), RATE(20, 1), RATE(55, 2), RATE(110, 3),
	RATE(60, 4), RATE(90, 5), RATE(120, 6), RATE(180, 7),
	RATE(240, 8), RATE(360, 9), RATE(480, 10), RATE(540, 11),
};

static struct ieee80211_supported_band mt6630_band_2ghz = {
	.channels = mt6630_channels_2ghz,
	.n_channels = ARRAY_SIZE(mt6630_channels_2ghz),
	.bitrates = mt6630_rates,
	.n_bitrates = ARRAY_SIZE(mt6630_rates),
};

static struct ieee80211_supported_band mt6630_band_5ghz = {
	.channels = mt6630_channels_5ghz,
	.n_channels = ARRAY_SIZE(mt6630_channels_5ghz),
	.bitrates = mt6630_rates + 4,	/* no CCK on 5 GHz */
	.n_bitrates = ARRAY_SIZE(mt6630_rates) - 4,
};

/* ---- probe / remove ---- */

static int mt6630_wlan_probe(struct sdio_func *func,
			     const struct sdio_device_id *id)
{
	struct ieee80211_hw *hw;
	struct mt6630_wlan *wl;
	u32 wcir = 0;
	int ret, i;

	if (func->num != MT6630_WLAN_FUNC)
		return -ENODEV;
	if (!mt6630_wmt_ready())
		return -EPROBE_DEFER;
	/* we poke CCCR IENx by hand for the chained interrupt (see
	 * mt6630_wlan_irq_mode_work); fn0 writes need this quirk */
	func->card->quirks |= MMC_QUIRK_LENIENT_FN0;

	hw = ieee80211_alloc_hw(sizeof(*wl), &mt6630_wlan_ops);
	if (!hw)
		return -ENOMEM;
	wl = hw->priv;
	wl->hw = hw;
	wl->func = func;
	mutex_init(&wl->tx_lock);
	mutex_init(&wl->cmd_lock);
	spin_lock_init(&wl->resp_lock);
	init_completion(&wl->resp_done);
	init_completion(&wl->scan_done);
	init_completion(&wl->ch_grant);
	init_completion(&wl->tx_done);
	skb_queue_head_init(&wl->tx_skbs);
	INIT_WORK(&wl->tx_work, mt6630_wlan_tx_work);
	INIT_WORK(&wl->irq_mode_work, mt6630_wlan_irq_mode_work);
	INIT_DELAYED_WORK(&wl->scan_timeout, mt6630_wlan_scan_timeout);
	sdio_set_drvdata(func, wl);

	wl->tx_buf = devm_kmalloc(&func->dev, MT6630_XFER_BUF_SIZE,
				  GFP_KERNEL);
	wl->rx_buf = devm_kmalloc(&func->dev, MT6630_XFER_BUF_SIZE,
				  GFP_KERNEL);
	wl->int_buf = devm_kmalloc(&func->dev, INT_STRUCT_LEN, GFP_KERNEL);
	if (!wl->tx_buf || !wl->rx_buf || !wl->int_buf) {
		ret = -ENOMEM;
		goto err_free_hw;
	}

	sdio_claim_host(func);
	ret = sdio_enable_func(func);
	if (ret)
		goto err_release;
	ret = sdio_set_block_size(func, MT6630_SDIO_BLK_SIZE);
	if (ret)
		goto err_disable;
	ret = mt6630_wlan_own_clear(func);
	if (ret)
		goto err_disable;

	wcir = sdio_readl(func, MCR_WCIR, &ret);
	if (ret || (wcir & WCIR_CHIP_ID) != MT6630_SDIO_DEVICE_ID) {
		dev_err(&func->dev, "bad chip id 0x%08x (%d)\n", wcir, ret);
		ret = ret ? ret : -ENODEV;
		goto err_disable;
	}
	sdio_release_host(func);

	ret = mt6630_wlan_load_firmware(wl);
	if (ret)
		goto err_disable_unclaimed;

	sdio_claim_host(func);
	for (i = 0; i < READY_POLL_RETRIES; i++) {
		wcir = sdio_readl(func, MCR_WCIR, &ret);
		if (ret || (wcir & WCIR_WLAN_READY))
			break;
		msleep(10);
	}
	sdio_release_host(func);
	if (!(wcir & WCIR_WLAN_READY)) {
		dev_err(&func->dev, "fw ready timeout\n");
		ret = -ETIMEDOUT;
		goto err_disable_unclaimed;
	}
	dev_info(&func->dev, "MT6630 WLAN firmware running\n");

	wl->rx_thread = kthread_run(mt6630_wlan_rx_thread, wl,
				    "mt6630-wlan-rx");
	if (IS_ERR(wl->rx_thread)) {
		ret = PTR_ERR(wl->rx_thread);
		goto err_disable_unclaimed;
	}

	ret = mt6630_wlan_query_caps(wl);
	if (ret) {
		dev_err(&func->dev, "capability query failed: %d\n", ret);
		goto err_stop_thread;
	}

	ret = mt6630_wlan_basic_config(wl);
	if (ret) {
		dev_err(&func->dev, "basic config failed: %d\n", ret);
		goto err_stop_thread;
	}

	/* the fw eeprom MAC is all-zero on this board; the bootloader has
	 * the factory address in the device tree */
	if (!is_valid_ether_addr(wl->macaddr)) {
		if (mt6630_wlan_idme_mac(wl) == 0)
			dev_info(&func->dev, "MAC from idme: %pM\n",
				 wl->macaddr);
		else
			eth_random_addr(wl->macaddr);
	}

	hw->wiphy->bands[NL80211_BAND_2GHZ] = &mt6630_band_2ghz;
	hw->wiphy->bands[NL80211_BAND_5GHZ] = &mt6630_band_5ghz;
	hw->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	hw->wiphy->max_scan_ssids = SCAN_MAX_SSIDS;
	hw->wiphy->max_scan_ie_len = SCAN_MAX_IE;
	ieee80211_hw_set(hw, SIGNAL_DBM);
	ieee80211_hw_set(hw, HAS_RATE_CONTROL);	/* fw autorate */
	/* fw watches beacons (EVENT_ID_BSS_BEACON_TIMEOUT) */
	ieee80211_hw_set(hw, CONNECTION_MONITOR);
	hw->queues = 4;
	SET_IEEE80211_DEV(hw, &func->dev);
	SET_IEEE80211_PERM_ADDR(hw, wl->macaddr);

	ret = ieee80211_register_hw(hw);
	if (ret)
		goto err_stop_thread;

	dev_info(&func->dev, "registered %s (%pM)\n",
		 wiphy_name(hw->wiphy), wl->macaddr);

	mt6630_wlan_dev = wl;
	if (irq_rx)	/* set via modprobe.d: honor it, but boot polled */
		schedule_work(&wl->irq_mode_work);
	return 0;

err_stop_thread:
	kthread_stop(wl->rx_thread);
err_disable_unclaimed:
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
	goto err_free_hw;
err_disable:
	sdio_disable_func(func);
err_release:
	sdio_release_host(func);
err_free_hw:
	ieee80211_free_hw(hw);
	return ret;
}

static void mt6630_wlan_remove(struct sdio_func *func)
{
	struct mt6630_wlan *wl = sdio_get_drvdata(func);

	mt6630_wlan_dev = NULL;
	ieee80211_unregister_hw(wl->hw);
	cancel_work_sync(&wl->irq_mode_work);
	if (wl->irq_active) {
		mt6630_wmt_unregister_wlan_isr();
		sdio_claim_host(func);
		sdio_writeb(func, WHLPCR_INT_EN_CLR, MCR_WHLPCR, NULL);
		sdio_release_host(func);
		wl->irq_active = false;
		kthread_unpark(wl->rx_thread);
	}
	cancel_work_sync(&wl->tx_work);
	cancel_delayed_work_sync(&wl->scan_timeout);
	ieee80211_purge_tx_queue(wl->hw, &wl->tx_skbs);
	kthread_stop(wl->rx_thread);
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
	ieee80211_free_hw(wl->hw);
}

/* ---- power management ----
 *
 * The MMC_PM_KEEP_POWER request is the load-bearing part, and the long
 * explanation of why lives next to the same call in mt6630-wmt.c: without it
 * the mmc core power-cycles the chip through the pwrseq, the runtime-downloaded
 * firmware is gone, and everything after resume fails -EINVAL. Both function
 * drivers ask, because either one may be the only one bound.
 */
static int mt6630_wlan_suspend(struct device *dev)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct mt6630_wlan *wl = sdio_get_drvdata(func);
	int ret;

	ret = sdio_set_host_pm_flags(func, MMC_PM_KEEP_POWER);
	if (ret) {
		dev_err(dev,
			"cannot keep the MT6630 powered across suspend (%d); is keep-power-in-suspend missing from the mmc node?\n",
			ret);
		return ret;
	}

	/* Stop everything of ours that can reach the bus.
	 *
	 * mmc_card_suspended() gates SDIO *interrupt* delivery only, not
	 * ordinary transfers, and a kthread is not frozen along with userspace.
	 * So in polled mode the RX thread would go on issuing CMD53s into a
	 * controller that msdc_runtime_suspend() has already clock-gated. In
	 * interrupt mode it is parked already and RX arrives through the
	 * chained handler, which the core does stop.
	 *
	 * mac80211 has quiesced by the time we get here -- the wiphy device is
	 * a child of this one, so wiphy_suspend() ran first and took the
	 * interface down -- so these cancels are only for work already queued.
	 */
	if (!READ_ONCE(wl->irq_active))
		kthread_park(wl->rx_thread);
	cancel_work_sync(&wl->tx_work);
	cancel_delayed_work_sync(&wl->scan_timeout);
	/* Waits out a mode switch that is mid-flight; one merely queued is
	 * dropped, which resume re-queues below. */
	cancel_work_sync(&wl->irq_mode_work);

	return 0;
}

static int mt6630_wlan_resume(struct device *dev)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct mt6630_wlan *wl = sdio_get_drvdata(func);

	if (!READ_ONCE(wl->irq_active))
		kthread_unpark(wl->rx_thread);

	/* Re-run a mode switch that suspend cancelled. The work reconciles
	 * against wl->irq_active and returns immediately if there is nothing
	 * to do, so this is free in the normal case. */
	if (READ_ONCE(irq_rx) != READ_ONCE(wl->irq_active))
		schedule_work(&wl->irq_mode_work);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mt6630_wlan_pm_ops,
				mt6630_wlan_suspend, mt6630_wlan_resume);

static const struct sdio_device_id mt6630_wlan_ids[] = {
	{ SDIO_DEVICE(SDIO_VENDOR_ID_MEDIATEK, MT6630_SDIO_DEVICE_ID) },
	{ /* end */ },
};
MODULE_DEVICE_TABLE(sdio, mt6630_wlan_ids);

static struct sdio_driver mt6630_wlan_driver = {
	.name		= "mt6630-wlan",
	.id_table	= mt6630_wlan_ids,
	.probe		= mt6630_wlan_probe,
	.remove		= mt6630_wlan_remove,
	.drv		= {
		.pm	= pm_sleep_ptr(&mt6630_wlan_pm_ops),
	},
};
module_sdio_driver(mt6630_wlan_driver);

MODULE_FIRMWARE(MT6630_FW_RAM_CODE);
MODULE_AUTHOR("amazon-suez mainline bringup");
MODULE_DESCRIPTION("mac80211 driver for MT6630 on SDIO");
MODULE_LICENSE("GPL");
