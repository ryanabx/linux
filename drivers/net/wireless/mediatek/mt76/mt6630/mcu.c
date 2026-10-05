// SPDX-License-Identifier: BSD-3-Clause-Clear
/* Copyright (C) 2026 Ryan Brue <ryanbrue.dev@gmail.com> */

#include <linux/firmware.h>
#include <linux/iopoll.h>
#include <linux/mmc/sdio_func.h>

#include "mt6630.h"
#include "mac.h"

#define MCU_CMD_QUERY			BIT(8)

#define MT6630_FW_SIGNATURE		0x574b544d	/* "MTKW" */
#define MT6630_FW_MAX_SECTIONS		8

struct mt6630_fw_header {
	__le32 signature;
	__le32 crc;
	__le32 n_sections;
	__le32 rsv;
	struct {
		__le32 offset;
		__le32 rsv;
		__le32 len;
		__le32 addr;
	} sections[];
} __packed;

static u32 mt6630_read_whisr(struct sdio_func *func)
{
	return sdio_readl(func, MCR_WHISR, NULL);
}

static u32 mt6630_read_wcir(struct sdio_func *func)
{
	return sdio_readl(func, MCR_WCIR, NULL);
}

static u32 mt6630_read_wcir_claimed(struct sdio_func *func)
{
	u32 val;

	sdio_claim_host(func);
	val = sdio_readl(func, MCR_WCIR, NULL);
	sdio_release_host(func);

	return val;
}

/* Until the firmware runs, nothing answers interrupts: commands are sent and
 * acknowledged by polling, one at a time, as the vendor driver does.
 */
static int mt6630_init_cmd_send(struct mt6630_dev *dev, u16 pq_id, u8 cid,
				u8 seq, const void *data, int len)
{
	struct sdio_func *func = dev->mt76.sdio.func;
	struct mt6630_mcu_txd *txd;
	int size, ret;
	u32 status;
	u8 *buf;

	/* the payload, padded to four bytes, then a zero word */
	size = round_up(sizeof(*txd) + len, 4) + 4;
	if (size > func->cur_blksize)
		size = round_up(size, func->cur_blksize);

	buf = kzalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	txd = (struct mt6630_mcu_txd *)buf;
	txd->len = cpu_to_le16(sizeof(*txd) + len);
	txd->pq_id = cpu_to_le16(pq_id);
	txd->cid = cid;
	txd->pkt_type = MCU_PKT_ID;
	txd->seq = seq;
	memcpy(txd + 1, data, len);

	sdio_claim_host(func);
	ret = sdio_writesb(func, MCR_WTDR1, buf, size);
	if (!ret)
		ret = readx_poll_timeout(mt6630_read_whisr, func, status,
					 status & WHIER_TX_DONE_INT_EN, 50,
					 2000000);
	sdio_release_host(func);

	kfree(buf);

	return ret;
}

static int mt6630_init_evt_wait(struct mt6630_dev *dev, u8 seq)
{
	struct sdio_func *func = dev->mt76.sdio.func;
	struct mt6630_mcu_rxd *rxd;
	u32 len;
	u8 *buf;
	int ret;

	sdio_claim_host(func);
	ret = read_poll_timeout(sdio_readl, len, len & RX0_PACKET_LENGTH, 50,
				2000000, false, func, MCR_WRPLR, NULL);
	sdio_release_host(func);
	if (ret)
		return ret;

	len = FIELD_GET(RX0_PACKET_LENGTH, len);
	/* the header and the result byte */
	if (len < sizeof(*rxd) + 1)
		return -EIO;

	len = round_up(len + 4, 4);
	if (len > func->cur_blksize)
		len = round_up(len, func->cur_blksize);

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	sdio_claim_host(func);
	ret = sdio_readsb(func, buf, MCR_WRDR0, len);
	sdio_release_host(func);
	if (ret)
		goto out;

	rxd = (struct mt6630_mcu_rxd *)buf;
	if (rxd->eid != MCU_INIT_EVENT_CMD_RESULT || rxd->seq != seq ||
	    buf[sizeof(*rxd)])
		ret = -EIO;
out:
	kfree(buf);

	return ret;
}

static int mt6630_init_cmd(struct mt6630_dev *dev, u8 cid, const void *data,
			   int len, bool wait_resp)
{
	u8 seq = ++dev->mt76.mcu.msg_seq ?: ++dev->mt76.mcu.msg_seq;
	int ret;

	ret = mt6630_init_cmd_send(dev, MCU_PQ_ID_CMD, cid, seq, data, len);
	if (ret || !wait_resp)
		return ret;

	return mt6630_init_evt_wait(dev, seq);
}

static int mt6630_load_firmware(struct mt6630_dev *dev)
{
	const struct mt6630_fw_header *hdr;
	const struct firmware *fw;
	u32 i, n;
	int ret;

	ret = request_firmware(&fw, MT6630_FIRMWARE, dev->mt76.dev);
	if (ret)
		return ret;

	hdr = (const struct mt6630_fw_header *)fw->data;
	if (fw->size < sizeof(*hdr) ||
	    le32_to_cpu(hdr->signature) != MT6630_FW_SIGNATURE) {
		ret = -EINVAL;
		goto out;
	}

	n = le32_to_cpu(hdr->n_sections);
	if (!n || n > MT6630_FW_MAX_SECTIONS ||
	    struct_size(hdr, sections, n) > fw->size) {
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < n; i++) {
		u32 offset = le32_to_cpu(hdr->sections[i].offset);
		u32 len = le32_to_cpu(hdr->sections[i].len);
		struct {
			__le32 addr;
			__le32 len;
			__le32 mode;
		} req = {
			.addr = hdr->sections[i].addr,
			.len = cpu_to_le32(len),
			/* the image is decrypted on its way in */
			.mode = cpu_to_le32(DL_MODE_ENCRYPT | DL_MODE_NEED_RSP |
					    (i ? 0 : DL_MODE_RESET_SEC_IV)),
		};
		u32 pos;

		if (offset > fw->size || len > fw->size - offset) {
			ret = -EINVAL;
			goto out;
		}

		ret = mt6630_init_cmd(dev, MCU_INIT_CMD_TARGET_ADDRESS_LEN_REQ,
				      &req, sizeof(req), true);
		if (ret)
			goto out;

		for (pos = 0; pos < len; pos += MT6630_FW_CHUNK_SIZE) {
			ret = mt6630_init_cmd_send(dev, MCU_PQ_ID_FW, 0, 0,
						   fw->data + offset + pos,
						   min_t(u32, len - pos,
							 MT6630_FW_CHUNK_SIZE));
			if (ret)
				goto out;
		}
	}
out:
	release_firmware(fw);

	return ret;
}

static int mt6630_start_firmware(struct mt6630_dev *dev)
{
	struct sdio_func *func = dev->mt76.sdio.func;
	struct {
		__le32 override;
		__le32 addr;
	} req = {
		.override = cpu_to_le32(FW_START_OVERRIDE),
		.addr = cpu_to_le32(MT6630_FW_START_ADDR),
	};
	u32 val;
	int ret;

	/* answered by the started firmware raising WLAN_READY, not an event */
	ret = mt6630_init_cmd(dev, MCU_INIT_CMD_FW_START_REQ, &req,
			      sizeof(req), false);
	if (ret)
		return ret;

	sdio_claim_host(func);
	ret = readx_poll_timeout(mt6630_read_wcir, func, val,
				 val & WCIR_WLAN_READY, 10000, 2000000);
	sdio_release_host(func);

	return ret;
}

int mt6630_mcu_init(struct mt6630_dev *dev)
{
	int ret;

	ret = mt6630_load_firmware(dev);
	if (ret) {
		dev_err(dev->mt76.dev, "Failed to load firmware (%d)\n", ret);
		return ret;
	}

	ret = mt6630_start_firmware(dev);
	if (ret)
		dev_err(dev->mt76.dev, "Firmware did not start (%d)\n", ret);

	return ret;
}

static int
mt6630_mcu_send_message(struct mt76_dev *mdev, struct sk_buff *skb,
			int cmd, int *wait_seq)
{
	struct mt6630_dev *dev = container_of(mdev, struct mt6630_dev, mt76);
	struct mt6630_mcu_txd *txd;
	u8 seq;
	int ret;

	seq = ++mdev->mcu.msg_seq ?: ++mdev->mcu.msg_seq;
	if (wait_seq)
		*wait_seq = seq;

	txd = (struct mt6630_mcu_txd *)skb_push(skb, sizeof(*txd));
	txd->len = cpu_to_le16(skb->len);
	txd->pq_id = cpu_to_le16(MCU_PQ_ID_CMD);
	txd->cid = cmd;
	txd->pkt_type = MCU_PKT_ID;
	txd->set_query = cmd & MCU_CMD_QUERY ? MCU_Q_QUERY : MCU_Q_SET;
	txd->seq = seq;

	ret = mt76_tx_queue_skb_raw(dev, mdev->q_mcu[MT_MCUQ_WM], skb, 0);
	if (ret)
		return ret;

	mt76_queue_kick(dev, mdev->q_mcu[MT_MCUQ_WM]);

	return 0;
}

static int
mt6630_mcu_parse_response(struct mt76_dev *mdev, int cmd,
			  struct sk_buff *skb, int seq)
{
	struct mt6630_mcu_rxd *rxd;

	if (!skb) {
		dev_err(mdev->dev, "Message %02x (seq %d) timeout\n",
			cmd & 0xff, seq);
		return -ETIMEDOUT;
	}

	rxd = (struct mt6630_mcu_rxd *)skb->data;
	if (seq != rxd->seq)
		return -EAGAIN;

	skb_pull(skb, sizeof(*rxd));

	return 0;
}

const struct mt76_mcu_ops mt6630_mcu_ops = {
	.headroom = sizeof(struct mt6630_mcu_txd),
	.mcu_skb_send_msg = mt6630_mcu_send_message,
	.mcu_parse_response = mt6630_mcu_parse_response,
};

/* Tell the firmware to stop, the way the vendor driver does before it
 * unloads: without it a later probe cannot download the firmware again.
 */
int mt6630_mcu_stop(struct mt6630_dev *dev)
{
	struct mt6630_nic_power_ctrl req = {
		.power_mode = MT6630_POWER_MODE_OFF,
	};
	struct sdio_func *func = dev->mt76.sdio.func;
	u32 val;
	int ret;

	ret = mt76_mcu_send_msg(&dev->mt76, MCU_CMD_NIC_POWER_CTRL, &req,
				sizeof(req), false);
	if (ret)
		return ret;

	/* the host is claimed per read, so the TX worker can send the
	 * command meanwhile
	 */
	return readx_poll_timeout(mt6630_read_wcir_claimed, func, val,
				  !(val & WCIR_WLAN_READY), 1000, 2000000);
}

int mt6630_mcu_get_nic_capability(struct mt6630_dev *dev)
{
	struct mt6630_nic_capability *cap;
	struct sk_buff *skb;
	int ret;

	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, sizeof(*cap));
	if (!skb)
		return -ENOMEM;

	/* the query carries a buffer the size of its answer */
	skb_put_zero(skb, sizeof(*cap));
	ret = mt76_mcu_skb_send_and_get_msg(&dev->mt76, skb,
					    MCU_CMD_GET_NIC_CAPABILITY |
					    MCU_CMD_QUERY, true, &skb);
	if (ret)
		return ret;

	if (skb->len < sizeof(*cap)) {
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	cap = (struct mt6630_nic_capability *)skb->data;
	dev->mt76.phy.cap.has_2ghz = true;
	dev->mt76.phy.cap.has_5ghz = !cap->hw_5g_disabled;
	memcpy(dev->mt76.phy.macaddr, cap->mac_addr, ETH_ALEN);

	dev_info(dev->mt76.dev, "Firmware version 0x%04x\n",
		 le16_to_cpu(cap->fw_version));

	dev_kfree_skb(skb);

	return 0;
}

int mt6630_mcu_bss_activate(struct mt6630_dev *dev, struct ieee80211_vif *vif,
			    bool enable)
{
	struct mt6630_bss_activate req = {
		.bss_idx = MT6630_BSS_IDX,
		.active = enable,
		.network_type = MT6630_NETWORK_INFRA,
		.omac_idx = MT6630_OMAC_IDX,
		.bmc_wlan_idx = MT6630_WTBL_BMC,
	};

	ether_addr_copy(req.bss_addr, vif->addr);

	return mt76_mcu_send_msg(&dev->mt76, MCU_CMD_BSS_ACTIVATE_CTRL, &req,
				 sizeof(req), false);
}

int mt6630_mcu_set_ps_profile(struct mt6630_dev *dev, u8 profile)
{
	struct mt6630_ps_profile req = {
		.bss_idx = MT6630_BSS_IDX,
		.profile = profile,
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_CMD_POWER_SAVE_MODE, &req,
				 sizeof(req), false);
}

static u8 mt6630_mcu_band(enum nl80211_band band)
{
	return band == NL80211_BAND_5GHZ ? MT6630_BAND_5G : MT6630_BAND_2G;
}

static u8 mt6630_mcu_sco(const struct cfg80211_chan_def *chandef)
{
	int lower = chandef->center_freq1 - cfg80211_chandef_get_width(chandef) / 2;

	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_40:
	case NL80211_CHAN_WIDTH_80:
		/* the 40 MHz half of the primary channel */
		return (chandef->chan->center_freq - lower) / 20 % 2 ?
		       MT6630_SCO_SCB : MT6630_SCO_SCA;
	default:
		return MT6630_SCO_SCN;
	}
}

static void mt6630_mcu_vht_width(const struct cfg80211_chan_def *chandef,
				 u8 *width, u8 *seg1)
{
	if (chandef->width != NL80211_CHAN_WIDTH_80) {
		*width = MT6630_VHT_WIDTH_20_40;
		*seg1 = 0;
		return;
	}

	*width = MT6630_VHT_WIDTH_80;
	*seg1 = ieee80211_frequency_to_channel(chandef->center_freq1);
}

/* Join requests the channel from the firmware's channel arbiter, which
 * otherwise keeps the radio wherever its own scan or idle logic left it.
 */
int mt6630_mcu_ch_privilege(struct mt6630_dev *dev,
			    struct ieee80211_channel *chan)
{
	struct mt6630_ch_privilege req = {
		.bss_idx = MT6630_BSS_IDX,
		.action = MT6630_CH_PRIV_REQ,
		.primary_channel = chan->hw_value,
		.sco = mt6630_mcu_sco(&dev->mt76.phy.chandef),
		.band = mt6630_mcu_band(chan->band),
		.req_type = MT6630_CH_PRIV_TYPE_JOIN,
		.max_interval = cpu_to_le32(MT6630_CH_PRIV_JOIN_MS),
	};
	int ret;

	mt6630_mcu_vht_width(&dev->mt76.phy.chandef, &req.width,
			     &req.center_seg1);
	req.token = ++dev->ch_token;
	reinit_completion(&dev->ch_grant);

	ret = mt76_mcu_send_msg(&dev->mt76, MCU_CMD_CH_PRIVILEGE, &req,
				sizeof(req), false);
	if (ret)
		return ret;

	/* a grant waits for a scan in progress to finish */
	if (!wait_for_completion_timeout(&dev->ch_grant, 5 * HZ))
		return -ETIMEDOUT;

	return 0;
}

/* mt76_rates: four CCK rates, then eight OFDM; 5 GHz has no CCK */
static unsigned long mt6630_mcu_rates(enum nl80211_band band, u32 rates)
{
	/* the mandatory rates, until the BSS has said otherwise */
	if (!rates)
		rates = band == NL80211_BAND_5GHZ ? BIT(0) | BIT(2) | BIT(4) :
						    GENMASK(3, 0);

	return band == NL80211_BAND_5GHZ ? (unsigned long)rates << 4 : rates;
}

static u16 mt6630_mcu_rate_set(enum nl80211_band band, u32 rates)
{
	unsigned long r = mt6630_mcu_rates(band, rates);
	u16 set = 0;
	int i;

	/* the firmware's rate set has two PBCC rates between CCK and OFDM */
	for_each_set_bit(i, &r, ARRAY_SIZE(mt76_rates))
		set |= BIT(i < 4 ? i : i + 2);

	return set;
}

/* management frames and retries go out at the lowest basic rate */
static u16 mt6630_mcu_default_rate(enum nl80211_band band, u32 basic_rates)
{
	unsigned long r = mt6630_mcu_rates(band, basic_rates);

	return mt6630_mac_rate_code(mt76_rates[__ffs(r)].hw_value);
}

static void mt6630_mcu_fill_rlm(struct mt6630_dev *dev,
				struct ieee80211_bss_conf *conf,
				struct mt6630_bss_rlm *rlm, bool ht, bool vht)
{
	struct cfg80211_chan_def *chandef = &dev->mt76.phy.chandef;
	enum nl80211_band band = chandef->chan->band;

	rlm->bss_idx = MT6630_BSS_IDX;
	rlm->band = mt6630_mcu_band(band);
	rlm->primary_channel = chandef->chan->hw_value;
	rlm->erp_protect = conf->use_cts_prot;
	rlm->short_preamble = conf->use_short_preamble;
	rlm->short_slot = conf->use_short_slot;
	/* the vendor driver fills this from the basic rates too */
	rlm->vht_basic_mcs =
		cpu_to_le16(mt6630_mcu_rate_set(band, conf->basic_rates));

	if (ht) {
		u16 op_mode = conf->ht_operation_mode;

		rlm->sco = mt6630_mcu_sco(chandef);
		rlm->ht_protect = op_mode & IEEE80211_HT_OP_MODE_PROTECTION;
		rlm->gf_mode =
			!!(op_mode & IEEE80211_HT_OP_MODE_NON_GF_STA_PRSNT);
		rlm->tx_rifs = MT6630_HT_RIFS_DISALLOWED;
		rlm->ht_op_info1 = rlm->sco;
		if (rlm->sco)
			rlm->ht_op_info1 |= IEEE80211_HT_PARAM_CHAN_WIDTH_ANY;
		rlm->ht_op_info2 = cpu_to_le16(op_mode);
	}

	if (vht)
		mt6630_mcu_vht_width(chandef, &rlm->vht_width,
				     &rlm->vht_freq_seg1);
}

static void mt6630_mcu_ap_caps(struct ieee80211_vif *vif, bool *ht, bool *vht)
{
	struct ieee80211_sta *sta;

	*ht = false;
	*vht = false;

	rcu_read_lock();
	sta = ieee80211_find_sta(vif, vif->bss_conf.bssid);
	if (sta) {
		*ht = sta->deflink.ht_cap.ht_supported;
		*vht = sta->deflink.vht_cap.vht_supported;
	}
	rcu_read_unlock();
}

/* What an AP changes in its beacons after association: protection,
 * preamble and slot time, as the vendor driver resyncs them
 */
int mt6630_mcu_set_rlm(struct mt6630_dev *dev, struct ieee80211_vif *vif)
{
	struct mt6630_bss_rlm req = {};
	bool ht, vht;

	mt6630_mcu_ap_caps(vif, &ht, &vht);
	mt6630_mcu_fill_rlm(dev, &vif->bss_conf, &req, ht, vht);

	return mt76_mcu_send_msg(&dev->mt76, MCU_CMD_SET_BSS_RLM_PARAM, &req,
				 sizeof(req), false);
}

int mt6630_mcu_set_bss_info(struct mt6630_dev *dev, struct ieee80211_vif *vif,
			    bool connected)
{
	struct ieee80211_bss_conf *conf = &vif->bss_conf;
	struct cfg80211_chan_def *chandef = &dev->mt76.phy.chandef;
	enum nl80211_band band = chandef->chan->band;
	bool g5 = band == NL80211_BAND_5GHZ;
	struct mt6630_bss_info req = {
		.bss_idx = MT6630_BSS_IDX,
		.conn_state = !connected,
		.ap_sta_idx = MT6630_STA_REC_NOT_FOUND,
		.rlm = {
			.bss_idx = MT6630_BSS_IDX,
		},
	};

	if (connected) {
		struct ieee80211_sta *sta;
		bool ht = false, vht = false;
		u16 rates = 0;

		rcu_read_lock();
		sta = ieee80211_find_sta(vif, conf->bssid);
		if (sta) {
			struct mt6630_sta *msta;

			msta = (struct mt6630_sta *)sta->drv_priv;
			rates = sta->deflink.supp_rates[band];
			ht = sta->deflink.ht_cap.ht_supported;
			vht = sta->deflink.vht_cap.vht_supported;
			req.ap_sta_idx = msta->sta_idx;
		}
		rcu_read_unlock();

		req.ssid_len = vif->cfg.ssid_len;
		memcpy(req.ssid, vif->cfg.ssid, vif->cfg.ssid_len);
		ether_addr_copy(req.bssid, conf->bssid);
		req.is_qbss = conf->qos;
		req.op_rate_set = cpu_to_le16(mt6630_mcu_rate_set(band, rates));
		req.basic_rate_set =
			cpu_to_le16(mt6630_mcu_rate_set(band, conf->basic_rates));
		req.hw_default_rate =
			cpu_to_le16(mt6630_mcu_default_rate(band,
							    conf->basic_rates));
		req.non_ht_phy_type = g5 ? MT6630_PHY_TYPE_OFDM :
					   MT6630_PHY_TYPE_ERP;
		req.phy_type_set = g5 ? MT6630_PHY_SET_OFDM :
					MT6630_PHY_SET_DSSS |
					MT6630_PHY_SET_ERP;
		req.bmc_wlan_idx = MT6630_WTBL_BMC;

		if (ht)
			req.phy_type_set |= MT6630_PHY_SET_HT;
		if (vht)
			req.phy_type_set |= MT6630_PHY_SET_VHT;
		mt6630_mcu_fill_rlm(dev, conf, &req.rlm, ht, vht);
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_CMD_SET_BSS_INFO, &req,
				 sizeof(req), false);
}

int mt6630_mcu_sta_update(struct mt6630_dev *dev, struct ieee80211_vif *vif,
			  struct ieee80211_sta *sta, bool assoc)
{
	struct mt6630_sta *msta = (struct mt6630_sta *)sta->drv_priv;
	enum nl80211_band band = dev->mt76.phy.chandef.chan->band;
	bool g5 = band == NL80211_BAND_5GHZ;
	struct mt6630_sta_rec req = {
		.sta_idx = msta->sta_idx,
		.sta_type = MT6630_STA_TYPE_LEGACY_AP,
		.aid = cpu_to_le16(vif->cfg.aid),
		.listen_interval =
			cpu_to_le16(dev->mt76.hw->conf.listen_interval),
		.bss_idx = MT6630_BSS_IDX,
		.phy_type_set = g5 ? MT6630_PHY_SET_OFDM :
				     MT6630_PHY_SET_DSSS | MT6630_PHY_SET_ERP,
		.basic_rate_set =
			cpu_to_le16(mt6630_mcu_rate_set(band,
							vif->bss_conf.basic_rates)),
		.is_qos = sta->wme,
		.rcpi = MT6630_RCPI_INIT,
		.wlan_idx = msta->wcid.idx,
		.bmc_wlan_idx = MT6630_WTBL_BMC,
		.rts_policy = MT6630_RTS_POLICY_LEGACY,
		.hw_default_rate =
			cpu_to_le16(mt6630_mcu_default_rate(band,
							    vif->bss_conf.basic_rates)),
	};

	ether_addr_copy(req.addr, sta->addr);

	/* a class 1 record before authentication makes the hardware ACK the
	 * AP; the class 3 one after association opens the data path, and
	 * only once the firmware has answered it
	 */
	if (!assoc)
		return mt76_mcu_send_msg(&dev->mt76,
					 MCU_CMD_UPDATE_STA_RECORD, &req,
					 sizeof(req), false);

	req.state = MT6630_STA_STATE_3;
	req.need_resp = true;
	req.non_ht_rate_set =
		cpu_to_le16(mt6630_mcu_rate_set(band,
						sta->deflink.supp_rates[band]));

	if (sta->deflink.ht_cap.ht_supported) {
		struct ieee80211_sta_ht_cap *ht = &sta->deflink.ht_cap;

		req.phy_type_set |= MT6630_PHY_SET_HT;
		req.is_qos = true;
		req.mcs_set = ht->mcs.rx_mask[0];
		req.mcs32 = ht->mcs.rx_mask[4] & BIT(0);
		memcpy(req.rx_mcs_bitmask, ht->mcs.rx_mask,
		       sizeof(req.rx_mcs_bitmask));
		req.rx_highest_rate = ht->mcs.rx_highest;
		req.tx_rate_info = cpu_to_le32(ht->mcs.tx_params);
		/* the vendor driver never transmits STBC */
		req.ht_cap_info = cpu_to_le16(ht->cap &
					      ~IEEE80211_HT_CAP_TX_STBC);
		req.ampdu_param =
			u8_encode_bits(ht->ampdu_factor,
				       IEEE80211_HT_AMPDU_PARM_FACTOR) |
			u8_encode_bits(ht->ampdu_density,
				       IEEE80211_HT_AMPDU_PARM_DENSITY);
		/* the firmware negotiates block-ack in both directions */
		req.tx_ampdu = MT6630_FEATURE_ENABLED;
		req.rx_ampdu = MT6630_FEATURE_ENABLED;
		req.tx_ba_size = IEEE80211_MAX_AMPDU_BUF_HT;
		req.rx_ba_size = IEEE80211_MAX_AMPDU_BUF_HT;
	}

	if (sta->deflink.vht_cap.vht_supported) {
		struct ieee80211_sta_vht_cap *vht = &sta->deflink.vht_cap;

		req.phy_type_set |= MT6630_PHY_SET_VHT;
		req.vht_cap_info = cpu_to_le32(vht->cap &
					       ~IEEE80211_VHT_CAP_TXSTBC);
		req.vht_rx_mcs_map = vht->vht_mcs.rx_mcs_map;
		req.vht_rx_highest_rate = vht->vht_mcs.rx_highest;
		req.vht_tx_mcs_map = vht->vht_mcs.tx_mcs_map;
		req.vht_tx_highest_rate = vht->vht_mcs.tx_highest;
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_CMD_UPDATE_STA_RECORD, &req,
				 sizeof(req), true);
}

int mt6630_mcu_sta_remove(struct mt6630_dev *dev, struct mt6630_sta *msta)
{
	struct mt6630_sta_remove req = {
		.sta_idx = msta->sta_idx,
		.bss_idx = MT6630_BSS_IDX,
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_CMD_REMOVE_STA_RECORD, &req,
				 sizeof(req), false);
}

/* Without this the firmware treats the BSS as gone once the join's channel
 * grant expires, and all reception stops a few seconds after association.
 */
int mt6630_mcu_bss_connected(struct mt6630_dev *dev,
			     struct ieee80211_vif *vif)
{
	struct mt6630_bss_connected req = {
		.bss_idx = MT6630_BSS_IDX,
		.dtim_period = vif->bss_conf.dtim_period ?: 1,
		.aid = cpu_to_le16(vif->cfg.aid),
		.beacon_interval = cpu_to_le16(vif->bss_conf.beacon_int),
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_CMD_PM_BSS_CONNECTED, &req,
				 sizeof(req), false);
}

int mt6630_mcu_bss_abort(struct mt6630_dev *dev)
{
	struct mt6630_bss_abort req = {
		.bss_idx = MT6630_BSS_IDX,
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_CMD_PM_BSS_ABORT, &req,
				 sizeof(req), false);
}

int mt6630_mcu_add_key(struct mt6630_dev *dev, struct ieee80211_vif *vif,
		       struct mt6630_sta *msta, struct ieee80211_key_conf *key,
		       enum set_key_cmd cmd)
{
	bool pairwise = key->flags & IEEE80211_KEY_FLAG_PAIRWISE;
	struct mt6630_key req = {
		.add = cmd == SET_KEY,
		.tx_key = pairwise,
		.key_type = pairwise,
		.bss_idx = MT6630_BSS_IDX,
		.key_idx = key->keyidx,
		.key_len = key->keylen,
	};

	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
		req.algorithm = MT6630_CIPHER_WEP40;
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		req.algorithm = MT6630_CIPHER_WEP104;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		req.algorithm = MT6630_CIPHER_TKIP;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		req.algorithm = MT6630_CIPHER_CCMP;
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (key->keylen > sizeof(req.key))
		return -EINVAL;

	/* group keys are installed against the AP as well */
	ether_addr_copy(req.peer_addr, vif->bss_conf.bssid);
	req.wlan_idx = pairwise ? msta->wcid.idx : MT6630_WTBL_BMC;
	memcpy(req.key, key->key, key->keylen);

	/* the firmware wants the RX MIC key before the TX one */
	if (key->cipher == WLAN_CIPHER_SUITE_TKIP) {
		memcpy(req.key + 16, key->key + 24, 8);
		memcpy(req.key + 24, key->key + 16, 8);
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_CMD_ADD_REMOVE_KEY, &req,
				 sizeof(req), false);
}

int mt6630_mcu_set_wmm(struct mt6630_dev *dev, u8 queue,
		       const struct ieee80211_tx_queue_params *params)
{
	/* mac80211 orders the queues VO, VI, BE, BK; the firmware BE, BK,
	 * VI, VO
	 */
	static const u8 to_fw[] = { 3, 2, 0, 1 };
	struct mt6630_wmm_params *req = &dev->wmm;
	int ac = to_fw[queue];

	req->bss_idx = MT6630_BSS_IDX;
	req->is_qbss = true;
	req->ac[ac].cw_min = cpu_to_le16(params->cw_min);
	req->ac[ac].cw_max = cpu_to_le16(params->cw_max);
	req->ac[ac].txop = cpu_to_le16(params->txop);
	req->ac[ac].aifsn = cpu_to_le16(params->aifs);
	req->ac[ac].acm = params->acm;

	return mt76_mcu_send_msg(&dev->mt76, MCU_CMD_UPDATE_WMM_PARMS, req,
				 sizeof(*req), false);
}

int mt6630_mcu_hw_scan(struct mt6630_dev *dev, struct ieee80211_vif *vif,
		       struct ieee80211_scan_request *scan_req)
{
	struct cfg80211_scan_request *sreq = &scan_req->req;
	int n_ssids = 0, i, ret;
	struct mt6630_scan_req *req;
	struct sk_buff *skb;

	skb = mt76_mcu_msg_alloc(&dev->mt76, NULL, sizeof(*req));
	if (!skb)
		return -ENOMEM;

	req = (struct mt6630_scan_req *)skb_put_zero(skb, sizeof(*req));
	req->seq = ++dev->scan_seq ?: ++dev->scan_seq;
	req->bss_idx = MT6630_BSS_IDX;
	req->scan_type = sreq->n_ssids ? MT6630_SCAN_TYPE_ACTIVE :
					 MT6630_SCAN_TYPE_PASSIVE;
	req->ssid_type = MT6630_SCAN_SSID_WILDCARD;
	req->probe_req_num = sreq->n_ssids ? MT6630_SCAN_PROBE_REQS : 0;

	for (i = 0; i < min(sreq->n_ssids, MT6630_SCAN_SSIDS); i++) {
		if (!sreq->ssids[i].ssid_len)
			continue;

		req->ssids[n_ssids].len = cpu_to_le32(sreq->ssids[i].ssid_len);
		memcpy(req->ssids[n_ssids].ssid, sreq->ssids[i].ssid,
		       sreq->ssids[i].ssid_len);
		n_ssids++;
	}
	if (n_ssids)
		req->ssid_type |= MT6630_SCAN_SSID_SPECIFIED;
	req->ssids_num = n_ssids;

	req->channel_type = MT6630_SCAN_CHANNEL_SPECIFIED;
	req->channels_num = min_t(u8, sreq->n_channels, MT6630_SCAN_CHANNELS);
	for (i = 0; i < req->channels_num; i++) {
		struct ieee80211_channel *chan = sreq->channels[i];

		req->channels[i].band = mt6630_mcu_band(chan->band);
		req->channels[i].channel = chan->hw_value;
	}

	if (sreq->ie_len <= sizeof(req->ies)) {
		req->ies_len = cpu_to_le16(sreq->ie_len);
		memcpy(req->ies, sreq->ie, sreq->ie_len);
	}

	/* before the send: the answer can come before it returns */
	set_bit(0, &dev->scan_pending);
	ret = mt76_mcu_skb_send_msg(&dev->mt76, skb, MCU_CMD_SCAN_REQ, false);
	if (ret) {
		clear_bit(0, &dev->scan_pending);
		return ret;
	}
	/* a scan the firmware drops would leave mac80211 waiting for ever */
	ieee80211_queue_delayed_work(dev->mt76.hw, &dev->scan_work, 10 * HZ);

	return 0;
}

int mt6630_mcu_cancel_hw_scan(struct mt6630_dev *dev)
{
	struct mt6630_scan_cancel req = {
		.seq = dev->scan_seq,
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_CMD_SCAN_CANCEL, &req,
				 sizeof(req), false);
}

void mt6630_mcu_scan_done(struct mt6630_dev *dev, bool aborted)
{
	struct cfg80211_scan_info info = {
		.aborted = aborted,
	};

	if (test_and_clear_bit(0, &dev->scan_pending))
		ieee80211_scan_completed(dev->mt76.hw, &info);
}

void mt6630_scan_work(struct work_struct *work)
{
	struct mt6630_dev *dev = container_of(work, struct mt6630_dev,
					      scan_work.work);

	dev_warn(dev->mt76.dev, "Scan did not complete\n");
	mt6630_mcu_scan_done(dev, true);
}

static void mt6630_mcu_beacon_loss(void *priv, u8 *mac,
				   struct ieee80211_vif *vif)
{
	if (vif->type == NL80211_IFTYPE_STATION && vif->cfg.assoc)
		ieee80211_connection_loss(vif);
}

/* Firmware station records map onto WTBL entries one up */
static void mt6630_mcu_queue_ba(struct mt6630_dev *dev, u8 sta_idx, u8 tid,
				u16 ssn, u16 size, bool start)
{
	struct mt6630_ba_event *ev;

	if (tid >= IEEE80211_NUM_TIDS || sta_idx + 1 >= MT6630_WTBL_SIZE)
		return;

	ev = kzalloc_obj(*ev, GFP_ATOMIC);
	if (!ev)
		return;

	ev->wlan_idx = sta_idx + 1;
	ev->tid = tid;
	ev->ssn = ssn;
	ev->size = size ?: IEEE80211_MAX_AMPDU_BUF_HT;
	ev->start = start;

	/* mt6630_stop() has emptied the list, and mac80211's workqueue may
	 * be gone
	 */
	spin_lock_bh(&dev->ba_lock);
	if (test_bit(MT76_STATE_RUNNING, &dev->mt76.phy.state)) {
		list_add_tail(&ev->list, &dev->ba_list);
		ieee80211_queue_work(dev->mt76.hw, &dev->ba_work);
		ev = NULL;
	}
	spin_unlock_bh(&dev->ba_lock);

	kfree(ev);
}

void mt6630_mcu_rx_event(struct mt6630_dev *dev, struct sk_buff *skb)
{
	struct mt6630_mcu_rxd *rxd = (struct mt6630_mcu_rxd *)skb->data;
	void *data = skb->data + sizeof(*rxd);
	int len = skb->len - sizeof(*rxd);

	if (len < 0)
		goto out;

	switch (rxd->eid) {
	case MCU_EVENT_SCAN_DONE: {
		struct mt6630_scan_done_event *ev = data;

		if (len < sizeof(*ev) || ev->seq != dev->scan_seq)
			break;

		cancel_delayed_work(&dev->scan_work);
		mt6630_mcu_scan_done(dev, false);
		break;
	}
	case MCU_EVENT_CH_PRIVILEGE: {
		struct mt6630_ch_privilege_event *ev = data;

		if (len >= sizeof(*ev) && ev->token == dev->ch_token &&
		    !ev->status)
			complete(&dev->ch_grant);
		break;
	}
	case MCU_EVENT_TX_DONE: {
		struct mt6630_tx_done_event *ev = data;

		if (len >= sizeof(*ev))
			mt6630_mac_tx_done(dev, ev->wlan_idx, ev->pid,
					   !ev->status);
		break;
	}
	case MCU_EVENT_BSS_BEACON_TIMEOUT:
		ieee80211_iterate_active_interfaces_atomic(dev->mt76.hw,
							   IEEE80211_IFACE_ITER_RESUME_ALL,
							   mt6630_mcu_beacon_loss,
							   dev);
		break;
	case MCU_EVENT_RX_ADDBA: {
		struct mt6630_rx_addba_event *ev = data;
		u16 param;

		if (len < sizeof(*ev))
			break;

		param = le16_to_cpu(ev->ba_param);
		mt6630_mcu_queue_ba(dev, ev->sta_idx,
				    u16_get_bits(param,
						 IEEE80211_ADDBA_PARAM_TID_MASK),
				    IEEE80211_SEQ_TO_SN(le16_to_cpu(ev->start_seq)),
				    u16_get_bits(param,
						 IEEE80211_ADDBA_PARAM_BUF_SIZE_MASK),
				    true);
		break;
	}
	case MCU_EVENT_RX_DELBA: {
		struct mt6630_rx_delba_event *ev = data;

		if (len >= sizeof(*ev))
			mt6630_mcu_queue_ba(dev, ev->sta_idx, ev->tid, 0, 0,
					    false);
		break;
	}
	case MCU_EVENT_DEBUG_MSG: {
		struct mt6630_debug_msg_event *ev = data;

		if (len > sizeof(*ev))
			dev_dbg(dev->mt76.dev, "fw: %.*s\n",
				min_t(int, le16_to_cpu(ev->msg_size),
				      len - sizeof(*ev)), ev->msg);
		break;
	}
	case MCU_EVENT_STA_UPDATE_FREE_QUOTA:
		break;
	default:
		/* everything else answers a command */
		if (rxd->seq) {
			mt76_mcu_rx_event(&dev->mt76, skb);
			return;
		}
		break;
	}
out:
	dev_kfree_skb(skb);
}
