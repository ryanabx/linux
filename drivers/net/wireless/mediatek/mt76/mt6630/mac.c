// SPDX-License-Identifier: BSD-3-Clause-Clear
/* Copyright (C) 2026 Ryan Brue <ryanbrue.dev@gmail.com> */

#include <linux/etherdevice.h>

#include "mt6630.h"
#include "mac.h"

#define MT6630_WTBL_NO_STA		31

#define MT6630_TX_COUNT_MGMT		30
#define MT6630_TX_COUNT_DATA		7

static void
mt6630_mac_write_txwi(struct mt6630_dev *dev, __le32 *txwi,
		      struct sk_buff *skb, enum mt76_txq_id qid,
		      struct mt76_wcid *wcid, struct ieee80211_key_conf *key,
		      bool port_ctrl, int pid)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	bool mcast = is_multicast_ether_addr(hdr->addr1);
	__le16 fc = hdr->frame_control;
	int hdr_len = ieee80211_get_hdrlen_from_skb(skb);
	bool mgmt = ieee80211_is_mgmt(fc);
	bool fixed_rate = mgmt;
	u8 p_idx, q_idx, wlan_idx;
	u16 tx_count, rate;
	u32 val;

	if (wcid->sta)
		wlan_idx = wcid->idx;
	else if (mcast)
		wlan_idx = MT6630_WTBL_BMC;
	else
		wlan_idx = MT6630_WTBL_NO_STA;

	/* management frames go through the MCU, which also sends them at
	 * the basic rate, as it does EAPOL frames
	 */
	if (mgmt) {
		p_idx = MT_TX_PORT_MCU;
		q_idx = MT_TX_MCU_Q_MGMT;
	} else {
		p_idx = MT_TX_PORT_LMAC;
		/* the LMAC numbers the AC queues BK, BE, VI, VO */
		q_idx = ieee80211_is_data_qos(fc) ? IEEE80211_AC_BK - qid :
						    MT_TX_LMAC_Q_NON_QOS;
		fixed_rate = port_ctrl;
	}

	val = FIELD_PREP(MT_TXD0_TX_BYTES, skb->len + MT_TXD_SIZE) |
	      FIELD_PREP(MT_TXD0_P_IDX, p_idx) |
	      FIELD_PREP(MT_TXD0_Q_IDX, q_idx);
	txwi[0] = cpu_to_le32(val);

	val = MT_TXD1_LONG_FORMAT |
	      FIELD_PREP(MT_TXD1_WLAN_IDX, wlan_idx) |
	      FIELD_PREP(MT_TXD1_HDR_FORMAT, MT_HDR_FORMAT_802_11) |
	      FIELD_PREP(MT_TXD1_HDR_INFO, hdr_len / 2) |
	      FIELD_PREP(MT_TXD1_OWN_MAC, MT6630_OMAC_IDX);
	if (ieee80211_is_data_qos(fc))
		val |= FIELD_PREP(MT_TXD1_TID, ieee80211_get_tid(hdr));
	if (mcast || (info->flags & IEEE80211_TX_CTL_NO_ACK))
		val |= MT_TXD1_NO_ACK;
	if (key)
		val |= MT_TXD1_PROTECTED;
	txwi[1] = cpu_to_le32(val);

	val = FIELD_PREP(MT_TXD2_FRAME_TYPE,
			 le16_get_bits(fc, IEEE80211_FCTL_FTYPE)) |
	      FIELD_PREP(MT_TXD2_SUB_TYPE,
			 le16_get_bits(fc, IEEE80211_FCTL_STYPE));
	if (mcast)
		val |= MT_TXD2_MULTICAST;
	if (fixed_rate)
		val |= MT_TXD2_FIX_RATE;
	txwi[2] = cpu_to_le32(val);

	tx_count = mgmt ? MT6630_TX_COUNT_MGMT : MT6630_TX_COUNT_DATA;
	txwi[3] = cpu_to_le32(FIELD_PREP(MT_TXD3_REM_TX_COUNT, tx_count));
	txwi[4] = 0;

	val = FIELD_PREP(MT_TXD5_PID, pid);
	if (pid >= MT_PACKET_ID_FIRST)
		val |= MT_TXD5_TX_STATUS_MCU;
	txwi[5] = cpu_to_le32(val);

	/* the lowest rate of the band: OFDM 6M, or CCK 1M */
	rate = mt6630_mac_rate_code(dev->mt76.phy.chandef.chan->band ==
				    NL80211_BAND_5GHZ ? mt76_rates[4].hw_value :
							mt76_rates[0].hw_value);
	txwi[6] = fixed_rate ? cpu_to_le32(FIELD_PREP(MT_TXD6_TX_RATE, rate)) :
			       0;
}

/* mt76_rates' hardware values carry the PHY type above the rate index */
u16 mt6630_mac_rate_code(u16 hw_value)
{
	return FIELD_PREP(MT_TX_RATE_MODE, hw_value >> 8) |
	       FIELD_PREP(MT_TX_RATE_IDX, hw_value & 0xff);
}

int mt6630_tx_prepare_skb(struct mt76_dev *mdev, void *txwi_ptr,
			  enum mt76_txq_id qid, struct mt76_wcid *wcid,
			  struct ieee80211_sta *sta,
			  struct mt76_tx_info *tx_info)
{
	struct mt6630_dev *dev = container_of(mdev, struct mt6630_dev, mt76);
	struct sk_buff *skb = tx_info->skb;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	/* read before the status tracking reuses this part of the cb */
	struct ieee80211_key_conf *key = info->control.hw_key;
	bool port_ctrl = info->control.flags &
			 IEEE80211_TX_CTRL_PORT_CTRL_PROTO;
	__le32 *txwi;
	int pid, err;

	if (!wcid)
		wcid = &mdev->global_wcid;

	err = skb_cow_head(skb, MT_TXD_SIZE);
	if (err)
		return err;

	pid = mt76_tx_status_skb_add(mdev, wcid, skb);
	txwi = (__le32 *)(skb->data - MT_TXD_SIZE);
	mt6630_mac_write_txwi(dev, txwi, skb, qid, wcid, key, port_ctrl, pid);
	skb_push(skb, MT_TXD_SIZE);

	err = mt76_skb_adjust_pad(skb, round_up(skb->len, 4) - skb->len);
	if (err)
		idr_remove(&wcid->pktid, pid);

	return err;
}

void mt6630_tx_complete_skb(struct mt76_dev *mdev,
			    struct mt76_queue_entry *e)
{
	skb_pull(e->skb, MT_TXD_SIZE);
	mt76_tx_complete_skb(mdev, e->wcid, e->skb);
}

void mt6630_mac_tx_done(struct mt6630_dev *dev, u8 wlan_idx, u8 pid,
			bool acked)
{
	struct mt76_dev *mdev = &dev->mt76;
	struct mt76_wcid *wcid;
	struct sk_buff_head list;
	struct sk_buff *skb;

	rcu_read_lock();

	wcid = rcu_dereference(mdev->wcid[wlan_idx]);
	if (!wcid && wlan_idx == MT6630_WTBL_BMC)
		wcid = &mdev->global_wcid;
	if (!wcid)
		goto out;

	mt76_tx_status_lock(mdev, &list);
	skb = mt76_tx_status_skb_get(mdev, wcid, pid, &list);
	if (skb) {
		struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);

		if (acked)
			info->flags |= IEEE80211_TX_STAT_ACK;
		mt76_tx_status_skb_done(mdev, skb, &list);
	}
	mt76_tx_status_unlock(mdev, &list);
out:
	rcu_read_unlock();
}

/* The firmware translates every data frame to 802.3, even before the port
 * is authorized; mac80211 cannot take EAPOL that way, so the 802.11 header
 * is put back from what RX group 4 kept of it.
 */
static int mt6630_mac_rx_rebuild_hdr(struct sk_buff *skb, __le32 *rxd_g4)
{
	struct ieee80211_hdr *hdr;
	u8 ta[ETH_ALEN];
	struct ethhdr eth;
	u16 seq_ctrl, qos;
	__le16 fc;
	int hdr_len;

	if (skb->len < ETH_HLEN)
		return -EINVAL;

	fc = cpu_to_le16(le32_get_bits(rxd_g4[0], MT_RXD4_FC));
	if (!ieee80211_is_data(fc) ||
	    ieee80211_has_tods(fc) || !ieee80211_has_fromds(fc))
		return -EINVAL;

	/* the new header may overlap group 4 */
	memcpy(ta, (u8 *)rxd_g4 + 2, ETH_ALEN);
	seq_ctrl = le32_get_bits(rxd_g4[2], MT_RXD6_SEQ_CTRL);
	qos = le32_get_bits(rxd_g4[2], MT_RXD6_QOS_CTL);
	memcpy(&eth, skb->data, ETH_HLEN);
	skb_pull(skb, ETH_HLEN - 2);

	/* LLC/SNAP in front of the ethertype that is left */
	memcpy(skb_push(skb, sizeof(rfc1042_header)), rfc1042_header,
	       sizeof(rfc1042_header));

	hdr_len = ieee80211_hdrlen(fc);
	hdr = skb_push(skb, hdr_len);
	memset(hdr, 0, hdr_len);
	hdr->frame_control = fc;
	ether_addr_copy(hdr->addr1, eth.h_dest);
	ether_addr_copy(hdr->addr2, ta);
	ether_addr_copy(hdr->addr3, eth.h_source);
	hdr->seq_ctrl = cpu_to_le16(seq_ctrl);
	/* the payload is a single MSDU now */
	if (ieee80211_is_data_qos(fc))
		put_unaligned_le16(qos & ~IEEE80211_QOS_CTL_A_MSDU_PRESENT,
				   ieee80211_get_qos_ctl(hdr));

	return 0;
}

static int mt6630_mac_fill_rx(struct mt6630_dev *dev, struct sk_buff *skb)
{
	struct mt76_rx_status *status = (struct mt76_rx_status *)skb->cb;
	__le32 *rxd = (__le32 *)skb->data;
	__le32 *rxd_g4 = NULL, *rxd_g1 = NULL, *rxv = NULL;
	struct ieee80211_hdr *hdr;
	u32 rxd0 = le32_to_cpu(rxd[0]);
	u32 rxd1 = le32_to_cpu(rxd[1]);
	u32 rxd2 = le32_to_cpu(rxd[2]);
	int len = MT_RXD_SIZE;
	u8 channel, idx;

	/* the groups follow the base descriptor in the order 4, 1, 2, 3 */
	if (rxd0 & MT_RXD0_GROUP_4) {
		rxd_g4 = rxd + len / 4;
		len += MT_RXD_GROUP_4_SIZE;
	}

	if (rxd0 & MT_RXD0_GROUP_1) {
		rxd_g1 = rxd + len / 4;
		len += MT_RXD_GROUP_1_SIZE;
	}

	if (rxd0 & MT_RXD0_GROUP_2)
		len += MT_RXD_GROUP_2_SIZE;

	if (rxd0 & MT_RXD0_GROUP_3) {
		rxv = rxd + len / 4;
		len += MT_RXD_GROUP_3_SIZE;
	}

	if (rxd1 & MT_RXD1_NORMAL_HDR_OFFSET)
		len += 2;

	if (skb->len < len)
		return -EINVAL;

	memset(status, 0, sizeof(*status));

	channel = FIELD_GET(MT_RXD1_NORMAL_CH_FREQ, rxd1);
	status->band = channel > 14 ? NL80211_BAND_5GHZ : NL80211_BAND_2GHZ;
	status->freq = ieee80211_channel_to_frequency(channel, status->band);

	if (rxd2 & MT_RXD2_NORMAL_FCS_ERR)
		status->flag |= RX_FLAG_FAILED_FCS_CRC;
	if (rxd2 & MT_RXD2_NORMAL_TKIP_MIC_ERR)
		status->flag |= RX_FLAG_MMIC_ERROR;

	if (FIELD_GET(MT_RXD2_NORMAL_SEC_MODE, rxd2) &&
	    !(rxd2 & (MT_RXD2_NORMAL_CLM | MT_RXD2_NORMAL_CM |
		      MT_RXD2_NORMAL_ICV_ERR))) {
		status->flag |= RX_FLAG_DECRYPTED;
		status->flag |= RX_FLAG_IV_STRIPPED | RX_FLAG_MMIC_STRIPPED;
	}

	idx = FIELD_GET(MT_RXD2_NORMAL_WLAN_IDX, rxd2);
	status->wcid = rcu_dereference(dev->mt76.wcid[idx]);

	if (rxd_g1 && (status->flag & RX_FLAG_DECRYPTED)) {
		u8 *pn = (u8 *)rxd_g1;

		status->iv[0] = pn[5];
		status->iv[1] = pn[4];
		status->iv[2] = pn[3];
		status->iv[3] = pn[2];
		status->iv[4] = pn[1];
		status->iv[5] = pn[0];
	}

	if (rxv) {
		struct ieee80211_supported_band *sband;
		u32 rxv1 = le32_to_cpu(rxv[0]);
		u32 rxv3 = le32_to_cpu(rxv[2]);
		int idx = FIELD_GET(MT_RXV1_TX_RATE, rxv1);
		bool cck = false;

		if (status->band == NL80211_BAND_5GHZ)
			sband = &dev->mt76.phy.sband_5g.sband;
		else
			sband = &dev->mt76.phy.sband_2g.sband;

		switch (FIELD_GET(MT_RXV1_TX_MODE, rxv1)) {
		case MT_PHY_TYPE_CCK:
			cck = true;
			fallthrough;
		case MT_PHY_TYPE_OFDM:
			idx = mt76_get_rate(&dev->mt76, sband, idx, cck);
			break;
		case MT_PHY_TYPE_HT_GF:
		case MT_PHY_TYPE_HT:
			status->encoding = RX_ENC_HT;
			if (idx > 7)
				return -EINVAL;
			if (rxv1 & MT_RXV1_HT_SHORT_GI)
				status->enc_flags |= RX_ENC_FLAG_SHORT_GI;
			break;
		case MT_PHY_TYPE_VHT:
			status->encoding = RX_ENC_VHT;
			status->nss = 1;
			idx &= MT_RXV1_VHT_MCS;
			if (idx > 9)
				return -EINVAL;
			if (rxv1 & MT_RXV1_HT_SHORT_GI)
				status->enc_flags |= RX_ENC_FLAG_SHORT_GI;
			break;
		default:
			return -EINVAL;
		}
		status->rate_idx = idx;

		switch (FIELD_GET(MT_RXV1_FRAME_MODE, rxv1)) {
		case 1:
			status->bw = RATE_INFO_BW_40;
			break;
		case 2:
			status->bw = RATE_INFO_BW_80;
			break;
		}

		status->chains = BIT(0);
		status->chain_signal[0] = FIELD_GET(MT_RXV3_RCPI0, rxv3) / 2 -
					  110;
		status->signal = status->chain_signal[0];
	} else {
		status->flag |= RX_FLAG_NO_SIGNAL_VAL;
	}

	skb_pull(skb, len);

	if (rxd1 & MT_RXD1_NORMAL_HDR_TRANS) {
		if (!rxd_g4 || mt6630_mac_rx_rebuild_hdr(skb, rxd_g4))
			return -EINVAL;
	}

	hdr = (struct ieee80211_hdr *)skb->data;
	if (skb->len < sizeof(hdr->frame_control) ||
	    skb->len < ieee80211_hdrlen(hdr->frame_control))
		return -EINVAL;

	if (ieee80211_is_data_qos(hdr->frame_control)) {
		status->aggr = (rxd1 & MT_RXD1_NORMAL_U2M) &&
			       !ieee80211_is_qos_nullfunc(hdr->frame_control);
		status->qos_ctl = *ieee80211_get_qos_ctl(hdr);
		status->seqno = IEEE80211_SEQ_TO_SN(le16_to_cpu(hdr->seq_ctrl));
	}

	return 0;
}

void mt6630_mac_ba_flush(struct mt6630_dev *dev)
{
	struct mt6630_ba_event *ev, *tmp;

	spin_lock_bh(&dev->ba_lock);
	list_for_each_entry_safe(ev, tmp, &dev->ba_list, list) {
		list_del(&ev->list);
		kfree(ev);
	}
	spin_unlock_bh(&dev->ba_lock);
}

void mt6630_mac_ba_work(struct work_struct *work)
{
	struct mt6630_dev *dev = container_of(work, struct mt6630_dev,
					      ba_work);
	struct mt6630_ba_event *ev, *tmp;
	struct mt76_wcid *wcid;
	LIST_HEAD(list);

	spin_lock_bh(&dev->ba_lock);
	list_splice_init(&dev->ba_list, &list);
	spin_unlock_bh(&dev->ba_lock);

	mutex_lock(&dev->mt76.mutex);
	list_for_each_entry_safe(ev, tmp, &list, list) {
		wcid = rcu_dereference_protected(dev->mt76.wcid[ev->wlan_idx],
						 lockdep_is_held(&dev->mt76.mutex));
		if (wcid && wcid->sta) {
			if (ev->start)
				mt76_rx_aggr_start(&dev->mt76, wcid, ev->tid,
						   ev->ssn, ev->size);
			else
				mt76_rx_aggr_stop(&dev->mt76, wcid, ev->tid);
		}

		list_del(&ev->list);
		kfree(ev);
	}
	mutex_unlock(&dev->mt76.mutex);
}

void mt6630_queue_rx_skb(struct mt76_dev *mdev, enum mt76_rxq_id q,
			 struct sk_buff *skb, u32 *info)
{
	struct mt6630_dev *dev = container_of(mdev, struct mt6630_dev, mt76);
	u32 rxd0;

	if (skb->len < sizeof(rxd0)) {
		dev_kfree_skb(skb);
		return;
	}

	rxd0 = le32_to_cpu(((__le32 *)skb->data)[0]);
	switch (FIELD_GET(MT_RXD0_PKT_TYPE, rxd0)) {
	case PKT_TYPE_SW:
		if (FIELD_GET(MT_RXD0_SW_PKT_TYPE, rxd0) ==
		    MT_RXD0_SW_PKT_EVENT) {
			mt6630_mcu_rx_event(dev, skb);
			return;
		}
		fallthrough;
	case PKT_TYPE_NORMAL:
		if (skb->len >= MT_RXD_SIZE && !mt6630_mac_fill_rx(dev, skb)) {
			mt76_rx(&dev->mt76, q, skb);
			return;
		}
		fallthrough;
	default:
		dev_kfree_skb(skb);
		break;
	}
}

int mt6630_mac_sta_add(struct mt76_dev *mdev, struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta)
{
	struct mt6630_dev *dev = container_of(mdev, struct mt6630_dev, mt76);
	struct mt6630_sta *msta = (struct mt6630_sta *)sta->drv_priv;
	int idx, ret;

	idx = mt76_wcid_alloc(mdev->wcid_mask, MT6630_WTBL_SIZE);
	if (idx < 0)
		return -ENOSPC;

	msta->wcid.idx = idx;
	msta->wcid.sta = 1;
	/* firmware station records count from zero, WTBL entries from one */
	msta->sta_idx = idx - 1;

	ret = mt6630_mcu_sta_update(dev, vif, sta, false);
	if (ret)
		mt76_wcid_mask_clear(mdev->wcid_mask, idx);

	return ret;
}

void mt6630_mac_sta_remove(struct mt76_dev *mdev, struct ieee80211_vif *vif,
			   struct ieee80211_sta *sta)
{
	struct mt6630_dev *dev = container_of(mdev, struct mt6630_dev, mt76);
	struct mt6630_sta *msta = (struct mt6630_sta *)sta->drv_priv;

	mt6630_mcu_sta_remove(dev, msta);
}
