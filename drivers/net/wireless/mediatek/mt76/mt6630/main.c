// SPDX-License-Identifier: BSD-3-Clause-Clear
/* Copyright (C) 2026 Ryan Brue <ryanbrue.dev@gmail.com> */

#include <linux/etherdevice.h>

#include "mt6630.h"
#include "mac.h"

static int mt6630_start(struct ieee80211_hw *hw)
{
	struct mt6630_dev *dev = mt6630_hw_dev(hw);

	set_bit(MT76_STATE_RUNNING, &dev->mt76.phy.state);

	return 0;
}

static void mt6630_stop(struct ieee80211_hw *hw, bool suspend)
{
	struct mt6630_dev *dev = mt6630_hw_dev(hw);

	spin_lock_bh(&dev->ba_lock);
	clear_bit(MT76_STATE_RUNNING, &dev->mt76.phy.state);
	spin_unlock_bh(&dev->ba_lock);

	cancel_delayed_work_sync(&dev->scan_work);
	cancel_work_sync(&dev->ba_work);
	mt6630_mac_ba_flush(dev);
}

static int mt6630_add_interface(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif)
{
	struct mt6630_vif *mvif = (struct mt6630_vif *)vif->drv_priv;
	struct mt6630_dev *dev = mt6630_hw_dev(hw);
	struct mt76_txq *mtxq;
	int ret;

	mutex_lock(&dev->mt76.mutex);

	if (dev->vif) {
		ret = -EBUSY;
		goto out;
	}

	mvif->mt76.idx = MT6630_BSS_IDX;
	mvif->mt76.omac_idx = MT6630_OMAC_IDX;

	/* the firmware misses the AP's answers while it dozes */
	ret = mt6630_mcu_set_ps_profile(dev, MT6630_PS_CONTINUOUS_ACTIVE);
	if (ret)
		goto out;

	ret = mt6630_mcu_bss_activate(dev, vif, true);
	if (ret)
		goto out;

	mvif->sta.wcid.idx = MT6630_WTBL_BMC;
	mvif->sta.wcid.hw_key_idx = -1;
	mt76_wcid_init(&mvif->sta.wcid, 0);
	rcu_assign_pointer(dev->mt76.wcid[MT6630_WTBL_BMC], &mvif->sta.wcid);

	if (vif->txq) {
		mtxq = (struct mt76_txq *)vif->txq->drv_priv;
		mtxq->wcid = MT6630_WTBL_BMC;
	}

	dev->vif = mvif;
out:
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static void mt6630_remove_interface(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif)
{
	struct mt6630_vif *mvif = (struct mt6630_vif *)vif->drv_priv;
	struct mt6630_dev *dev = mt6630_hw_dev(hw);

	mutex_lock(&dev->mt76.mutex);
	mt6630_mcu_bss_activate(dev, vif, false);
	rcu_assign_pointer(dev->mt76.wcid[MT6630_WTBL_BMC], NULL);
	mt76_wcid_cleanup(&dev->mt76, &mvif->sta.wcid);
	dev->vif = NULL;
	mutex_unlock(&dev->mt76.mutex);
}

static int mt6630_set_channel(struct mt76_phy *phy)
{
	/* the firmware tunes the radio itself, for scans and for the joins
	 * it grants channel time to
	 */
	return 0;
}

static int mt6630_config(struct ieee80211_hw *hw, int radio_idx, u32 changed)
{
	struct mt6630_dev *dev = mt6630_hw_dev(hw);

	if (changed & IEEE80211_CONF_CHANGE_CHANNEL)
		return mt76_update_channel(&dev->mt76.phy);

	return 0;
}

static void mt6630_configure_filter(struct ieee80211_hw *hw,
				    unsigned int changed_flags,
				    unsigned int *total_flags, u64 multicast)
{
	*total_flags &= FIF_ALLMULTI | FIF_BCN_PRBRESP_PROMISC;
}

static void mt6630_bss_info_changed(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif,
				    struct ieee80211_bss_conf *info,
				    u64 changed)
{
	struct mt6630_dev *dev = mt6630_hw_dev(hw);
	struct ieee80211_sta *sta;

	mutex_lock(&dev->mt76.mutex);

	if (!(changed & BSS_CHANGED_ASSOC)) {
		if (vif->cfg.assoc &&
		    (changed & (BSS_CHANGED_ERP_CTS_PROT |
				BSS_CHANGED_ERP_PREAMBLE |
				BSS_CHANGED_ERP_SLOT | BSS_CHANGED_HT)))
			mt6630_mcu_set_rlm(dev, vif);
	} else if (vif->cfg.assoc) {
		/* the vendor's order: the connected BSS, then the AP's class 3
		 * record, then the power management side of the connection
		 */
		mt6630_mcu_set_bss_info(dev, vif, true);

		rcu_read_lock();
		sta = ieee80211_find_sta(vif, info->bssid);
		rcu_read_unlock();
		if (sta)
			mt6630_mcu_sta_update(dev, vif, sta, true);

		/* the connection resets the firmware's power save state */
		mt6630_mcu_set_ps_profile(dev, MT6630_PS_CONTINUOUS_ACTIVE);
		mt6630_mcu_bss_connected(dev, vif);
	} else {
		/* releases the channel the connection held */
		mt6630_mcu_bss_abort(dev);
		mt6630_mcu_set_bss_info(dev, vif, false);
	}

	mutex_unlock(&dev->mt76.mutex);
}

static int mt6630_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
			  struct ieee80211_vif *vif, struct ieee80211_sta *sta,
			  struct ieee80211_key_conf *key)
{
	struct mt6630_vif *mvif = (struct mt6630_vif *)vif->drv_priv;
	struct mt6630_dev *dev = mt6630_hw_dev(hw);
	struct mt6630_sta *msta;
	struct mt76_wcid *wcid;
	int ret;

	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
	case WLAN_CIPHER_SUITE_TKIP:
	case WLAN_CIPHER_SUITE_CCMP:
		break;
	default:
		return -EOPNOTSUPP;
	}

	msta = sta ? (struct mt6630_sta *)sta->drv_priv : &mvif->sta;
	wcid = &msta->wcid;

	mutex_lock(&dev->mt76.mutex);

	if (key->flags & IEEE80211_KEY_FLAG_PAIRWISE)
		wcid->hw_key_idx = cmd == SET_KEY ? key->keyidx : -1;

	/* mac80211 encrypts robust management frames; BIP stays in
	 * software too, as the cipher is not offloaded
	 */
	if (key->flags & IEEE80211_KEY_FLAG_RX_MGMT)
		key->flags |= IEEE80211_KEY_FLAG_SW_MGMT_TX;

	mt76_wcid_key_setup(&dev->mt76, wcid,
			    cmd == SET_KEY ? key : NULL);

	ret = mt6630_mcu_add_key(dev, vif, msta, key, cmd);

	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static int mt6630_conf_tx(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			  unsigned int link_id, u16 queue,
			  const struct ieee80211_tx_queue_params *params)
{
	struct mt6630_dev *dev = mt6630_hw_dev(hw);
	int ret;

	mutex_lock(&dev->mt76.mutex);
	ret = mt6630_mcu_set_wmm(dev, queue, params);
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static void mt6630_mgd_prepare_tx(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif,
				  struct ieee80211_prep_tx_info *info)
{
	struct mt6630_dev *dev = mt6630_hw_dev(hw);
	struct ieee80211_channel *chan = dev->mt76.phy.chandef.chan;

	/* the grant lasts the whole join, association and EAPOL included,
	 * and the connected BSS holds the channel after it
	 */
	if (info->subtype != IEEE80211_STYPE_AUTH)
		return;

	mutex_lock(&dev->mt76.mutex);
	if (mt6630_mcu_ch_privilege(dev, chan))
		dev_warn(dev->mt76.dev, "No channel grant for channel %d\n",
			 chan->hw_value);
	mutex_unlock(&dev->mt76.mutex);
}

static int mt6630_hw_scan(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			  struct ieee80211_scan_request *req)
{
	struct mt6630_dev *dev = mt6630_hw_dev(hw);
	int ret;

	mutex_lock(&dev->mt76.mutex);
	ret = mt6630_mcu_hw_scan(dev, vif, req);
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static void mt6630_cancel_hw_scan(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif)
{
	struct mt6630_dev *dev = mt6630_hw_dev(hw);

	cancel_delayed_work_sync(&dev->scan_work);

	mutex_lock(&dev->mt76.mutex);
	mt6630_mcu_cancel_hw_scan(dev);
	mutex_unlock(&dev->mt76.mutex);

	/* the firmware stops the scan without reporting it done */
	mt6630_mcu_scan_done(dev, true);
}

static int mt6630_pre_channel_switch(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif,
				     struct ieee80211_channel_switch *chsw)
{
	/* The firmware stays on the old channel, so a switch would leave the
	 * link up with nothing getting through; have mac80211 disconnect
	 * instead and the connection re-made on the new channel.
	 */
	return -EOPNOTSUPP;
}

static void mt6630_flush(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			 u32 queues, bool drop)
{
	struct mt6630_dev *dev = mt6630_hw_dev(hw);

	wait_event_timeout(dev->mt76.tx_wait,
			   !mt76_has_tx_pending(&dev->mt76.phy), HZ / 2);
}

static void mt6630_tx(struct ieee80211_hw *hw,
		      struct ieee80211_tx_control *control,
		      struct sk_buff *skb)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_vif *vif = info->control.vif;
	struct mt6630_dev *dev = mt6630_hw_dev(hw);
	struct mt76_wcid *wcid = &dev->mt76.global_wcid;

	if (control->sta) {
		struct mt6630_sta *msta;

		msta = (struct mt6630_sta *)control->sta->drv_priv;
		wcid = &msta->wcid;
	} else if (vif) {
		struct mt6630_vif *mvif;

		mvif = (struct mt6630_vif *)vif->drv_priv;
		wcid = &mvif->sta.wcid;
	}

	mt76_tx(&dev->mt76.phy, control->sta, wcid, skb);
}

static int mt6630_set_rts_threshold(struct ieee80211_hw *hw, int radio_idx,
				    u32 val)
{
	/* the firmware applies the station record's RTS policy */
	return 0;
}

const struct ieee80211_ops mt6630_ops = {
	.add_chanctx = ieee80211_emulate_add_chanctx,
	.remove_chanctx = ieee80211_emulate_remove_chanctx,
	.change_chanctx = ieee80211_emulate_change_chanctx,
	.switch_vif_chanctx = ieee80211_emulate_switch_vif_chanctx,
	.tx = mt6630_tx,
	.start = mt6630_start,
	.stop = mt6630_stop,
	.add_interface = mt6630_add_interface,
	.remove_interface = mt6630_remove_interface,
	.config = mt6630_config,
	.configure_filter = mt6630_configure_filter,
	.bss_info_changed = mt6630_bss_info_changed,
	.sta_state = mt76_sta_state,
	.sta_pre_rcu_remove = mt76_sta_pre_rcu_remove,
	.set_key = mt6630_set_key,
	.conf_tx = mt6630_conf_tx,
	.mgd_prepare_tx = mt6630_mgd_prepare_tx,
	.pre_channel_switch = mt6630_pre_channel_switch,
	.hw_scan = mt6630_hw_scan,
	.cancel_hw_scan = mt6630_cancel_hw_scan,
	.flush = mt6630_flush,
	.set_rts_threshold = mt6630_set_rts_threshold,
	.wake_tx_queue = mt76_wake_tx_queue,
	.get_txpower = mt76_get_txpower,
	.get_antenna = mt76_get_antenna,
};

const struct mt76_driver_ops mt6630_drv_ops = {
	.txwi_size = MT_TXD_SIZE,
	.tx_prepare_skb = mt6630_tx_prepare_skb,
	.tx_complete_skb = mt6630_tx_complete_skb,
	.rx_skb = mt6630_queue_rx_skb,
	.sta_add = mt6630_mac_sta_add,
	.sta_remove = mt6630_mac_sta_remove,
	.set_channel = mt6630_set_channel,
};
