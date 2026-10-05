// SPDX-License-Identifier: BSD-3-Clause-Clear
/* Copyright (C) 2026 Ryan Brue <ryanbrue.dev@gmail.com> */

#include "mt6630.h"

static int mt6630_init_wiphy(struct mt76_dev *mdev)
{
	struct ieee80211_hw *hw = mdev->hw;
	struct wiphy *wiphy = hw->wiphy;
	int i;

	/* The firmware negotiates block-ack itself; mac80211 is told about
	 * the RX sessions it sets up, and never starts TX ones.
	 */
	__clear_bit(IEEE80211_HW_AMPDU_AGGREGATION, hw->flags);
	__clear_bit(IEEE80211_HW_SUPPORTS_AMSDU_IN_AMPDU, hw->flags);

	/* what the vendor driver offers: no LDPC, beamforming or TX STBC */
	mdev->phy.sband_5g.sband.vht_cap.cap &= ~(IEEE80211_VHT_CAP_RXLDPC |
						   IEEE80211_VHT_CAP_TXSTBC);

	/* 40 MHz is a 5 GHz feature on this chip, as in the vendor driver */
	mdev->phy.sband_2g.sband.ht_cap.cap &= ~(IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
						  IEEE80211_HT_CAP_SGI_40);

	/* station mode only: the firmware keeps one infrastructure BSS */
	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	wiphy->flags &= ~(WIPHY_FLAG_SUPPORTS_TDLS | WIPHY_FLAG_AP_UAPSD |
			  WIPHY_FLAG_HAS_CHANNEL_SWITCH | WIPHY_FLAG_IBSS_RSN);
	__clear_bit(IEEE80211_HW_AP_LINK_PS, hw->flags);

	/* the firmware's channel plan stops at 165 */
	for (i = 0; i < mdev->phy.sband_5g.sband.n_channels; i++) {
		struct ieee80211_channel *chan;

		chan = &mdev->phy.sband_5g.sband.channels[i];
		if (chan->hw_value > 165)
			chan->flags |= IEEE80211_CHAN_DISABLED;
	}

	wiphy->max_scan_ssids = MT6630_SCAN_SSIDS;
	wiphy->max_scan_ie_len = MT6630_SCAN_IE_LEN;

	ieee80211_hw_set(hw, HAS_RATE_CONTROL);
	ieee80211_hw_set(hw, CONNECTION_MONITOR);

	return 0;
}

/* Nothing to poll: the firmware keeps the statistics, and mt76 core
 * only needs the work to exist, as mt76_set_channel() cancels it.
 */
static void mt6630_mac_work(struct work_struct *work)
{
}

int mt6630_register_device(struct mt6630_dev *dev)
{
	struct ieee80211_hw *hw = dev->mt76.hw;
	int ret;

	init_completion(&dev->ch_grant);
	INIT_DELAYED_WORK(&dev->mt76.phy.mac_work, mt6630_mac_work);
	INIT_DELAYED_WORK(&dev->scan_work, mt6630_scan_work);
	INIT_WORK(&dev->ba_work, mt6630_mac_ba_work);
	spin_lock_init(&dev->ba_lock);
	INIT_LIST_HEAD(&dev->ba_list);

	ret = mt6630_mcu_get_nic_capability(dev);
	if (ret)
		return ret;

	/* the firmware's address is usually blank */
	ret = mt76_eeprom_override(&dev->mt76.phy);
	if (ret)
		return ret;

	dev->mt76.phy.antenna_mask = BIT(0);
	dev->mt76.phy.chainmask = BIT(0);

	/* entry 0 is the BSS's broadcast/multicast one */
	mt76_wcid_mask_set(dev->mt76.wcid_mask, MT6630_WTBL_BMC);

	hw->queues = 4;
	hw->max_tx_fragments = 1;
	hw->max_rx_aggregation_subframes = IEEE80211_MAX_AMPDU_BUF_HT;
	hw->sta_data_size = sizeof(struct mt6630_sta);
	hw->vif_data_size = sizeof(struct mt6630_vif);

	dev->mt76.init_wiphy = mt6630_init_wiphy;

	return mt76_register_device(&dev->mt76, true, mt76_rates,
				    ARRAY_SIZE(mt76_rates));
}

void mt6630_unregister_device(struct mt6630_dev *dev)
{
	mt76_unregister_device(&dev->mt76);
	cancel_delayed_work_sync(&dev->scan_work);
	cancel_work_sync(&dev->ba_work);
	mt6630_mac_ba_flush(dev);
}
