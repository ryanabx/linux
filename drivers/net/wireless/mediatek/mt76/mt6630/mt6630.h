/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/* Copyright (C) 2026 Ryan Brue <ryanbrue.dev@gmail.com> */

#ifndef __MT6630_H
#define __MT6630_H

#include "../mt76.h"
#include "regs.h"
#include "mcu.h"

#define MT6630_FIRMWARE		"mediatek/mt6630/WIFI_RAM_CODE_MT6630"

/* The firmware keeps one station-mode BSS; the indices below are the ones
 * the driver hands it, and WTBL entry 0 serves broadcast/multicast.
 */
#define MT6630_BSS_IDX		0
#define MT6630_OMAC_IDX		0
#define MT6630_WTBL_BMC		0
#define MT6630_WTBL_SIZE	20

#define MT6630_TX_RING_PAGES	64
#define MT6630_TX_MCU_PAGES	16

struct mt6630_sta {
	struct mt76_wcid wcid; /* must be first */

	u8 sta_idx;
};

struct mt6630_vif {
	struct mt76_vif_link mt76; /* must be first */

	struct mt6630_sta sta;
};

/* The enhanced interrupt status the chip returns in one read at WHISR */
struct mt6630_sdio_intr {
	u32 isr;
	u16 tx_cnt[16];
	u16 rx_num[2];
	u16 rx_len[2][16];
	u32 rec_mb[2];
} __packed;

struct mt6630_dev {
	struct mt76_dev mt76; /* must be first */

	struct mt6630_vif *vif;

	/* the connac1 counter layout mt76s_refill_sched_quota() reads */
	u32 tx_quota[8];
	int tx_balance;

	u8 ch_token;
	struct completion ch_grant;

	u8 scan_seq;
	unsigned long scan_pending;
	struct delayed_work scan_work;

	struct mt6630_wmm_params wmm;

	/* block-ack sessions the firmware set up, applied from process
	 * context
	 */
	struct work_struct ba_work;
	spinlock_t ba_lock; /* protects ba_list */
	struct list_head ba_list;
};

struct mt6630_ba_event {
	struct list_head list;
	u8 wlan_idx;
	u8 tid;
	u16 ssn;
	u16 size;
	bool start;
};

enum {
	MT6630_HIF_TX_FFA = 14,
	MT6630_HIF_TX_CPU = 15,
};

extern const struct ieee80211_ops mt6630_ops;
extern const struct mt76_mcu_ops mt6630_mcu_ops;
extern const struct mt76_driver_ops mt6630_drv_ops;

static inline struct mt6630_dev *mt6630_hw_dev(struct ieee80211_hw *hw)
{
	struct mt76_phy *phy = hw->priv;

	return container_of(phy->dev, struct mt6630_dev, mt76);
}

int mt6630_register_device(struct mt6630_dev *dev);
void mt6630_unregister_device(struct mt6630_dev *dev);

int mt6630_mcu_init(struct mt6630_dev *dev);
int mt6630_mcu_stop(struct mt6630_dev *dev);
int mt6630_mcu_get_nic_capability(struct mt6630_dev *dev);
int mt6630_mcu_bss_activate(struct mt6630_dev *dev, struct ieee80211_vif *vif,
			    bool enable);
int mt6630_mcu_set_ps_profile(struct mt6630_dev *dev, u8 profile);
int mt6630_mcu_ch_privilege(struct mt6630_dev *dev,
			    struct ieee80211_channel *chan);
int mt6630_mcu_set_rlm(struct mt6630_dev *dev, struct ieee80211_vif *vif);
int mt6630_mcu_set_bss_info(struct mt6630_dev *dev, struct ieee80211_vif *vif,
			    bool connected);
int mt6630_mcu_sta_update(struct mt6630_dev *dev, struct ieee80211_vif *vif,
			  struct ieee80211_sta *sta, bool assoc);
int mt6630_mcu_sta_remove(struct mt6630_dev *dev, struct mt6630_sta *msta);
int mt6630_mcu_bss_connected(struct mt6630_dev *dev,
			     struct ieee80211_vif *vif);
int mt6630_mcu_bss_abort(struct mt6630_dev *dev);
int mt6630_mcu_add_key(struct mt6630_dev *dev, struct ieee80211_vif *vif,
		       struct mt6630_sta *msta, struct ieee80211_key_conf *key,
		       enum set_key_cmd cmd);
int mt6630_mcu_set_wmm(struct mt6630_dev *dev, u8 queue,
		       const struct ieee80211_tx_queue_params *params);
int mt6630_mcu_hw_scan(struct mt6630_dev *dev, struct ieee80211_vif *vif,
		       struct ieee80211_scan_request *scan_req);
int mt6630_mcu_cancel_hw_scan(struct mt6630_dev *dev);
void mt6630_mcu_scan_done(struct mt6630_dev *dev, bool aborted);
void mt6630_mcu_rx_event(struct mt6630_dev *dev, struct sk_buff *skb);

int mt6630_tx_prepare_skb(struct mt76_dev *mdev, void *txwi_ptr,
			  enum mt76_txq_id qid, struct mt76_wcid *wcid,
			  struct ieee80211_sta *sta,
			  struct mt76_tx_info *tx_info);
void mt6630_tx_complete_skb(struct mt76_dev *mdev,
			    struct mt76_queue_entry *e);
void mt6630_queue_rx_skb(struct mt76_dev *mdev, enum mt76_rxq_id q,
			 struct sk_buff *skb, u32 *info);
void mt6630_mac_tx_done(struct mt6630_dev *dev, u8 wlan_idx, u8 pid,
			bool acked);
int mt6630_mac_sta_add(struct mt76_dev *mdev, struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta);
void mt6630_mac_sta_remove(struct mt76_dev *mdev, struct ieee80211_vif *vif,
			   struct ieee80211_sta *sta);
void mt6630_scan_work(struct work_struct *work);
void mt6630_mac_ba_work(struct work_struct *work);
u16 mt6630_mac_rate_code(u16 hw_value);
void mt6630_mac_ba_flush(struct mt6630_dev *dev);

#endif
