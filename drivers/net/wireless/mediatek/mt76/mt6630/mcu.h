/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/* Copyright (C) 2026 Ryan Brue <ryanbrue.dev@gmail.com> */

#ifndef __MT6630_MCU_H
#define __MT6630_MCU_H

/* Commands share the TXD's first two words: the byte count, the port and
 * queue, then a command ID where the TXD has its WLAN index.
 */
struct mt6630_mcu_txd {
	__le16 len;
	__le16 pq_id;
	u8 cid;
	u8 pkt_type;
	u8 set_query;
	u8 seq;
} __packed;

struct mt6630_mcu_rxd {
	__le16 len;
	__le16 pkt_type;
	u8 eid;
	u8 seq;
	u8 rsv[2];
} __packed;

#define MCU_PQ_ID_CMD			0x8000
#define MCU_PQ_ID_FW			0xc000
#define MCU_PKT_ID			0xa0

#define MCU_Q_QUERY			0
#define MCU_Q_SET			1

/* Commands understood before the firmware runs */
#define MCU_INIT_CMD_TARGET_ADDRESS_LEN_REQ	0x01
#define MCU_INIT_CMD_FW_START_REQ	0x02
#define MCU_INIT_EVENT_CMD_RESULT	0x01

#define DL_MODE_ENCRYPT			BIT(0)
#define DL_MODE_RESET_SEC_IV		BIT(3)
#define DL_MODE_NEED_RSP		BIT(31)

#define FW_START_OVERRIDE		BIT(0)

#define MT6630_FW_START_ADDR		0x00091400
#define MT6630_FW_CHUNK_SIZE		2048

enum {
	MCU_CMD_SCAN_REQ = 0x03,
	MCU_CMD_NIC_POWER_CTRL = 0x04,
	MCU_CMD_POWER_SAVE_MODE = 0x05,
	MCU_CMD_ADD_REMOVE_KEY = 0x07,
	MCU_CMD_BSS_ACTIVATE_CTRL = 0x11,
	MCU_CMD_SET_BSS_INFO = 0x12,
	MCU_CMD_UPDATE_STA_RECORD = 0x13,
	MCU_CMD_REMOVE_STA_RECORD = 0x14,
	MCU_CMD_PM_BSS_CONNECTED = 0x16,
	MCU_CMD_PM_BSS_ABORT = 0x17,
	MCU_CMD_SET_BSS_RLM_PARAM = 0x19,
	MCU_CMD_SCAN_CANCEL = 0x1b,
	MCU_CMD_CH_PRIVILEGE = 0x1c,
	MCU_CMD_UPDATE_WMM_PARMS = 0x1d,
	MCU_CMD_GET_NIC_CAPABILITY = 0x80,
};

enum {
	MCU_EVENT_RX_ADDBA = 0x0a,
	MCU_EVENT_RX_DELBA = 0x0b,
	MCU_EVENT_SCAN_DONE = 0x0d,
	MCU_EVENT_TX_DONE = 0x0f,
	MCU_EVENT_CH_PRIVILEGE = 0x10,
	MCU_EVENT_BSS_BEACON_TIMEOUT = 0x13,
	MCU_EVENT_STA_UPDATE_FREE_QUOTA = 0x16,
	MCU_EVENT_DEBUG_MSG = 0x27,
};

struct mt6630_nic_capability {
	__le16 product_id;
	__le16 fw_version;
	__le16 driver_version;
	u8 hw_5g_disabled;
	u8 eeprom_used;
	u8 mac_addr[ETH_ALEN];
	u8 mac_addr_endian;
	u8 rsv;
	u8 rf_version;
	u8 phy_version;
	u8 rf_cal_fail;
	u8 bb_cal_fail;
	u8 date_code[16];
	__le32 feature_flag[2];
	__le32 compile_flag[2];
	u8 rsv2[64];
} __packed;

enum {
	MT6630_PS_CONTINUOUS_ACTIVE,
	MT6630_PS_MAX_PS,
	MT6630_PS_FAST_PS,
};

enum {
	MT6630_NETWORK_INFRA,
};

/* -60 dBm: a starting point for the firmware's rate control */
#define MT6630_RCPI_INIT		100

enum {
	MT6630_PHY_TYPE_ERP = 1,
	MT6630_PHY_TYPE_OFDM = 3,
};

#define MT6630_PHY_SET_DSSS		BIT(0)
#define MT6630_PHY_SET_ERP		BIT(1)
#define MT6630_PHY_SET_OFDM		BIT(3)
#define MT6630_PHY_SET_HT		BIT(4)
#define MT6630_PHY_SET_VHT		BIT(5)

/* the VHT operation element's channel width */
#define MT6630_VHT_WIDTH_20_40		0
#define MT6630_VHT_WIDTH_80		1

enum {
	MT6630_SCO_SCN = 0,
	MT6630_SCO_SCA = 1,
	MT6630_SCO_SCB = 3,
};

/* per-peer features are "auto", "disabled" or "enabled", counted from 0 */
#define MT6630_FEATURE_ENABLED		2
#define MT6630_HT_RIFS_DISALLOWED	1

#define MT6630_BAND_2G			1
#define MT6630_BAND_5G			2

#define MT6630_STA_TYPE_LEGACY_AP	0x41
#define MT6630_STA_REC_NOT_FOUND	0xfe
#define MT6630_RTS_POLICY_LEGACY	3

enum {
	MT6630_STA_STATE_1,
	MT6630_STA_STATE_2,
	MT6630_STA_STATE_3,
};

struct mt6630_bss_activate {
	u8 bss_idx;
	u8 active;
	u8 network_type;
	u8 omac_idx;
	u8 bss_addr[ETH_ALEN];
	u8 bmc_wlan_idx;
	u8 rsv;
} __packed;

struct mt6630_ps_profile {
	u8 bss_idx;
	u8 profile;
	u8 rsv[2];
} __packed;

struct mt6630_bss_rlm {
	u8 bss_idx;
	u8 band;
	u8 primary_channel;
	u8 sco;
	u8 erp_protect;
	u8 ht_protect;
	u8 gf_mode;
	u8 tx_rifs;
	__le16 ht_op_info3;
	__le16 ht_op_info2;
	u8 ht_op_info1;
	u8 short_preamble;
	u8 short_slot;
	u8 vht_width;
	u8 vht_freq_seg1;
	u8 vht_freq_seg2;
	__le16 vht_basic_mcs;
} __packed;

struct mt6630_bss_info {
	u8 bss_idx;
	u8 conn_state;
	u8 op_mode;
	u8 ssid_len;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	u8 bssid[ETH_ALEN];
	u8 is_qbss;
	u8 rsv;
	__le16 op_rate_set;
	__le16 basic_rate_set;
	u8 ap_sta_idx;
	u8 rsv2;
	__le16 hw_default_rate;
	u8 non_ht_phy_type;
	u8 auth_mode;
	u8 enc_status;
	u8 phy_type_set;
	u8 wapi_mode;
	u8 is_ap;
	u8 bmc_wlan_idx;
	u8 hidden_ssid;
	u8 disconnect_th;
	u8 rsv3[3];
	__le32 private_data;
	struct mt6630_bss_rlm rlm;
} __packed;

struct mt6630_sta_rec {
	u8 sta_idx;
	u8 sta_type;
	u8 addr[ETH_ALEN];
	__le16 aid;
	__le16 listen_interval;
	u8 bss_idx;
	u8 phy_type_set;
	__le16 non_ht_rate_set;
	__le16 basic_rate_set;
	u8 is_qos;
	u8 is_uapsd;
	u8 state;
	u8 mcs_set;
	u8 mcs32;
	u8 rsv;
	u8 rx_mcs_bitmask[10];
	__le16 rx_highest_rate;
	__le32 tx_rate_info;
	__le16 ht_cap_info;
	__le16 ht_ext_cap;
	__le32 tx_bf_cap;
	u8 ampdu_param;
	u8 asel_cap;
	u8 rcpi;
	u8 need_resp;
	u8 uapsd_ac;
	u8 uapsd_sp;
	u8 wlan_idx;
	u8 bmc_wlan_idx;
	__le32 vht_cap_info;
	__le16 vht_rx_mcs_map;
	__le16 vht_rx_highest_rate;
	__le16 vht_tx_mcs_map;
	__le16 vht_tx_highest_rate;
	u8 rts_policy;
	u8 rsv2;
	u8 traffic_type;
	u8 tx_gf;
	u8 tx_sgi;
	u8 tx_stbc;
	__le16 hw_default_rate;
	u8 tx_ampdu;
	u8 rx_ampdu;
	u8 rsv3[2];
	__le32 fixed_phy_rate;
	__le16 max_link_speed;
	__le16 min_link_speed;
	__le32 flags;
	u8 tx_ba_size;
	u8 rx_ba_size;
	u8 rsv4[30];
} __packed;

struct mt6630_sta_remove {
	u8 action;
	u8 sta_idx;
	u8 bss_idx;
	u8 rsv;
} __packed;

struct mt6630_bss_connected {
	u8 bss_idx;
	u8 dtim_period;
	__le16 aid;
	__le16 beacon_interval;
	__le16 atim_window;
	u8 is_uapsd;
	u8 uapsd_delivery_ac;
	u8 uapsd_trigger_ac;
	u8 rsv;
} __packed;

struct mt6630_bss_abort {
	u8 bss_idx;
	u8 rsv[3];
} __packed;

enum {
	MT6630_CIPHER_WEP40 = 1,
	MT6630_CIPHER_TKIP = 2,
	MT6630_CIPHER_CCMP = 4,
	MT6630_CIPHER_WEP104 = 5,
};

struct mt6630_key {
	u8 add;
	u8 tx_key;
	u8 key_type;
	u8 is_authenticator;
	u8 peer_addr[ETH_ALEN];
	u8 bss_idx;
	u8 algorithm;
	u8 key_idx;
	u8 key_len;
	u8 wlan_idx;
	u8 rsv;
	u8 key[32];
	u8 rsc[16];
} __packed;

/* AC order is BE, BK, VI, VO; the values are mac80211's, not exponents */
struct mt6630_wmm_params {
	struct {
		__le16 cw_min;
		__le16 cw_max;
		__le16 txop;
		__le16 aifsn;
		u8 guard_time;
		u8 acm;
	} __packed ac[IEEE80211_NUM_ACS];
	u8 bss_idx;
	u8 is_qbss;
	u8 rsv[2];
} __packed;

#define MT6630_CH_PRIV_REQ		0
#define MT6630_CH_PRIV_TYPE_JOIN	0
#define MT6630_CH_PRIV_JOIN_MS		4000

struct mt6630_ch_privilege {
	u8 bss_idx;
	u8 token;
	u8 action;
	u8 primary_channel;
	u8 sco;
	u8 band;
	u8 width;
	u8 center_seg1;
	u8 center_seg2;
	u8 req_type;
	u8 rsv[2];
	__le32 max_interval;
} __packed;

struct mt6630_ch_privilege_event {
	u8 bss_idx;
	u8 token;
	u8 status;
	u8 primary_channel;
	u8 sco;
	u8 band;
	u8 width;
	u8 center_seg1;
	u8 center_seg2;
	u8 req_type;
	u8 rsv[2];
	__le32 grant_interval;
} __packed;

#define MT6630_SCAN_PROBE_REQS		2	/* per channel */
#define MT6630_SCAN_SSIDS		4
#define MT6630_SCAN_CHANNELS		32
#define MT6630_SCAN_IE_LEN		600

#define MT6630_SCAN_TYPE_PASSIVE	0
#define MT6630_SCAN_TYPE_ACTIVE		1
#define MT6630_SCAN_SSID_WILDCARD	BIT(0)
#define MT6630_SCAN_SSID_SPECIFIED	BIT(2)
#define MT6630_SCAN_CHANNEL_SPECIFIED	4

struct mt6630_scan_req {
	u8 seq;
	u8 bss_idx;
	u8 scan_type;
	u8 ssid_type;
	u8 ssids_num;
	u8 probe_req_num;
	u8 rsv[2];
	struct {
		__le32 len;
		u8 ssid[IEEE80211_MAX_SSID_LEN];
	} __packed ssids[MT6630_SCAN_SSIDS];
	__le16 probe_delay;
	__le16 dwell_time;
	__le16 timeout;
	u8 channel_type;
	u8 channels_num;
	struct {
		u8 band;
		u8 channel;
	} __packed channels[MT6630_SCAN_CHANNELS];
	__le16 ies_len;
	u8 ies[MT6630_SCAN_IE_LEN];
} __packed;

struct mt6630_scan_cancel {
	u8 seq;
	u8 is_ext_channel;
	u8 rsv[2];
} __packed;

struct mt6630_scan_done_event {
	u8 seq;
	u8 sparse_channel_valid;
	u8 band;
	u8 channel;
} __packed;

struct mt6630_tx_done_event {
	u8 pid;
	u8 status;
	__le16 seq;
	u8 wlan_idx;
	u8 tx_count;
	__le16 tx_rate;
	u8 flag;
	u8 rsv[7];
} __packed;

struct mt6630_nic_power_ctrl {
	u8 power_mode;
	u8 rsv[3];
} __packed;

#define MT6630_POWER_MODE_OFF		1

struct mt6630_debug_msg_event {
	__le16 id;
	u8 type;
	u8 flags;
	__le32 value;
	__le16 msg_size;
	u8 rsv[2];
	u8 msg[];
} __packed;

struct mt6630_rx_addba_event {
	u8 sta_idx;
	u8 token;
	__le16 ba_param;
	__le16 timeout;
	__le16 start_seq;
} __packed;

struct mt6630_rx_delba_event {
	u8 sta_idx;
	u8 tid;
	u8 rsv[2];
} __packed;

struct mt6630_beacon_timeout_event {
	u8 bss_idx;
	u8 reason;
	u8 rsv[2];
} __packed;

#endif
