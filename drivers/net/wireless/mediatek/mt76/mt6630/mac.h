/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/* Copyright (C) 2026 Ryan Brue <ryanbrue.dev@gmail.com> */

#ifndef __MT6630_MAC_H
#define __MT6630_MAC_H

/* The descriptors are the MT7603 ones, with a seven-word TXD */

#define MT_RXD0_LENGTH			GENMASK(15, 0)
#define MT_RXD0_SW_PKT_TYPE		GENMASK(19, 16)
#define MT_RXD0_GROUP_1			BIT(25)
#define MT_RXD0_GROUP_2			BIT(26)
#define MT_RXD0_GROUP_3			BIT(27)
#define MT_RXD0_GROUP_4			BIT(28)
#define MT_RXD0_PKT_TYPE		GENMASK(31, 29)

enum mt6630_rx_pkt_type {
	PKT_TYPE_TXS		= 0,
	PKT_TYPE_TXRXV		= 1,
	PKT_TYPE_NORMAL		= 2,
	PKT_TYPE_RX_DUP_RFB	= 3,
	PKT_TYPE_RX_TMR		= 4,
	PKT_TYPE_RETRIEVE	= 5,
	PKT_TYPE_SW		= 7,
};

/* PKT_TYPE_SW carries either an event or a frame the firmware passed on */
#define MT_RXD0_SW_PKT_EVENT		0
#define MT_RXD0_SW_PKT_FRAME		1

#define MT_RXD1_NORMAL_HDR_TRANS	BIT(23)
#define MT_RXD1_NORMAL_HDR_OFFSET	BIT(22)
#define MT_RXD1_NORMAL_MAC_HDR_LEN	GENMASK(21, 16)
#define MT_RXD1_NORMAL_CH_FREQ		GENMASK(15, 8)
#define MT_RXD1_NORMAL_BCAST		BIT(3)
#define MT_RXD1_NORMAL_MCAST		BIT(2)
#define MT_RXD1_NORMAL_U2M		BIT(1)

#define MT_RXD2_NORMAL_FRAG		BIT(27)
#define MT_RXD2_NORMAL_NULL_FRAME	BIT(28)
#define MT_RXD2_NORMAL_NDATA		BIT(29)
#define MT_RXD2_NORMAL_LEN_MISMATCH	BIT(22)
#define MT_RXD2_NORMAL_TKIP_MIC_ERR	BIT(21)
#define MT_RXD2_NORMAL_ICV_ERR		BIT(20)
#define MT_RXD2_NORMAL_CLM		BIT(19)
#define MT_RXD2_NORMAL_CM		BIT(18)
#define MT_RXD2_NORMAL_FCS_ERR		BIT(17)
#define MT_RXD2_NORMAL_SEC_MODE		GENMASK(15, 12)
#define MT_RXD2_NORMAL_TID		GENMASK(11, 8)
#define MT_RXD2_NORMAL_WLAN_IDX		GENMASK(7, 0)

#define MT_RXD_SIZE			16
#define MT_RXD_GROUP_1_SIZE		16
#define MT_RXD_GROUP_2_SIZE		8
#define MT_RXD_GROUP_3_SIZE		24
#define MT_RXD_GROUP_4_SIZE		16

/* Group 4 keeps the 802.11 header fields an 802.3-translated frame lost */
#define MT_RXD4_FC			GENMASK(15, 0)
#define MT_RXD6_SEQ_CTRL		GENMASK(15, 0)
#define MT_RXD6_QOS_CTL			GENMASK(31, 16)

#define MT_RXV1_TX_RATE			GENMASK(6, 0)
#define MT_RXV1_VHT_MCS			GENMASK(3, 0)
#define MT_RXV1_HT_SHORT_GI		BIT(19)
#define MT_RXV1_FRAME_MODE		GENMASK(16, 15)
#define MT_RXV1_TX_MODE			GENMASK(14, 12)
#define MT_RXV3_RCPI0			GENMASK(15, 8)

enum mt6630_tx_header_format {
	MT_HDR_FORMAT_802_3,
	MT_HDR_FORMAT_CMD,
	MT_HDR_FORMAT_802_11,
	MT_HDR_FORMAT_802_11_EXT,
};

#define MT_TXD_SIZE			(7 * 4)

#define MT_TXD0_P_IDX			BIT(31)
#define MT_TXD0_Q_IDX			GENMASK(30, 27)
#define MT_TXD0_TX_BYTES		GENMASK(15, 0)

#define MT_TXD1_OWN_MAC			GENMASK(31, 26)
#define MT_TXD1_PROTECTED		BIT(23)
#define MT_TXD1_TID			GENMASK(22, 20)
#define MT_TXD1_NO_ACK			BIT(19)
#define MT_TXD1_HDR_PAD			GENMASK(18, 16)
#define MT_TXD1_LONG_FORMAT		BIT(15)
#define MT_TXD1_HDR_FORMAT		GENMASK(14, 13)
#define MT_TXD1_HDR_INFO		GENMASK(12, 8)
#define MT_TXD1_WLAN_IDX		GENMASK(7, 0)

#define MT_TXD2_FIX_RATE		BIT(31)
#define MT_TXD2_MULTICAST		BIT(10)
#define MT_TXD2_FRAME_TYPE		GENMASK(5, 4)
#define MT_TXD2_SUB_TYPE		GENMASK(3, 0)

#define MT_TXD3_REM_TX_COUNT		GENMASK(15, 11)

#define MT_TXD5_TX_STATUS_MCU		BIT(9)
#define MT_TXD5_PID			GENMASK(7, 0)

#define MT_TXD6_TX_RATE			GENMASK(29, 18)
#define MT_TXD6_FIXED_RATE		BIT(0)

#define MT_TX_RATE_MODE			GENMASK(8, 6)
#define MT_TX_RATE_IDX			GENMASK(5, 0)

/* Port 0 is the LMAC with its AC queues, port 1 the MCU */
#define MT_TX_PORT_LMAC			0
#define MT_TX_PORT_MCU			1
#define MT_TX_MCU_Q_MGMT		1
#define MT_TX_LMAC_Q_NON_QOS		4

#endif
