/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/* Copyright (C) 2026 Ryan Brue <ryanbrue.dev@gmail.com> */

#ifndef __MT6630_REGS_H
#define __MT6630_REGS_H

#include "../sdio.h"

/* Function-1 bits that mt76/sdio.h does not carry */
#define WCIR_CHIP_ID			GENMASK(15, 0)
#define WCIR_REVISION			GENMASK(19, 16)
#define MT6630_CHIP_ID			0x6630

#define WCIR_WLAN_READY			BIT(21)

#define WHISR_FW_ASSERT			BIT(31)

#endif
