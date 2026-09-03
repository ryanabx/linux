/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Interface between the MT6630 WMT control-plane driver (SDIO function 2)
 * and the WLAN driver (function 1).
 */
#ifndef _MT6630_WMT_H
#define _MT6630_WMT_H

#include <linux/types.h>

/* True once the ROM patches are downloaded and FUNC_CTRL WIFI ON has been
 * acked: only then may the function-1 driver download the WLAN firmware. */
bool mt6630_wmt_ready(void);

/* The chip has no CCCR INTx, so only ONE function may claim the SDIO irq
 * (the mmc core's single-handler fast path). The WMT driver owns it and
 * chains the WLAN driver's handler: fn runs from sdio_irq_thread context
 * with the host claimed. Unregister does not synchronize -- the caller
 * must claim the host afterwards to be sure no invocation is in flight. */
void mt6630_wmt_register_wlan_isr(void (*fn)(void *ctx), void *ctx);
void mt6630_wmt_unregister_wlan_isr(void);

#endif
