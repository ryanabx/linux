// SPDX-License-Identifier: GPL-2.0
/*
 * Bring-up aid, not for upstream: an earlycon that logs into one page of
 * RAM, for boards with no serial console that keep RAM across a warm reset.
 *
 *   earlycon=ramlog,0x4447e000 keep_bootcon
 *
 * The page holds a small header and a ring of the most recent output, so
 * the last thing printed before a hang can be read back after the reset.
 */

#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/serial_core.h>
#include <linux/stddef.h>

#define RAMLOG_MAGIC	0x474c4d52	/* "RMLG" */

struct ramlog {
	u32 magic;
	u32 pos;	/* next write offset into buf */
	u32 wraps;	/* times buf has wrapped */
	u32 reserved;
	char buf[];
};

#define RAMLOG_BUF_SIZE	(PAGE_SIZE - offsetof(struct ramlog, buf))

static void ramlog_write(struct console *con, const char *s, unsigned int n)
{
	struct earlycon_device *dev = con->data;
	struct ramlog __iomem *log = (struct ramlog __iomem *)dev->port.membase;
	u32 pos = readl(&log->pos);

	if (pos >= RAMLOG_BUF_SIZE)
		pos = 0;

	while (n--) {
		writeb(*s++, &log->buf[pos]);
		if (++pos == RAMLOG_BUF_SIZE) {
			pos = 0;
			writel(readl(&log->wraps) + 1, &log->wraps);
		}
	}
	writel(pos, &log->pos);
}

static int __init ramlog_setup(struct earlycon_device *dev, const char *opt)
{
	struct ramlog __iomem *log = (struct ramlog __iomem *)dev->port.membase;

	if (!log || (dev->port.mapbase & ~PAGE_MASK))
		return -ENODEV;

	writel(0, &log->pos);
	writel(0, &log->wraps);
	writel(RAMLOG_MAGIC, &log->magic);
	dev->con->write = ramlog_write;
	return 0;
}
EARLYCON_DECLARE(ramlog, ramlog_setup);
