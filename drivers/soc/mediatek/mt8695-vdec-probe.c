// SPDX-License-Identifier: GPL-2.0-only
/*
 * DO NOT UPSTREAM: MT8695 video decoder power-on and register probe test.
 *
 * Powers the decoder the way the vendor kernel's videocodec driver does
 * (VDEC power domain, the three decoder muxes, the vdecsoc and vdeccore
 * gates, then the extra word the vendor writes at vdecsoc +0xa8), and
 * prints the decoder's control registers. Nothing is decoded and no
 * decode register is written. Nothing happens at probe: reading
 * /sys/kernel/debug/mt8695-vdec-probe does it, and each step and register
 * name goes to the console first, so a read that hangs shows on screen.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/printk.h>
#include <linux/seq_file.h>

#define VDEC_BASE	0x16000000
#define VDEC_SIZE	0x40000

/* Offsets from the vendor's videocodec_kernel_driver.c */
#define SOC_GCON	0x0f000
#define LAT_MISC	0x10000
#define CORE_VLD	0x20000	/* reading +0x0 hangs; see below */
#define CORE_MISC	0x25000
#define CORE_GCON	0x2f000

struct vdec_probe {
	struct device *dev;
	void __iomem *base;
	struct clk_bulk_data *clks;
	int num_clks;
};

static const struct {
	const char *name;
	u32 off;
} vdec_regs[] = {
	{ "sys +0x00", 0x00000 },
	{ "sys +0x04", 0x00004 },
	{ "sys +0x08", 0x00008 },
	{ "sys +0x0c", 0x0000c },
	{ "soc gcon +0xa0", SOC_GCON + 0xa0 },
	{ "soc gcon +0xa8", SOC_GCON + 0xa8 },
	{ "soc gcon +0xb0", SOC_GCON + 0xb0 },
	{ "core gcon +0x0", CORE_GCON + 0x0 },
	{ "core gcon +0x8", CORE_GCON + 0x8 },
	{ "lat misc +0xa0", LAT_MISC + 0xa0 },
	{ "lat misc +0xa4", LAT_MISC + 0xa4 },
	{ "lat misc +0xa8", LAT_MISC + 0xa8 },
	{ "lat misc +0xac", LAT_MISC + 0xac },
	{ "core misc +0xa0", CORE_MISC + 0xa0 },
	{ "core misc +0xa4", CORE_MISC + 0xa4 },
	{ "core misc +0xa8", CORE_MISC + 0xa8 },
	{ "core misc +0xac", CORE_MISC + 0xac },
	/*
	 * Not core vld +0x00 (CORE_VLD): reading it hung the bus with the
	 * domain, the muxes and all four gates on (2026-10-27).
	 */
};

/* Say what comes next on the console, and wait until it is shown */
static void vdec_probe_step(struct vdec_probe *vp, const char *what)
{
	dev_emerg(vp->dev, "next: %s\n", what);
	pr_flush(1000, true);
}

static int vdec_probe_on(struct vdec_probe *vp)
{
	int ret;

	vdec_probe_step(vp, "power domain on");
	ret = pm_runtime_resume_and_get(vp->dev);
	if (ret)
		return ret;
	vdec_probe_step(vp, "clocks on");
	ret = clk_bulk_prepare_enable(vp->num_clks, vp->clks);
	if (ret) {
		pm_runtime_put_sync(vp->dev);
		return ret;
	}
	/* The vendor driver writes this word after the soc gate (+0xa0) */
	vdec_probe_step(vp, "write soc gcon +0xa8");
	writel(0x1, vp->base + SOC_GCON + 0xa8);
	return 0;
}

static void vdec_probe_off(struct vdec_probe *vp)
{
	clk_bulk_disable_unprepare(vp->num_clks, vp->clks);
	pm_runtime_put_sync(vp->dev);
}

static void vdec_probe_dump(struct vdec_probe *vp, struct seq_file *s)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vdec_regs); i++) {
		u32 val;

		vdec_probe_step(vp, vdec_regs[i].name);
		val = readl(vp->base + vdec_regs[i].off);

		/* Also on the console, which ramoops keeps if a later read hangs */
		dev_emerg(vp->dev, "%-16s 0x%08x: 0x%08x\n", vdec_regs[i].name,
			  VDEC_BASE + vdec_regs[i].off, val);
		if (s)
			seq_printf(s, "%-16s 0x%08x: 0x%08x\n", vdec_regs[i].name,
				   VDEC_BASE + vdec_regs[i].off, val);
	}
}

static int vdec_probe_show(struct seq_file *s, void *unused)
{
	struct vdec_probe *vp = s->private;
	int ret;

	ret = vdec_probe_on(vp);
	if (ret)
		return ret;
	vdec_probe_dump(vp, s);
	vdec_probe_off(vp);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(vdec_probe);

static int mt8695_vdec_probe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vdec_probe *vp;
	int ret;

	vp = devm_kzalloc(dev, sizeof(*vp), GFP_KERNEL);
	if (!vp)
		return -ENOMEM;
	vp->dev = dev;

	/* Overlaps the gate blocks' own mappings, so not requested */
	vp->base = devm_ioremap(dev, VDEC_BASE, VDEC_SIZE);
	if (!vp->base)
		return -ENOMEM;

	vp->num_clks = devm_clk_bulk_get_all(dev, &vp->clks);
	if (vp->num_clks < 0)
		return dev_err_probe(dev, vp->num_clks, "clocks\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	dev_info(dev, "%d clocks; read /sys/kernel/debug/mt8695-vdec-probe\n",
		 vp->num_clks);
	debugfs_create_file("mt8695-vdec-probe", 0400, NULL, vp, &vdec_probe_fops);
	return 0;
}

static const struct of_device_id mt8695_vdec_probe_of_match[] = {
	{ .compatible = "mediatek,mt8695-vdec-probe-test" },
	{ /* sentinel */ }
};

static struct platform_driver mt8695_vdec_probe_driver = {
	.probe = mt8695_vdec_probe_probe,
	.driver = {
		.name = "mt8695-vdec-probe",
		.of_match_table = mt8695_vdec_probe_of_match,
	},
};
module_platform_driver(mt8695_vdec_probe_driver);

MODULE_DESCRIPTION("MT8695 video decoder probe test (not for upstream)");
MODULE_LICENSE("GPL");
