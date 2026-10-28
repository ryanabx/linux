// SPDX-License-Identifier: GPL-2.0-only
/*
 * DO NOT UPSTREAM: MT8695 video decoder power-on and register probe test.
 *
 * Powers the decoder the way the vendor kernel's videocodec driver does
 * (VDEC power domain, the three decoder muxes, the vdecsoc and vdeccore
 * gates, then the extra word the vendor writes at vdecsoc +0xa8), and
 * prints the decoder's control registers. Nothing happens at probe:
 * reading /sys/kernel/debug/mt8695-vdec-probe does it, and each step and
 * register name goes to the console first, so a read that hangs shows on
 * screen. That file decodes nothing and writes no decode register.
 *
 * /sys/kernel/debug/mt8695-vdec-setup powers the decoder the same way,
 * then runs the setup the vendor library does before its first VLD access
 * (its register log of an MPEG-2 decode on Fire OS) and reads the two VLD
 * words that the library reads first. Nothing is decoded there either.
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

/*
 * The setup's registers. The vendor library maps sys at 0x16000000, VLD at
 * 0x16020000 and misc at 0x16025000. Names after mainline's MT8173 VP8
 * code (vdec_vp8_if.c), which writes the same VLD and misc offsets and the
 * same +0x18 in its gate block.
 */
#define SYS_08		0x00008
#define SYS_18		0x00018
#define VLD_SRST	(CORE_VLD + 0x108)
#define VLD_EC		(CORE_VLD + 0x0ec)
#define VLD_F4		(CORE_VLD + 0x0f4)
#define MISC_SYS_SEL	(CORE_MISC + 0x084)
#define MISC_IRQ_CFG	(CORE_MISC + 0x0a4)
#define MISC_SPEC_CON	(CORE_MISC + 0x0c8)
#define MISC_FUNC_CON	(CORE_MISC + 0x0cc)
#define MISC_DCM_CON	(CORE_MISC + 0x0ec)
#define MISC_SRST	(CORE_MISC + 0x0f4)
#define MISC_178	(CORE_MISC + 0x178)
#define GCON_DCM_CON	(CORE_GCON + 0x018)

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

/* The setup's registers outside VLD, read before and after it */
static const struct {
	const char *name;
	u32 off;
} vdec_setup_regs[] = {
	{ "sys +0x08", SYS_08 },
	{ "sys +0x18", SYS_18 },
	{ "core gcon +0x18", GCON_DCM_CON },
	{ "misc sys_sel", MISC_SYS_SEL },
	{ "misc irq_cfg", MISC_IRQ_CFG },
	{ "misc spec_con", MISC_SPEC_CON },
	{ "misc func_con", MISC_FUNC_CON },
	{ "misc dcm_con", MISC_DCM_CON },
	{ "misc srst", MISC_SRST },
	{ "misc +0x178", MISC_178 },
};

/*
 * The vendor's writes before its first VLD read, in its order (frame 0 of
 * the MPEG-2 log). The two gate writes that the clocks already did (core
 * gcon +0x0, soc gcon +0xa0) are left out.
 */
static const struct {
	const char *name;
	u32 off;
	u32 val;
	bool set;	/* OR val into the register instead of writing it */
} vdec_setup_seq[] = {
	{ "misc irq_cfg", MISC_IRQ_CFG, 0x10 },
	{ "misc irq_cfg", MISC_IRQ_CFG, 0x0 },
	{ "sys +0x08", SYS_08, 0x1 },
	{ "core gcon +0x18", GCON_DCM_CON, 0x1, true },
	{ "vld srst", VLD_SRST, 0x101 },
	{ "misc spec_con", MISC_SPEC_CON, 0xfe },
	{ "misc func_con", MISC_FUNC_CON, 0x3f6a15fd },
	{ "misc spec_con", MISC_SPEC_CON, 0xfffe },
	{ "misc func_con", MISC_FUNC_CON, 0x7f6a11df },
	{ "misc +0x178", MISC_178, 0xfffffffb },
	{ "misc srst", MISC_SRST, 0x0 },
	{ "misc sys_sel", MISC_SYS_SEL, 0x2 },	/* MPEG-2 */
	{ "vld srst", VLD_SRST, 0x0 },
	{ "sys +0x18", SYS_18, 0x1 },
	{ "misc dcm_con", MISC_DCM_CON, 0x1 },
};

/* The vendor's first VLD reads; it got 0x00030000 and 0 */
static const struct {
	const char *name;
	u32 off;
} vdec_setup_vld_regs[] = {
	{ "vld +0xf4", VLD_F4 },
	{ "vld +0xec", VLD_EC },
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

static void vdec_probe_read(struct vdec_probe *vp, struct seq_file *s,
			    const char *name, u32 off)
{
	u32 val;

	vdec_probe_step(vp, name);
	val = readl(vp->base + off);

	/* Also on the console, which ramoops keeps if a later read hangs */
	dev_emerg(vp->dev, "%-16s 0x%08x: 0x%08x\n", name, VDEC_BASE + off, val);
	if (s)
		seq_printf(s, "%-16s 0x%08x: 0x%08x\n", name, VDEC_BASE + off,
			   val);
}

static void vdec_probe_dump(struct vdec_probe *vp, struct seq_file *s)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vdec_regs); i++)
		vdec_probe_read(vp, s, vdec_regs[i].name, vdec_regs[i].off);
}

static void vdec_probe_dump_setup(struct vdec_probe *vp, struct seq_file *s)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vdec_setup_regs); i++)
		vdec_probe_read(vp, s, vdec_setup_regs[i].name,
				vdec_setup_regs[i].off);
}

static void vdec_probe_setup(struct vdec_probe *vp, struct seq_file *s)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vdec_setup_seq); i++) {
		void __iomem *reg = vp->base + vdec_setup_seq[i].off;
		u32 val = vdec_setup_seq[i].val;
		char what[48];

		snprintf(what, sizeof(what), "write %s %s 0x%x",
			 vdec_setup_seq[i].name,
			 vdec_setup_seq[i].set ? "|=" : "=", val);
		vdec_probe_step(vp, what);
		if (vdec_setup_seq[i].set)
			val |= readl(reg);
		writel(val, reg);
		seq_printf(s, "%s\n", what);
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

static int vdec_setup_show(struct seq_file *s, void *unused)
{
	struct vdec_probe *vp = s->private;
	int i, ret;

	ret = vdec_probe_on(vp);
	if (ret)
		return ret;
	seq_puts(s, "before:\n");
	vdec_probe_dump_setup(vp, s);
	vdec_probe_setup(vp, s);
	seq_puts(s, "after:\n");
	vdec_probe_dump_setup(vp, s);
	for (i = 0; i < ARRAY_SIZE(vdec_setup_vld_regs); i++)
		vdec_probe_read(vp, s, vdec_setup_vld_regs[i].name,
				vdec_setup_vld_regs[i].off);
	vdec_probe_off(vp);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(vdec_setup);

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

	dev_info(dev, "%d clocks; read /sys/kernel/debug/mt8695-vdec-{probe,setup}\n",
		 vp->num_clks);
	debugfs_create_file("mt8695-vdec-probe", 0400, NULL, vp, &vdec_probe_fops);
	debugfs_create_file("mt8695-vdec-setup", 0400, NULL, vp, &vdec_setup_fops);
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
