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
 *
 * /sys/kernel/debug/mt8695-vdec-replay decodes one picture: the first
 * MPEG-2 picture of the stream written to /sys/kernel/debug/mt8695-vdec-bs,
 * with the vendor library's register list for that picture (frame 0 of its
 * log of the 1080p test clip) and our own buffers, through the M4U. The
 * picture lands in /sys/kernel/debug/mt8695-vdec-fb, in block mode. Every
 * read is compared with the value the vendor library got.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/printk.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>

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

/* The replay's blocks, named as the vendor library names them */
#define SYS		0x00000
#define VLD		CORE_VLD
#define VLD_TOP		(CORE_VLD + 0x800)
#define MC		0x21000
#define MISC		CORE_MISC

/* 1920x1088 in block mode: luma, then chroma at half its size */
#define VDEC_FB_Y_SIZE	(1920 * 1088)
#define VDEC_FB_SIZE	(VDEC_FB_Y_SIZE * 3 / 2)
#define VDEC_BS_SIZE	SZ_256K

struct vdec_probe {
	struct device *dev;
	void __iomem *base;
	struct clk_bulk_data *clks;
	int num_clks;

	void *bs;
	dma_addr_t bs_iova;
	size_t bs_len;
	void *fb;
	dma_addr_t fb_iova;
	struct debugfs_blob_wrapper fb_blob;
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

/*
 * Write the vendor's word at vdecsoc +0xa8 before larb1 resumes (Y), or
 * only after it (N), to see whether larb1 needs it: writable at
 * /sys/module/mt8695_vdec_probe/parameters/soc_word_first.
 */
static bool soc_word_first = true;
module_param(soc_word_first, bool, 0644);

/*
 * Enable the clocks before the runtime PM get (Y, the vendor's order), or
 * after it (N): with the soc gate among the power domain's clocks, larb1
 * should resume either way.
 */
static bool clocks_first = true;
module_param(clocks_first, bool, 0644);

/*
 * With clocks_first=N, which of the three muxes (the node's first three
 * clocks: bit 0 vdec_sel, 1 vdec_slow_sel, 2 vdec_lae_sel) go on before the
 * runtime PM get. The gates never do: their blocks are in the domain.
 */
static unsigned int pre_muxes;
module_param(pre_muxes, uint, 0644);

/* Say what comes next on the console, and wait until it is shown */
static void vdec_probe_step(struct vdec_probe *vp, const char *what)
{
	dev_emerg(vp->dev, "next: %s\n", what);
	pr_flush(1000, true);
}

static void vdec_probe_read(struct vdec_probe *vp, struct seq_file *s,
			    const char *name, u32 off);

/*
 * In the vendor driver's order: the muxes and the soc gates (with the extra
 * word at vdecsoc +0xa8), and only then the larbs. Through the M4U, the
 * runtime PM get also resumes larb1, whose port setup hangs the bus while
 * the soc gate is still off.
 */
static int vdec_probe_on(struct vdec_probe *vp)
{
	int ret;

	if (!clocks_first) {
		int i;

		for (i = 0; i < 3 && i < vp->num_clks; i++) {
			if (!(pre_muxes & BIT(i)))
				continue;
			vdec_probe_step(vp, vp->clks[i].id ?: "a mux");
			clk_prepare_enable(vp->clks[i].clk);
		}
		vdec_probe_step(vp, "power domain and larb1 on, before the clocks");
		ret = pm_runtime_resume_and_get(vp->dev);
		if (ret)
			return ret;
	}
	vdec_probe_step(vp, "clocks on");
	ret = clk_bulk_prepare_enable(vp->num_clks, vp->clks);
	if (ret)
		goto err_put;
	vdec_probe_read(vp, NULL, "soc gcon +0xa8", SOC_GCON + 0xa8);
	/* The vendor driver writes this word after the soc gate (+0xa0) */
	if (soc_word_first) {
		vdec_probe_step(vp, "write soc gcon +0xa8");
		writel(0x1, vp->base + SOC_GCON + 0xa8);
	}
	if (clocks_first) {
		vdec_probe_step(vp, "power domain and larb1 on");
		ret = pm_runtime_resume_and_get(vp->dev);
		if (ret) {
			clk_bulk_disable_unprepare(vp->num_clks, vp->clks);
			return ret;
		}
	}
	if (!soc_word_first) {
		vdec_probe_step(vp, "write soc gcon +0xa8 after larb1");
		writel(0x1, vp->base + SOC_GCON + 0xa8);
	}
	return 0;

err_put:
	if (!clocks_first)
		pm_runtime_put_sync(vp->dev);
	return ret;
}

static void vdec_probe_off(struct vdec_probe *vp)
{
	int i;

	pm_runtime_put_sync(vp->dev);
	clk_bulk_disable_unprepare(vp->num_clks, vp->clks);
	if (!clocks_first)
		for (i = 0; i < 3 && i < vp->num_clks; i++)
			if (pre_muxes & BIT(i))
				clk_disable_unprepare(vp->clks[i].clk);
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

/*
 * Read the two gate blocks' words with nothing powered: whether the clock
 * controllers can do without runtime PM, as MT8192's do. Check
 * pm_genpd_summary for "vdec off" first; this file powers nothing.
 */
static int vdec_offread_show(struct seq_file *s, void *unused)
{
	struct vdec_probe *vp = s->private;

	vdec_probe_read(vp, s, "soc gcon +0xa0", SOC_GCON + 0xa0);
	vdec_probe_read(vp, s, "core gcon +0x0", CORE_GCON + 0x0);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(vdec_offread);

struct vdec_replay {
	struct vdec_probe *vp;
	struct seq_file *s;
	int mismatches;
};

static const char *vdec_block_name(u32 block)
{
	switch (block) {
	case SYS:
		return "SYS";
	case VLD:
		return "VLD";
	case VLD_TOP:
		return "VLD_TOP";
	case MC:
		return "MC";
	case MISC:
		return "MISC";
	}
	return "?";
}

/* Write word @word of @block, logged the way the vendor's log has it */
static void rp_w(struct vdec_replay *r, u32 block, u32 word, u32 val)
{
	char what[48];

	snprintf(what, sizeof(what), "%s(%u) = 0x%08x", vdec_block_name(block),
		 word, val);
	vdec_probe_step(r->vp, what);
	writel(val, r->vp->base + block + word * 4);
	seq_printf(r->s, "%s\n", what);
}

/* Read word @word of @block and compare it with what the vendor read */
static u32 rp_r(struct vdec_replay *r, u32 block, u32 word, u32 expect)
{
	char what[48];
	u32 val;

	snprintf(what, sizeof(what), "%s(%u)", vdec_block_name(block), word);
	vdec_probe_step(r->vp, what);
	val = readl(r->vp->base + block + word * 4);
	if (val != expect)
		r->mismatches++;
	dev_emerg(r->vp->dev, "%s: 0x%08x (vendor 0x%08x)\n", what, val, expect);
	seq_printf(r->s, "%s: 0x%08x (vendor 0x%08x)%s\n", what, val, expect,
		   val == expect ? "" : " MISMATCH");
	return val;
}

static int rp_poll(struct vdec_replay *r, u32 off, u32 mask, u32 timeout_us,
		   const char *what)
{
	u32 val;
	int ret;

	vdec_probe_step(r->vp, what);
	ret = readl_poll_timeout(r->vp->base + off, val, val & mask, 10,
				 timeout_us);
	dev_emerg(r->vp->dev, "%s: %s, 0x%08x\n", what,
		  ret ? "timed out" : "done", val);
	seq_printf(r->s, "%s: %s, 0x%08x\n", what, ret ? "TIMED OUT" : "done",
		   val);
	return ret;
}

/* What the vendor's barrel shifter reads returned: the sequence header on */
static const u32 vdec_replay_bits[] = {
	0xb3780438, 0x3835ffff, 0xffe45800, 0x000001b5, 0xb5144a00,
	0x00010000, 0x00000001, 0x01b80008, 0x08004000, 0x00000100,
	0x00000fff, 0xfff80000, 0x0001b58f, 0x8ffff341, 0x41800000,
};

/*
 * Frame 0 of the vendor's log after the setup, with our buffers: the
 * bitstream window in VLD words 44-46 and 68, the frame buffer in MC words
 * 0-3 (luma >> 9, chroma >> 8; both reference slots point at the output,
 * as for the vendor's I picture) and 532/533 (end and start >> 4).
 */
static void vdec_replay_frame(struct vdec_replay *r)
{
	struct vdec_probe *vp = r->vp;
	u32 bs = vp->bs_iova, bs_end = vp->bs_iova + vp->bs_len;
	u32 y = vp->fb_iova, c = vp->fb_iova + VDEC_FB_Y_SIZE;
	u32 val;
	int i;

	rp_r(r, VLD, 61, 0x00030000);
	rp_w(r, VLD, 45, bs >> 6);
	rp_w(r, VLD, 46, bs_end >> 6);
	rp_w(r, VLD, 44, bs);
	rp_w(r, VLD, 68, bs_end);
	rp_w(r, VLD, 66, 0x100);
	rp_w(r, VLD, 66, 0x0);
	rp_w(r, VLD, 39, 0x00030fff);
	val = rp_r(r, VLD, 59, 0x0);
	rp_w(r, VLD, 59, val | 0x10000000);
	rp_w(r, VLD, 35, 0x00100000);
	/* The vendor's wait(`VDEC_INI_FETCH_RDY == 1) */
	if (rp_poll(r, VLD + 0xe8, BIT(0), 100000, "fetch ready (VLD +0xe8)"))
		return;	/* the shifter reads below would hang the bus */
	rp_w(r, VLD, 35, 0x00800000);

	rp_r(r, VLD, 60, 0x000001b3);
	for (i = 0; i < ARRAY_SIZE(vdec_replay_bits); i++)
		rp_r(r, VLD, 24, vdec_replay_bits[i]);
	rp_r(r, VLD, 16, 0x00000101);
	rp_r(r, VLD, 60, 0x00000101);

	rp_w(r, VLD, 34, 0xe020ffff);
	rp_w(r, VLD, 36, 0x444001ff);
	rp_w(r, VLD_TOP, 28, 0x04400780);
	rp_w(r, MC, 608, 0x78);
	rp_w(r, VLD, 65, 0x0);
	rp_w(r, VLD, 65, 0x400);
	rp_w(r, VLD, 37, 0x1ff);
	rp_w(r, VLD, 38, 0x3ff);
	rp_w(r, MC, 0, y >> 9);
	rp_w(r, MC, 1, c >> 8);
	rp_w(r, MC, 2, y >> 9);
	rp_w(r, MC, 3, c >> 8);
	rp_w(r, MC, 0, y >> 9);
	rp_w(r, MC, 1, c >> 8);
	rp_w(r, MC, 532, (y + VDEC_FB_SIZE) >> 4);
	rp_w(r, MC, 533, y >> 4);
	rp_w(r, MC, 534, 0x1);
	rp_w(r, MC, 8, 0x0);
	rp_w(r, MC, 9, 0x0);
	rp_w(r, MC, 130, 1920);
	rp_w(r, MC, 131, 1088);
	rp_w(r, VLD, 35, 0x004000ff);

	/* The vendor waits for SPI 212; poll its status bit instead */
	if (rp_poll(r, MISC_IRQ_CFG, BIT(16), 1000000, "decode done (misc +0xa4)"))
		return;
	/* Clear it as the vendor's ISR does */
	val = readl(vp->base + MISC_IRQ_CFG);
	writel(val | 0x11, vp->base + MISC_IRQ_CFG);
	writel((val | 0x11) & ~0x10, vp->base + MISC_IRQ_CFG);
	rp_r(r, SYS, 0, 0x0);	/* the vendor stops if bit 4 is set */

	rp_w(r, MISC, 50, 0xfe);
	rp_w(r, MISC, 51, 0x3f6a151d);
	rp_w(r, SYS, 0, 0x1);
	rp_r(r, MC, 10, 0x77);	/* the last macroblock: column 119 */
	rp_r(r, MC, 11, 0x43);	/* and row 67 */
}

static int vdec_replay_show(struct seq_file *s, void *unused)
{
	struct vdec_probe *vp = s->private;
	struct vdec_replay r = { .vp = vp, .s = s };
	int ret;

	if (!vp->bs_len) {
		seq_puts(s, "write the stream to mt8695-vdec-bs first\n");
		return 0;
	}
	seq_printf(s, "bitstream 0x%llx, %zu bytes; frame buffer 0x%llx\n",
		   (u64)vp->bs_iova, vp->bs_len, (u64)vp->fb_iova);
	memset(vp->fb, 0x5a, VDEC_FB_SIZE);

	ret = vdec_probe_on(vp);
	if (ret)
		return ret;
	vdec_probe_setup(vp, s);
	vdec_replay_frame(&r);
	vdec_probe_off(vp);
	seq_printf(s, "%d reads differ from the vendor's\n", r.mismatches);
	return 0;
}

/* Room for the whole output, so that seq_file never runs it twice */
static int vdec_replay_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, vdec_replay_show, inode->i_private, SZ_64K);
}

static const struct file_operations vdec_replay_fops = {
	.owner = THIS_MODULE,
	.open = vdec_replay_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* The stream; a write at offset 0 starts it over */
static ssize_t vdec_bs_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct vdec_probe *vp = file->private_data;
	ssize_t ret;

	if (*ppos == 0)
		vp->bs_len = 0;
	ret = simple_write_to_buffer(vp->bs, VDEC_BS_SIZE, ppos, buf, count);
	if (ret > 0)
		vp->bs_len = max_t(size_t, vp->bs_len, *ppos);
	return ret;
}

static const struct file_operations vdec_bs_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = vdec_bs_write,
	.llseek = default_llseek,
};

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

	/* Through the M4U (larb1's ports), as the vendor's buffers were */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	vp->bs = dmam_alloc_coherent(dev, VDEC_BS_SIZE, &vp->bs_iova, GFP_KERNEL);
	vp->fb = dmam_alloc_coherent(dev, VDEC_FB_SIZE, &vp->fb_iova, GFP_KERNEL);
	if (!vp->bs || !vp->fb)
		return -ENOMEM;
	vp->fb_blob.data = vp->fb;
	vp->fb_blob.size = VDEC_FB_SIZE;

	dev_info(dev, "%d clocks, bitstream at 0x%llx, frame buffer at 0x%llx; see /sys/kernel/debug/mt8695-vdec-*\n",
		 vp->num_clks, (u64)vp->bs_iova, (u64)vp->fb_iova);
	debugfs_create_file("mt8695-vdec-probe", 0400, NULL, vp, &vdec_probe_fops);
	debugfs_create_file("mt8695-vdec-setup", 0400, NULL, vp, &vdec_setup_fops);
	debugfs_create_file("mt8695-vdec-offread", 0400, NULL, vp, &vdec_offread_fops);
	debugfs_create_file("mt8695-vdec-replay", 0400, NULL, vp, &vdec_replay_fops);
	debugfs_create_file("mt8695-vdec-bs", 0200, NULL, vp, &vdec_bs_fops);
	debugfs_create_blob("mt8695-vdec-fb", 0400, NULL, &vp->fb_blob);
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
