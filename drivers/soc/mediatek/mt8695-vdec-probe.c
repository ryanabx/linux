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
 * /sys/kernel/debug/mt8695-vdec-replay decodes the first pictures of the
 * MPEG-2 stream written to /sys/kernel/debug/mt8695-vdec-bs (one of two
 * 1080p test clips, picked by the clip parameter: I and P, or I, P and B
 * in decode order), with the vendor library's register lists for them
 * (from its logs of those clips) and our own buffers, through the M4U.
 * Each picture lands in its own frame buffer, /sys/kernel/debug/
 * mt8695-vdec-fb, -fb1 and -fb2, in block mode. The reads the vendor
 * library made are compared with what it got.
 *
 * /sys/kernel/debug/mt8695-vdec-h264 decodes the first pictures of an
 * H.264 clip (I, P, B in decode order; how many: the h264_npics parameter)
 * by replaying the vendor library's register logs of them, the LAT then
 * the core for each (mt8695-vdec-h264-pics.h, made by the work repo's
 * tools/vdec/h264-replay-gen.py with the inputs, which go to
 * mt8695-vdec-h264-bs first). Each of the vendor's buffers gets one of
 * ours. The LAT reaches memory through larb7, so a second node
 * (mediatek,mt8695-vdec-probe-test-lat) carries its ports; the IOMMU puts
 * both in one domain. The pictures land in mt8695-vdec-h264-fb0 and on;
 * the CRCs the decoder computes (with UFO on) are compared with the
 * vendor's.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
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
#include <linux/string_helpers.h>

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
#define VDEC_RAW_SIZE	SZ_512K
#define VDEC_NUM_FB	3

/* One step of a replay table from the vendor's H.264 log */
enum { RP_W, RP_R, RP_WAIT, RP_IRQ };

struct rp_op {
	u8 op;
	const char *name;	/* side and block, as the log names them */
	u32 base;		/* the block, from 0x16000000 */
	u16 word;
	s8 region;		/* -1, or val is an offset into this region */
	u8 shift;		/* and the word takes the address >> shift */
	u32 val;
};

/* One of the vendor's buffers, and for an input where it is in the upload */
struct rp_region {
	const char *name;
	u32 size;
	u32 in_off, in_len;
};

/* A picture: the LAT's steps, then the core's */
struct rp_pic {
	const struct rp_op *lat;
	int n_lat;
	const struct rp_op *core;
	int n_core;
};

#include "mt8695-vdec-h264-pics.h"

#define H264_NUM_REGIONS	ARRAY_SIZE(h264_regions)
#define H264_RAW_SIZE		SZ_1M

struct vdec_probe {
	struct device *dev;
	void __iomem *base;
	struct clk_bulk_data *clks;
	int num_clks;

	void *raw;		/* the stream as written */
	size_t raw_len;
	void *bs;		/* the picture's chunk of it, for the decoder */
	dma_addr_t bs_iova;
	void *fb[VDEC_NUM_FB];
	dma_addr_t fb_iova[VDEC_NUM_FB];
	struct debugfs_blob_wrapper fb_blob[VDEC_NUM_FB];

	void *h264[H264_NUM_REGIONS];
	dma_addr_t h264_iova[H264_NUM_REGIONS];
	struct debugfs_blob_wrapper h264_blob[H264_NUM_REGIONS];
	void *h264_raw;		/* the inputs as written */
	size_t h264_raw_len;
};

/* The LAT node: only there for larb7's ports and runtime PM */
static struct device *vdec_probe_lat_dev;

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
 * after it (N, the default: a driver's order). With the slow mux and the
 * soc gate among the power domain's clocks, larb1 resumes on its own, and
 * the gate blocks have no runtime PM, so Y now writes the gates while the
 * domain is off: don't use it.
 */
static bool clocks_first;
module_param(clocks_first, bool, 0644);

/*
 * With clocks_first=N, which of the three muxes (the node's first three
 * clocks: bit 0 vdec_sel, 1 vdec_slow_sel, 2 vdec_lae_sel) go on before the
 * runtime PM get. The gates never do: their blocks are in the domain.
 */
static unsigned int pre_muxes;
module_param(pre_muxes, uint, 0644);

/* The replay's clip: 0, I and P (mpeg2-1080p30-ip); 1, I, P, B (-ipb) */
static unsigned int clip;
module_param(clip, uint, 0644);

/* Print every H.264 replay step on the console first (slow, but a hang shows) */
static bool h264_trace = true;
module_param(h264_trace, bool, 0644);

/*
 * Write the vendor's UFO (compressed output) words, SOCUFO 3 and 31-35
 * (Y), or leave them out for a plain block-mode picture (N)
 */
static bool h264_ufo = true;
module_param(h264_ufo, bool, 0644);

/*
 * MC word 664, written with the UFO words: -1 as logged, -2 not at all (the
 * vendor library's path without UFO), else this value
 */
static int h264_mc664 = -1;
module_param(h264_mc664, int, 0644);

/* How many of the table's pictures the H.264 replay decodes */
static unsigned int h264_npics = 1;
module_param(h264_npics, uint, 0644);

/*
 * Leave out writes to vdecsoc (0x1600f000), to see which a driver needs:
 * bit 0 word 42 (+0xa8), bit 1 word 62 (+0xf8, the LAT's), bit 2 word 60
 * (+0xf0, the core's), bit 3 word 1 (+0x04). Word 40 is the soc gate.
 */
static unsigned int h264_skip_soc;
module_param(h264_skip_soc, uint, 0644);

/*
 * For mt8695-vdec-larb7: what goes on before the LAT node's runtime PM get
 * resumes larb7, after the core node's (the domain and larb1). Bits 0-6
 * the node's clocks in order (vdec_sel, vdec_slow_sel, vdec_lae_sel, the
 * soc gate, larb7's gate, the core's vdec gate, larb1's gate), bit 7 the
 * write of vdecsoc +0xa8.
 */
static unsigned int larb7_pre;
module_param(larb7_pre, uint, 0644);

/* Leave the decoder powered after the H.264 replay (until the next reboot) */
static bool h264_keep_on;
module_param(h264_keep_on, bool, 0644);

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

/* Shift the barrel shifter @bits (1-31) with one read, and log it */
static void rp_shift(struct vdec_replay *r, u32 bits)
{
	char what[48];
	u32 val;

	snprintf(what, sizeof(what), "VLD(%u)", bits);
	vdec_probe_step(r->vp, what);
	val = readl(r->vp->base + VLD + bits * 4);
	seq_printf(r->s, "%s: 0x%08x\n", what, val);
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

/*
 * The start of each frame in the vendor's log after the setup: its chunk of
 * the stream, copied to the start of our bitstream buffer (the vendor's
 * chunks started 64-byte aligned), as the bitstream window in VLD words
 * 44-46 and 68, then the fetch.
 */
static int vdec_replay_start(struct vdec_replay *r, size_t off, size_t len)
{
	struct vdec_probe *vp = r->vp;
	u32 bs = vp->bs_iova, bs_end = vp->bs_iova + len;
	u32 val;

	if (off + len > vp->raw_len || len > VDEC_BS_SIZE) {
		seq_printf(r->s, "the stream is too short for bytes %zu-%zu\n",
			   off, off + len);
		return -EINVAL;
	}
	memcpy(vp->bs, vp->raw + off, len);

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
		return -ETIMEDOUT;	/* the shifter reads would hang the bus */
	rp_w(r, VLD, 35, 0x00800000);
	return 0;
}

/* The picture parameters both frames share: the 1080p size */
static void vdec_replay_size(struct vdec_replay *r)
{
	rp_w(r, VLD, 36, 0x444001ff);
	rp_w(r, VLD_TOP, 28, 0x04400780);
	rp_w(r, MC, 608, 0x78);
	rp_w(r, VLD, 65, 0x0);
	rp_w(r, VLD, 65, 0x400);
	rp_w(r, VLD, 37, 0x1ff);
	rp_w(r, VLD, 38, 0x3ff);
}

/* Reference slot @slot (MC words 0/1 or 2/3) is frame buffer @fb */
static void vdec_replay_slot(struct vdec_replay *r, int slot, int fb)
{
	u32 y = r->vp->fb_iova[fb], c = y + VDEC_FB_Y_SIZE;

	rp_w(r, MC, 2 * slot, y >> 9);
	rp_w(r, MC, 2 * slot + 1, c >> 8);
}

/* Output to frame buffer @fb (start and end >> 4), from slot @slot, and go */
static int vdec_replay_finish(struct vdec_replay *r, int fb, int slot)
{
	struct vdec_probe *vp = r->vp;
	u32 y = vp->fb_iova[fb];
	u32 val;

	rp_w(r, MC, 532, (y + VDEC_FB_SIZE) >> 4);
	rp_w(r, MC, 533, y >> 4);
	rp_w(r, MC, 534, 0x1);
	rp_w(r, MC, 8, 0x0);
	rp_w(r, MC, 9, slot);
	rp_w(r, MC, 130, 1920);
	rp_w(r, MC, 131, 1088);
	rp_w(r, VLD, 35, 0x004000ff);

	/* The vendor waits for SPI 212; poll its status bit instead */
	if (rp_poll(r, MISC_IRQ_CFG, BIT(16), 1000000, "decode done (misc +0xa4)"))
		return -ETIMEDOUT;
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
	return 0;
}

/*
 * One picture of a clip, from the vendor's log of it: its chunk of the
 * stream, VLD word 34 (the picture's type and f_codes), which frame buffer
 * it goes to, its references, and MC word 9 (the output's slot: 0 or 1
 * for the two reference slots, 4 for the B slot in MC words 39/40).
 */
struct vdec_pic {
	u32 off, len;
	u32 start_code;		/* what VLD word 60 finds first */
	u32 vld34;
	int out, fwd, bwd;	/* frame buffers; -1: none */
	u32 mc9;
};

static const struct vdec_pic vdec_clip_ip[] = {
	{ 0, 83095, 0x1b3, 0xe020ffff, 0, -1, -1, 0 },
	{ 82519, 131053, 0x100, 0xe04033ff, 1, 0, -1, 1 },
};

static const struct vdec_pic vdec_clip_ipb[] = {
	{ 0, 83095, 0x1b3, 0xe020ffff, 0, -1, -1, 0 },
	{ 82519, 137632, 0x100, 0xe04044ff, 1, 0, -1, 1 },
	{ 219575, 90380, 0x100, 0xe0604411, 2, 0, 1, 4 },
};

/*
 * The barrel shifter: reading VLD word n shifts the stream n bits, then
 * returns the next 32; word 60 returns the start code at the position.
 * After word 60 found the picture's first start code, shift to the first
 * slice's (00 00 01 01, found in the chunk) in 24-bit steps and the rest,
 * as the vendor library does over the headers it parses itself.
 */
static int vdec_replay_to_slice(struct vdec_replay *r, const struct vdec_pic *pic)
{
	const u8 *d = r->vp->raw + pic->off;
	u32 bits, i;

	for (i = 0; i + 4 <= pic->len; i++)
		if (d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1 && d[i + 3] == 1)
			break;
	if (i + 4 > pic->len) {
		seq_puts(r->s, "no slice in the chunk\n");
		return -EINVAL;
	}
	/* Position at the first start code; the chunks begin with it or 00s */
	for (bits = 0; d[bits / 8] == 0 && d[bits / 8 + 1] == 0 &&
	     d[bits / 8 + 2] == 0; bits += 8)
		;
	bits = i * 8 - bits;
	seq_printf(r->s, "shift %u bits to the slice\n", bits);
	while (bits >= 24) {
		rp_shift(r, 24);
		bits -= 24;
	}
	if (bits)
		rp_shift(r, bits);
	return 0;
}

static int vdec_replay_pic(struct vdec_replay *r, const struct vdec_pic *pic)
{
	int ret;

	ret = vdec_replay_start(r, pic->off, pic->len);
	if (ret)
		return ret;
	rp_r(r, VLD, 60, pic->start_code);
	ret = vdec_replay_to_slice(r, pic);
	if (ret)
		return ret;
	rp_r(r, VLD, 60, 0x101);

	rp_w(r, VLD, 34, pic->vld34);
	vdec_replay_size(r);
	/* The vendor first points both slots (and the B slot) at the output */
	vdec_replay_slot(r, 0, pic->out);
	vdec_replay_slot(r, 1, pic->out);
	if (pic->mc9 == 4) {
		rp_w(r, MC, 39, r->vp->fb_iova[pic->out] >> 9);
		rp_w(r, MC, 40, (r->vp->fb_iova[pic->out] + VDEC_FB_Y_SIZE) >> 8);
	}
	if (pic->fwd >= 0) {
		vdec_replay_slot(r, 0, pic->fwd);
		vdec_replay_slot(r, 1, pic->bwd >= 0 ? pic->bwd : pic->out);
		rp_w(r, MC, 6, 0x0);
	} else {
		vdec_replay_slot(r, 0, pic->out);
	}
	return vdec_replay_finish(r, pic->out, pic->mc9);
}

static u32 rp_val(struct vdec_probe *vp, const struct rp_op *o)
{
	if (o->region < 0)
		return o->val;
	return (u32)(vp->h264_iova[o->region] + o->val) >> o->shift;
}

/*
 * After a wait that timed out: soft-reset VLD on both sides, as the
 * vendor's reset starts, so that powering off doesn't meet a busy decoder
 */
static void rp_reset(struct vdec_replay *r)
{
	void __iomem *base = r->vp->base;

	vdec_probe_step(r->vp, "reset VLD after a timeout");
	writel(0x101, base + 0x20000 + 66 * 4);
	writel(0x101, base + 0x11000 + 66 * 4);
	udelay(10);
	writel(0x0, base + 0x20000 + 66 * 4);
	writel(0x0, base + 0x11000 + 66 * 4);
	seq_puts(r->s, "VLD reset on both sides\n");
}

static int rp_run(struct vdec_replay *r, const struct rp_op *ops, int n)
{
	struct vdec_probe *vp = r->vp;
	int i, ret;

	for (i = 0; i < n; i++) {
		const struct rp_op *o = &ops[i];
		void __iomem *reg = vp->base + o->base + o->word * 4;
		u32 want = rp_val(vp, o), val;
		char what[48];

		snprintf(what, sizeof(what), "%d %s(%u)", i, o->name, o->word);
		if (h264_trace)
			vdec_probe_step(vp, what);
		if (o->op == RP_W && o->base == 0x0f000 &&
		    ((o->word == 42 && (h264_skip_soc & BIT(0))) ||
		     (o->word == 62 && (h264_skip_soc & BIT(1))) ||
		     (o->word == 60 && (h264_skip_soc & BIT(2))) ||
		     (o->word == 1 && (h264_skip_soc & BIT(3))))) {
			seq_printf(r->s, "%s = 0x%08x skipped\n", what, want);
			continue;
		}
		/* SOCUFO, at 0x16000800, is the only block there */
		if (o->op == RP_W && !h264_ufo && o->base == 0x00800) {
			seq_printf(r->s, "%s = 0x%08x skipped\n", what, want);
			continue;
		}
		if (o->op == RP_W && !strcmp(o->name, "core MC") &&
		    o->word == 664) {
			seq_printf(r->s, "%s was 0x%08x\n", what, readl(reg));
			if (h264_mc664 == -2) {
				seq_printf(r->s, "%s = 0x%08x skipped\n", what,
					   want);
				continue;
			}
			if (h264_mc664 >= 0)
				want = h264_mc664;
		}
		switch (o->op) {
		case RP_W:
			writel(want, reg);
			seq_printf(r->s, "%s = 0x%08x\n", what, want);
			break;
		case RP_R:
			val = readl(reg);
			if (val != want)
				r->mismatches++;
			seq_printf(r->s, "%s: 0x%08x (vendor 0x%08x)%s\n", what,
				   val, want, val == want ? "" : " MISMATCH");
			break;
		case RP_WAIT:
			ret = readl_poll_timeout(reg, val, val & want, 10, 100000);
			seq_printf(r->s, "%s & 0x%x: %s, 0x%08x\n", what, want,
				   ret ? "TIMED OUT" : "done", val);
			if (ret) {
				rp_reset(r);
				return ret;	/* the shifter would hang the bus */
			}
			break;
		case RP_IRQ:
			/* The vendor waits for the interrupt; poll its bit */
			ret = readl_poll_timeout(reg, val, val & want, 10, 1000000);
			seq_printf(r->s, "%s & 0x%x (interrupt): %s, 0x%08x\n",
				   what, want, ret ? "TIMED OUT" : "done", val);
			dev_emerg(vp->dev, "%s interrupt: %s, 0x%08x\n", o->name,
				  ret ? "timed out" : "done", val);
			if (ret) {
				rp_reset(r);
				return ret;
			}
			/* Clear it as the vendor's ISR does */
			writel(val | 0x11, reg);
			writel((val | 0x11) & ~0x10, reg);
			break;
		}
	}
	return 0;
}

static int vdec_h264_show(struct seq_file *s, void *unused)
{
	struct vdec_probe *vp = s->private;
	struct vdec_replay r = { .vp = vp, .s = s };
	int i, n, ret;

	if (!vdec_probe_lat_dev) {
		seq_puts(s, "no LAT node\n");
		return 0;
	}
	for (i = 0; i < H264_NUM_REGIONS; i++) {
		const struct rp_region *rg = &h264_regions[i];

		if (rg->in_off + rg->in_len > vp->h264_raw_len) {
			seq_puts(s, "write the inputs to mt8695-vdec-h264-bs first\n");
			return 0;
		}
		seq_printf(s, "%s at 0x%llx\n", rg->name, (u64)vp->h264_iova[i]);
		/* The frame buffers filled, to see what the decoder wrote */
		memset(vp->h264[i], rg->name[0] == 'F' ? 0x5a : 0, rg->size);
		memcpy(vp->h264[i], vp->h264_raw + rg->in_off, rg->in_len);
	}

	ret = vdec_probe_on(vp);
	if (ret)
		return ret;
	/* larb7 after the clocks, as the vendor's lat power-on does */
	vdec_probe_step(vp, "LAT node on (larb7)");
	ret = pm_runtime_resume_and_get(vdec_probe_lat_dev);
	if (ret) {
		vdec_probe_off(vp);
		return ret;
	}
	/* What the vdecsoc words hold before the vendor's writes */
	for (i = 0; i < 4; i++) {
		static const u32 soc_words[] = { 0x04, 0xa8, 0xf0, 0xf8 };

		seq_printf(s, "vdecsoc +0x%02x before: 0x%08x\n", soc_words[i],
			   readl(vp->base + 0x0f000 + soc_words[i]));
	}
	n = min_t(int, h264_npics, ARRAY_SIZE(h264_pics));
	for (i = 0, ret = 0; i < n && !ret; i++) {
		const struct rp_pic *pic = &h264_pics[i];

		seq_printf(s, "== picture %d, LAT\n", i);
		ret = rp_run(&r, pic->lat, pic->n_lat);
		if (ret)
			break;
		seq_printf(s, "== picture %d, core\n", i);
		ret = rp_run(&r, pic->core, pic->n_core);
	}
	seq_printf(s, "%s; %d reads differ from the vendor's\n",
		   ret ? "stopped" : "done", r.mismatches);
	if (h264_keep_on) {
		seq_puts(s, "decoder left on\n");
		return 0;
	}
	vdec_probe_step(vp, "LAT node off (larb7)");
	pm_runtime_put_sync(vdec_probe_lat_dev);
	vdec_probe_step(vp, "decoder off");
	vdec_probe_off(vp);
	vdec_probe_step(vp, "decoder is off");
	return 0;
}

/* Which of the clocks, or the +0xa8 word, larb7 needs before it resumes */
static int vdec_larb7_show(struct seq_file *s, void *unused)
{
	struct vdec_probe *vp = s->private;
	int i, ret;

	if (!vdec_probe_lat_dev)
		return -ENODEV;
	vdec_probe_step(vp, "core node on (domain, larb1)");
	ret = pm_runtime_resume_and_get(vp->dev);
	if (ret)
		return ret;
	for (i = 0; i < vp->num_clks && i < 7; i++) {
		if (!(larb7_pre & BIT(i)))
			continue;
		vdec_probe_step(vp, vp->clks[i].id ?: "a clock");
		clk_prepare_enable(vp->clks[i].clk);
	}
	if (larb7_pre & BIT(7)) {
		vdec_probe_step(vp, "write soc gcon +0xa8");
		writel(0x1, vp->base + SOC_GCON + 0xa8);
	}
	vdec_probe_step(vp, "LAT node on (larb7)");
	ret = pm_runtime_resume_and_get(vdec_probe_lat_dev);
	seq_printf(s, "larb7_pre 0x%x: larb7 resume %d\n", larb7_pre, ret);
	dev_emerg(vp->dev, "larb7 resumed: %d\n", ret);
	if (!ret)
		pm_runtime_put_sync(vdec_probe_lat_dev);
	for (i = 0; i < vp->num_clks && i < 7; i++)
		if (larb7_pre & BIT(i))
			clk_disable_unprepare(vp->clks[i].clk);
	pm_runtime_put_sync(vp->dev);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(vdec_larb7);

static int vdec_h264_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, vdec_h264_show, inode->i_private, SZ_1M);
}

static const struct file_operations vdec_h264_fops = {
	.owner = THIS_MODULE,
	.open = vdec_h264_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int vdec_replay_show(struct seq_file *s, void *unused)
{
	struct vdec_probe *vp = s->private;
	struct vdec_replay r = { .vp = vp, .s = s };
	const struct vdec_pic *pics;
	int i, n, ret;

	if (!vp->raw_len) {
		seq_puts(s, "write the stream to mt8695-vdec-bs first\n");
		return 0;
	}
	pics = clip ? vdec_clip_ipb : vdec_clip_ip;
	n = clip ? ARRAY_SIZE(vdec_clip_ipb) : ARRAY_SIZE(vdec_clip_ip);
	seq_printf(s, "clip %u, stream %zu bytes; bitstream 0x%llx\n", clip,
		   vp->raw_len, (u64)vp->bs_iova);
	for (i = 0; i < VDEC_NUM_FB; i++)
		memset(vp->fb[i], 0x5a, VDEC_FB_SIZE);

	ret = vdec_probe_on(vp);
	if (ret)
		return ret;
	for (i = 0; i < n && !ret; i++) {
		seq_printf(s, "== picture %d, frame buffer %d (0x%llx)\n", i,
			   pics[i].out, (u64)vp->fb_iova[pics[i].out]);
		vdec_probe_setup(vp, s);
		ret = vdec_replay_pic(&r, &pics[i]);
	}
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
		vp->raw_len = 0;
	if (*ppos >= VDEC_RAW_SIZE)
		return -ENOSPC;
	ret = simple_write_to_buffer(vp->raw, VDEC_RAW_SIZE, ppos, buf, count);
	if (ret > 0)
		vp->raw_len = max_t(size_t, vp->raw_len, *ppos);
	return ret;
}

/* The H.264 inputs, as tools/vdec/h264-replay-gen.py wrote them */
static ssize_t vdec_h264_bs_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct vdec_probe *vp = file->private_data;
	ssize_t ret;

	if (*ppos == 0)
		vp->h264_raw_len = 0;
	if (*ppos >= H264_RAW_SIZE)
		return -ENOSPC;
	ret = simple_write_to_buffer(vp->h264_raw, H264_RAW_SIZE, ppos, buf,
				     count);
	if (ret > 0)
		vp->h264_raw_len = max_t(size_t, vp->h264_raw_len, *ppos);
	return ret;
}

static const struct file_operations vdec_h264_bs_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = vdec_h264_bs_write,
	.llseek = default_llseek,
};

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
	int i, ret;

	if (of_device_is_compatible(dev->of_node,
				    "mediatek,mt8695-vdec-probe-test-lat")) {
		ret = devm_pm_runtime_enable(dev);
		if (ret)
			return ret;
		vdec_probe_lat_dev = dev;
		return 0;
	}

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
	vp->raw = devm_kzalloc(dev, VDEC_RAW_SIZE, GFP_KERNEL);
	vp->bs = dmam_alloc_coherent(dev, VDEC_BS_SIZE, &vp->bs_iova, GFP_KERNEL);
	if (!vp->raw || !vp->bs)
		return -ENOMEM;
	for (i = 0; i < VDEC_NUM_FB; i++) {
		vp->fb[i] = dmam_alloc_coherent(dev, VDEC_FB_SIZE, &vp->fb_iova[i],
						GFP_KERNEL);
		if (!vp->fb[i])
			return -ENOMEM;
		vp->fb_blob[i].data = vp->fb[i];
		vp->fb_blob[i].size = VDEC_FB_SIZE;
	}
	vp->h264_raw = devm_kzalloc(dev, H264_RAW_SIZE, GFP_KERNEL);
	if (!vp->h264_raw)
		return -ENOMEM;
	for (i = 0; i < H264_NUM_REGIONS; i++) {
		vp->h264[i] = dmam_alloc_coherent(dev, h264_regions[i].size,
						  &vp->h264_iova[i], GFP_KERNEL);
		if (!vp->h264[i])
			return -ENOMEM;
		vp->h264_blob[i].data = vp->h264[i];
		vp->h264_blob[i].size = h264_regions[i].size;
	}

	dev_info(dev, "%d clocks, bitstream at 0x%llx; see /sys/kernel/debug/mt8695-vdec-*\n",
		 vp->num_clks, (u64)vp->bs_iova);
	debugfs_create_file("mt8695-vdec-probe", 0400, NULL, vp, &vdec_probe_fops);
	debugfs_create_file("mt8695-vdec-setup", 0400, NULL, vp, &vdec_setup_fops);
	debugfs_create_file("mt8695-vdec-offread", 0400, NULL, vp, &vdec_offread_fops);
	debugfs_create_file("mt8695-vdec-replay", 0400, NULL, vp, &vdec_replay_fops);
	debugfs_create_file("mt8695-vdec-bs", 0200, NULL, vp, &vdec_bs_fops);
	debugfs_create_blob("mt8695-vdec-fb", 0400, NULL, &vp->fb_blob[0]);
	debugfs_create_blob("mt8695-vdec-fb1", 0400, NULL, &vp->fb_blob[1]);
	debugfs_create_blob("mt8695-vdec-fb2", 0400, NULL, &vp->fb_blob[2]);
	debugfs_create_file("mt8695-vdec-h264", 0400, NULL, vp, &vdec_h264_fops);
	debugfs_create_file("mt8695-vdec-h264-bs", 0200, NULL, vp, &vdec_h264_bs_fops);
	debugfs_create_file("mt8695-vdec-larb7", 0400, NULL, vp, &vdec_larb7_fops);
	/* Each of the replay's buffers, as mt8695-vdec-h264-fb0 and so on */
	for (i = 0; i < H264_NUM_REGIONS; i++) {
		char lower[8], name[32];

		string_lower(lower, h264_regions[i].name);
		snprintf(name, sizeof(name), "mt8695-vdec-h264-%s", lower);
		debugfs_create_blob(name, 0400, NULL, &vp->h264_blob[i]);
	}
	return 0;
}

static const struct of_device_id mt8695_vdec_probe_of_match[] = {
	{ .compatible = "mediatek,mt8695-vdec-probe-test" },
	{ .compatible = "mediatek,mt8695-vdec-probe-test-lat" },
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
