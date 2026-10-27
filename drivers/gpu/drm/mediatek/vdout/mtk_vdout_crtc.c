// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek VDOUT: the CRTC
 *
 * The format timing generator (FMT) makes the CEA timings from a table of
 * modes, the OSD premix in front of it mixes the planes, and RGB2HDMI
 * takes its YCbCr output to the HDMI TX, converting it to RGB when the
 * link carries RGB. The register sequences follow MediaTek's MT8695
 * display and HDMI drivers; registers no vendor kernel code writes are set
 * to the values the bootloader leaves.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/regmap.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_modes.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "mtk_vdout_drv.h"

/* FMT at 0x000, sys1 at 0x500, RGB2HDMI at 0x800, premix and sys2 at 0xc00 */
#define FMT_MODE		0x094
#define FMT_ACTIVE_X		0x0a0
#define FMT_ACTIVE_Y0		0x0a4
#define FMT_ACTIVE_Y1		0x0a8
#define FMT_CTRL		0x0ac
#define  FMT_CTRL_ON		BIT(0)
#define  FMT_CTRL_FMTM		BIT(1)
#define  FMT_CTRL_FTRST		BIT(10)
#define  FMT_CTRL_SHADOW_EN	BIT(28)
#define  FMT_CTRL_TRIGGER	BIT(31)
#define FMT_BG			0x0b8
#define FMT_PLANE_OFF		0x0f8
#define  FMT_PLN2_OFF		BIT(16)	/* the sub video plane */
/* The sub video plane's window, in the FMT's second instance */
#define FMT_SUB			0x300

#define VDOUT_CLK		0x508
#define VSYNC_PULSE_VDO_OUT(id)	(0x520 + 4 * (id))
#define  VDOUT_CLK_HD_HALF	BIT(30)
#define VDTCLK_CFG3		0x578
#define VSYNC_PULSE_VDO(id)	(0x580 + 4 * (id))
#define VSYNC_PULSE_OSD_UHD	0x574
#define VSYNC_PULSE_OSD_FHD	0x57c
#define HDMI_CONFIG0_SUB	0x598
#define  VM_BYPASS		BIT(4)
#define HDMI_CONFIG_SUB		0x59c
#define  HDMI_SUB_BIT8		BIT(8)
#define  VM_SEL			GENMASK(12, 11)
#define  SELF_OPT_HDMI		BIT(18)
#define  VDOUT_CLK_HDMI_HD	BIT(19)
#define  VDOUT_CLK_HDMI_1080P	BIT(21)
#define  RGB2HDMI_296M_EN	BIT(23)
#define  HDMI_PRGS27M		BIT(24)
#define  HDMI_422_TO_420	BIT(27)
#define VDOUT_SHADOW		0x5b0
#define SYS_CFG_CC		0x5cc
#define  VM_SEL_DOLBY3		BIT(7)

#define R2H_CTRL		0x800
#define R2H_TIME0		0x81c
#define R2H_TIME1		0x820
#define R2H_TIME2		0x824
#define R2H_TIME3		0x828
#define R2H_DELAY		0x840
#define R2H_CLIP		0x850
#define R2H_MATRIX		0x860
#define R2H_709			0x864
#define R2H_SYNC_DELAY		0x8b4
#define R2H_HSYNC_DELAY		0x8c0

#define PREMIX_CFG07		0xc1c
#define  PREMIX_SHADOW_UPDATE	BIT(12)
#define PREMIX_CFG08		0xc20
#define PREMIX_MIX1		0xc24	/* the FHD plane */
#define PREMIX_MIX2		0xc28	/* the UHD plane, above it */
#define  PREMIX_MIX_PREMUL	BIT(0)
#define VDTCLK_CONFIG4		0xc30
#define PREMIX_SDR2HDR		0xc34
#define PREMIX_CFG0E		0xc38

/* sys2 interrupts: status bits are low while pending, cleared by a 1 then 0 */
#define VDOUT_INT_CLR		0xc50
#define VDOUT_INT_STA		0xc54
#define  VDOUT_INT_FMT_VSYNC	BIT(9)

/* mmsys */
#define MMSYS_HDMI_REFCLK	0x228
#define  MMSYS_HDMI_REFCLK_SD	BIT(16)

struct reg_val {
	u16 off;
	u32 val;
};

/*
 * Values the bootloader leaves and no vendor kernel code writes, from a
 * dump of the working 1080p output.
 */
static const struct reg_val vdout_boot_regs[] = {
	/* FMT */
	{ 0x00c, 0x00900000 }, { 0x018, 0x00160000 }, { 0x01c, 0x76543210 },
	{ 0x028, 0x00000179 }, { 0x02c, 0x00010001 }, { 0x04c, 0x00030003 },
	{ 0x05c, 0x00808010 }, { 0x060, 0x00df0682 }, { 0x064, 0x00130102 },
	{ 0x068, 0x011a0209 }, { 0x090, 0x00030005 }, { 0x098, 0x2a000000 },
	{ 0x0bc, 0x00240001 }, { 0x0c4, 0x000010eb }, { 0x0d0, 0x00df0682 },
	{ 0x0e0, 0x00130102 }, { 0x0e8, 0x00010006 }, { 0x0f0, 0x011a0209 },
	/* sys1 */
	{ 0x500, 0x8027002a }, { 0x510, 0x00000002 }, { 0x514, 0xcccc4444 },
	{ 0x518, 0x001f4444 }, { 0x528, 0x0000f810 }, { 0x53c, 0x0000fe01 },
	{ 0x540, 0x0000fe01 }, { 0x544, 0x00808010 }, { 0x5c0, 0x00000010 },
	{ 0x5c4, 0x3ff00ffc }, { 0x5f0, 0x10000000 }, { 0x5f8, 0x00000020 },
	/* RGB2HDMI gamma, gain and matrix setup */
	{ 0x804, 0x20000000 }, { 0x808, 0x60004000 }, { 0x80c, 0xa0008000 },
	{ 0x810, 0xe000c000 }, { 0x814, 0x80808080 }, { 0x818, 0x80808080 },
	{ 0x838, 0x80010000 }, { 0x854, 0xeb001000 }, { 0x868, 0x00eb0100 },
	{ 0x890, 0xec3f8080 }, { 0x894, 0xfc06f60f }, { 0x898, 0x7505bac4 },
	{ 0x8a0, 0xf4006080 }, { 0x8a4, 0xffcb07f7 }, { 0x8a8, 0x50973b34 },
	{ 0x8b0, 0x06b4020d }, { 0x8b8, 0xd4006400 }, { 0x8bc, 0x00004100 },
	{ 0x8d4, 0x00000008 },
	/* OSD premix */
	{ 0xc00, 0x00000001 }, { 0xc14, 0x00000001 }, { 0xc28, 0x0000000c },
	{ 0xc2c, 0x0000ff00 },
};

/*
 * Per-mode values from MediaTek's HDMI and display drivers. The /1.001
 * rates differ from the integer ones only in TVDPLL.
 */
struct mtk_vdout_mode {
	struct drm_display_mode mode;
	unsigned long tvdpll;
	bool hd_half;		/* 74.25 MHz pixel clock: TVDPLL / 8 */
	bool sd;		/* 27 MHz pixel clock, TMDS reference from OSDPLL */
	u32 fmt_mode;
	u32 vsync_pulse_osd;
	struct mtk_vdout_osd_timing osd;
	u32 r2h_time[3];	/* 0x81c, 0x820, 0x824 and 0x828 */
	u32 r2h_sync_delay;
	u32 r2h_delay;
	struct mtk_vdout_vdo_timing vdo;
};

#define MODE_1080P(clock)						\
	DRM_MODE("1920x1080", DRM_MODE_TYPE_DRIVER, clock, 1920, 2008,	\
		 2052, 2200, 0, 1080, 1084, 1089, 1125, 0,		\
		 DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC)
#define MODE_720P(clock)						\
	DRM_MODE("1280x720", DRM_MODE_TYPE_DRIVER, clock, 1280, 1390,	\
		 1430, 1650, 0, 720, 725, 730, 750, 0,			\
		 DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC)

#define VDOUT_1080P(clock, rate) {					\
	.mode = { MODE_1080P(clock) },					\
	.tvdpll = rate,							\
	.fmt_mode = 0xdf20,						\
	.vsync_pulse_osd = 0x08c60874,					\
	.osd = {							\
		.sync = 0x00403465, .htotal = 0x898,			\
		.start = 0x00c10000, .vstart = 0x002a002a,		\
		.sync_thres = 0x357,					\
	},								\
	.r2h_time = { 0x002c0005, 0x00c10840, 0x002a0461 },		\
	.r2h_sync_delay = 0x00010025,					\
	.vdo = {							\
		.hstart = 0xc1, .vstart = 0x2a,				\
		.path_delay = 0x08ca0886, .fmt_mode = 0x0000c220,	\
		.fmt_ratio = 0x60, .fmt_delay = 0x08c80885,		\
		.hdwn = { 5, 6, 5, 5 },					\
	},								\
}

#define VDOUT_720P(clock, rate) {					\
	.mode = { MODE_720P(clock) },					\
	.tvdpll = rate,							\
	.hd_half = true,						\
	.fmt_mode = 0xf120,						\
	.vsync_pulse_osd = 0x05d8064e,					\
	.osd = {							\
		.sync = 0x004032ee, .htotal = 0x672,			\
		.start = 0x01050000, .vstart = 0x001a001a,		\
		.sync_thres = 0x23a,					\
	},								\
	.r2h_time = { 0x00280005, 0x01050604, 0x001a02e9 },		\
	.r2h_sync_delay = 0x00010026, .r2h_delay = 0x00111000,		\
	.vdo = {							\
		.hstart = 0x105, .vstart = 0x1a,			\
		.path_delay = 0x05dc0660, .fmt_mode = 0x0000e220,	\
		.fmt_ratio = 0x2120, .fmt_delay = 0x8002064a,		\
		.hdwn = { 5, 6, 5, 7 },					\
	},								\
}

/*
 * The 50 Hz HD modes: the 60 Hz timing with a longer line, the FMT in its
 * PAL mode (and, for 1080p50, its default vsync width)
 */
#define VDOUT_1080P50 {						\
	.mode = { DRM_MODE("1920x1080", DRM_MODE_TYPE_DRIVER, 148500,	\
			   1920, 2448, 2492, 2640, 0,			\
			   1080, 1084, 1089, 1125, 0,			\
			   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },	\
	.tvdpll = 594000000,						\
	.fmt_mode = 0xc000d420,						\
	.vsync_pulse_osd = 0x08c60a2c,					\
	.osd = {							\
		.sync = 0x00403465, .htotal = 0xa50,			\
		.start = 0x00c10000, .vstart = 0x002a002a,		\
		.sync_thres = 0x384,					\
	},								\
	.r2h_time = { 0x002c0005, 0x00c10840, 0x002a0461 },		\
	.r2h_sync_delay = 0x00010025,					\
	.vdo = {							\
		.hstart = 0xc1, .vstart = 0x2a,				\
		.path_delay = 0x08ca0a3e, .fmt_mode = 0xc000c220,	\
		.fmt_ratio = 0x60, .fmt_delay = 0x08c80a28,		\
		.hdwn = { 5, 6, 5, 5 },					\
	},								\
}

#define VDOUT_720P50 {							\
	.mode = { DRM_MODE("1280x720", DRM_MODE_TYPE_DRIVER, 74250,	\
			   1280, 1720, 1760, 1980, 0, 720, 725, 730, 750, 0,\
			   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },	\
	.tvdpll = 594000000,						\
	.hd_half = true,						\
	.fmt_mode = 0xc000f120,						\
	.vsync_pulse_osd = 0x05d80798,					\
	.osd = {							\
		.sync = 0x004032ee, .htotal = 0x7bc,			\
		.start = 0x01050000, .vstart = 0x001a001a,		\
		.sync_thres = 0x258,					\
	},								\
	.r2h_time = { 0x00280005, 0x01050604, 0x001a02e9 },		\
	.r2h_sync_delay = 0x00010026, .r2h_delay = 0x00111000,		\
	.vdo = {							\
		.hstart = 0x105, .vstart = 0x1a,			\
		.path_delay = 0x05dc07aa, .fmt_mode = 0xc000e220,	\
		.fmt_ratio = 0x2120, .fmt_delay = 0x80020794,		\
		.hdwn = { 5, 6, 5, 7 },					\
	},								\
}

/* 720x480p59.94 (CEA VIC 2 and 3): MediaTek has no 27.027 MHz variant */
#define VDOUT_480P {							\
	.mode = { DRM_MODE("720x480", DRM_MODE_TYPE_DRIVER, 27000,	\
			   720, 736, 798, 858, 0, 480, 489, 495, 525, 0,\
			   DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },	\
	.tvdpll = 594000000,						\
	.sd = true,							\
	.fmt_mode = 0x9420,						\
	.vsync_pulse_osd = 0x04160333,					\
	.osd = {							\
		.sync = 0x0040320d, .htotal = 0x35a,			\
		.start = 0x007b0000, .vstart = 0x002b002b,		\
		.sync_thres = 0x18f,					\
	},								\
	.r2h_time = { 0x033e060c, 0x007b034a, 0x002b020a },		\
	.r2h_sync_delay = 0x00010021, .r2h_delay = 0x00111000,		\
	.vdo = {							\
		.hstart = 0x7b, .vstart = 0x2b,				\
		.path_delay = 0x041a034b, .fmt_mode = 0x00008220,	\
		.fmt_ratio = 0x20, .fmt_delay = 0x80020334,		\
		.hdwn = { 2, 3, 3, 3 },					\
	},								\
}

/* 720x576p50 (CEA VIC 17 and 18): the FMT in its PAL mode */
#define VDOUT_576P {							\
	.mode = { DRM_MODE("720x576", DRM_MODE_TYPE_DRIVER, 27000,	\
			   720, 732, 796, 864, 0, 576, 581, 586, 625, 0,\
			   DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },	\
	.tvdpll = 594000000,						\
	.sd = true,							\
	.fmt_mode = 0xc0009420,						\
	.vsync_pulse_osd = 0x04de0339,					\
	.osd = {							\
		.sync = 0x00403271, .htotal = 0x360,			\
		.start = 0x00850000, .vstart = 0x002d002d,		\
		.sync_thres = 0x1f4,					\
	},								\
	.r2h_time = { 0x03400005, 0x00850354, 0x002d026c },		\
	.r2h_sync_delay = 0x00010021, .r2h_delay = 0x00111000,		\
	.vdo = {							\
		.hstart = 0x85, .vstart = 0x2d,				\
		.path_delay = 0x04e20351, .fmt_mode = 0xc0008220,	\
		.fmt_ratio = 0x20, .fmt_delay = 0x8002035a,		\
		.hdwn = { 2, 3, 3, 3 },					\
	},								\
}

/* The first one is what the bootloader sets up */
static const struct mtk_vdout_mode mtk_vdout_modes[] = {
	VDOUT_1080P(148352, 593400000),
	VDOUT_1080P(148500, 594000000),
	VDOUT_720P(74250, 594000000),
	VDOUT_720P(74176, 593400000),
	VDOUT_1080P50,
	VDOUT_720P50,
	VDOUT_480P,
	VDOUT_576P,
};

static inline struct mtk_vdout *crtc_to_vdout(struct drm_crtc *crtc)
{
	return container_of(crtc, struct mtk_vdout, crtc);
}

static void rmw(void __iomem *base, u32 off, u32 mask, u32 val)
{
	writel((readl(base + off) & ~mask) | (val & mask), base + off);
}

/* FMT registers are shadowed once FMT_CTRL_SHADOW_EN is set */
static void fmt_write(struct mtk_vdout *vd, u32 off, u32 val)
{
	writel(val, vd->regs + off);
	rmw(vd->regs, FMT_CTRL, FMT_CTRL_TRIGGER, FMT_CTRL_TRIGGER);
}

static void fmt_reset(struct mtk_vdout *vd)
{
	rmw(vd->regs, FMT_CTRL, FMT_CTRL_FTRST, 0);
	rmw(vd->regs, FMT_CTRL, FMT_CTRL_FTRST, FMT_CTRL_FTRST);
	rmw(vd->regs, FMT_CTRL, FMT_CTRL_FTRST, 0);
}

static void mtk_vdout_fmt_setup(struct mtk_vdout *vd, const struct mtk_vdout_mode *m)
{
	void __iomem *v = vd->regs;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vdout_boot_regs); i++)
		writel(vdout_boot_regs[i].val, v + vdout_boot_regs[i].off);

	/* fmt_hal_set_mode(): the totals follow from the mode bits */
	writel(0, v + 0x0d4);
	writel(0, v + 0x0e4);
	writel(m->fmt_mode, v + FMT_MODE);
	writel(FMT_CTRL_FMTM, v + FMT_CTRL);
	writel(0, v + 0x024);
	fmt_reset(vd);
	rmw(v, FMT_CTRL, FMT_CTRL_ON, FMT_CTRL_ON);
	rmw(v, FMT_CTRL, FMT_CTRL_SHADOW_EN, FMT_CTRL_SHADOW_EN);

	fmt_write(vd, FMT_MODE, m->fmt_mode);
	fmt_write(vd, FMT_BG, 0x00808010);		/* YCbCr black */
	fmt_write(vd, FMT_PLANE_OFF, FMT_PLN2_OFF);	/* the OSD premix only */
	fmt_write(vd, FMT_ACTIVE_X, 0);
	fmt_write(vd, FMT_ACTIVE_Y0, 0);
	fmt_write(vd, FMT_ACTIVE_Y1, 0);
	/* The bootloader leaves a sub plane window, hidden only by PLN2_OFF */
	fmt_write(vd, FMT_SUB + FMT_ACTIVE_X, 0);
	fmt_write(vd, FMT_SUB + FMT_ACTIVE_Y0, 0);
	fmt_write(vd, FMT_SUB + FMT_ACTIVE_Y1, 0);
	fmt_reset(vd);

	/* vdout_sys_hal_set_hdmi(): the HD clock path, or the 27 MHz one */
	rmw(v, HDMI_CONFIG_SUB, SELF_OPT_HDMI | VDOUT_CLK_HDMI_HD | HDMI_PRGS27M,
	    m->sd ? HDMI_PRGS27M : SELF_OPT_HDMI | VDOUT_CLK_HDMI_HD);
	rmw(v, HDMI_CONFIG_SUB, VDOUT_CLK_HDMI_1080P,
	    m->hd_half || m->sd ? 0 : VDOUT_CLK_HDMI_1080P);
	rmw(v, VDOUT_CLK, VDOUT_CLK_HD_HALF, m->hd_half ? VDOUT_CLK_HD_HALF : 0);
	rmw(v, VDTCLK_CFG3, BIT(16) | BIT(17), 0);
	rmw(v, HDMI_CONFIG_SUB, RGB2HDMI_296M_EN, 0);
	rmw(v, VDTCLK_CONFIG4, BIT(7) | BIT(10) | BIT(11) | BIT(16) | BIT(17), 0);
	rmw(v, HDMI_CONFIG0_SUB, VM_BYPASS, VM_BYPASS);

	writel(0xf, v + VDOUT_SHADOW);

	/* Video mark input: FMT */
	rmw(v, SYS_CFG_CC, VM_SEL_DOLBY3, 0);
	rmw(v, HDMI_CONFIG_SUB, VM_SEL | HDMI_SUB_BIT8, HDMI_SUB_BIT8);
	rmw(v, FMT_CTRL, FMT_CTRL_TRIGGER, FMT_CTRL_TRIGGER);

	/* The UHD plane runs two pixels behind the FHD plane */
	fmt_write(vd, VSYNC_PULSE_OSD_FHD, m->vsync_pulse_osd);
	fmt_write(vd, VSYNC_PULSE_OSD_UHD, m->vsync_pulse_osd + 2);
}

/* A video plane's vsync pulse, delayed from the FMT's */
void mtk_vdout_video_delay(struct mtk_vdout *vd, enum mtk_vdout_vdo_id id,
			   u32 delay)
{
	fmt_write(vd, VSYNC_PULSE_VDO(id), delay);
	fmt_write(vd, VSYNC_PULSE_VDO_OUT(id), delay);
}

/*
 * A video plane's window in the mixer, begin << 16 | end in each direction
 * (the same lines for both fields), or 0 for none
 */
void mtk_vdout_video_window(struct mtk_vdout *vd, enum mtk_vdout_vdo_id id,
			    u32 h, u32 v)
{
	u32 base = id == MTK_VDOUT_VDO_SUB ? FMT_SUB : 0;

	fmt_write(vd, base + FMT_ACTIVE_X, h);
	fmt_write(vd, base + FMT_ACTIVE_Y0, v);
	fmt_write(vd, base + FMT_ACTIVE_Y1, v);
}

/* The mixer always takes the main plane; the sub plane can be left out */
void mtk_vdout_video_mix(struct mtk_vdout *vd, enum mtk_vdout_vdo_id id,
			 bool on)
{
	if (id != MTK_VDOUT_VDO_SUB)
		return;
	rmw(vd->regs, FMT_PLANE_OFF, FMT_PLN2_OFF, on ? 0 : FMT_PLN2_OFF);
	rmw(vd->regs, FMT_CTRL, FMT_CTRL_TRIGGER, FMT_CTRL_TRIGGER);
}

/* The OSD path's HDR block as the bootloader leaves it */
static const struct reg_val hdr_boot_regs[] = {
	{ 0x000, 0x01000012 }, { 0x054, 0x0fff0fff }, { 0x058, 0x00000fff },
	{ 0x0a8, 0x01000012 }, { 0x100, 0x0b000012 }, { 0x154, 0x0ff00ff0 },
	{ 0x158, 0x00000ff0 }, { 0x1a8, 0x0b000012 },
};

/*
 * No windows, shadowed; the FHD plane is mix1, the UHD plane mix2 above
 * it. The mixed output goes through the HDR block, whose RGB to YCbCr
 * conversion the FMT needs.
 */
static void mtk_vdout_premix_setup(struct mtk_vdout *vd)
{
	void __iomem *v = vd->regs;
	unsigned int i;

	writel(0x00071800, v + PREMIX_CFG07);
	writel(0x2e262524, v + PREMIX_CFG08);
	writel(0x8000000c, v + PREMIX_MIX1);
	writel(0x0000000c, v + PREMIX_MIX2);
	rmw(v, PREMIX_CFG0E, GENMASK(23, 16), 0);
	writel(0, v + PREMIX_SDR2HDR);

	for (i = 0; i < ARRAY_SIZE(hdr_boot_regs); i++)
		writel(hdr_boot_regs[i].val, vd->hdr_regs + hdr_boot_regs[i].off);
}

/* A plane's per-pixel alpha, premultiplied, against the planes below it */
void mtk_vdout_premix_alpha(struct mtk_vdout *vd, enum mtk_vdout_osd_id id,
			    bool premultiplied)
{
	u32 reg = id == MTK_VDOUT_OSD_UHD ? PREMIX_MIX2 : PREMIX_MIX1;

	rmw(vd->regs, reg, PREMIX_MIX_PREMUL,
	    premultiplied ? PREMIX_MIX_PREMUL : 0);
	rmw(vd->regs, PREMIX_CFG07, PREMIX_SHADOW_UPDATE, PREMIX_SHADOW_UPDATE);
}

/*
 * vHDMIVideoOutput() and the data enable timing. The pipeline carries
 * YCbCr; for RGB on the link, RGB2HDMI's 3x3 matrix converts it.
 */
static void mtk_vdout_r2h_setup(struct mtk_vdout *vd, const struct mtk_vdout_mode *m,
				enum drm_output_color_format format, bool limited)
{
	void __iomem *v = vd->regs;
	bool rgb = format == DRM_OUTPUT_COLOR_FORMAT_RGB444;
	bool full = !rgb || !limited;

	rmw(v, HDMI_CONFIG_SUB, HDMI_422_TO_420, 0);
	regmap_update_bits(vd->mmsys, MMSYS_HDMI_REFCLK, MMSYS_HDMI_REFCLK_SD,
			   m->sd ? MMSYS_HDMI_REFCLK_SD : 0);

	rmw(v, R2H_CTRL, BIT(21) | BIT(20) | BIT(16) | BIT(3) | BIT(2),
	    rgb ? BIT(2) : 0);
	rmw(v, R2H_CTRL, BIT(0) | BIT(6) | BIT(11), BIT(0) | BIT(6) | BIT(11));
	writel(1, v + R2H_HSYNC_DELAY);
	rmw(v, R2H_CTRL, BIT(8) | BIT(10), BIT(8) | BIT(10));
	/*
	 * RGB: BT.709 to RGB, 16-235 (mode 4) or 0-255 (mode 6). The OSD path
	 * makes BT.709 YCbCr at every resolution, so SD modes take the same
	 * matrix; MediaTek's driver picks BT.601 for them, which shifts the
	 * colours.
	 */
	rmw(v, R2H_MATRIX, BIT(31), rgb ? BIT(31) : 0);
	rmw(v, R2H_709, GENMASK(30, 28), rgb ? (full ? 6 : 4) << 28 : 0);
	writel(full ? 0xfeff0100 : 0xebff0000, v + R2H_CLIP);
	rmw(v, R2H_CTRL, BIT(25), full ? BIT(25) : 0);
	writel(m->r2h_delay, v + R2H_DELAY);

	writel(m->r2h_time[0], v + R2H_TIME0);
	rmw(v, R2H_TIME1, 0x9fff1fff, m->r2h_time[1]);
	rmw(v, R2H_TIME2, 0x07ffffff, m->r2h_time[2]);
	rmw(v, R2H_TIME3, 0x07ffffff, m->r2h_time[2]);
	writel(m->r2h_sync_delay, v + R2H_SYNC_DELAY);
}

static void mtk_vdout_clear_vsync(struct mtk_vdout *vd)
{
	writel(VDOUT_INT_FMT_VSYNC, vd->regs + VDOUT_INT_CLR);
	writel(0, vd->regs + VDOUT_INT_CLR);
}

static irqreturn_t mtk_vdout_irq(int irq, void *data)
{
	struct mtk_vdout *vd = data;

	if (readl(vd->regs + VDOUT_INT_STA) & VDOUT_INT_FMT_VSYNC)
		return IRQ_NONE;
	mtk_vdout_clear_vsync(vd);
	drm_crtc_handle_vblank(&vd->crtc);
	return IRQ_HANDLED;
}

static const struct mtk_vdout_mode *mtk_vdout_find_mode(const struct drm_display_mode *mode)
{
	unsigned int i;

	/* The FMT makes the CEA timings, sync polarities included */
	for (i = 0; i < ARRAY_SIZE(mtk_vdout_modes); i++)
		if (drm_mode_match(mode, &mtk_vdout_modes[i].mode,
				   DRM_MODE_MATCH_TIMINGS | DRM_MODE_MATCH_CLOCK |
				   DRM_MODE_MATCH_FLAGS))
			return &mtk_vdout_modes[i];
	return NULL;
}

static enum drm_mode_status mtk_vdout_crtc_mode_valid(struct drm_crtc *crtc,
						      const struct drm_display_mode *mode)
{
	return mtk_vdout_find_mode(mode) ? MODE_OK : MODE_BAD;
}

static int mtk_vdout_crtc_atomic_check(struct drm_crtc *crtc,
				       struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (crtc_state->enable && !mtk_vdout_find_mode(&crtc_state->adjusted_mode))
		return -EINVAL;
	return 0;
}

static void mtk_vdout_crtc_atomic_enable(struct drm_crtc *crtc,
					 struct drm_atomic_commit *state)
{
	struct mtk_vdout *vd = crtc_to_vdout(crtc);
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	const struct mtk_vdout_mode *m = mtk_vdout_find_mode(&crtc_state->adjusted_mode);
	enum drm_output_color_format format = DRM_OUTPUT_COLOR_FORMAT_RGB444;
	struct drm_connector_state *conn_state;
	struct drm_connector *conn;
	bool limited = false;
	int i, ret;

	if (WARN_ON(!m))
		return;

	for_each_new_connector_in_state(state, conn, conn_state, i) {
		if (conn_state->crtc != crtc)
			continue;
		format = conn_state->hdmi.output_format;
		limited = conn_state->hdmi.is_limited_range;
	}

	/*
	 * The bridge set up after this takes its PHY PLL from TVDPLL. A FMT
	 * that already runs the mode is left alone.
	 */
	if (m != vd->cur) {
		ret = clk_set_rate(vd->tvdpll, m->tvdpll);
		if (ret)
			drm_err(vd->drm, "TVDPLL to %lu Hz: %d\n", m->tvdpll, ret);
		mtk_vdout_fmt_setup(vd, m);
		vd->cur = m;
		vd->cur_vdo = &m->vdo;
	}
	mtk_vdout_premix_setup(vd);
	for (i = 0; i < MTK_VDOUT_OSD_NUM; i++)
		if (vd->osd[i])
			mtk_vdout_osd_setup(vd->osd[i], &m->mode, &m->osd);
	mtk_vdout_r2h_setup(vd, m, format, limited);

	drm_crtc_vblank_on(crtc);
}

static void mtk_vdout_crtc_atomic_disable(struct drm_crtc *crtc,
					  struct drm_atomic_commit *state)
{
	struct mtk_vdout *vd = crtc_to_vdout(crtc);
	unsigned int i;

	drm_crtc_vblank_off(crtc);

	/*
	 * The timing generator keeps running for the TX, which must not be
	 * reset; only the planes go off.
	 */
	for (i = 0; i < MTK_VDOUT_OSD_NUM; i++)
		if (vd->osd[i])
			mtk_vdout_osd_off(vd->osd[i]);
	for (i = 0; i < MTK_VDOUT_VDO_NUM; i++)
		if (vd->vdo[i])
			mtk_vdout_vdo_off(vd->vdo[i]);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event && !crtc->state->active) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static void mtk_vdout_crtc_atomic_flush(struct drm_crtc *crtc,
					struct drm_atomic_commit *state)
{
	struct drm_pending_vblank_event *event = crtc->state->event;

	if (!event)
		return;
	crtc->state->event = NULL;

	/* The OSD latches what the planes wrote at the next vsync */
	spin_lock_irq(&crtc->dev->event_lock);
	if (drm_crtc_vblank_get(crtc) == 0)
		drm_crtc_arm_vblank_event(crtc, event);
	else
		drm_crtc_send_vblank_event(crtc, event);
	spin_unlock_irq(&crtc->dev->event_lock);
}

static const struct drm_crtc_helper_funcs mtk_vdout_crtc_helper_funcs = {
	.mode_valid = mtk_vdout_crtc_mode_valid,
	.atomic_check = mtk_vdout_crtc_atomic_check,
	.atomic_enable = mtk_vdout_crtc_atomic_enable,
	.atomic_disable = mtk_vdout_crtc_atomic_disable,
	.atomic_flush = mtk_vdout_crtc_atomic_flush,
};

/* There is no interrupt mask: the vsync interrupt is gated at the GIC */
static int mtk_vdout_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct mtk_vdout *vd = crtc_to_vdout(crtc);

	mtk_vdout_clear_vsync(vd);
	enable_irq(vd->irq);
	return 0;
}

static void mtk_vdout_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct mtk_vdout *vd = crtc_to_vdout(crtc);

	/* Called under the vblank locks the handler takes */
	disable_irq_nosync(vd->irq);
}

static const struct drm_crtc_funcs mtk_vdout_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = mtk_vdout_crtc_enable_vblank,
	.disable_vblank = mtk_vdout_crtc_disable_vblank,
};

int mtk_vdout_crtc_init(struct mtk_vdout *vd, struct drm_plane *primary)
{
	struct drm_crtc *crtc = &vd->crtc;
	int ret;

	ret = devm_request_irq(vd->dev, vd->irq, mtk_vdout_irq, IRQF_NO_AUTOEN,
			       dev_name(vd->dev), vd);
	if (ret)
		return ret;

	ret = drmm_crtc_init_with_planes(vd->drm, crtc, primary, NULL,
					 &mtk_vdout_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(crtc, &mtk_vdout_crtc_helper_funcs);

	return 0;
}
