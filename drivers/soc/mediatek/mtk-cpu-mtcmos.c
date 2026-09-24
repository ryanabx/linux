// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT8173 CPU MTCMOS power sequences.
 *
 * Which firmware an MT8173 board ships decides who powers a secondary core's
 * MTCMOS domain. Under upstream ARM Trusted Firmware, as on mt8173-elm, BL31
 * does it and nothing here is needed. Under MediaTek's own MT8173 Android
 * BL31, as on the Amazon Fire HD 10 (2017), PSCI CPU_ON sets the core's entry
 * point but leaves the domain off, and CPU_OFF parks the core but leaves the
 * domain on, so the kernel has to run both halves itself.
 *
 * The sequences are spm_mtcmos_ctrl_cpu1(), spm_mtcmos_ctrl_cpu4() and
 * spm_mtcmos_ctrl_cpusys1() from the vendor 3.18 kernel's
 * drivers/misc/mediatek/base/power/mt8173/mtcmos.c, with every offset and bit
 * from its mt_spm.h and mtcmos.h.
 *
 * The two clusters differ:
 *
 *  - A53 ("CA7"): per-core PWR_CON and L1_PDN registers, and the cluster is
 *    already powered because the boot CPU lives in it.
 *  - A72 ("CA15"): a shared L1_PDN register with per-core bits, and the whole
 *    cluster (CPUTOP + L2) must be powered first, which additionally means
 *    lifting the VCA15 supply isolation.
 *
 * The vendor's cluster power-up also clears the TOP_AXI bus protection for the
 * CA15 ADB and deasserts ACINACTM in MCUCFG. Both are omitted here, because
 * the only thing that asserts them is the cluster power-DOWN half of
 * spm_mtcmos_ctrl_cpusys1(), and this driver never powers a cluster down --
 * measured, on this board: the boot chain leaves both clear, and all four
 * cores come up, hotplug and suspend included, without either. Anyone adding
 * cluster power-down has to bring both back, along with the infracfg and
 * MCUCFG handles they need.
 *
 * Only one write in all of this has to happen after the firmware's CPU_ON:
 * the final PWR_RST_B, which is what releases the core. Everything else may
 * run as early as the caller likes. The CPU power API below is therefore
 * deliberately not a genpd: a genpd provider does not exist until platform
 * drivers probe, which is after smp_init(), and the CPU domains must not be
 * powered before the bring-up path asks for them anyway.
 *
 * The SPM registers are the scpsys syscon, which drivers/pmdomain/mediatek
 * uses for the peripheral power domains and mtk-spm-suspend uses for
 * suspend-to-RAM. All three take a regmap on that one node rather than
 * mapping the block again, so the CPU PWR_CONs and the domain PWR_CONs they
 * are interleaved with have a single owner and a single lock.
 */

#include <linux/bits.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/psci.h>
#include <linux/regmap.h>
#include <linux/soc/mediatek/mtk-cpu-mtcmos.h>

/* SPM register offsets within the scpsys block. */
#define SPM_PWR_STATUS		0x60c
#define SPM_PWR_STATUS_2ND	0x610

/* bits in a per-core SPM_CA7_CPUn_PWR_CON */
#define PWR_RST_B		BIT(0)
#define PWR_ISO			BIT(1)
#define PWR_ON			BIT(2)
#define PWR_ON_2ND		BIT(3)
#define PWR_CLK_DIS		BIT(4)
#define SRAM_CKISO		BIT(5)
#define SRAM_ISOINT_B		BIT(6)

/* bits in a per-core SPM_CA7_CPUn_L1_PDN */
#define L1_PDN			BIT(0)
#define L1_PDN_ACK		BIT(8)

/* A72 ("CA15") cluster */
#define SPM_CA15_CPU_PWR_CON(n)	(0x2a0 + 4 * (n))
#define SPM_CA15_CPUTOP_PWR_CON	0x2b0
#define SPM_CA15_L1_PWR_CON	0x2b4
#define SPM_CA15_L2_PWR_CON	0x2b8
#define SPM_SLEEP_DUAL_VCORE_PWR_CON 0x404

#define CA15_CPU(n)		BIT(16 + (n))
#define CA15_CPUTOP		BIT(15)
#define CA15_L1_PDN(n)		BIT(n)
#define CA15_L1_PDN_ACK(n)	BIT(8 + (n))
#define CA15_L2_PDN		BIT(0)
#define CA15_L2_PDN_ACK		BIT(8)
#define VCA15_PWR_ISO		BIT(13)

/*
 * SPM_SLEEP_TIMER_STA reports each core's STANDBYWFI. The vendor waits on it
 * before cutting a core's power, so a power-down can never land on a core that
 * is still fetching.
 */
#define SPM_SLEEP_TIMER_STA	0x720
#define CA7_STANDBYWFI(n)	BIT(16 + (n))
#define CA15_STANDBYWFI(n)	BIT(20 + (n))

/* The vendor spins forever on these; bound it instead. */
#define SPM_POLL_US		10000

#define MT8173_CORES_PER_CLUSTER	2

struct mtk_cpu_pwr {
	u16 pwr_con;
	u16 l1_pdn;
	u32 status;
};

/*
 * Indexed by the core's aff0. The PWR_CON offsets are not contiguous, hence
 * the table. MT8173 is two A53s and two A72s, so the SPM's CA7_CPU2 and
 * CA7_CPU3 registers describe cores this SoC does not have and are omitted.
 */
static const struct mtk_cpu_pwr mt8173_ca7_pwr[] = {
	{ .pwr_con = 0x200, .l1_pdn = 0x25c, .status = BIT(9)  },
	{ .pwr_con = 0x218, .l1_pdn = 0x264, .status = BIT(10) },
};

static struct regmap *spm_regmap;

static int spm_wait(u32 off, u32 mask, u32 want)
{
	u32 val;

	return regmap_read_poll_timeout_atomic(spm_regmap, off, val,
					       (val & mask) == want,
					       1, SPM_POLL_US);
}

/*
 * Power up the whole A72 cluster. Only needed for the first core of that
 * cluster; spm_mtcmos_ctrl_cpu4() guards this on CA15_CPUTOP already being
 * powered, and so do we.
 */
static int mtk_ca15_cluster_on(void)
{
	u32 sta, sta_2nd;
	int ret;

	ret = regmap_read(spm_regmap, SPM_PWR_STATUS, &sta);
	if (!ret)
		ret = regmap_read(spm_regmap, SPM_PWR_STATUS_2ND, &sta_2nd);
	if (ret)
		return ret;

	if ((sta & CA15_CPUTOP) && (sta_2nd & CA15_CPUTOP))
		return 0;

	/* lift supply isolation for the CA15 rail */
	regmap_clear_bits(spm_regmap, SPM_SLEEP_DUAL_VCORE_PWR_CON,
			  VCA15_PWR_ISO);

	regmap_set_bits(spm_regmap, SPM_CA15_CPUTOP_PWR_CON, PWR_ON);
	ret = spm_wait(SPM_PWR_STATUS, CA15_CPUTOP, CA15_CPUTOP);
	if (ret)
		return ret;

	regmap_set_bits(spm_regmap, SPM_CA15_CPUTOP_PWR_CON, PWR_ON_2ND);
	ret = spm_wait(SPM_PWR_STATUS_2ND, CA15_CPUTOP, CA15_CPUTOP);
	if (ret)
		return ret;

	regmap_clear_bits(spm_regmap, SPM_CA15_CPUTOP_PWR_CON, PWR_CLK_DIS);

	regmap_clear_bits(spm_regmap, SPM_CA15_L2_PWR_CON, CA15_L2_PDN);
	ret = spm_wait(SPM_CA15_L2_PWR_CON, CA15_L2_PDN_ACK, 0);
	if (ret)
		return ret;

	regmap_set_bits(spm_regmap, SPM_CA15_CPUTOP_PWR_CON, SRAM_ISOINT_B);
	regmap_clear_bits(spm_regmap, SPM_CA15_CPUTOP_PWR_CON, SRAM_CKISO);
	regmap_clear_bits(spm_regmap, SPM_CA15_CPUTOP_PWR_CON, PWR_ISO);

	return regmap_set_bits(spm_regmap, SPM_CA15_CPUTOP_PWR_CON, PWR_RST_B);
}

static int mtk_ca15_cpu_on(unsigned int core)
{
	u32 pwr_con = SPM_CA15_CPU_PWR_CON(core);
	int ret;

	ret = mtk_ca15_cluster_on();
	if (ret)
		return ret;

	regmap_set_bits(spm_regmap, pwr_con, PWR_ON);
	ret = spm_wait(SPM_PWR_STATUS, CA15_CPU(core), CA15_CPU(core));
	if (ret)
		return ret;

	regmap_set_bits(spm_regmap, pwr_con, PWR_ON_2ND);
	ret = spm_wait(SPM_PWR_STATUS_2ND, CA15_CPU(core), CA15_CPU(core));
	if (ret)
		return ret;

	regmap_clear_bits(spm_regmap, SPM_CA15_L1_PWR_CON, CA15_L1_PDN(core));
	ret = spm_wait(SPM_CA15_L1_PWR_CON, CA15_L1_PDN_ACK(core), 0);
	if (ret)
		return ret;

	regmap_clear_bits(spm_regmap, pwr_con, SRAM_CKISO);
	regmap_clear_bits(spm_regmap, pwr_con, PWR_ISO);
	regmap_set_bits(spm_regmap, pwr_con, PWR_RST_B);

	return 0;
}

static int mtk_ca7_cpu_on(unsigned int core)
{
	const struct mtk_cpu_pwr *p = &mt8173_ca7_pwr[core];
	int ret;

	regmap_set_bits(spm_regmap, p->pwr_con, PWR_ON);
	udelay(1);
	regmap_set_bits(spm_regmap, p->pwr_con, PWR_ON_2ND);

	ret = spm_wait(SPM_PWR_STATUS, p->status, p->status);
	if (!ret)
		ret = spm_wait(SPM_PWR_STATUS_2ND, p->status, p->status);
	if (ret)
		return ret;

	regmap_clear_bits(spm_regmap, p->pwr_con, PWR_ISO);
	regmap_clear_bits(spm_regmap, p->l1_pdn, L1_PDN);

	ret = spm_wait(p->l1_pdn, L1_PDN_ACK, 0);
	if (ret)
		return ret;

	udelay(1);
	regmap_set_bits(spm_regmap, p->pwr_con, SRAM_ISOINT_B);
	regmap_clear_bits(spm_regmap, p->pwr_con, SRAM_CKISO);
	regmap_clear_bits(spm_regmap, p->pwr_con, PWR_CLK_DIS);
	regmap_set_bits(spm_regmap, p->pwr_con, PWR_RST_B);

	return 0;
}

/*
 * The power-down sequences, from the STA_POWER_DOWN branches of
 * spm_mtcmos_ctrl_cpu1() and spm_mtcmos_ctrl_cpu4(). Each follows the
 * vendor's order for its cluster (the two are not mirror images of the
 * power-up paths, nor of each other), and they are not optional: this
 * firmware's PSCI CPU_OFF parks the core but leaves the MTCMOS domain
 * untouched, so without them a re-onlined core finds PWR_RST_B already
 * asserted, never sees the rising edge that restarts it, and fails to come
 * online -- the same symptom as having no MTCMOS support at all.
 */
static int mtk_ca7_cpu_off(unsigned int core)
{
	const struct mtk_cpu_pwr *p = &mt8173_ca7_pwr[core];
	int ret;

	regmap_set_bits(spm_regmap, p->pwr_con, PWR_ISO);
	regmap_set_bits(spm_regmap, p->pwr_con, SRAM_CKISO);
	regmap_clear_bits(spm_regmap, p->pwr_con, SRAM_ISOINT_B);
	regmap_set_bits(spm_regmap, p->l1_pdn, L1_PDN);

	ret = spm_wait(p->l1_pdn, L1_PDN_ACK, L1_PDN_ACK);
	if (ret)
		return ret;

	regmap_clear_bits(spm_regmap, p->pwr_con, PWR_RST_B);
	regmap_set_bits(spm_regmap, p->pwr_con, PWR_CLK_DIS);
	regmap_clear_bits(spm_regmap, p->pwr_con, PWR_ON);
	regmap_clear_bits(spm_regmap, p->pwr_con, PWR_ON_2ND);

	ret = spm_wait(SPM_PWR_STATUS, p->status, 0);
	if (!ret)
		ret = spm_wait(SPM_PWR_STATUS_2ND, p->status, 0);

	return ret;
}

/*
 * Only the core is powered down, never the cluster. spm_mtcmos_ctrl_cpu4()
 * goes on to call spm_mtcmos_ctrl_cpusys1() once the last A72 core is off,
 * which additionally re-asserts the CA15 ADB bus protection, ACINACTM and the
 * VCA15 supply isolation. Leaving CPUTOP powered only leaks; getting that
 * teardown wrong strands the cluster, and the power-up path above already
 * guards on CPUTOP so it stays correct either way.
 */
static int mtk_ca15_cpu_off(unsigned int core)
{
	u32 pwr_con = SPM_CA15_CPU_PWR_CON(core);
	int ret;

	regmap_set_bits(spm_regmap, pwr_con, SRAM_CKISO);
	regmap_set_bits(spm_regmap, SPM_CA15_L1_PWR_CON, CA15_L1_PDN(core));

	ret = spm_wait(SPM_CA15_L1_PWR_CON, CA15_L1_PDN_ACK(core),
		       CA15_L1_PDN_ACK(core));
	if (ret)
		return ret;

	regmap_set_bits(spm_regmap, pwr_con, PWR_ISO);
	regmap_clear_bits(spm_regmap, pwr_con, PWR_ON);
	regmap_clear_bits(spm_regmap, pwr_con, PWR_ON_2ND);

	ret = spm_wait(SPM_PWR_STATUS, CA15_CPU(core), 0);
	if (!ret)
		ret = spm_wait(SPM_PWR_STATUS_2ND, CA15_CPU(core), 0);
	if (ret)
		return ret;

	return regmap_clear_bits(spm_regmap, pwr_con, PWR_RST_B);
}

static int mtk_cpu_mtcmos_check(unsigned int cluster, unsigned int core)
{
	if (!spm_regmap)
		return -ENODEV;

	if (cluster >= 2 || core >= MT8173_CORES_PER_CLUSTER)
		return -EINVAL;

	return 0;
}

/**
 * mtk_cpu_mtcmos_power_on - power up one CPU core's MTCMOS domain
 * @cluster: the core's affinity level 1
 * @core: the core's affinity level 0
 *
 * Must be called after the firmware's CPU_ON for that core: the last write of
 * the sequence releases the core's reset, and a core released before CPU_ON
 * either fetches from a reset vector the firmware has not written yet or
 * arrives at a warm-boot entry the firmware is not expecting it at.
 */
int mtk_cpu_mtcmos_power_on(unsigned int cluster, unsigned int core)
{
	int ret;

	ret = mtk_cpu_mtcmos_check(cluster, core);
	if (ret)
		return ret;

	ret = cluster ? mtk_ca15_cpu_on(core) : mtk_ca7_cpu_on(core);
	if (ret)
		pr_err("cluster %u core %u: MTCMOS power-up failed (%d)\n",
		       cluster, core, ret);

	return ret;
}

/**
 * mtk_cpu_mtcmos_power_off - power down one CPU core's MTCMOS domain
 * @cluster: the core's affinity level 1
 * @core: the core's affinity level 0
 *
 * Must be called only once the core has stopped, which the caller is expected
 * to have established through the firmware; this additionally waits for the
 * SPM to report the core's STANDBYWFI, because cutting power to a core that is
 * still executing is not a recoverable mistake.
 */
int mtk_cpu_mtcmos_power_off(unsigned int cluster, unsigned int core)
{
	u32 wfi = cluster ? CA15_STANDBYWFI(core) : CA7_STANDBYWFI(core);
	int ret;

	ret = mtk_cpu_mtcmos_check(cluster, core);
	if (ret)
		return ret;

	ret = spm_wait(SPM_SLEEP_TIMER_STA, wfi, wfi);
	if (ret) {
		pr_err("cluster %u core %u: no STANDBYWFI; leaving its power domain on\n",
		       cluster, core);
		return ret;
	}

	ret = cluster ? mtk_ca15_cpu_off(core) : mtk_ca7_cpu_off(core);
	if (ret)
		pr_err("cluster %u core %u: MTCMOS power-down failed (%d)\n",
		       cluster, core, ret);

	return ret;
}

static struct regmap *mtk_cpu_mtcmos_syscon(const char *compatible)
{
	struct device_node *np;
	struct regmap *regmap;

	np = of_find_compatible_node(NULL, NULL, compatible);
	if (!np)
		return ERR_PTR(-ENODEV);

	regmap = syscon_node_to_regmap(np);
	of_node_put(np);

	return regmap;
}

/**
 * mtk_cpu_mtcmos_init - find the register block the sequences need
 *
 * Idempotent, and callable as early as the CPU bring-up path's own prepare
 * hook: syscon_node_to_regmap() works there, long before platform drivers
 * probe.
 */
int mtk_cpu_mtcmos_init(void)
{
	struct regmap *regmap;

	if (spm_regmap)
		return 0;

	regmap = mtk_cpu_mtcmos_syscon("mediatek,mt8173-scpsys");
	if (IS_ERR(regmap)) {
		pr_err("no regmap for the scpsys syscon (%pe); cannot power up secondaries\n",
		       regmap);
		return PTR_ERR(regmap);
	}

	spm_regmap = regmap;

	return 0;
}

/*
 * Shape B4: the board's cpu nodes keep enable-method = "psci". The power-up
 * runs from the one place that has nowhere else to go -- straight after the
 * firmware's CPU_ON -- and the power-down runs from a stock CPU hotplug
 * teardown callback, because every state below CPUHP_TEARDOWN_CPU is torn down
 * after it, i.e. after the core is already dead.
 *
 * MPIDR affinity levels 0 and 1 are the SPM's core and cluster index; only the
 * low byte of each is used on this SoC. The mapping is resolved from the DT
 * cpu nodes once, because of_cpu_device_node_get() works before the CPU
 * devices exist -- it falls back to matching cpu_logical_map().
 */
#define MTK_CPU_MTCMOS_MAX_CPUS	4

static struct {
	u8 cluster;
	u8 core;
	bool valid;
} mtk_cpu_map[MTK_CPU_MTCMOS_MAX_CPUS];

static int mtk_cpu_mtcmos_resolve(unsigned int cpu)
{
	struct device_node *np;
	u64 mpidr;
	u32 reg;

	if (cpu >= MTK_CPU_MTCMOS_MAX_CPUS)
		return -EINVAL;

	if (mtk_cpu_map[cpu].valid)
		return 0;

	np = of_cpu_device_node_get(cpu);
	if (!np)
		return -ENODEV;

	if (of_property_read_u64(np, "reg", &mpidr)) {
		if (of_property_read_u32(np, "reg", &reg)) {
			of_node_put(np);
			return -EINVAL;
		}
		mpidr = reg;
	}
	of_node_put(np);

	mtk_cpu_map[cpu].core = mpidr & 0xff;
	mtk_cpu_map[cpu].cluster = (mpidr >> 8) & 0xff;
	mtk_cpu_map[cpu].valid = true;

	return 0;
}

static int mtk_cpu_mtcmos_up(unsigned int cpu)
{
	int ret = mtk_cpu_mtcmos_resolve(cpu);

	if (ret)
		return ret;

	return mtk_cpu_mtcmos_power_on(mtk_cpu_map[cpu].cluster,
				       mtk_cpu_map[cpu].core);
}

static int mtk_cpu_mtcmos_down(unsigned int cpu)
{
	int ret = mtk_cpu_mtcmos_resolve(cpu);

	if (ret)
		return ret;

	return mtk_cpu_mtcmos_power_off(mtk_cpu_map[cpu].cluster,
					mtk_cpu_map[cpu].core);
}

/*
 * PROTOTYPE KEYING. Which boards need this is a property of their firmware;
 * the binding for saying so is the open question.
 *
 * early_initcall runs after smp_prepare_cpus() and before smp_init(), so the
 * power-up is registered before the first secondary is brought up. The
 * teardown state is registered later, from a normal initcall, because nothing
 * can go offline before that.
 */
static int __init mtk_cpu_mtcmos_register_up(void)
{
	int ret;

	if (!of_machine_is_compatible("amazon,suez"))
		return 0;

	ret = mtk_cpu_mtcmos_init();
	if (ret)
		return ret;

	return psci_set_cpu_power_on(mtk_cpu_mtcmos_up);
}
early_initcall(mtk_cpu_mtcmos_register_up);

static int __init mtk_cpu_mtcmos_register_down(void)
{
	if (!spm_regmap)
		return 0;

	return cpuhp_setup_state_nocalls(CPUHP_BP_PREPARE_DYN,
					 "soc/mtk-cpu-mtcmos",
					 NULL, mtk_cpu_mtcmos_down) < 0 ?
	       -EINVAL : 0;
}
device_initcall(mtk_cpu_mtcmos_register_down);
