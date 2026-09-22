// SPDX-License-Identifier: GPL-2.0-only
/*
 * Battery driver for the MediaTek MT6397 PMIC's fuel gauge, running the
 * battery-meter algorithm of MediaTek's downstream kernels
 * (drivers/power/mt81xx, CONFIG_SOC_BY_HW_FG).
 *
 * Copyright (C) 2026 Ryan Brue <ryanbrue.dev@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/iio/consumer.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/time.h>
#include <linux/units.h>
#include <linux/workqueue.h>

/* Clock gates for the gauge; it runs from the RTC's 32 kHz oscillator. */
#define MT6397_TOP_CKPDN		0x0102
#define MT6397_TOP_CKPDN_FGADC_CK	(BIT(6) | BIT(5))

#define MT6397_FGADC_CON0		0x0618
#define MT6397_FGADC_CON0_MODE		GENMASK(7, 0)
#define MT6397_FGADC_CON0_MODE_SETUP	0x28
#define MT6397_FGADC_CON0_MODE_ON	0x29
#define MT6397_FGADC_CON0_LATCHED	BIT(10)
#define MT6397_FGADC_CON0_CMD		GENMASK(15, 8)
#define MT6397_FGADC_CON0_CMD_IDLE	0x00
#define MT6397_FGADC_CON0_CMD_LATCH	0x02
#define MT6397_FGADC_CON0_CMD_CLEAR	0x08
#define MT6397_FGADC_CON0_CMD_RESET	0x71
#define MT6397_FGADC_CON0_CMD_RESET_LATCH 0x73

/*
 * The charge accumulator is a 36-bit register, CAR[35:0], spread over three
 * 16-bit words: CON1 holds bits 35:32 in its low nibble, CON2 bits 31:16 and
 * CON3 bits 15:0. The vendor's meter reads a 16-bit window of it, bits 29:14
 * (CON3[15:14] as the low two bits, CON2[13:0] above them), and takes the
 * sign from bit 35.
 */
#define MT6397_FGADC_CON1		0x061a
#define MT6397_FGADC_CON1_CAR_SIGN	BIT(3)
#define MT6397_FGADC_CON2		0x061c
#define MT6397_FGADC_CON2_CAR_HIGH	GENMASK(13, 0)
#define MT6397_FGADC_CON3		0x061e
#define MT6397_FGADC_CON3_CAR_LOW	GENMASK(15, 14)

#define MT6397_FGADC_CON8		0x0628

#define MT6397_FGADC_TIMEOUT_US		100000

/*
 * How long to wait for the gauge's first conversion after enabling it. It
 * takes about 170 ms from cold on an MT8173 board; this is a generous bound
 * on that rather than a tight one.
 */
#define MT6397_FGADC_FIRST_SAMPLE_US	(500 * USEC_PER_MSEC)

/*
 * One LSB is 158.122 nA * 1000 across a 20 mOhm sense resistor, and scales
 * inversely with the resistor a board actually fits.
 */
#define MT6397_FGADC_NA_PER_LSB		158122
#define MT6397_FGADC_REF_SENSE_MOHM	20

/* One accumulator LSB, likewise referred to a 20 mOhm sense resistor. */
#define MT6397_FGADC_NAH_PER_LSB	359860

/*
 * The AUXADC latches the battery channel at PMIC power-on, before any load:
 * the "hardware OCV". Same 1:4 divider as the live channel, plus the vendor's
 * fixed correction.
 */
#define MT6397_AUXADC_ADC20		0x053c
#define MT6397_AUXADC_ADC20_VAL		GENMASK(9, 0)
#define MT6397_AUXADC_FULL_SCALE_MV	1200
#define MT6397_AUXADC_PRECISION		1024
#define MT6397_AUXADC_VBAT_DIVIDER	4
#define MT6397_BAT_HW_OCV_TUNE_MV	8

/* The battery-meter task period and its tunables, as the vendor ships them. */
#define MT6397_BAT_PERIOD_S		10
#define MT6397_BAT_SYSTEM_OFF_MV	3400
#define MT6397_BAT_VBAT_AVG_SIZE	18
#define MT6397_BAT_MIN_ERROR_OFFSET_MV	1000
#define MT6397_BAT_TEMP_AVG_SIZE	12
#define MT6397_BAT_R_COMP_ITERATIONS	5
#define MT6397_BAT_POWERON_DELTA_CAP	45
#define MT6397_BAT_SYNC_TO_REAL_S	30
/*
 * The 100 % ramp step. This is the vendor's cust_soc_jeita_sync_time, which
 * is what its meter uses with JEITA support built in (as it is for this
 * board); the 10 s onehundred_percent_tracking_time is the non-JEITA path.
 */
#define MT6397_BAT_FULL_TRACK_S		60

/*
 * A run has to cover most of the pack to measure it: the same error at each
 * end is a larger share of a shorter run. The vendor uses the same threshold.
 */
#define MT6397_BAT_LEARN_MIN_SPAN	85

struct mt6397_bat_point {
	s32 x;		/* depth of discharge in %, or resistance in mOhm */
	s32 mv;
};

struct mt6397_bat_profile {
	int temp_c;
	int qmax_mah;
	int qmax_hc_mah;	/* Qmax under the table's load current */
	struct mt6397_bat_point *ocv;
	struct mt6397_bat_point *res;
};

struct mt6397_battery {
	struct device *dev;
	struct regmap *regmap;
	struct iio_channel *vbat;
	struct nvmem_cell *soc_cell;
	struct thermal_zone_device *tz;
	struct power_supply *psy;
	struct power_supply *charger;
	struct delayed_work work;
	/* serialises the meter's state against the power_supply reads */
	struct mutex lock;

	/* board and battery data */
	u32 sense_mohm;
	u32 car_tune;
	bool ocv2cv;
	int cv_current_01ma;
	int poweron_low_cap_tol;
	struct mt6397_bat_profile prof[POWER_SUPPLY_OCV_TEMP_MAX];
	int nprof;
	int npts;
	/* the tables re-interpolated for the present temperature */
	struct mt6397_bat_point *ocv_t;
	struct mt6397_bat_point *res_t;

	/* temperature */
	int temp_mc;
	int temp_c;
	int temp_buf[MT6397_BAT_TEMP_AVG_SIZE];
	int temp_sum, temp_idx, temp_last_avg;
	bool temp_init;

	/* voltage and current */
	int vbat_mv;		/* live terminal voltage */
	int zcv_mv;		/* compensated and averaged open-circuit voltage */
	int vavg_buf[MT6397_BAT_VBAT_AVG_SIZE];
	int vavg_sum, vavg_idx, vavg_mv;
	int cur_01ma;		/* magnitude, in 0.1 mA like the vendor */
	bool charging;		/* sign of the current */
	int cur_factor;		/* average load as a percentage of cv_current */
	int cur_buf[MT6397_BAT_TEMP_AVG_SIZE];
	int cur_sum, cur_idx;
	bool cur_init;
	int car_uah;		/* since the last accumulator reset */
	int rbat_mohm;
	int comp_mv;

	/* the gauge */
	int qmax_mah, qmax_aging_mah, qmax_hc_mah;
	int qmax_health;	/* learned capacity per mille of design, 1000 = as new */
	int dod0, dod1;
	int cap_by_v, cap_by_c;
	int soc;		/* after the load compensation */
	int ui_soc;		/* what is reported; -1 until the first pass */
	unsigned int sync_counter;
	unsigned int full_counter;
	bool refresh_ui;
	bool full;
	bool full_reset_pending;
	bool reached_full;	/* showed 100 % on this charger session */
	bool charger_online;
	int status;
	int hw_ocv_mv, sw_ocv_mv, rtc_soc;
};

/* ------------------------------------------------------------------------ */
/* Fuel gauge hardware access                                               */

static int mt6397_battery_fg_cmd(struct mt6397_battery *bat, unsigned int cmd)
{
	return regmap_update_bits(bat->regmap, MT6397_FGADC_CON0,
				  MT6397_FGADC_CON0_CMD,
				  FIELD_PREP(MT6397_FGADC_CON0_CMD, cmd));
}

static int mt6397_battery_fg_latch(struct mt6397_battery *bat, unsigned int cmd)
{
	unsigned int reg;
	int ret;

	ret = mt6397_battery_fg_cmd(bat, cmd);
	if (ret)
		return ret;

	return regmap_read_poll_timeout(bat->regmap, MT6397_FGADC_CON0, reg,
					reg & MT6397_FGADC_CON0_LATCHED,
					100, MT6397_FGADC_TIMEOUT_US);
}

static int mt6397_battery_fg_release(struct mt6397_battery *bat)
{
	unsigned int reg;
	int ret;

	ret = mt6397_battery_fg_cmd(bat, MT6397_FGADC_CON0_CMD_CLEAR);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(bat->regmap, MT6397_FGADC_CON0, reg,
				       !(reg & MT6397_FGADC_CON0_LATCHED),
				       100, MT6397_FGADC_TIMEOUT_US);
	if (ret)
		return ret;

	return mt6397_battery_fg_cmd(bat, MT6397_FGADC_CON0_CMD_IDLE);
}

/* Signed, positive into the battery, in uA. */
static int mt6397_battery_read_current(struct mt6397_battery *bat, int *curr_ua)
{
	unsigned int reg;
	int raw, ret;

	ret = mt6397_battery_fg_latch(bat, MT6397_FGADC_CON0_CMD_LATCH);
	if (ret)
		return ret;

	ret = regmap_read(bat->regmap, MT6397_FGADC_CON8, &reg);
	if (ret)
		return ret;

	/* Not two's complement: a magnitude with the sign folded in as 65535 - raw. */
	raw = reg > 32767 ? -(65535 - (int)reg) : (int)reg;

	*curr_ua = div_s64((s64)raw * MT6397_FGADC_NA_PER_LSB *
			   MT6397_FGADC_REF_SENSE_MOHM * bat->car_tune,
			   (s64)bat->sense_mohm * 1000 * 100);

	return mt6397_battery_fg_release(bat);
}

static int mt6397_battery_read_car_raw(struct mt6397_battery *bat,
				       unsigned int cmd, int *raw)
{
	unsigned int con1, con2, con3, car;
	int ret;

	ret = mt6397_battery_fg_latch(bat, cmd);
	if (ret)
		return ret;

	ret = regmap_read(bat->regmap, MT6397_FGADC_CON1, &con1);
	if (ret)
		return ret;

	ret = regmap_read(bat->regmap, MT6397_FGADC_CON2, &con2);
	if (ret)
		return ret;

	ret = regmap_read(bat->regmap, MT6397_FGADC_CON3, &con3);
	if (ret)
		return ret;

	car = FIELD_GET(MT6397_FGADC_CON2_CAR_HIGH, con2) << 2 |
	      FIELD_GET(MT6397_FGADC_CON3_CAR_LOW, con3);

	/*
	 * Not two's complement: the window is a magnitude, and CAR[35] says
	 * whether the accumulator has gone negative, i.e. net discharge, in
	 * which case the vendor reads it as (window - 0xffff). An all-ones
	 * window is treated as zero, as the vendor does.
	 */
	if (car == 0xffff)
		*raw = 0;
	else if (con1 & MT6397_FGADC_CON1_CAR_SIGN)
		*raw = (int)car - 0xffff;
	else
		*raw = car;

	return mt6397_battery_fg_release(bat);
}

/* Signed, positive for charge that went into the battery, in uAh. */
static int mt6397_battery_read_car(struct mt6397_battery *bat, int *charge_uah)
{
	int raw, ret;

	ret = mt6397_battery_read_car_raw(bat, MT6397_FGADC_CON0_CMD_LATCH, &raw);
	if (ret)
		return ret;

	*charge_uah = div_s64((s64)raw * MT6397_FGADC_NAH_PER_LSB *
			      MT6397_FGADC_REF_SENSE_MOHM * bat->car_tune,
			      (s64)bat->sense_mohm * 1000 * 100);

	return 0;
}

/* Zero the accumulator, as the vendor does whenever it re-anchors the gauge. */
static int mt6397_battery_reset_car(struct mt6397_battery *bat)
{
	int i, raw, ret;

	for (i = 0; i < 10; i++) {
		ret = mt6397_battery_fg_cmd(bat, MT6397_FGADC_CON0_CMD_RESET);
		if (ret)
			return ret;

		ret = mt6397_battery_read_car_raw(bat,
						  MT6397_FGADC_CON0_CMD_RESET_LATCH,
						  &raw);
		if (ret)
			return ret;

		if (!raw)
			return 0;
	}

	dev_warn(bat->dev, "charge accumulator would not reset (%d)\n", raw);
	return -EIO;
}

static int mt6397_battery_fg_enable(struct mt6397_battery *bat)
{
	int curr_ua = 0, err, ret;

	ret = regmap_clear_bits(bat->regmap, MT6397_TOP_CKPDN,
				MT6397_TOP_CKPDN_FGADC_CK);
	if (ret)
		return ret;

	/* current mode, auto-calibration, 32 kHz; then enable */
	ret = regmap_update_bits(bat->regmap, MT6397_FGADC_CON0,
				 MT6397_FGADC_CON0_MODE,
				 FIELD_PREP(MT6397_FGADC_CON0_MODE,
					    MT6397_FGADC_CON0_MODE_SETUP));
	if (ret)
		return ret;

	ret = regmap_update_bits(bat->regmap, MT6397_FGADC_CON0,
				 MT6397_FGADC_CON0_MODE,
				 FIELD_PREP(MT6397_FGADC_CON0_MODE,
					    MT6397_FGADC_CON0_MODE_ON));
	if (ret)
		return ret;

	ret = mt6397_battery_reset_car(bat);
	if (ret)
		return ret;

	/*
	 * The boot seed compensates the open-circuit voltage for the load, so
	 * a current of zero would seed the gauge several percent out.
	 */
	err = read_poll_timeout(mt6397_battery_read_current, ret,
				ret || curr_ua, USEC_PER_MSEC,
				MT6397_FGADC_FIRST_SAMPLE_US, false, bat,
				&curr_ua);
	if (err || ret)
		dev_warn(bat->dev, "no current reading after enabling the gauge\n");

	return 0;
}

static int mt6397_battery_read_hw_ocv(struct mt6397_battery *bat, int *mv)
{
	unsigned int reg;
	int ret;

	ret = regmap_read(bat->regmap, MT6397_AUXADC_ADC20, &reg);
	if (ret)
		return ret;

	*mv = FIELD_GET(MT6397_AUXADC_ADC20_VAL, reg) * MT6397_AUXADC_VBAT_DIVIDER *
	      MT6397_AUXADC_FULL_SCALE_MV / MT6397_AUXADC_PRECISION +
	      MT6397_BAT_HW_OCV_TUNE_MV;

	return 0;
}

/* The last state of charge, in a byte of the PMIC RTC's spare storage. */
static int mt6397_battery_rtc_get_soc(struct mt6397_battery *bat)
{
	size_t len;
	int soc;
	u8 *buf;

	buf = nvmem_cell_read(bat->soc_cell, &len);
	if (IS_ERR(buf))
		return 0;

	soc = len ? *buf : 0;
	kfree(buf);

	/* Anything out of range is not something this driver wrote. */
	return soc > 100 ? 0 : soc;
}

static void mt6397_battery_rtc_set_soc(struct mt6397_battery *bat, int soc)
{
	u8 val = clamp(soc, 0, 100);

	if (mt6397_battery_rtc_get_soc(bat) == val)
		return;

	nvmem_cell_write(bat->soc_cell, &val, sizeof(val));
}

/* ------------------------------------------------------------------------ */
/* Tables                                                                   */

/*
 * Profiles are ordered by falling voltage. Interpolate the other column, and
 * clamp to the end value beyond either end, as the vendor's lookups do.
 */
static int mt6397_bat_x_by_mv(const struct mt6397_bat_point *pts, int n, int mv)
{
	int i;

	if (mv > pts[0].mv)
		return pts[0].x;
	if (mv < pts[n - 1].mv)
		return pts[n - 1].x;

	for (i = 0; i < n - 1; i++) {
		if (mv <= pts[i].mv && mv >= pts[i + 1].mv) {
			if (pts[i].mv == pts[i + 1].mv)
				return pts[i].x;
			return pts[i].x + (pts[i].mv - mv) *
			       (pts[i + 1].x - pts[i].x) /
			       (pts[i].mv - pts[i + 1].mv);
		}
	}

	return pts[n - 1].x;
}

static int mt6397_bat_lerp(int lo, int hi, int t, int t_lo, int t_hi)
{
	if (t_hi == t_lo)
		return lo;
	return lo + (t - t_lo) * (hi - lo) / (t_hi - t_lo);
}

/* Pick the two profiles bracketing a temperature, clamping to the ends. */
static void mt6397_bat_bracket(struct mt6397_battery *bat, int *temp_c,
			       int *lo, int *hi)
{
	int i;

	/* Nothing looks a profile up before they are built; don't index none. */
	if (bat->nprof <= 0) {
		*lo = 0;
		*hi = 0;
		return;
	}

	if (*temp_c <= bat->prof[0].temp_c) {
		*temp_c = bat->prof[0].temp_c;
		*lo = 0;
		*hi = bat->nprof > 1 ? 1 : 0;
		return;
	}

	for (i = 1; i < bat->nprof; i++) {
		if (*temp_c <= bat->prof[i].temp_c) {
			*lo = i - 1;
			*hi = i;
			return;
		}
	}

	*temp_c = bat->prof[bat->nprof - 1].temp_c;
	*lo = bat->nprof > 1 ? bat->nprof - 2 : 0;
	*hi = bat->nprof - 1;
}

static void mt6397_bat_construct_tables(struct mt6397_battery *bat, int temp_c)
{
	int lo, hi, i;

	mt6397_bat_bracket(bat, &temp_c, &lo, &hi);

	for (i = 0; i < bat->npts; i++) {
		bat->ocv_t[i].x = bat->prof[hi].ocv[i].x;
		bat->ocv_t[i].mv = mt6397_bat_lerp(bat->prof[lo].ocv[i].mv,
						   bat->prof[hi].ocv[i].mv,
						   temp_c, bat->prof[lo].temp_c,
						   bat->prof[hi].temp_c);
		bat->res_t[i].x = mt6397_bat_lerp(bat->prof[lo].res[i].x,
						  bat->prof[hi].res[i].x,
						  temp_c, bat->prof[lo].temp_c,
						  bat->prof[hi].temp_c);
		bat->res_t[i].mv = mt6397_bat_lerp(bat->prof[lo].res[i].mv,
						   bat->prof[hi].res[i].mv,
						   temp_c, bat->prof[lo].temp_c,
						   bat->prof[hi].temp_c);
	}
}

static int mt6397_bat_qmax(struct mt6397_battery *bat, int temp_c, bool hc)
{
	int lo, hi;

	mt6397_bat_bracket(bat, &temp_c, &lo, &hi);

	if (hc)
		return mt6397_bat_lerp(bat->prof[lo].qmax_hc_mah,
				       bat->prof[hi].qmax_hc_mah, temp_c,
				       bat->prof[lo].temp_c, bat->prof[hi].temp_c);

	return mt6397_bat_lerp(bat->prof[lo].qmax_mah, bat->prof[hi].qmax_mah,
			       temp_c, bat->prof[lo].temp_c, bat->prof[hi].temp_c);
}

static void mt6397_bat_update_qmax(struct mt6397_battery *bat)
{
	bat->qmax_mah = mt6397_bat_qmax(bat, bat->temp_c, false);
	bat->qmax_hc_mah = mt6397_bat_qmax(bat, bat->temp_c, true);
	bat->qmax_aging_mah = DIV_ROUND_CLOSEST(bat->qmax_mah * bat->qmax_health,
						1000);
}

/* Capacity in % from an open-circuit voltage, at the present temperature. */
static int mt6397_bat_cap_by_mv(struct mt6397_battery *bat, int mv)
{
	return 100 - mt6397_bat_x_by_mv(bat->ocv_t, bat->npts, mv);
}

/*
 * The vendor's ZCV tables were taken under a fixed load (cv_current). With a
 * heavier average load the pack reaches the system-off voltage sooner, so its
 * effective Qmax is smaller: walk the table until OCV - I*R drops below the
 * cut-off, and take what the pack had given up by the point before it from
 * that point's own depth of discharge.
 */
static int mt6397_bat_qmax_by_current(struct mt6397_battery *bat, int i_01ma)
{
	int idx, qmax = bat->qmax_hc_mah;

	if (!bat->res_t[0].x || !bat->ocv_t[0].mv)
		return qmax;

	for (idx = 0; idx < bat->npts - 1; idx++) {
		int vdrop = i_01ma * bat->res_t[idx].x / 10000;

		if (bat->ocv_t[idx].mv - vdrop < MT6397_BAT_SYSTEM_OFF_MV) {
			int dod = bat->ocv_t[idx ? idx - 1 : 0].x;

			qmax = max(dod, 1) * bat->qmax_mah / 100;
			break;
		}
	}

	return qmax;
}

/* ------------------------------------------------------------------------ */
/* Measurements                                                             */

/*
 * I*R in mV, iterated because the resistance is itself looked up by voltage.
 * Negative while charging, so voltage + compensation is the OCV either way.
 */
static int mt6397_bat_compensate(struct mt6397_battery *bat, int mv)
{
	int v2 = mv, comp = 0, i;

	for (i = 0; i <= MT6397_BAT_R_COMP_ITERATIONS; i++) {
		bat->rbat_mohm = mt6397_bat_x_by_mv(bat->res_t, bat->npts, v2);
		comp = DIV_ROUND_CLOSEST(bat->cur_01ma *
					 (bat->rbat_mohm + (int)bat->sense_mohm),
					 10000);
		if (bat->charging)
			comp = -comp;
		v2 = mv + comp;
	}

	bat->comp_mv = comp;
	return comp;
}

static void mt6397_bat_update_temp(struct mt6397_battery *bat)
{
	int temp_c, avg, i;

	if (thermal_zone_get_temp(bat->tz, &bat->temp_mc))
		bat->temp_mc = bat->temp_c * MILLIDEGREE_PER_DEGREE;
	temp_c = DIV_ROUND_CLOSEST(bat->temp_mc, MILLIDEGREE_PER_DEGREE);
	bat->temp_c = temp_c;

	if (!bat->temp_init) {
		for (i = 0; i < MT6397_BAT_TEMP_AVG_SIZE; i++)
			bat->temp_buf[i] = temp_c;
		bat->temp_sum = temp_c * MT6397_BAT_TEMP_AVG_SIZE;
		bat->temp_last_avg = temp_c;
		bat->temp_init = true;
		mt6397_bat_construct_tables(bat, temp_c);
		mt6397_bat_update_qmax(bat);
	}

	bat->temp_sum -= bat->temp_buf[bat->temp_idx];
	bat->temp_sum += temp_c;
	bat->temp_buf[bat->temp_idx] = temp_c;
	avg = bat->temp_sum / MT6397_BAT_TEMP_AVG_SIZE;
	bat->temp_idx = (bat->temp_idx + 1) % MT6397_BAT_TEMP_AVG_SIZE;

	if (avg != bat->temp_last_avg) {
		mt6397_bat_construct_tables(bat, temp_c);
		mt6397_bat_update_qmax(bat);
		bat->temp_last_avg = avg;
	}
}

static int mt6397_bat_read_vi(struct mt6397_battery *bat)
{
	int mv, ua, ret;

	ret = iio_read_channel_processed(bat->vbat, &mv);
	if (ret < 0)
		return ret;

	ret = mt6397_battery_read_current(bat, &ua);
	if (ret)
		return ret;

	bat->vbat_mv = mv;
	bat->charging = ua > 0;
	bat->cur_01ma = abs(ua) / 100;

	return 0;
}

static void mt6397_bat_update_current_factor(struct mt6397_battery *bat)
{
	int i, avg;

	if (bat->charging) {
		bat->cur_init = false;
		bat->cur_factor = 100;
		return;
	}

	if (!bat->cur_init) {
		for (i = 0; i < MT6397_BAT_TEMP_AVG_SIZE; i++)
			bat->cur_buf[i] = bat->cur_01ma;
		bat->cur_sum = bat->cur_01ma * MT6397_BAT_TEMP_AVG_SIZE;
		bat->cur_init = true;
	}

	bat->cur_sum -= bat->cur_buf[bat->cur_idx];
	bat->cur_sum += bat->cur_01ma;
	bat->cur_buf[bat->cur_idx] = bat->cur_01ma;
	avg = bat->cur_sum / MT6397_BAT_TEMP_AVG_SIZE;
	bat->cur_idx = (bat->cur_idx + 1) % MT6397_BAT_TEMP_AVG_SIZE;

	bat->cur_factor = bat->cv_current_01ma ?
			  avg * 100 / bat->cv_current_01ma : 100;
}

/* ------------------------------------------------------------------------ */
/* The gauge                                                                */

static int mt6397_bat_dod1(struct mt6397_battery *bat)
{
	int dod1;

	bat->dod0 = clamp(bat->dod0, 0, 100);
	dod1 = bat->dod0 - (int)div_s64((s64)bat->car_uah * 100,
					(s64)bat->qmax_aging_mah * 1000);

	return clamp(dod1, 0, 100);
}

/* Scale a depth of discharge by how much load has shrunk the usable Qmax. */
static int mt6397_bat_transform_dod(struct mt6397_battery *bat, int d)
{
	int c_0ma = bat->qmax_mah, c_load;

	if (!bat->charging && bat->cur_factor > 100) {
		int i_avg = bat->cur_factor * (bat->cv_current_01ma / 100);

		c_load = min(bat->qmax_hc_mah,
			     mt6397_bat_qmax_by_current(bat, i_avg));
	} else {
		c_load = bat->qmax_hc_mah;
	}

	if (c_0ma > c_load && c_load)
		d += (c_0ma - c_load) * d / c_load;

	return min(d, 100);
}

static void mt6397_bat_meter_run(struct mt6397_battery *bat)
{
	int i, zcv, offset;

	mt6397_bat_update_temp(bat);

	if (mt6397_bat_read_vi(bat))
		return;

	zcv = bat->vbat_mv + mt6397_bat_compensate(bat, bat->vbat_mv);

	if (bat->ocv2cv)
		mt6397_bat_update_current_factor(bat);

	if (mt6397_battery_read_car(bat, &bat->car_uah))
		return;

	/* The vendor's sliding average of the compensated voltage. */
	if (!bat->vavg_mv) {
		for (i = 0; i < MT6397_BAT_VBAT_AVG_SIZE; i++)
			bat->vavg_buf[i] = zcv;
		bat->vavg_sum = zcv * MT6397_BAT_VBAT_AVG_SIZE;
		bat->vavg_mv = zcv;
	}
	offset = abs(zcv - bat->vavg_mv);
	if (offset <= MT6397_BAT_MIN_ERROR_OFFSET_MV) {
		bat->vavg_sum -= bat->vavg_buf[bat->vavg_idx];
		bat->vavg_sum += zcv;
		bat->vavg_buf[bat->vavg_idx] = zcv;
		bat->vavg_mv = bat->vavg_sum / MT6397_BAT_VBAT_AVG_SIZE;
		bat->vavg_idx = (bat->vavg_idx + 1) % MT6397_BAT_VBAT_AVG_SIZE;
	}
	bat->zcv_mv = bat->vavg_mv;

	bat->cap_by_v = mt6397_bat_cap_by_mv(bat, bat->zcv_mv);

	bat->dod1 = mt6397_bat_dod1(bat);
	bat->cap_by_c = max(100 - bat->dod1, 1);

	if (bat->ocv2cv)
		bat->soc = 100 - mt6397_bat_transform_dod(bat, 100 - bat->cap_by_c);
	else
		bat->soc = bat->cap_by_c;
}

/* Re-anchor the coulomb counter at the displayed percentage. */
static void mt6397_bat_meter_reset(struct mt6397_battery *bat, int ui_soc)
{
	if (!mt6397_battery_reset_car(bat))
		bat->car_uah = 0;
	bat->dod0 = 100 - ui_soc;
	bat->dod1 = bat->dod0;
}

/*
 * The charge that passes between two known states of the pack measures what it
 * now holds, which is what CHARGE_FULL is for: the vendor's meter recomputes it
 * when a charge that began deeply discharged terminates
 * (fg_qmax_update_for_aging()), and cpcap-battery learns the same figure from
 * its own coulomb counter between full and empty. Without it the meter counts
 * against the design capacity for the life of the pack; measured on a cell that
 * had lost 12 %, the percentage fell too slowly and read ten points high near
 * empty.
 *
 * Both ends are used, because they fail in different places. Charging measures
 * against the depth the pack was seeded at, which on an aged pack is itself
 * optimistic -- so that direction converges slowly or not at all. Discharging
 * from a full re-anchor to the cut-off spans the whole pack and needs no seed.
 *
 * Kept as a proportion rather than an absolute, so that the capacity still
 * follows temperature the way the profiles describe.
 */
static void mt6397_bat_learn_qmax(struct mt6397_battery *bat, int span)
{
	int learned, health;

	if (span < MT6397_BAT_LEARN_MIN_SPAN || !bat->car_uah ||
	    bat->qmax_mah <= 0)
		return;

	learned = DIV_ROUND_CLOSEST(abs(bat->car_uah) / 1000 * 100, span);
	health = DIV_ROUND_CLOSEST(learned * 1000, bat->qmax_mah);

	/*
	 * Well outside this and the starting estimate was wrong, not the pack:
	 * keep what we had rather than take a figure the meter cannot recover
	 * from until the next deep discharge.
	 */
	if (health < 500 || health > 1100)
		return;

	if (health == bat->qmax_health)
		return;

	dev_info(bat->dev,
		 "learned %d mAh (%d %% of design) from %d mAh across %d %% of the pack\n",
		 learned, health / 10, abs(bat->car_uah) / 1000, span);

	bat->qmax_health = health;
	mt6397_bat_update_qmax(bat);
}

static void mt6397_bat_read_charger(struct mt6397_battery *bat)
{
	union power_supply_propval val;
	bool was_full = bat->full;

	bat->charger_online = false;
	bat->full = false;
	bat->status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (!bat->charger)
		return;

	if (!power_supply_get_property(bat->charger, POWER_SUPPLY_PROP_ONLINE, &val))
		bat->charger_online = val.intval;

	if (!bat->charger_online) {
		bat->reached_full = false;
		return;
	}

	if (!power_supply_get_property(bat->charger, POWER_SUPPLY_PROP_STATUS, &val)) {
		bat->status = val.intval;
		bat->full = val.intval == POWER_SUPPLY_STATUS_FULL;
	} else {
		bat->status = POWER_SUPPLY_STATUS_CHARGING;
	}

	/*
	 * A pack that already reads 100 % when the cable goes in should not
	 * drop back to 99 %, but it is not evidence that the charger has
	 * finished. Record that it has been full rather than that it is full:
	 * marking it full here feeds a branch below that pins ui_soc at 100,
	 * which pins it here in turn, and the displayed value never comes down
	 * again while the cable is in -- measured at 100 % across 4102 mAh of
	 * discharge. The vendor's meter takes bat_full from the charger alone.
	 */
	if (bat->ui_soc == 100)
		bat->reached_full = true;

	if (bat->full && !was_full) {
		/* Charged up to full from wherever the meter thought it was. */
		mt6397_bat_learn_qmax(bat, bat->dod0);
		bat->full_reset_pending = true;
	}
}

/*
 * The displayed percentage, which only ever moves one point at a time and sits
 * at 99 % until the charger reports full.
 */
static void mt6397_bat_update_ui(struct mt6397_battery *bat)
{
	bool reset = false;

	if (bat->ui_soc < 0)
		bat->ui_soc = bat->soc;

	if (bat->charger_online) {
		if (bat->vbat_mv <= MT6397_BAT_SYSTEM_OFF_MV) {
			/* Discharged to the cut-off; the span is what is left. */
			mt6397_bat_learn_qmax(bat, 100 - bat->dod0);
			if (bat->ui_soc > 0)
				bat->ui_soc--;
			reset = true;
		} else if (bat->full) {
			if (bat->ui_soc >= 100) {
				bat->ui_soc = 100;
				bat->reached_full = true;
				if (bat->full_reset_pending) {
					reset = true;
					bat->full_reset_pending = false;
				}
			} else {
				bat->full_counter += MT6397_BAT_PERIOD_S;
				if (bat->full_counter >= MT6397_BAT_FULL_TRACK_S) {
					bat->full_counter = 0;
					bat->ui_soc++;
					reset = true;
				}
			}
		} else {
			/*
			 * Once the pack has been reported full, the charger
			 * cycling back into a top-up charge is not a reason to
			 * show 99 %: the vendor keeps 100 % through its
			 * recharging state, and so does this.
			 */
			if (bat->ui_soc >= 99 && bat->charging && !bat->reached_full)
				bat->ui_soc = 99;
			bat->full_counter = 0;
		}
	} else if (bat->vbat_mv <= MT6397_BAT_SYSTEM_OFF_MV) {
		mt6397_bat_learn_qmax(bat, 100 - bat->dod0);
		if (bat->ui_soc > 0)
			bat->ui_soc--;
		reset = true;
	}

	if (reset) {
		mt6397_bat_meter_reset(bat, bat->ui_soc);
	} else if (!bat->full) {
		if (bat->ui_soc > bat->soc && bat->ui_soc != 1) {
			bat->sync_counter += MT6397_BAT_PERIOD_S;
			if (bat->refresh_ui ||
			    bat->sync_counter >= MT6397_BAT_SYNC_TO_REAL_S) {
				bat->ui_soc--;
				bat->sync_counter = 0;
				bat->refresh_ui = false;
			}
		} else {
			bat->sync_counter = 0;
			if (bat->charger_online && bat->ui_soc < bat->soc) {
				if (bat->soc - bat->ui_soc > 1)
					bat->ui_soc++;
				else
					bat->ui_soc = bat->soc;
			}
		}

		if (bat->ui_soc == 100 && !bat->reached_full)
			bat->ui_soc = 99;
		if (bat->ui_soc <= 0)
			bat->ui_soc = 1;
	}

	/*
	 * What goes in the RTC is the coulomb-counted value when the load
	 * correction is in use, so that the next boot starts from a figure it
	 * can correct itself, and the displayed one otherwise -- as the
	 * vendor's meter does either way, with the same floor of 1 %.
	 */
	mt6397_battery_rtc_set_soc(bat, max(bat->ocv2cv ? bat->cap_by_c :
					    bat->ui_soc, 1));
}

static void mt6397_bat_work(struct work_struct *work)
{
	struct mt6397_battery *bat = container_of(to_delayed_work(work),
						  struct mt6397_battery, work);
	int ui_before, status_before;

	scoped_guard(mutex, &bat->lock) {
		ui_before = bat->ui_soc;
		status_before = bat->status;

		mt6397_bat_read_charger(bat);
		mt6397_bat_meter_run(bat);
		mt6397_bat_update_ui(bat);

		dev_dbg(bat->dev,
			"t=%d v=%d zcv=%d i=%s%d.%d car=%d rbat=%d dod0=%d dod1=%d capv=%d capc=%d soc=%d ui=%d qmax=%d/%d f=%d\n",
			bat->temp_c, bat->vbat_mv, bat->zcv_mv,
			bat->charging ? "+" : "-", bat->cur_01ma / 10,
			bat->cur_01ma % 10, bat->car_uah, bat->rbat_mohm,
			bat->dod0, bat->dod1, bat->cap_by_v, bat->cap_by_c,
			bat->soc, bat->ui_soc, bat->qmax_mah, bat->qmax_hc_mah,
			bat->cur_factor);
	}

	if (ui_before != bat->ui_soc || status_before != bat->status)
		power_supply_changed(bat->psy);

	schedule_delayed_work(&bat->work, MT6397_BAT_PERIOD_S * HZ);
}

/*
 * Seed the depth of discharge from three candidates in the vendor's order of
 * trust: the latched power-on OCV, the load-compensated live voltage, and what
 * the previous boot left in the RTC.
 */
static void mt6397_bat_init_soc(struct mt6397_battery *bat)
{
	int hw_cap, sw_cap, cap, rtc;

	mt6397_bat_update_temp(bat);
	mt6397_bat_read_charger(bat);

	if (mt6397_bat_read_vi(bat)) {
		bat->vbat_mv = 3800;
		bat->cur_01ma = 0;
	}
	bat->sw_ocv_mv = bat->vbat_mv + mt6397_bat_compensate(bat, bat->vbat_mv);
	sw_cap = mt6397_bat_cap_by_mv(bat, bat->sw_ocv_mv);

	if (mt6397_battery_read_hw_ocv(bat, &bat->hw_ocv_mv))
		bat->hw_ocv_mv = bat->sw_ocv_mv;
	hw_cap = mt6397_bat_cap_by_mv(bat, bat->hw_ocv_mv);

	cap = hw_cap;
	if (abs(sw_cap - hw_cap) > 5)
		cap = sw_cap;

	rtc = mt6397_battery_rtc_get_soc(bat);
	bat->rtc_soc = rtc;
	if (rtc > 1 && rtc >= sw_cap + 15) {
		rtc--;
		mt6397_battery_rtc_set_soc(bat, rtc);
	}
	if (bat->charger_online && rtc > 1 && sw_cap - rtc > 10) {
		rtc++;
		mt6397_battery_rtc_set_soc(bat, rtc);
	}

	if (rtc &&
	    (abs(rtc - cap) < MT6397_BAT_POWERON_DELTA_CAP ||
	     abs(rtc - sw_cap) < MT6397_BAT_POWERON_DELTA_CAP) &&
	    (cap > bat->poweron_low_cap_tol || bat->charger_online))
		cap = rtc;

	if (!cap && bat->charger_online)
		cap = 1;

	dev_info(bat->dev,
		 "hw_ocv=%d mV (%d%%) sw_ocv=%d mV (%d%%) rtc=%d%% -> %d%%, t=%d C, qmax=%d mAh\n",
		 bat->hw_ocv_mv, hw_cap, bat->sw_ocv_mv, sw_cap, bat->rtc_soc,
		 cap, bat->temp_c, bat->qmax_mah);

	bat->dod0 = 100 - cap;
	bat->dod1 = bat->dod0;
	bat->cap_by_c = cap;
	bat->cap_by_v = cap;
	bat->soc = cap;
	bat->ui_soc = -1;
	bat->cur_factor = 100;
}

/* ------------------------------------------------------------------------ */
/* power_supply                                                             */

/*
 * Chemistry and the design capacity and voltages are absent on purpose: the
 * core serves them from the battery node, and naming one here overrides it.
 */
static const enum power_supply_property mt6397_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_SCOPE,
};

static int mt6397_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mt6397_battery *bat = power_supply_get_drvdata(psy);
	int ret = 0;

	guard(mutex)(&bat->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat->status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bat->vbat_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = bat->zcv_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW: {
		int ua;

		ret = mt6397_battery_read_current(bat, &ua);
		if (!ret)
			val->intval = ua;
		break;
	}
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bat->ui_soc < 0 ? bat->soc : bat->ui_soc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = mt6397_battery_read_car(bat, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = bat->qmax_aging_mah * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = bat->qmax_aging_mah * 1000 / 100 * bat->cap_by_c;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = DIV_ROUND_CLOSEST(bat->temp_mc, 100);
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void mt6397_battery_external_power_changed(struct power_supply *psy)
{
	struct mt6397_battery *bat = power_supply_get_drvdata(psy);

	/* re-run now so a plug or unplug is reflected without waiting a period */
	mod_delayed_work(system_percpu_wq, &bat->work, HZ);
}

static const struct power_supply_desc mt6397_battery_desc = {
	.name			= "mt6397-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property		= mt6397_battery_get_property,
	.external_power_changed	= mt6397_battery_external_power_changed,
	.properties		= mt6397_battery_properties,
	.num_properties		= ARRAY_SIZE(mt6397_battery_properties),
};

/* ------------------------------------------------------------------------ */
/* Device tree                                                              */

static int mt6397_bat_parse_dt(struct mt6397_battery *bat)
{
	struct device *dev = bat->dev;
	u32 v;
	int ret;

	ret = device_property_read_u32(dev, "shunt-resistor-micro-ohms", &v);
	if (ret || v < 1000)
		return dev_err_probe(dev, ret ?: -EINVAL,
				     "missing or too small shunt-resistor-micro-ohms\n");
	/* The meter works in milliohms throughout, as the vendor's does. */
	bat->sense_mohm = v / 1000;

	bat->car_tune = 100;
	device_property_read_u32(dev, "mediatek,car-tune-value", &bat->car_tune);

	bat->ocv2cv = device_property_read_bool(dev, "mediatek,ocv2cv-transform");

	v = 600000;
	device_property_read_u32(dev, "mediatek,cv-current-microamp", &v);
	bat->cv_current_01ma = v / 100;

	v = 0;
	device_property_read_u32(dev, "mediatek,poweron-low-capacity-tolerance", &v);
	bat->poweron_low_cap_tol = v;

	return 0;
}

/*
 * The resistance profile matching ocv-capacity-table-<index>: pairs of
 * open-circuit voltage and resistance, in micro-units, kept in milli-units.
 */
static int mt6397_bat_read_resistance(struct mt6397_battery *bat, int index,
				      struct mt6397_bat_point **out)
{
	struct device *dev = bat->dev;
	struct mt6397_bat_point *pts;
	int i, n, ret;

	char *name __free(kfree) = kasprintf(GFP_KERNEL,
					     "mediatek,ocv-resistance-table-%d",
					     index);
	if (!name)
		return -ENOMEM;

	n = device_property_count_u32(dev, name);
	if (n < 0)
		return dev_err_probe(dev, n, "%s\n", name);
	if (n != bat->npts * 2)
		return dev_err_probe(dev, -EINVAL, "%s: %d points, expected %d\n",
				     name, n / 2, bat->npts);

	u32 *buf __free(kfree) = kcalloc(n, sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = device_property_read_u32_array(dev, name, buf, n);
	if (ret)
		return ret;

	pts = devm_kcalloc(dev, bat->npts, sizeof(*pts), GFP_KERNEL);
	if (!pts)
		return -ENOMEM;

	for (i = 0; i < bat->npts; i++) {
		pts[i].mv = buf[i * 2] / 1000;
		pts[i].x = buf[i * 2 + 1] / 1000;
	}

	*out = pts;
	return 0;
}

/*
 * The OCV tables and their temperatures come from the battery node via the
 * power-supply core; the resistance and capacity figures have no generic
 * equivalent and sit on this node, indexed the same way.
 */
static int mt6397_bat_build_profiles(struct mt6397_battery *bat,
				     struct power_supply_battery_info *info)
{
	u32 qmax[POWER_SUPPLY_OCV_TEMP_MAX], qmax_hc[POWER_SUPPLY_OCV_TEMP_MAX];
	struct device *dev = bat->dev;
	int i, j, n, ret;

	for (n = 0; n < POWER_SUPPLY_OCV_TEMP_MAX; n++)
		if (!info->ocv_table[n])
			break;
	if (!n)
		return dev_err_probe(dev, -EINVAL,
				     "the battery has no ocv-capacity-table\n");
	bat->nprof = n;
	bat->npts = info->ocv_table_size[0];
	if (bat->npts < 2)
		return dev_err_probe(dev, -EINVAL,
				     "ocv-capacity-table-0: needs at least two points\n");

	ret = device_property_read_u32_array(dev, "mediatek,qmax-microamp-hours",
					     qmax, n);
	if (ret)
		return dev_err_probe(dev, ret, "mediatek,qmax-microamp-hours\n");

	ret = device_property_read_u32_array(dev,
					     "mediatek,qmax-high-current-microamp-hours",
					     qmax_hc, n);
	if (ret)
		memcpy(qmax_hc, qmax, n * sizeof(*qmax_hc));

	for (i = 0; i < n; i++) {
		const struct power_supply_battery_ocv_table *ocv = info->ocv_table[i];
		struct mt6397_bat_point *pts;

		if (i && info->ocv_temp[i] <= info->ocv_temp[i - 1])
			return dev_err_probe(dev, -EINVAL,
					     "ocv-capacity-celsius must ascend\n");
		if (info->ocv_table_size[i] != bat->npts)
			return dev_err_probe(dev, -EINVAL,
					     "ocv-capacity-table-%d: %d points, expected %d\n",
					     i, info->ocv_table_size[i], bat->npts);

		pts = devm_kcalloc(dev, bat->npts, sizeof(*pts), GFP_KERNEL);
		if (!pts)
			return -ENOMEM;

		/*
		 * The meter works in depth of discharge, the binding in
		 * capacity; both tables run from full to empty.
		 */
		for (j = 0; j < bat->npts; j++) {
			pts[j].mv = ocv[j].ocv / 1000;
			pts[j].x = 100 - ocv[j].capacity;
		}

		bat->prof[i].temp_c = info->ocv_temp[i];
		bat->prof[i].qmax_mah = qmax[i] / 1000;
		bat->prof[i].qmax_hc_mah = qmax_hc[i] / 1000;
		bat->prof[i].ocv = pts;

		ret = mt6397_bat_read_resistance(bat, i, &bat->prof[i].res);
		if (ret)
			return ret;
	}

	bat->ocv_t = devm_kcalloc(dev, bat->npts, sizeof(*bat->ocv_t), GFP_KERNEL);
	bat->res_t = devm_kcalloc(dev, bat->npts, sizeof(*bat->res_t), GFP_KERNEL);
	if (!bat->ocv_t || !bat->res_t)
		return -ENOMEM;

	return 0;
}

static int mt6397_battery_probe(struct platform_device *pdev)
{
	struct power_supply_battery_info *info;
	struct power_supply_config psy_cfg = {};
	struct device *dev = &pdev->dev;
	struct mt6397_chip *chip = dev_get_drvdata(dev->parent);
	struct mt6397_battery *bat;
	int ret;

	bat = devm_kzalloc(dev, sizeof(*bat), GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	bat->dev = dev;
	bat->regmap = chip->regmap;
	bat->temp_c = 25;
	/* Until a charge from near empty measures otherwise, assume a new pack. */
	bat->qmax_health = 1000;

	ret = devm_mutex_init(dev, &bat->lock);
	if (ret)
		return ret;

	ret = mt6397_bat_parse_dt(bat);
	if (ret)
		return ret;

	bat->vbat = devm_iio_channel_get(dev, "voltage");
	if (IS_ERR(bat->vbat))
		return dev_err_probe(dev, PTR_ERR(bat->vbat),
				     "failed to get the voltage channel\n");

	/*
	 * The RTC keeps a byte for exactly this, and owning the register is
	 * what lets it serialise the write against its own alarm handling.
	 */
	bat->soc_cell = devm_nvmem_cell_get(dev, "state-of-charge");
	if (IS_ERR(bat->soc_cell))
		return dev_err_probe(dev, PTR_ERR(bat->soc_cell),
				     "failed to get the state-of-charge cell\n");

	/* The thermistor is its own NTC device, read through a thermal zone. */
	bat->tz = thermal_zone_get_zone_by_name("battery-thermal");
	if (IS_ERR(bat->tz)) {
		/* -ENODEV here usually means the zone has not probed yet. */
		ret = PTR_ERR(bat->tz);
		return dev_err_probe(dev, ret == -ENODEV ? -EPROBE_DEFER : ret,
				     "failed to get the battery thermal zone\n");
	}

	/* the charger tells us whether it is there and whether it is done */
	bat->charger = devm_power_supply_get_by_reference(dev, "power-supplies");
	if (IS_ERR(bat->charger)) {
		if (PTR_ERR(bat->charger) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		bat->charger = NULL;
	}

	ret = mt6397_battery_fg_enable(bat);
	if (ret)
		return dev_err_probe(dev, ret, "failed to start the fuel gauge\n");

	psy_cfg.drv_data = bat;
	psy_cfg.fwnode = dev_fwnode(dev);
	bat->psy = devm_power_supply_register(dev, &mt6397_battery_desc, &psy_cfg);
	if (IS_ERR(bat->psy))
		return dev_err_probe(dev, PTR_ERR(bat->psy),
				     "failed to register the power supply\n");

	ret = power_supply_get_battery_info(bat->psy, &info);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the battery info\n");

	/* The profiles are copied out of it, so it is not needed after this. */
	ret = mt6397_bat_build_profiles(bat, info);
	power_supply_put_battery_info(bat->psy, info);
	if (ret)
		return ret;

	ret = devm_delayed_work_autocancel(dev, &bat->work, mt6397_bat_work);
	if (ret)
		return ret;

	scoped_guard(mutex, &bat->lock)
		mt6397_bat_init_soc(bat);

	platform_set_drvdata(pdev, bat);
	schedule_delayed_work(&bat->work, 0);

	return 0;
}

static int mt6397_battery_suspend(struct device *dev)
{
	struct mt6397_battery *bat = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&bat->work);
	return 0;
}

static int mt6397_battery_resume(struct device *dev)
{
	struct mt6397_battery *bat = dev_get_drvdata(dev);

	/* The accumulator counted through the sleep; resync the display now. */
	scoped_guard(mutex, &bat->lock)
		bat->refresh_ui = true;
	schedule_delayed_work(&bat->work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mt6397_battery_pm_ops,
				mt6397_battery_suspend, mt6397_battery_resume);

static const struct of_device_id mt6397_battery_of_match[] = {
	{ .compatible = "mediatek,mt6397-battery" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6397_battery_of_match);

static struct platform_driver mt6397_battery_driver = {
	.driver = {
		.name = "mt6397-battery",
		.of_match_table = mt6397_battery_of_match,
		.pm = pm_sleep_ptr(&mt6397_battery_pm_ops),
	},
	.probe = mt6397_battery_probe,
};
module_platform_driver(mt6397_battery_driver);

MODULE_AUTHOR("Ryan Brue <ryanbrue.dev@gmail.com>");
MODULE_DESCRIPTION("MediaTek MT6397 PMIC fuel gauge battery driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_CONSUMER");
