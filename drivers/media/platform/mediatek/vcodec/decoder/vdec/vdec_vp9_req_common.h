/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: George Sun <george.sun@mediatek.com>
 */

#ifndef _VDEC_VP9_REQ_COMMON_H_
#define _VDEC_VP9_REQ_COMMON_H_

#include <linux/types.h>
#include <media/v4l2-vp9.h>

/*
 * struct vdec_vp9_slice_frame_counts - vp9 counts tables footprint
 */
struct vdec_vp9_slice_frame_counts {
	union {
		struct {
			u32 band_0[3];
			u32 padding0[1];
			u32 band_1_5[5][6];
			u32 padding1[2];
		} eob_branch[4][2][2];
		u32 eob_branch_space[256 * 4];
	};

	struct {
		u32 band_0[3][4];
		u32 band_1_5[5][6][4];
	} coef_probs[4][2][2];

	u32 intra_inter[4][2];
	u32 comp_inter[5][2];
	u32 comp_inter_padding[2];
	u32 comp_ref[5][2];
	u32 comp_ref_padding[2];
	u32 single_ref[5][2][2];
	u32 inter_mode[7][4];
	u32 y_mode[4][12];
	u32 uv_mode[10][10];
	u32 partition[16][4];
	u32 switchable_interp[4][4];

	u32 tx_p8x8[2][2];
	u32 tx_p16x16[2][4];
	u32 tx_p32x32[2][4];

	u32 skip[3][4];

	u32 joint[4];

	struct {
		u32 sign[2];
		u32 class0[2];
		u32 classes[12];
		u32 bits[10][2];
		u32 padding[4];
		u32 class0_fp[2][4];
		u32 fp[4];
		u32 class0_hp[2];
		u32 hp[2];
	} mvcomp[2];

	u32 reserved[126][4];
};

/**
 * struct vdec_vp9_slice_counts_map - vp9 counts tables to map
 *                                    v4l2_vp9_frame_symbol_counts
 * @skip:	skip counts.
 * @y_mode:	Y prediction mode counts.
 * @filter:	interpolation filter counts.
 * @sign:	motion vector sign counts.
 * @classes:	motion vector class counts.
 * @class0:	motion vector class0 bit counts.
 * @bits:	motion vector bits counts.
 * @class0_fp:	motion vector class0 fractional bit counts.
 * @fp:	motion vector fractional bit counts.
 * @class0_hp:	motion vector class0 high precision fractional bit counts.
 * @hp:	motion vector high precision fractional bit counts.
 */
struct vdec_vp9_slice_counts_map {
	u32 skip[3][2];
	u32 y_mode[4][10];
	u32 filter[4][3];
	u32 sign[2][2];
	u32 classes[2][11];
	u32 class0[2][2];
	u32 bits[2][10][2];
	u32 class0_fp[2][2][4];
	u32 fp[2][4];
	u32 class0_hp[2][2];
	u32 hp[2][2];
};

void vdec_vp9_slice_counts_map_helper(struct vdec_vp9_slice_counts_map *counts_map,
				      struct vdec_vp9_slice_frame_counts *counts,
				      struct v4l2_vp9_frame_symbol_counts *counts_helper);

#endif
