// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: George Sun <george.sun@mediatek.com>
 */

#include <linux/string.h>

#include "vdec_vp9_req_common.h"

/* clang stack usage explodes if this is inlined */
static noinline_for_stack
void vdec_vp9_slice_map_counts_eob_coef(unsigned int i, unsigned int j, unsigned int k,
					struct vdec_vp9_slice_frame_counts *counts,
					struct v4l2_vp9_frame_symbol_counts *counts_helper)
{
	u32 l = 0, m;

	/*
	 * helper eo -> mtk eo
	 * helpre e1 -> mtk c3
	 * helper c0 -> c0
	 * helper c1 -> c1
	 * helper c2 -> c2
	 */
	for (m = 0; m < 3; m++) {
		counts_helper->coeff[i][j][k][l][m] =
			(u32 (*)[3]) & counts->coef_probs[i][j][k].band_0[m];
		counts_helper->eob[i][j][k][l][m][0] =
			&counts->eob_branch[i][j][k].band_0[m];
		counts_helper->eob[i][j][k][l][m][1] =
			&counts->coef_probs[i][j][k].band_0[m][3];
	}

	for (l = 1; l < 6; l++) {
		for (m = 0; m < 6; m++) {
			counts_helper->coeff[i][j][k][l][m] =
				(u32 (*)[3]) & counts->coef_probs[i][j][k].band_1_5[l - 1][m];
			counts_helper->eob[i][j][k][l][m][0] =
				&counts->eob_branch[i][j][k].band_1_5[l - 1][m];
			counts_helper->eob[i][j][k][l][m][1] =
				&counts->coef_probs[i][j][k].band_1_5[l - 1][m][3];
		}
	}
}

void vdec_vp9_slice_counts_map_helper(struct vdec_vp9_slice_counts_map *counts_map,
				      struct vdec_vp9_slice_frame_counts *counts,
				      struct v4l2_vp9_frame_symbol_counts *counts_helper)
{
	int i, j, k;

	counts_helper->partition = &counts->partition;
	counts_helper->intra_inter = &counts->intra_inter;
	counts_helper->tx32p = &counts->tx_p32x32;
	counts_helper->tx16p = &counts->tx_p16x16;
	counts_helper->tx8p = &counts->tx_p8x8;
	counts_helper->uv_mode = &counts->uv_mode;

	counts_helper->comp = &counts->comp_inter;
	counts_helper->comp_ref = &counts->comp_ref;
	counts_helper->single_ref = &counts->single_ref;
	counts_helper->mv_mode = &counts->inter_mode;
	counts_helper->mv_joint = &counts->joint;

	for (i = 0; i < ARRAY_SIZE(counts_map->skip); i++)
		memcpy(counts_map->skip[i], counts->skip[i],
		       sizeof(counts_map->skip[0]));
	counts_helper->skip = &counts_map->skip;

	for (i = 0; i < ARRAY_SIZE(counts_map->y_mode); i++)
		memcpy(counts_map->y_mode[i], counts->y_mode[i],
		       sizeof(counts_map->y_mode[0]));
	counts_helper->y_mode = &counts_map->y_mode;

	for (i = 0; i < ARRAY_SIZE(counts_map->filter); i++)
		memcpy(counts_map->filter[i], counts->switchable_interp[i],
		       sizeof(counts_map->filter[0]));
	counts_helper->filter = &counts_map->filter;

	for (i = 0; i < ARRAY_SIZE(counts_map->sign); i++)
		memcpy(counts_map->sign[i], counts->mvcomp[i].sign,
		       sizeof(counts_map->sign[0]));
	counts_helper->sign = &counts_map->sign;

	for (i = 0; i < ARRAY_SIZE(counts_map->classes); i++)
		memcpy(counts_map->classes[i], counts->mvcomp[i].classes,
		       sizeof(counts_map->classes[0]));
	counts_helper->classes = &counts_map->classes;

	for (i = 0; i < ARRAY_SIZE(counts_map->class0); i++)
		memcpy(counts_map->class0[i], counts->mvcomp[i].class0,
		       sizeof(counts_map->class0[0]));
	counts_helper->class0 = &counts_map->class0;

	for (i = 0; i < ARRAY_SIZE(counts_map->bits); i++)
		for (j = 0; j < ARRAY_SIZE(counts_map->bits[0]); j++)
			memcpy(counts_map->bits[i][j], counts->mvcomp[i].bits[j],
			       sizeof(counts_map->bits[0][0]));
	counts_helper->bits = &counts_map->bits;

	for (i = 0; i < ARRAY_SIZE(counts_map->class0_fp); i++)
		for (j = 0; j < ARRAY_SIZE(counts_map->class0_fp[0]); j++)
			memcpy(counts_map->class0_fp[i][j], counts->mvcomp[i].class0_fp[j],
			       sizeof(counts_map->class0_fp[0][0]));
	counts_helper->class0_fp = &counts_map->class0_fp;

	for (i = 0; i < ARRAY_SIZE(counts_map->fp); i++)
		memcpy(counts_map->fp[i], counts->mvcomp[i].fp,
		       sizeof(counts_map->fp[0]));
	counts_helper->fp = &counts_map->fp;

	for (i = 0; i < ARRAY_SIZE(counts_map->class0_hp); i++)
		memcpy(counts_map->class0_hp[i], counts->mvcomp[i].class0_hp,
		       sizeof(counts_map->class0_hp[0]));
	counts_helper->class0_hp = &counts_map->class0_hp;

	for (i = 0; i < ARRAY_SIZE(counts_map->hp); i++)
		memcpy(counts_map->hp[i], counts->mvcomp[i].hp, sizeof(counts_map->hp[0]));

	counts_helper->hp = &counts_map->hp;

	for (i = 0; i < 4; i++)
		for (j = 0; j < 2; j++)
			for (k = 0; k < 2; k++)
				vdec_vp9_slice_map_counts_eob_coef(i, j, k, counts, counts_helper);
}
