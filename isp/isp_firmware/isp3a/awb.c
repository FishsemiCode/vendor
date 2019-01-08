/****************************************************************************
 * apps/external/isp/isp_firmware/isp3a/awb.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Pinecone <pinecone@pinecone.net>

 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <math.h>

#include "defines.h"
#include "main_loop.h"
#include "isp3a.h"

int awb_read_process(int pipe_id)
{
	int i = 0; 
	unsigned int attr0 = 0;

	for (i = 0; i < 256; i++) {
		attr0 = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AWB_RAM_ADDR);
		middle_group[pipe_id]->awb_sta_intr_wire = (attr0 >> 8) & 0xff;

		if (middle_group[pipe_id]->awb_sta_intr_wire == 1) {
			middle_group[pipe_id]->awb_ram_addr = i;
			setreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AWB_RAM_ADDR, middle_group[pipe_id]->awb_ram_addr);
			middle_group[pipe_id]->awb_ram_r_dout[i] = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AWB_RAM_R_DOUT);
			middle_group[pipe_id]->awb_ram_g_dout[i] = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AWB_RAM_G_DOUT);
			middle_group[pipe_id]->awb_ram_b_dout[i] = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AWB_RAM_B_DOUT);

		}
	}

	return 0;
}

int update_CTWB_weight(int pipe_id, uint8_t *weight)
{
	int i = 0;
	/* generate weight from global control according to exp info */
	/* for now just use low map alone */
	int exp_idx = middle_group[pipe_id]->exp_idx;
	uint8_t  map_trans_0 = global_control[pipe_id]->awb_map_trans_band0;
	uint8_t  map_trans_1 = global_control[pipe_id]->awb_map_trans_band1;

	uint32_t map_th0 = global_control[pipe_id]->awb_map_th0;
	uint32_t map_th1 = map_th0 + map_th0 * map_trans_0 / 0x20;
	uint32_t map_th2 = global_control[pipe_id]->awb_map_th1;
	uint32_t map_th3 = map_th2 + map_th2 * map_trans_1 / 0x20;

	uint8_t *awb_map_mid = (uint8_t *)malloc(sizeof(uint8_t) * 256);
	uint8_t *awb_map_high = (uint8_t *)malloc(sizeof(uint8_t) * 256);
	uint8_t *awb_map_low = (uint8_t *)global_control[pipe_id]->awb_map_low;
	uint8_t *high_mask = (uint8_t *)global_control[pipe_id]->awb_map_high_mask;
	uint8_t *mid_mask = (uint8_t *)global_control[pipe_id]->awb_map_middle_mask;

	for (i = 0; i < 256; i++) {
		awb_map_mid[i] = (mid_mask[i/8] & (1 << (i%8))) ? (awb_map_low[i]) : (0);
		awb_map_high[i] = (high_mask[i/8] & (1 << (i%8))) ? (awb_map_low[i]) : (0);
	}
	for (i = 0; i <  256; i++) {
		if (exp_idx < map_th0) {
			weight[i] = awb_map_high[i];
		} else if (exp_idx < map_th1) {
			weight[i] = interp(exp_idx, map_th0, map_th1, awb_map_high[i], awb_map_mid[i]);
		} else if (exp_idx < map_th2) {
			weight[i] = awb_map_mid[i];
		} else if (exp_idx < map_th3) {
			weight[i] = interp(exp_idx, map_th2, map_th3, awb_map_mid[i], awb_map_low[i]);
		} else {
			weight[i] = awb_map_low[i];
		}
	}
	if (awb_map_mid != NULL)
	{
		free(awb_map_mid);
		awb_map_mid = NULL;
	}
	if (awb_map_high != NULL)
	{
		free(awb_map_high);
		awb_map_high = NULL;
	}
	return 0;
}

int awb_shift(int pipe_id)
{
	int i = 0;
	uint8_t ct_r_coef[6], ct_b_coef[6], r_shift, b_shift;
	uint16_t min_gain;
	uint32_t exp_idx = middle_group[pipe_id]->exp_idx;
	uint16_t ct = middle_group[pipe_id]->current_ct;

	for (i = 0; i < 6; i++) {
		if (exp_idx < global_control[pipe_id]->awb_shift_br_th[0]) {
			ct_r_coef[i] = global_control[pipe_id]->awb_shift_r_coef[0][i];
			ct_b_coef[i] = global_control[pipe_id]->awb_shift_b_coef[0][i];

		} else if (exp_idx < global_control[pipe_id]->awb_shift_br_th[1]) {
			ct_r_coef[i] = interp(exp_idx,
					global_control[pipe_id]->awb_shift_br_th[0],
					global_control[pipe_id]->awb_shift_br_th[1],
					global_control[pipe_id]->awb_shift_r_coef[0][i],
					global_control[pipe_id]->awb_shift_r_coef[1][i]);
			ct_b_coef[i] = interp(exp_idx,
					global_control[pipe_id]->awb_shift_br_th[0],
					global_control[pipe_id]->awb_shift_br_th[1],
					global_control[pipe_id]->awb_shift_b_coef[0][i],
					global_control[pipe_id]->awb_shift_b_coef[1][i]);

		} else if (exp_idx < global_control[pipe_id]->awb_shift_br_th[2]) {
			ct_r_coef[i] = interp(exp_idx,
					global_control[pipe_id]->awb_shift_br_th[1],
					global_control[pipe_id]->awb_shift_br_th[2],
					global_control[pipe_id]->awb_shift_r_coef[1][i],
					global_control[pipe_id]->awb_shift_r_coef[2][i]);
			ct_b_coef[i] = interp(exp_idx,
					global_control[pipe_id]->awb_shift_br_th[1],
					global_control[pipe_id]->awb_shift_br_th[2],
					global_control[pipe_id]->awb_shift_b_coef[1][i],
					global_control[pipe_id]->awb_shift_b_coef[2][i]);

		} else if (exp_idx < global_control[pipe_id]->awb_shift_br_th[3]) {
			ct_r_coef[i] = interp(exp_idx,
					global_control[pipe_id]->awb_shift_br_th[2],
					global_control[pipe_id]->awb_shift_br_th[3],
					global_control[pipe_id]->awb_shift_r_coef[2][i],
					global_control[pipe_id]->awb_shift_r_coef[3][i]);
			ct_b_coef[i] = interp(exp_idx,
					global_control[pipe_id]->awb_shift_br_th[2],
					global_control[pipe_id]->awb_shift_br_th[3],
					global_control[pipe_id]->awb_shift_b_coef[2][i],
					global_control[pipe_id]->awb_shift_b_coef[3][i]);

		} else {
			ct_r_coef[i] = global_control[pipe_id]->awb_shift_r_coef[3][i];
			ct_b_coef[i] = global_control[pipe_id]->awb_shift_b_coef[3][i];

		}
	}

	if (ct < global_control[pipe_id]->awb_shift_ct_th[0]) {
		r_shift = ct_r_coef[0];
		b_shift = ct_b_coef[0];
	} else if (ct < global_control[pipe_id]->awb_shift_ct_th[1]) {
		r_shift = interp(ct,
				global_control[pipe_id]->awb_shift_ct_th[0],
				global_control[pipe_id]->awb_shift_ct_th[1],
				ct_r_coef[0],
				ct_r_coef[1]);
		b_shift = interp(ct,
				global_control[pipe_id]->awb_shift_ct_th[0],
				global_control[pipe_id]->awb_shift_ct_th[1],
				ct_b_coef[0],
				ct_b_coef[1]);

	} else if (ct < global_control[pipe_id]->awb_shift_ct_th[2]) {
		r_shift = interp(ct,
				global_control[pipe_id]->awb_shift_ct_th[1],
				global_control[pipe_id]->awb_shift_ct_th[2],
				ct_r_coef[1],
				ct_r_coef[2]);
		b_shift = interp(ct,
				global_control[pipe_id]->awb_shift_ct_th[1],
				global_control[pipe_id]->awb_shift_ct_th[2],
				ct_b_coef[1],
				ct_b_coef[2]);

	} else if (ct < global_control[pipe_id]->awb_shift_ct_th[3]) {
		r_shift = interp(ct,
				global_control[pipe_id]->awb_shift_ct_th[2],
				global_control[pipe_id]->awb_shift_ct_th[3],
				ct_r_coef[2],
				ct_r_coef[3]);
		b_shift = interp(ct,
				global_control[pipe_id]->awb_shift_ct_th[2],
				global_control[pipe_id]->awb_shift_ct_th[3],
				ct_b_coef[2],
				ct_b_coef[3]);

	} else if (ct < global_control[pipe_id]->awb_shift_ct_th[4]) {
		r_shift = interp(ct,
				global_control[pipe_id]->awb_shift_ct_th[3],
				global_control[pipe_id]->awb_shift_ct_th[4],
				ct_r_coef[3],
				ct_r_coef[4]);
		b_shift = interp(ct,
				global_control[pipe_id]->awb_shift_ct_th[3],
				global_control[pipe_id]->awb_shift_ct_th[4],
				ct_b_coef[3],
				ct_b_coef[4]);

	} else if (ct < global_control[pipe_id]->awb_shift_ct_th[5]) {
		r_shift = interp(ct,
				global_control[pipe_id]->awb_shift_ct_th[4],
				global_control[pipe_id]->awb_shift_ct_th[5],
				ct_r_coef[4],
				ct_r_coef[5]);
		b_shift = interp(ct,
				global_control[pipe_id]->awb_shift_ct_th[4],
				global_control[pipe_id]->awb_shift_ct_th[5],
				ct_b_coef[4],
				ct_b_coef[5]);

	} else {
		r_shift = ct_r_coef[5];
		b_shift = ct_b_coef[5];
	}

	middle_group[pipe_id]->b_gain = middle_group[pipe_id]->b_gain_before_shift * b_shift / 0x80;
	middle_group[pipe_id]->g_gain = middle_group[pipe_id]->g_gain_before_shift;
	middle_group[pipe_id]->r_gain = middle_group[pipe_id]->r_gain_before_shift * r_shift / 0x80;
	min_gain = min(middle_group[pipe_id]->b_gain, min(middle_group[pipe_id]->g_gain, middle_group[pipe_id]->r_gain));
	middle_group[pipe_id]->b_gain = middle_group[pipe_id]->b_gain * 0x80 / min_gain;
	middle_group[pipe_id]->g_gain = middle_group[pipe_id]->g_gain * 0x80 / min_gain;
	middle_group[pipe_id]->r_gain = middle_group[pipe_id]->r_gain * 0x80 / min_gain;

	return 0;
}

int awb_calc_process(int pipe_id)
{
	int i = 0, j = 0;
	unsigned long long sumR = 0, sumGR = 0, sumGB = 0, sumB = 0;
	int tempBGain, tempGGain, tempRGain, minWbGain = 100000;
	uint8_t *weight = middle_group[pipe_id]->awb_weight_map;

	update_CTWB_weight(pipe_id, weight);

	for (i =0; i < 16; i++) {
		for (j = 0; j < 16; j++) {
			sumB += (int)((pow(2, (float)weight[i*16 + j]) - 1) * (middle_group[pipe_id]->awb_ram_b_dout[i*16 + j]));
			sumGB += (int)((pow(2, (float)weight[i*16 + j]) - 1) * (middle_group[pipe_id]->awb_ram_g_dout[i*16 + j]));
			sumGR += (int)((pow(2, (float)weight[i*16 + j]) - 1) * (middle_group[pipe_id]->awb_ram_g_dout[i*16 + j]));
			sumR += (int)((pow(2, (float)weight[i*16 + j]) - 1) * (middle_group[pipe_id]->awb_ram_r_dout[i*16 + j]));
		}
	}
	tempBGain = 0x80 * sumGB / sumB;
	minWbGain = min(minWbGain, tempBGain);
	tempGGain = 0x80;
	minWbGain = min(minWbGain, tempGGain);
	tempRGain = 0x80 * sumGR / sumR;
	minWbGain = min(minWbGain, tempRGain);

	middle_group[pipe_id]->b_gain_before_shift = tempBGain;
	middle_group[pipe_id]->g_gain_before_shift = tempGGain;
	middle_group[pipe_id]->r_gain_before_shift = tempRGain;

	middle_group[pipe_id]->b_gain = tempBGain;
	middle_group[pipe_id]->g_gain = tempGGain;
	middle_group[pipe_id]->r_gain = tempRGain;

	middle_group[pipe_id]->current_ct = 0x100 * tempBGain / tempRGain;

	/* add awb shift logic here */
	if (global_control[pipe_id]->awb_shift_en)
		awb_shift(pipe_id);

	return 0;
}

int awb_apply_process(int pipe_id)
{
	/* apply awb gain */
	setreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AWB_R_GAIN, middle_group[pipe_id]->r_gain);
	setreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AWB_G_GAIN, middle_group[pipe_id]->g_gain);
	setreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AWB_B_GAIN, middle_group[pipe_id]->b_gain);

	return 0;
}
