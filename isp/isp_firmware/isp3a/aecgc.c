/****************************************************************************
 * apps/external/isp/isp_firmware/isp3a/aecgc.c
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

#include "defines.h"
#include "main_loop.h"
#include "isp3a.h"

int aecgc_read_process(int pipe_id)
{
	int i = 0;
	for (i = 0; i < 22; i++) {
		middle_group[pipe_id]->r_hist[i] = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AE_R_HIST_BASE + i*4);
		middle_group[pipe_id]->gr_hist[i] = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AE_GR_HIST_BASE + i*4);
		middle_group[pipe_id]->gb_hist[i] = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AE_GB_HIST_BASE + i*4);
		middle_group[pipe_id]->b_hist[i] = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AE_B_HIST_BASE + i*4);

	}

	for (i = 0; i < 16; i++) {
		middle_group[pipe_id]->r_bri[i] = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AE_R_BRI_BASE + i*4);
		middle_group[pipe_id]->gr_bri[i] = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AE_GR_BRI_BASE + i*4);
		middle_group[pipe_id]->gb_bri[i] = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AE_GB_BRI_BASE + i*4);
		middle_group[pipe_id]->b_bri[i] = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AE_B_BRI_BASE + i*4);

	}

	return 0;
}

int get_mean_y(int pipe_id)
{
	int i;
	unsigned long long ae_r_sum = 0, ae_gr_sum = 0, ae_gb_sum = 0, ae_b_sum = 0;
	int ae_stat_r = 0, ae_stat_gr = 0, ae_stat_gb = 0, ae_stat_b = 0;
	uint32_t ae_grid_h_start, ae_grid_v_start, ae_grid_h_end, ae_grid_v_end;
	int ae_stat_size;

	for (i = 0; i < 16; i++) {
		ae_r_sum += middle_group[pipe_id]->r_bri[i] * global_control[pipe_id]->ae_weight[i] / 0x10;
		ae_gr_sum += middle_group[pipe_id]->gr_bri[i] * global_control[pipe_id]->ae_weight[i] / 0x10;
		ae_gb_sum += middle_group[pipe_id]->gb_bri[i] * global_control[pipe_id]->ae_weight[i] / 0x10;
		ae_b_sum += middle_group[pipe_id]->b_bri[i] * global_control[pipe_id]->ae_weight[i] / 0x10;
	}

	ae_grid_h_start = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AE_STAT_H_START);
	ae_grid_v_start = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AE_STAT_V_START);
	ae_grid_h_end = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AE_STAT_H_END);
	ae_grid_v_end = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_AE_STAT_V_END);

	ae_stat_size = (ae_grid_h_end - ae_grid_h_start) * (ae_grid_v_end - ae_grid_v_start);

	ae_stat_r = ae_r_sum / ae_stat_size;
	ae_stat_gr = ae_gr_sum / ae_stat_size;
	ae_stat_gb = ae_gb_sum / ae_stat_size;
	ae_stat_b = ae_b_sum / ae_stat_size;

	middle_group[pipe_id]->meany = (299 * ae_stat_r + 587 * ((ae_stat_gr + ae_stat_gb) / 2) + 114 * ae_stat_b) / 1000;

	return 0;
}

int aecgc_estimate(int pipe_id)
{
	uint32_t target = global_control[pipe_id]->ae_target;
	uint32_t stable_in = global_control[pipe_id]->ae_stable_range_in;
	uint32_t stable_out = global_control[pipe_id]->ae_stable_range_out;
	uint32_t mean_y = middle_group[pipe_id]->meany, new_meany;
	int32_t fast_step, slow_step, delta_meany = 0;
	uint32_t current_exp_idx = middle_group[pipe_id]->exp_idx;

	if ((middle_group[pipe_id]->ae_stable_flag == 1) &&
			(abs(mean_y - target) > stable_out)) {
		middle_group[pipe_id]->ae_stable_flag = 0;
	}

	if ((middle_group[pipe_id]->ae_stable_flag == 0) &&
			(abs(mean_y - target) < stable_in)) {
		middle_group[pipe_id]->ae_stable_flag = 1;
	}

	if (middle_group[pipe_id]->ae_stable_flag == 0) {
		fast_step = abs(mean_y - target) * global_control[pipe_id]->ae_coarse_step >> 6;
		slow_step = abs(mean_y - target) * global_control[pipe_id]->ae_fine_step >> 6;
		fast_step = max(fast_step, 4);
		slow_step = max(slow_step, 2);

		if (mean_y < target - stable_out) {
			/* lower case */
			if (mean_y + fast_step < target - stable_in) {
				delta_meany = fast_step;
			} else if (mean_y + fast_step > target + stable_in) {
				delta_meany = slow_step;
			} else {
				delta_meany = slow_step;
			}
		} else if (mean_y > target + stable_out) {
			/* higher case */
			if (mean_y - fast_step > target + stable_in) {
				delta_meany = -fast_step;
			} else if (mean_y - fast_step < target - stable_in) {
				delta_meany = -slow_step;
			} else {
				delta_meany = -slow_step;
			}
		}
		middle_group[pipe_id]->new_exp_idx_flag = 1;
	}

	new_meany = mean_y + delta_meany;
	middle_group[pipe_id]->new_exp_idx = current_exp_idx * new_meany / mean_y;

	return 0;
}

int aecgc_calc_process(int pipe_id)
{
	get_mean_y(pipe_id);
	aecgc_estimate(pipe_id);

	return 0;
}
int aecgc_apply_process(int pipe_id)
{
	/* fill expo table logic here */

	middle_group[pipe_id]->new_exp_idx_flag = 0;
	/* write exp & gain to sensor */

	return 0;
}

