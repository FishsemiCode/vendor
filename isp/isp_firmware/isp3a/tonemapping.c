/****************************************************************************
 * apps/external/isp/isp_firmware/isp3a/tonemapping.c
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

/* note: the calc consumption of convert 46-seg curve to 1024-seg curve seems
 * a little large. For low resolution and high frame rate case, when gamma process
 * is called at ae done or eof, there maybe is no enough time, so it is a alternative
 * option to call this process at sof */

int gamma_process(int pipe_id)
{
	int i = 0, j = 0;
	int mismatch_num = 0;

	/* 46 segment curve convert logic */
	uint16_t *xs = (uint16_t *)global_control[pipe_id]->gamma_index;
	uint16_t *ys_low = (uint16_t *)global_control[pipe_id]->gamma_val_low;
	uint16_t *ys_mid = (uint16_t *)global_control[pipe_id]->gamma_val_mid;
	uint16_t *ys_high = (uint16_t *)global_control[pipe_id]->gamma_val_high;
	uint16_t *ys = (uint16_t *)malloc(sizeof(uint16_t) * 46);

	uint8_t trans_0 = global_control[pipe_id]->gamma_bri_trans_band0;
	uint8_t trans_1 = global_control[pipe_id]->gamma_bri_trans_band1;
	uint32_t th_0 = global_control[pipe_id]->gamma_bri_th0;
	uint32_t th_1 = th_0 + th_0 * trans_0 / 0x20;
	uint32_t th_2 = global_control[pipe_id]->gamma_bri_th1;
	uint32_t th_3 = th_2 + th_2 * trans_1 / 0x20;
	uint32_t exp_idx = middle_group[pipe_id]->exp_idx;

	uint16_t *output = (uint16_t *)malloc(sizeof(uint16_t) * 1024);
	uint16_t qout;
	uint32_t w_attr, r_attr;

	for (i = 0; i < 46; i++) {
		if (exp_idx < th_0) {
			ys[i] = ys_high[i];
		} else if (exp_idx < th_1) {
			ys[i] = interp(exp_idx, th_0, th_1, ys_high[i], ys_mid[i]);
		} else if (exp_idx < th_2) {
			ys[i] = ys_mid[i];
		} else if (exp_idx < th_3) {
			ys[i] = interp(exp_idx, th_2, th_3, ys_mid[i], ys_low[i]);
		} else {
			ys[i] = ys_low[i];
		}
	}

	for (i = 0; i < 1024; i++) {
		if (i < xs[0]) {
			output[i] = interp(i, 0, xs[0], 0, ys[0]);
		} else if (i >= xs[45]) {
			output[i] = interp(i, xs[45], 1024, ys[45], 1024);
		} else {
			for (j = 0; j < 45; j++) {
				if ((i >= xs[j]) && (i < xs[j+1])) {
					output[i] = interp(i, xs[j], xs[j+1], ys[j], ys[j+1]);
				}
			}
		}
		output[i] = output[i] & 0x3ff;

		w_attr = (output[i] << 10) | i;
		setreg32(ISP1_BASE + pipe_id * ISP_BASE_OFFSET + REG_GAMMA_WRITE_ADDR, w_attr);
		setreg32(ISP1_BASE + pipe_id * ISP_BASE_OFFSET + REG_GAMMA_WRITE_ADDR, w_attr);

		r_attr = i;
		setreg32(ISP1_BASE + pipe_id * ISP_BASE_OFFSET + REG_GAMMA_QOUT_ADDR, r_attr);
		r_attr = getreg32(ISP1_BASE + pipe_id * ISP_BASE_OFFSET + REG_GAMMA_QOUT_ADDR);
		r_attr = getreg32(ISP1_BASE + pipe_id * ISP_BASE_OFFSET + REG_GAMMA_QOUT_ADDR);
		qout = (r_attr >> 10) & 0x3ff;
		if (qout != output[i])
			mismatch_num++;
	}

	if (mismatch_num != 0)
		syslog(LOG_INFO, "%d mismatch between input and qout in %s\n", mismatch_num, __func__);

	if (output != NULL) {
		free(output);
		output = NULL;
	}
	if (ys != NULL) {
		free(output);
		output = NULL;
	}

	return 0;
}

int stretch_process(int pipe_id)
{
	return 0;
}
