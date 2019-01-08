/****************************************************************************
 * apps/external/isp/isp_firmware/isp3a/lenc.c
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

int lenc_read_process(int pipe_id)
{
	return 0;
}
int lenc_calc_process(int pipe_id)
{
	return 0;
}
int lenc_apply_process(int pipe_id)
{
	int i = 0;
	uint32_t w_attr, r_attr;
	int r_mismatch_num = 0, g_mismatch_num = 0, b_mismatch_num = 0;
	bool if_show_mismatch_detail = false;
	for (i = 0; i < 256; i++) {
		w_attr = ((global_control[pipe_id]->lsc_r_coef[i] & 0xff) << 24) |
				((global_control[pipe_id]->lsc_g_coef[i] & 0xff) << 16) |
				((global_control[pipe_id]->lsc_b_coef[i] & 0xff) << 8) |
				((i & 0xff));

		setreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_LENC_COEF_ATTR, w_attr);
		r_attr = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_LENC_COEF_RD_ATTR);
		r_attr = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_LENC_COEF_RD_ATTR);
////	r_attr = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_LENC_COEF_RD_ATTR);
////	r_attr = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_LENC_COEF_RD_ATTR);
////	r_attr = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_LENC_COEF_RD_ATTR);
////	r_attr = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_LENC_COEF_RD_ATTR);
////	r_attr = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_LENC_COEF_RD_ATTR);
////	r_attr = getreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_LENC_COEF_RD_ATTR);

		middle_group[pipe_id]->lsc_r_coef[i] = (r_attr >> 16) & 0xff;
		middle_group[pipe_id]->lsc_g_coef[i] = (r_attr >> 8) & 0xff;
		middle_group[pipe_id]->lsc_b_coef[i] = (r_attr >> 0) & 0xff;

		if (middle_group[pipe_id]->lsc_r_coef[i] !=
				global_control[pipe_id]->lsc_r_coef[i]) {
			r_mismatch_num++;
			if (if_show_mismatch_detail == true)
				syslog(LOG_INFO, "global lsc_r_coef = 0x%x, readout lsc_r_coef = 0x%x, at index = %d\n", global_control[pipe_id]->lsc_r_coef[i], middle_group[pipe_id]->lsc_r_coef[i], i );
		}
		if (middle_group[pipe_id]->lsc_g_coef[i] !=
				global_control[pipe_id]->lsc_g_coef[i]) {
			g_mismatch_num++;
			if (if_show_mismatch_detail == true)
				syslog(LOG_INFO, "global lsc_g_coef = 0x%x, readout lsc_g_coef = 0x%x, at index = %d\n", global_control[pipe_id]->lsc_g_coef[i], middle_group[pipe_id]->lsc_g_coef[i], i );
		}
		if (middle_group[pipe_id]->lsc_b_coef[i] !=
				global_control[pipe_id]->lsc_b_coef[i]) {
			b_mismatch_num++;
			if (if_show_mismatch_detail == true)
				syslog(LOG_INFO, "global lsc_b_coef = 0x%x, readout lsc_b_coef = 0x%x, at index = %d\n", global_control[pipe_id]->lsc_b_coef[i], middle_group[pipe_id]->lsc_b_coef[i], i );
		}

	}

	if (if_show_mismatch_detail == false) {
		syslog(LOG_INFO, "%d mismatches in r, %d mismatches in g, %d mismatches in b\n", r_mismatch_num, g_mismatch_num, b_mismatch_num);
		r_mismatch_num = 0;
		g_mismatch_num = 0;
		b_mismatch_num = 0;

	}

	return 0;
}


