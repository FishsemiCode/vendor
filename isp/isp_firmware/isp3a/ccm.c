/****************************************************************************
 * apps/external/isp/isp_firmware/isp3a/ccm.c
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

int ccm_process(int pipe_id)
{
	int i = 0, m1, m2, m3, n1, n2, n3;
	int16_t cc[9], cc_boost[9];
	uint16_t ct = middle_group[pipe_id]->current_ct;
	uint16_t ct_0 = global_control[pipe_id]->cc_ct_th_0;
	uint16_t ct_1 = global_control[pipe_id]->cc_ct_th_1;
	uint16_t ct_2 = global_control[pipe_id]->cc_ct_th_2;
	uint16_t level_0 = global_control[pipe_id]->cc_level_0;
	uint16_t level_1 = global_control[pipe_id]->cc_level_1;
	uint16_t level = 0;
	uint32_t bri_0 = global_control[pipe_id]->cc_bri_th0;
	uint32_t bri_1 = global_control[pipe_id]->cc_bri_th1;
	int16_t *cc0 = global_control[pipe_id]->cc_coef[0];
	int16_t *cc1 = global_control[pipe_id]->cc_coef[1];
	int16_t *cc2 = global_control[pipe_id]->cc_coef[2];

	for (i = 0; i < 9; i++) {
		if (ct < ct_0) {
			cc[i] = cc0[i];
		} else if (ct < ct_1) {
			cc[i] = interp(ct, ct_0, ct_1, cc0[i], cc1[i]);
		} else if (ct < ct_2) {
			cc[i] = interp(ct, ct_1, ct_2, cc1[i], cc2[i]);
		} else {
			cc[i] = cc2[i];
		}
	}

	if (middle_group[pipe_id]->exp_idx < bri_0)
		level = level_0;
	else if (middle_group[pipe_id]->exp_idx < bri_1)
		level = interp(middle_group[pipe_id]->exp_idx, bri_0, bri_1, level_0, level_1);
	else
		level = level_1;

	for (i = 0; i < 9; i++) {
		m1 = (i < 3) ? (384 + 640 * level / 0x80) :
			((640 - 640 * level / 0x80) / 2);
		m2 = ((i >= 3) && (i < 6)) ? (384 + 640 * level / 0x80) :
			((640 - 640 * level / 0x80) / 2);
		m3 = ((i >= 6)) ? (384 + 640 * level / 0x80) :
			((640 - 640 * level / 0x80) / 2);
		n1 = cc[i%3];
		n2 = cc[(i%3) + 3];
		n3 = cc[(i%3) + 6];

		cc_boost[i] = (m1 * n1 + m2 * n2 + m3 * n3) / 1024;
	}

	/* apply cc to hardware */
	for (i = 0; i < 9; i++) {
		setreg32(ISP1_BASE + ISP_BASE_OFFSET * pipe_id + REG_CC_00 + 4*i, cc_boost[i]);
	}
	return 0;
}
