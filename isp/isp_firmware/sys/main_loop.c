/****************************************************************************
 * apps/external/isp/isp_firmware/main_loop.c
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

#include "nuttx/irq.h"
#include "defines.h"
#include "main_loop.h"
#include "intr.h"
#include "isp3a.h"

global_control_t *global_control[PIPE_NUM] = {
	[0] = (global_control_t *)(FW_ISP_SETTING_BASE + 0 * FW_ISP_SETTING_OFFSET),
	[1] = (global_control_t *)(FW_ISP_SETTING_BASE + 1 * FW_ISP_SETTING_OFFSET),
	[2] = (global_control_t *)(FW_ISP_SETTING_BASE + 2 * FW_ISP_SETTING_OFFSET),
	[3] = (global_control_t *)(FW_ISP_SETTING_BASE + 3 * FW_ISP_SETTING_OFFSET),
};

middle_group_t *middle_group[PIPE_NUM];

int sof_process(int pipe_id)
{
	syslog(LOG_INFO, "%s for pipeline %d\n", __func__, pipe_id);
	return 0;
}

int eof_process(int pipe_id)
{
	syslog(LOG_INFO, "%s for pipeline %d\n", __func__, pipe_id);
//	lenc_apply_process(pipe_id);
	ccm_process(pipe_id);
	gamma_process(pipe_id);
	return 0;
}

int awb_done_process(int pipe_id)
{
	syslog(LOG_INFO, "%s for pipeline %d\n", __func__, pipe_id);
	awb_read_process(pipe_id);
//	awb_calc_process(pipe_id);
	return 0;
}

int ae_done_process(int pipe_id)
{
	syslog(LOG_INFO, "%s for pipeline %d\n", __func__, pipe_id);
	aecgc_read_process(pipe_id);
	aecgc_calc_process(pipe_id);
	return 0;
}

void *isp_main_loop(void *arg)
{
	int pipe_id = 0;
	while (1) {
		sem_wait(&sem_mainloop);
		for (pipe_id = 0; pipe_id < PIPE_NUM; pipe_id++) {
			if (g_sof[pipe_id] == 1) {
				g_sof[pipe_id] = 0;
				sof_process(pipe_id);
			}

			if (g_eof[pipe_id] == 1) {
				g_eof[pipe_id] = 0;
				eof_process(pipe_id);
			}

			if (g_awb_done[pipe_id] == 1) {
				g_awb_done[pipe_id] = 0;
				awb_done_process(pipe_id);
			}

			if (g_ae_done[pipe_id] == 1) {
				g_ae_done[pipe_id] = 0;
				ae_done_process(pipe_id);
			}
		}
	}

}


