/****************************************************************************
 * apps/external/isp/isp_firmware/commandset.c
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
#include "intr.h"
#include "nuttx/arch.h"
#include "commandset.h"
#include "main_loop.h"

int initialize_hw(int pipe_id)
{
	setreg32(ISP1_BASE + pipe_id * ISP_BASE_OFFSET + REG_CC_EN, 1);
	setreg32(ISP1_BASE + pipe_id * ISP_BASE_OFFSET + REG_GAMMA_EN, global_control[pipe_id]->gamma_en);

	return 0;
}

int initialize(void)
{
	syslog(LOG_INFO, "%s\n", __func__);
	int i = 0;

	for (i = 0; i < PIPE_NUM; i++) {
		middle_group[i] = (middle_group_t *)malloc(sizeof(middle_group_t));
	}

	for (i = 0; i <256; i++) {
		global_control[0]->lsc_r_coef[i] = 255 - i;
		global_control[0]->lsc_g_coef[i] = 255 - i;
		global_control[0]->lsc_b_coef[i] = 255 - i;

	}

	for (i = 0; i < 16; i++) {
		global_control[0]->ae_weight[i] = 0x10;
	}

	for (i = 0; i < 3; i++) {
		global_control[0]->cc_coef[i][0] = 0x100;
		global_control[0]->cc_coef[i][1] = 0;
		global_control[0]->cc_coef[i][2] = 0;
		global_control[0]->cc_coef[i][3] = 0;
		global_control[0]->cc_coef[i][4] = 0x100;
		global_control[0]->cc_coef[i][5] = 0;
		global_control[0]->cc_coef[i][6] = 0;
		global_control[0]->cc_coef[i][7] = 0;
		global_control[0]->cc_coef[i][8] = 0x100;
	}

	global_control[0]->gamma_en = 1;
	i = 0;
	global_control[0]->gamma_index[i++] = 0x004;
	global_control[0]->gamma_index[i++] = 0x008;
	global_control[0]->gamma_index[i++] = 0x00C;
	global_control[0]->gamma_index[i++] = 0x010;
	global_control[0]->gamma_index[i++] = 0x014;
	global_control[0]->gamma_index[i++] = 0x018;
	global_control[0]->gamma_index[i++] = 0x01C;
	global_control[0]->gamma_index[i++] = 0x020;
	global_control[0]->gamma_index[i++] = 0x024;
	global_control[0]->gamma_index[i++] = 0x028;
	global_control[0]->gamma_index[i++] = 0x02C;
	global_control[0]->gamma_index[i++] = 0x030;
	global_control[0]->gamma_index[i++] = 0x034;
	global_control[0]->gamma_index[i++] = 0x038;
	global_control[0]->gamma_index[i++] = 0x03C;
	global_control[0]->gamma_index[i++] = 0x040;
	global_control[0]->gamma_index[i++] = 0x060;
	global_control[0]->gamma_index[i++] = 0x080;
	global_control[0]->gamma_index[i++] = 0x0A0;
	global_control[0]->gamma_index[i++] = 0x0C0;
	global_control[0]->gamma_index[i++] = 0x0E0;
	global_control[0]->gamma_index[i++] = 0x100;
	global_control[0]->gamma_index[i++] = 0x120;
	global_control[0]->gamma_index[i++] = 0x140;
	global_control[0]->gamma_index[i++] = 0x160;
	global_control[0]->gamma_index[i++] = 0x180;
	global_control[0]->gamma_index[i++] = 0x1A0;
	global_control[0]->gamma_index[i++] = 0x1C0;
	global_control[0]->gamma_index[i++] = 0x1E0;
	global_control[0]->gamma_index[i++] = 0x200;
	global_control[0]->gamma_index[i++] = 0x220;
	global_control[0]->gamma_index[i++] = 0x240;
	global_control[0]->gamma_index[i++] = 0x260;
	global_control[0]->gamma_index[i++] = 0x280;
	global_control[0]->gamma_index[i++] = 0x2A0;
	global_control[0]->gamma_index[i++] = 0x2C0;
	global_control[0]->gamma_index[i++] = 0x2E0;
	global_control[0]->gamma_index[i++] = 0x300;
	global_control[0]->gamma_index[i++] = 0x320;
	global_control[0]->gamma_index[i++] = 0x340;
	global_control[0]->gamma_index[i++] = 0x360;
	global_control[0]->gamma_index[i++] = 0x380;
	global_control[0]->gamma_index[i++] = 0x3A0;
	global_control[0]->gamma_index[i++] = 0x3C0;
	global_control[0]->gamma_index[i++] = 0x3E0;
	global_control[0]->gamma_index[i++] = 0x3FF;

	i = 0;
	global_control[0]->gamma_val_low[i++] = 0x00C;
	global_control[0]->gamma_val_low[i++] = 0x018;
	global_control[0]->gamma_val_low[i++] = 0x020;
	global_control[0]->gamma_val_low[i++] = 0x02C;
	global_control[0]->gamma_val_low[i++] = 0x038;
	global_control[0]->gamma_val_low[i++] = 0x044;
	global_control[0]->gamma_val_low[i++] = 0x04C;
	global_control[0]->gamma_val_low[i++] = 0x058;
	global_control[0]->gamma_val_low[i++] = 0x064;
	global_control[0]->gamma_val_low[i++] = 0x070;
	global_control[0]->gamma_val_low[i++] = 0x074;
	global_control[0]->gamma_val_low[i++] = 0x080;
	global_control[0]->gamma_val_low[i++] = 0x08C;
	global_control[0]->gamma_val_low[i++] = 0x098;
	global_control[0]->gamma_val_low[i++] = 0x0A0;
	global_control[0]->gamma_val_low[i++] = 0x0AC;
	global_control[0]->gamma_val_low[i++] = 0x110;
	global_control[0]->gamma_val_low[i++] = 0x174;
	global_control[0]->gamma_val_low[i++] = 0x1C4;
	global_control[0]->gamma_val_low[i++] = 0x210;
	global_control[0]->gamma_val_low[i++] = 0x24C;
	global_control[0]->gamma_val_low[i++] = 0x278;
	global_control[0]->gamma_val_low[i++] = 0x29C;
	global_control[0]->gamma_val_low[i++] = 0x2BC;
	global_control[0]->gamma_val_low[i++] = 0x2D8;
	global_control[0]->gamma_val_low[i++] = 0x2F0;
	global_control[0]->gamma_val_low[i++] = 0x308;
	global_control[0]->gamma_val_low[i++] = 0x320;
	global_control[0]->gamma_val_low[i++] = 0x334;
	global_control[0]->gamma_val_low[i++] = 0x344;
	global_control[0]->gamma_val_low[i++] = 0x354;
	global_control[0]->gamma_val_low[i++] = 0x364;
	global_control[0]->gamma_val_low[i++] = 0x370;
	global_control[0]->gamma_val_low[i++] = 0x37C;
	global_control[0]->gamma_val_low[i++] = 0x388;
	global_control[0]->gamma_val_low[i++] = 0x394;
	global_control[0]->gamma_val_low[i++] = 0x3A0;
	global_control[0]->gamma_val_low[i++] = 0x3AC;
	global_control[0]->gamma_val_low[i++] = 0x3B8;
	global_control[0]->gamma_val_low[i++] = 0x3C0;
	global_control[0]->gamma_val_low[i++] = 0x3CC;
	global_control[0]->gamma_val_low[i++] = 0x3D8;
	global_control[0]->gamma_val_low[i++] = 0x3E0;
	global_control[0]->gamma_val_low[i++] = 0x3EC;
	global_control[0]->gamma_val_low[i++] = 0x3F4;
	global_control[0]->gamma_val_low[i++] = 0x3FF;
	for (i = 0; i < 46; i++) {
		global_control[0]->gamma_val_mid[i] = global_control[0]->gamma_val_low[i];
		global_control[0]->gamma_val_high[i] = global_control[0]->gamma_val_low[i];
	}

	return 0;
}

int deinit(void)
{
	int i = 0;

	for (i = 0; i < PIPE_NUM; i++) {
		if (middle_group[i] != NULL) {
			free(middle_group[i]);
			middle_group[i] = NULL;
		}
	}

	return 0;
}

int set_format(stream_fmt_t *fmt)
{
	int port_index =0;
	int pipe_index = fmt->pipe_id;

	if (fmt->str_on_off_flag == true) {
		/* stream on case */
		/* initialize hw according to pipe id */
		initialize_hw(pipe_index);

		/* set isp soft rst */
		setreg32(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_SOFT_RSTN, 0);

		if (fmt->input_type == SRC_COLORBAR) {
			setreg32(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_ISP_COLORBAR, 0x08);
			setreg32(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_RD_PORT_WIDTH, fmt->pipe_width);
			setreg32(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_RD_PORT_HEIGHT, fmt->pipe_height);
			/* it is better to set a largr vb, else the gap between eof and next sof is to small */
			setreg32(ISP1_BASE + REG_RD_PORT_VB, 0x100);
		} else {
			setreg32(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_ISP_COLORBAR, 0x00);
		}

		/* enable intr */
		up_enable_irq(pipe_index);
		setreg32(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + INT_SOF_EOF_EN, ((1 << BIT_EOF) | (1 << BIT_SOF)));
		setreg32(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + INT_STAT_DONE_EN, ((1 << BIT_AE_DONE) | (1 << BIT_AWB_DONE)));

		/* set size */
		setreg32_mask(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_ISP_WIDTH, fmt->pipe_width, 0x7ff);
		setreg32_mask(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_ISP_HEIGHT, fmt->pipe_height, 0x7ff);

		for (port_index = 0; port_index < PORT_NUM; port_index++) {
			if (fmt->port_fmt[port_index].port_en) {
				setreg32_mask(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_PORT0_CTRL + port_index * PORT_OFFSET, 0x07, 0x07);
				setreg32_mask(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_PORT0_WIDTH + port_index * PORT_OFFSET, fmt->port_fmt[port_index].width, 0x7ff);
				setreg32_mask(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_PORT0_HEIGHT + port_index * PORT_OFFSET, fmt->port_fmt[port_index].height, 0x7ff);

			} else {
				setreg32_mask(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_PORT0_CTRL + port_index * PORT_OFFSET, 0x00, 0x07);
			}
		}
		setreg32(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_SOFT_RSTN, 1);

	} else {
		/* stream off case */
		/* disable isp intr */
		setreg32(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + INT_SOF_EOF_EN, 0);
		setreg32(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + INT_STAT_DONE_EN, 0);

		up_disable_irq(pipe_index);

		/* disable output ports */
		for (port_index = 0; port_index < PORT_NUM; port_index++) {
			setreg32_mask(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_PORT0_CTRL + port_index * PORT_OFFSET, 0x00, 0x07);
		}

		/* set isp soft rst */
		setreg32(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_SOFT_RSTN, 0);

	}

	return 0;
}
