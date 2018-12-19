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

int initialize(void)
{
	syslog(LOG_INFO, "%s\n", __func__);
	return 0;
}

int set_format(stream_fmt_t *fmt)
{
	int port_index =0;
	int pipe_index = fmt->pipe_id;

	if (fmt->str_on_off_flag == true) {
		/* stream on case */

		/* set isp soft rst */
		setreg32(ISP1_BASE + pipe_index * ISP_BASE_OFFSET + REG_SOFT_RSTN, 1);

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
