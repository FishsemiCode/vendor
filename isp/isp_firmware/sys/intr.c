/****************************************************************************
 * apps/external/isp/isp_firmware/intr.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>

#include "nuttx/irq.h"
#include "nuttx/arch.h"
#include "defines.h"
#include "intr.h"

unsigned int isp_frame_irq[PIPE_NUM];
unsigned int isp_stat_irq[PIPE_NUM];
unsigned int g_ispctl_irq;

unsigned int g_sof[PIPE_NUM];
unsigned int g_eof[PIPE_NUM];
unsigned int g_awb_done[PIPE_NUM];
unsigned int g_ae_done[PIPE_NUM];

irq_id_t irq_id_array[] = {
	ISP1_IRQ,
	ISP2_IRQ,
	ISP3_IRQ,
	ISP4_IRQ,
	ISPCTL_IRQ
};

int (*isp_isr[])(int, void *, void *) = {
	isp1_isr,
	isp2_isr,
	isp3_isr,
	isp4_isr,
	ispctl_isr,
};

int isp_isr_process(int irq, void *data, void *arg)
{
	int addr, val;
	switch (irq) {
	case ISP1_IRQ:
	case ISP2_IRQ:
	case ISP3_IRQ:
	case ISP4_IRQ:
		addr = ISP1_BASE + irq * ISP_BASE_OFFSET + INT_SOF_EOF;
		val = getreg32(addr);
		isp_frame_irq[irq] |= val;
		setreg32(addr, val & 0x3);

		addr = ISP1_BASE + irq * ISP_BASE_OFFSET + INT_STAT_DONE;
		val = getreg32(addr);
		isp_stat_irq[irq] |= val;
		setreg32(addr, val & 0x3);

		sem_post(&sem_irq_listener);
		break;
	case ISPCTL_IRQ:
		addr = INT_ISPCTL;
		val = getreg32(addr);
		g_ispctl_irq |= val;
		setreg32(addr, val);

		sem_post(&sem_cmd_listener);
		break;
	default:
		break;
	}
	return 0;
}

int isp1_isr(int irq, void *data, void *arg)
{
	return isp_isr_process(irq, data, arg);
}

int isp2_isr(int irq, void *data, void *arg)
{
	return isp_isr_process(irq, data, arg);
}

int isp3_isr(int irq, void *data, void *arg)
{
	return isp_isr_process(irq, data, arg);
}

int isp4_isr(int irq, void *data, void *arg)
{
	return isp_isr_process(irq, data, arg);
}

int ispctl_isr(int irq, void *data, void *arg)
{
	return isp_isr_process(irq, data, arg);
}

int init_irqs(void)
{
	int i = 0;
	int irq_num = sizeof(irq_id_array) / sizeof(irq_id_array[0]);

	for (i = 0; i < irq_num; i++)
		irq_attach(irq_id_array[i], isp_isr[i], NULL);

	/* enable ispctl irq alone first */
	up_enable_irq(ISPCTL_IRQ);
	return 0;
}

void *isp_irq_listener_thread(void *arg)
{
	int i = 0;
	while (1) {
		sem_wait(&sem_irq_listener);
		for (i = 0; i < PIPE_NUM; i++) {
			if (isp_frame_irq[i] & 0x2) {
				g_eof[i] = 1;
				set_bit(isp_frame_irq[i], 1, 0);
			}
			if (isp_frame_irq[i] & 0x1) {
				g_sof[i] = 1;
				set_bit(isp_frame_irq[i], 0, 0);
			}
			if (isp_stat_irq[i] & 0x2) {
				g_ae_done[i] = 1;
				set_bit(isp_stat_irq[i], 1, 0);
			}
			if (isp_stat_irq[i] & 0x1) {
				g_awb_done[i] = 1;
				set_bit(isp_stat_irq[i], 0, 0);
			}
		}
		sem_post(&sem_mainloop);
	}

}


