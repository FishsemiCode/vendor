/****************************************************************************
 * apps/external/isp/isp_hello/isp_hello_main.c
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
#include "isp_irq_def.h"

#define setreg32(addr, val)		(*(volatile uint32_t *)(addr) = (uint32_t)(val))
#define getreg32(addr)			(*(volatile uint32_t *)(addr))
#define setreg16(addr, val)		(*(volatile uint16_t *)(addr) = (uint16_t)(val))
#define getreg16(addr)			(*(volatile uint16_t *)(addr))
#define setreg8(addr, val)		(*(volatile uint8_t *)(addr) = (uint8_t)(val))
#define getreg8(addr)			(*(volatile uint8_t *)(addr))

#define set_bit(n, bit, x)	(n = (n & ~(1 << bit)) | ((x << bit)))
int isp_isr_process(int irq, void *data, void *arg);
int isp1_isr(int irq, void *data, void *arg);
int isp2_isr(int irq, void *data, void *arg);
int isp3_isr(int irq, void *data, void *arg);
int isp4_isr(int irq, void *data, void *arg);
int ispctl_isr(int irq, void *data, void *arg);

typedef enum {
	ISP1_IRQ = 0,
	ISP2_IRQ,
	ISP3_IRQ,
	ISP4_IRQ,
	ISPCTL_IRQ = 27
} irq_id_t;

irq_id_t irq_id_array[] = {
	ISP1_IRQ,
	ISP2_IRQ,
	ISP3_IRQ,
	ISP4_IRQ,
	ISPCTL_IRQ
};

static int (*isp_isr[])(int, void *, void *) = {
	isp1_isr,
	isp2_isr,
	isp3_isr,
	isp4_isr,
	ispctl_isr,
};

unsigned int g_frame_irq[4];
unsigned int g_stat_irq[4];
unsigned int g_ispctl_irq;

int isp_isr_process(int irq, void *data,  void *arg)
{
	int addr, val;
	switch (irq) {
	case ISP1_IRQ:
	case ISP2_IRQ:
	case ISP3_IRQ:
	case ISP4_IRQ:
		addr = ISP1_BASE + irq * ISP_BASE_OFFSET + INT_SOF_EOF;
		val = getreg32(addr);
		g_frame_irq[irq] |= val;
		setreg32(addr, val & 0x3);

		addr = ISP1_BASE + irq * ISP_BASE_OFFSET + INT_STAT_DONE;
		val = getreg32(addr);
		g_stat_irq[irq] |= val;
		setreg32(addr, val & 0x3);
		break;

	case ISPCTL_IRQ:
		addr = INT_ISPCTL;
		val = getreg32(addr);
		g_ispctl_irq |= val;
		setreg32(addr, val); // the set value should be 'val', but some logic in ispctl is wrong for now
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

static int init_irqs(void)
{
	int i = 0;
	int irq_num = sizeof(irq_id_array) / sizeof(irq_id_array[0]);
	for (i = 0; i < irq_num; i++) {
		irq_attach(irq_id_array[i], isp_isr[i], NULL);
		up_enable_irq(irq_id_array[i]);
		syslog(LOG_INFO, "enable irq id = %d\n", irq_id_array[i]);
		if ((i >= ISP1_IRQ) && (i <= ISP4_IRQ)) {
			setreg32(ISP1_BASE + i* ISP_BASE_OFFSET + INT_SOF_EOF_EN, ((1 << BIT_EOF) | (1 << BIT_SOF)));
			setreg32(ISP1_BASE + i* ISP_BASE_OFFSET + INT_STAT_DONE_EN, ((1 << BIT_AE_DONE) | (1 << BIT_AWB_DONE)));
		}
	}
	return 0;
}

static int start_single_stream(void)
{
	unsigned int *y_addr = (unsigned int *)0x80000000;
	unsigned int *u_addr = (unsigned int *)0x80200000;
	unsigned int *v_addr = (unsigned int *)0x80400000;

	setreg32(ISP1_BASE + REG_ISP_WIDTH, 176);
	setreg32(ISP1_BASE + REG_ISP_HEIGHT, 144);
	setreg32(ISP1_BASE + REG_AWB_STAT_WIDTH, 176);
	setreg32(ISP1_BASE + REG_AWB_STAT_HEIGHT, 144);
	setreg32(ISP1_BASE + REG_AE_STAT_WIDTH, 176);
	setreg32(ISP1_BASE + REG_AE_STAT_HEIGHT, 144);
	setreg32(ISP1_BASE + REG_PORT0_WIDTH, 176);
	setreg32(ISP1_BASE + REG_PORT0_HEIGHT, 144);
	setreg32(ISP1_BASE + REG_PORT1_WIDTH, 176);
	setreg32(ISP1_BASE + REG_PORT1_HEIGHT, 144);
	setreg32(ISP1_BASE + REG_PORT2_WIDTH, 176);
	setreg32(ISP1_BASE + REG_PORT2_HEIGHT, 144);
	setreg32(ISP1_BASE + REG_PORT3_WIDTH, 176);
	setreg32(ISP1_BASE + REG_PORT3_HEIGHT, 144);
	setreg32(ISP1_BASE + REG_RD_PORT_WIDTH, 176);
	setreg32(ISP1_BASE + REG_RD_PORT_HEIGHT, 144);
	setreg32(ISP1_BASE + REG_RD_PORT_VB, 0x100);

	setreg32(0xfa4500c0, 0x05);
	setreg32(0xfa4501a0, 0x03);

	setreg32(0xfa450010, 0x08);
	setreg32(0xfa450800, 0x07);
	setreg32(0xfa450860, y_addr);
	setreg32(0xfa450864, u_addr);
	setreg32(0xfa450868, v_addr);

	/* enable isp ctl irq here */
	setreg32(INT_ISPCTL_EN, 0x01);
	setreg32(INT_ISPCTL_BIT_EN, 0xffff); // the set value should be 2<<INT_ISPCTL_EN, according to which int was needed, for test, enable all the intr bits

	return 0;

}

void *isp_irq_listener_thread(void *arg)
{
	int i = 0;
	while (1) {
		for (i = 0; i < 4; i++) {
			if (g_frame_irq[i] & 0x2) {
	//			syslog(LOG_INFO, "get eof of pipeline %d!\n", i);
				set_bit(g_frame_irq[i], 1, 0);
			}
			if (g_frame_irq[i] & 0x1) {
	//			syslog(LOG_INFO, "get sof of pipeline %d!\n", i);
				set_bit(g_frame_irq[i], 0, 0);
			}
			if (g_stat_irq[i] & 0x1) {
	//			syslog(LOG_INFO, "get awb done of pipeline %d!\n", i);
				set_bit(g_stat_irq[i], 0, 0);
			}
			if (g_stat_irq[i] & 0x2) {
	//			syslog(LOG_INFO, "get ae done of pipeline %d!\n", i);
				set_bit(g_stat_irq[i], 1, 0);
			}

		}

		if (g_ispctl_irq & 0xffff) {
			syslog(LOG_INFO, "get isp ctl irq = 0x%x\n", g_ispctl_irq & 0xffff);
			g_ispctl_irq = 0;
		}

		usleep(1);
	}

}

void *isp_main_loop(void *arg)
{
	while (1) {
		usleep(1);
	}

}

static int verify_intr_task(int argc, char *argv[])
{
	init_irqs();
	start_single_stream();

	pthread_t isp_mainloop_thread;
	pthread_attr_t isp_mainloop_attr;
	struct sched_param isp_mainloop_param;
	pthread_attr_init(&isp_mainloop_attr);
	isp_mainloop_param.sched_priority = SCHED_PRIORITY_DEFAULT;
	pthread_attr_setschedparam(&isp_mainloop_attr, &isp_mainloop_param);
	pthread_attr_setstacksize(&isp_mainloop_attr, 2048);
	pthread_create(&isp_mainloop_thread, &isp_mainloop_attr, isp_main_loop, NULL);

	pthread_t isp_irq_thread;
	pthread_attr_t isp_irq_attr;
	struct sched_param isp_irq_param;
	pthread_attr_init(&isp_irq_attr);
	isp_irq_param.sched_priority = SCHED_PRIORITY_DEFAULT;
	pthread_attr_setschedparam(&isp_irq_attr, &isp_irq_param);
	pthread_attr_setstacksize(&isp_irq_attr, 2048);
	pthread_create(&isp_irq_thread, &isp_irq_attr, isp_irq_listener_thread, NULL);


	return 0;
}

/****************************************************************************
 * isp_hello_main
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int isp_verify_intr_main(int argc, char *argv[])
#endif
{
	int ret =0;

	ret = task_create(argv[0],
			CONFIG_ISP_VERIFY_INTR_PRIORITY,
			CONFIG_ISP_VERIFY_INTR_STACKSIZE,
			verify_intr_task,
			argv + 1);

	return ret > 0 ? 0 : ret;
}
