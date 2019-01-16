/****************************************************************************
 * apps/external/isp/isp_simple_host/isp_simple_host_main.c
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
#include <semaphore.h>
#include <memory.h>

#include "nuttx/irq.h"
#include "nuttx/arch.h"
#include "defines.h"

unsigned int g_ispctl_irq;
/* reserve 108 ~ 112k for cmd buffer */
stream_fmt_t *g_fmt = NULL;

#define ISPCTL_IRQ 27

pthread_mutex_t lock;
pthread_cond_t cond;

sem_t sem_cmd;

int initialize_fw(void)
{
	/* enable ispctl irqs */
	setreg32(INT_ISPCTL_EN, 0x1);
	setreg32(INT_ISPCTL_BIT_EN, 0xffff);

	setreg32(REG_ISPCTL_TRIGGER, ISPCTL_INIT_FW);
	pthread_mutex_lock(&lock);
	pthread_cond_wait(&cond, &lock);
	pthread_mutex_unlock(&lock);
	syslog(LOG_INFO, "%s done\n", __func__);

	return 0;
}

static void load_isp_setting(void)
{
	setreg32(0xfa4500c0, 0x05); // enable awb and awb stat
	setreg32(0xfa4501a0, 0x03); // enable ae and ae stat
	setreg32(0xfa4510c0, 0x05);
	setreg32(0xfa4511a0, 0x03);
	setreg32(0xfa4520c0, 0x05);
	setreg32(0xfa4521a0, 0x03);
	setreg32(0xfa4530c0, 0x05);
	setreg32(0xfa4531a0, 0x03);

	setreg32(0xfa4500e0, 176);
	setreg32(0xfa4500e4, 100); // smaller than height(144)
	setreg32(0xfa4501b0, 176);
	setreg32(0xfa4501b4, 100); // smaller than height(144)

	setreg32(0xfa4510e0, 176);
	setreg32(0xfa4510e4, 100); // smaller than height(144)
	setreg32(0xfa4511b0, 176);
	setreg32(0xfa4511b4, 100); // smaller than height(144)

	setreg32(0xfa4520e0, 176);
	setreg32(0xfa4520e4, 100); // smaller than height(144)
	setreg32(0xfa4521b0, 176);
	setreg32(0xfa4521b4, 100); // smaller than height(144)

	setreg32(0xfa4540e0, 176);
	setreg32(0xfa4540e4, 100); // smaller than height(144)
	setreg32(0xfa4541b0, 176);
	setreg32(0xfa4541b4, 100); // smaller than height(144)

	setreg32(0xfa4501b8, 0x2c);
	setreg32(0xfa4501bc, 0x58);
	setreg32(0xfa4501c0, 0x84);
	setreg32(0xfa4501c4, 0x18);
	setreg32(0xfa4501c8, 0x30);
	setreg32(0xfa4501cc, 0x48);

	setreg32(0xfa4511b8, 0x2c);
	setreg32(0xfa4511bc, 0x58);
	setreg32(0xfa4511c0, 0x84);
	setreg32(0xfa4511c4, 0x18);
	setreg32(0xfa4511c8, 0x30);
	setreg32(0xfa4511cc, 0x48);

	setreg32(0xfa4521b8, 0x2c);
	setreg32(0xfa4521bc, 0x58);
	setreg32(0xfa4521c0, 0x84);
	setreg32(0xfa4521c4, 0x18);
	setreg32(0xfa4521c8, 0x30);
	setreg32(0xfa4521cc, 0x48);

	setreg32(0xfa4531b8, 0x2c);
	setreg32(0xfa4531bc, 0x58);
	setreg32(0xfa4531c0, 0x84);
	setreg32(0xfa4531c4, 0x18);
	setreg32(0xfa4531c8, 0x30);
	setreg32(0xfa4531cc, 0x48);

}

#define F2F_MODE

int start_single_stream(void)
{
	if (g_fmt == NULL) {
		g_fmt = (stream_fmt_t *)FW_CMD_BUFFER_BASE;
	}

	memset(g_fmt, 0, sizeof(stream_fmt_t));
	g_fmt->pipe_id = 0;
	g_fmt->input_type = SRC_COLORBAR;
#ifdef F2F_MODE
	g_fmt->input_type = SRC_MEM;
	g_fmt->mem_src_addr = (uint32_t *)0x80800000;
	g_fmt->rd_port_fmt.port_en = true;
	g_fmt->rd_port_fmt.width = 176;
	g_fmt->rd_port_fmt.height = 144;
#endif
	g_fmt->str_on_off_flag = true;
	g_fmt->pipe_width = 176;
	g_fmt->pipe_height = 144;
	g_fmt->port_fmt[0].port_en = true;
	g_fmt->port_fmt[0].width = 176;
	g_fmt->port_fmt[0].height =144;
	g_fmt->port_fmt[1].port_en = true;
	g_fmt->port_fmt[1].width = 176;
	g_fmt->port_fmt[1].height =144;


	setreg32(REG_ISPCTL_TRIGGER, ISPCTL_SETFMT);
	pthread_mutex_lock(&lock);
	pthread_cond_wait(&cond, &lock);
	pthread_mutex_unlock(&lock);

	g_fmt = NULL;

	unsigned int *y_addr = (uint32_t *)0x80000000;
	unsigned int *u_addr = (uint32_t *)0x80200000;
	unsigned int *v_addr = (uint32_t *)0x80400000;

	setreg32(0xfa450860, y_addr);
	setreg32(0xfa450864, u_addr);
	setreg32(0xfa450868, v_addr);

	return 0;

}

void *ispctl_listener(void *arg)
{
	while (1) {
		sem_wait(&sem_cmd);
		if (g_ispctl_irq & (1 << ISPCTL_INIT_FW_DONE)) {
			syslog(LOG_INFO, "get ispctl done, g_ispctl_irq = 0x%x\n", g_ispctl_irq);
			g_ispctl_irq = 0;
			pthread_mutex_lock(&lock);
			pthread_cond_signal(&cond);
			pthread_mutex_unlock(&lock);
		}

		if (g_ispctl_irq & (1 << ISPCTL_SETFMT_DONE)) {
			syslog(LOG_INFO, "get set format done, g_ispctl_irq = 0x%x\n", g_ispctl_irq);
			g_ispctl_irq = 0;
			pthread_mutex_lock(&lock);
			pthread_cond_signal(&cond);
			pthread_mutex_unlock(&lock);
		}
	}
}

int host_ispctl_isr(int irq, void *data, void *arg)
{
	int addr, val;
	addr = INT_ISPCTL;
	val = getreg32(addr);
	g_ispctl_irq |= val;
	setreg32(addr, val);

	sem_post(&sem_cmd);

	return 0;
}

int host_init_irqs(void)
{
	irq_attach(ISPCTL_IRQ, host_ispctl_isr, NULL);
	up_enable_irq(ISPCTL_IRQ);

	return 0;
}
static int simple_host_task(int argc, char *argv[])
{
	pthread_mutex_init(&lock, NULL);
	pthread_cond_init(&cond, NULL);

	sem_init(&sem_cmd, 0, 0);

	pthread_t ispctl_listener_thread;
	pthread_attr_t ispctl_listener_attr;
	struct sched_param ispctl_listener_param;
	pthread_attr_init(&ispctl_listener_attr);
	ispctl_listener_param.sched_priority = SCHED_PRIORITY_DEFAULT;
	pthread_attr_setschedparam(&ispctl_listener_attr, &ispctl_listener_param);
	pthread_attr_setstacksize(&ispctl_listener_attr, 2048);
	pthread_create(&ispctl_listener_thread, &ispctl_listener_attr, ispctl_listener, NULL);

	host_init_irqs();
	initialize_fw();
	load_isp_setting();
	start_single_stream();

	return 0;
}

/****************************************************************************
 * isp_simple_host
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int isp_simple_host_main(int argc, char *argv[])
#endif
{
	int ret =0;

	ret = task_create(argv[0],
			CONFIG_ISP_SIMPLE_HOST_PRIORITY,
			CONFIG_ISP_SIMPLE_HOST_STACKSIZE,
			simple_host_task,
			argv + 1);

	return ret > 0 ? 0 : ret;
}
