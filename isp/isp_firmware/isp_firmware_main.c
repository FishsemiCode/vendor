/****************************************************************************
 * apps/external/isp/isp_firmware/isp_firmware_main.c
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
#include "defines.h"
#include "intr.h"
#include "main_loop.h"

#ifdef CONFIG_ISP_SIMPLE_HOST
#include "simple_host/command_listener.h"
#endif //CONFIG_ISP_SIMPLE_HOST

sem_t sem_cmd_listener, sem_irq_listener, sem_mainloop;

int isp_firmware_task(int argc, char *argv[])
{
	sem_init(&sem_cmd_listener, 0, 0);
	sem_init(&sem_irq_listener, 0, 0);
	sem_init(&sem_mainloop, 0, 0);

	init_irqs();

	pthread_t isp_mainloop_thread;
	pthread_attr_t isp_mainloop_attr;
	struct sched_param isp_mainloop_param;
	pthread_attr_init(&isp_mainloop_attr);
	isp_mainloop_param.sched_priority = SCHED_PRIORITY_DEFAULT;
	pthread_attr_setschedparam(&isp_mainloop_attr, &isp_mainloop_param);
	pthread_attr_setstacksize(&isp_mainloop_attr, 1024);
	pthread_create(&isp_mainloop_thread, &isp_mainloop_attr, isp_main_loop, NULL);

	pthread_t isp_irq_thread;
	pthread_attr_t isp_irq_attr;
	struct sched_param isp_irq_param;
	pthread_attr_init(&isp_irq_attr);
	isp_irq_param.sched_priority = SCHED_PRIORITY_DEFAULT;
	pthread_attr_setschedparam(&isp_irq_attr, &isp_irq_param);
	pthread_attr_setstacksize(&isp_irq_attr, 1024);
	pthread_create(&isp_irq_thread, &isp_irq_attr, isp_irq_listener_thread, NULL);

#ifdef CONFIG_ISP_SIMPLE_HOST
	pthread_t host_thread;
	pthread_attr_t host_thread_attr;
	struct sched_param host_thread_param;
	pthread_attr_init(&host_thread_attr);
	host_thread_param.sched_priority = SCHED_PRIORITY_DEFAULT;
	pthread_attr_setschedparam(&host_thread_attr, &host_thread_param);
	pthread_attr_setstacksize(&host_thread_attr, 1024);
	pthread_create(&host_thread, &host_thread_attr, command_lister, NULL);
#endif

	return 0;
}

/****************************************************************************
 * isp_firmware_main
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int isp_firmware_main(int argc, char *argv[])
#endif
{
	int ret =0;

	ret = task_create(argv[0],
			CONFIG_ISP_FIRMWARE_PRIORITY,
			CONFIG_ISP_FIRMWARE_STACKSIZE,
			isp_firmware_task,
			argv + 1);

	return ret > 0 ? 0 : ret;
}
