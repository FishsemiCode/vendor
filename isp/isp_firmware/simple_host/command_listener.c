/****************************************************************************
 * apps/external/isp/isp_firmware/simple_host/command_listener.c
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
#include "main_loop.h"
#include "commandset.h"

#include "command_listener.h"

#ifdef CONFIG_ISP_SIMPLE_HOST

stream_fmt_t *g_fmt;
bool stream_init_mode = true;

void *command_lister(void *arg)
{
	while (1) {
		sem_wait(&sem_cmd_listener);
		if (g_ispctl_irq & (1 << ISPCTL_INIT_FW)) {
			syslog(LOG_INFO, "get ispctl init fw\n");
			initialize();
			g_ispctl_irq = 0;
			setreg32(REG_ISPCTL_TRIGGER, ISPCTL_INIT_FW_DONE);
		}

		if (g_ispctl_irq & (1 << ISPCTL_DEINIT_FW)) {
			syslog(LOG_INFO, "get ispctl deinit fw\n");
			deinit();
			g_ispctl_irq = 0;
			setreg32(REG_ISPCTL_TRIGGER, ISPCTL_DEINIT_FW_DONE);
		}

		if (g_ispctl_irq & (1 << ISPCTL_SETFMT)) {
			syslog(LOG_INFO, "get set format cmd\n");
			g_fmt = (stream_fmt_t *)FW_CMD_BUFFER_BASE;
			set_format(g_fmt);
			g_ispctl_irq = 0;
			setreg32(REG_ISPCTL_TRIGGER, ISPCTL_SETFMT_DONE);
		}

	}
}

#endif //CONFIG_ISP_SIMPLE_HOST
