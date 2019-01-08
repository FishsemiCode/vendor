/****************************************************************************
 * apps/external/isp/isp_firmware/intr.h
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

#ifndef ISP_INTR_H
#define ISP_INTR_H

#include <semaphore.h>

extern unsigned int g_ispctl_irq;
extern unsigned int g_sof[PIPE_NUM];
extern unsigned int g_eof[PIPE_NUM];
extern unsigned int g_awb_done[PIPE_NUM];
extern unsigned int g_ae_done[PIPE_NUM];

typedef enum {
	ISP1_IRQ = 0,
	ISP2_IRQ,
	ISP3_IRQ,
	ISP4_IRQ,
	ISPCTL_IRQ = 27
} irq_id_t;

int init_irqs(void);
void *isp_irq_listener_thread(void *arg);

int isp_isr_process(int irq, void *data, void *arg);
int isp1_isr(int irq, void *data, void *arg);
int isp2_isr(int irq, void *data, void *arg);
int isp3_isr(int irq, void *data, void *arg);
int isp4_isr(int irq, void *data, void *arg);
int ispctl_isr(int irq, void *data, void *arg);

#endif
