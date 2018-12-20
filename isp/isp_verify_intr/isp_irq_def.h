/****************************************************************************
 * apps/external/isp/isp_verify_intr/isp_irq_def.h
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

#ifndef ISP_IRQ_DEF
#define ISP_IRQ_DEF

#define ISP1_BASE 0xFA450000
#define ISP2_BASE 0xFA451000
#define ISP3_BASE 0xFA452000
#define ISP4_BASE 0xFA453000
#define ISP_BASE_OFFSET	0x1000

#define INT_SOF_EOF_EN	0xa00
#define INT_SOF_EOF		0xa04
	#define BIT_EOF		1
	#define BIT_SOF		0
#define INT_STAT_DONE_EN	0xa18
#define INT_STAT_DONE		0xa1c
	#define BIT_AE_DONE		1
	#define BIT_AWB_DONE	0

#define REG_ISP_HEIGHT	0x08
#define REG_ISP_WIDTH	0x0c
#define REG_AWB_STAT_WIDTH	0xe0
#define REG_AWB_STAT_HEIGHT	0xe4
#define REG_AE_STAT_WIDTH	0x1b0
#define REG_AE_STAT_HEIGHT	0x1b4
#define REG_PORT0_HEIGHT	0x810
#define REG_PORT0_WIDTH		0x814
#define REG_PORT1_HEIGHT	0x828
#define REG_PORT1_WIDTH		0x82c
#define REG_PORT2_HEIGHT	0x840
#define REG_PORT2_WIDTH		0x844
#define REG_PORT3_HEIGHT	0x858
#define REG_PORT3_WIDTH		0x85c
#define REG_RD_PORT_HEIGHT	0x908
#define REG_RD_PORT_WIDTH	0x90c
#define REG_RD_PORT_VB		0x910
#define REG_RD_PORT_HB		0x914

#endif
