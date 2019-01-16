/****************************************************************************
 * apps/external/isp/isp_simple_host/defines.h
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

#ifndef ISP_DEFINES
#define ISP_DEFINES

#define PIPE_NUM	4
#define PORT_NUM	4

#define FW_CMD_BUFFER_BASE	0x1b000
#define FW_ISP_SETTING_BASE	0x1c000

#define INT_ISPCTL_EN	0xfa3e0404
#define INT_ISPCTL_BIT_EN	0xfa3e0408
#define INT_ISPCTL		0xfa3e040c
#define REG_ISPCTL_TRIGGER	0xfa3e0400
	#define ISPCTL_INIT_FW	0
	#define ISPCTL_SETFMT	1
	#define ISPCTL_BRACKET_CAP	2
	#define ISPCTL_3	3
	#define ISPCTL_4	4
	#define ISPCTL_5	5
	#define ISPCTL_6	6
	#define ISPCTL_7	7
	#define ISPCTL_INIT_FW_DONE	8
	#define ISPCTL_SETFMT_DONE	9
	#define ISPCTL_BRACKET_CAP_DONE	10
	#define ISPCTL_11	11
	#define ISPCTL_12	12
	#define ISPCTL_13	13
	#define ISPCTL_14	14
	#define ISPCTL_15	15

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

#ifdef CONFIG_ISP_SIMPLE_HOST
	#define INT_AXI_START_EN    0xa08
	#define INT_AXI_START       0xa0c
	#define INT_AXI_END_EN      0xa10
	#define INT_AXI_END         0xa14
#endif

#define REG_SOFT_RSTN	0x00
#define REG_ISP_HEIGHT	0x08
#define REG_ISP_WIDTH	0x0c
#define REG_ISP_COLORBAR	0x10
#define REG_AWB_STAT_WIDTH	0xe0
#define REG_AWB_STAT_HEIGHT	0xe4
#define REG_AE_STAT_WIDTH	0x1b0
#define REG_AE_STAT_HEIGHT	0x1b4
#define REG_PORT0_CTRL		0x800
#define REG_PORT0_HEIGHT	0x810
#define REG_PORT0_WIDTH		0x814
#define REG_PORT1_CTRL		0x800
#define REG_PORT1_HEIGHT	0x828
#define REG_PORT1_WIDTH		0x82c
#define REG_PORT2_CTRL		0x800
#define REG_PORT2_HEIGHT	0x840
#define REG_PORT2_WIDTH		0x844
#define REG_PORT3_CTRL		0x800
#define REG_PORT3_HEIGHT	0x858
#define REG_PORT3_WIDTH		0x85c
	#define PORT_OFFSET		0x18
#define REG_RD_PORT_HEIGHT	0x908
#define REG_RD_PORT_WIDTH	0x90c
#define REG_RD_PORT_VB		0x910
#define REG_RD_PORT_HB		0x914

/* operations */
#define setreg32(addr, val)		(*(volatile uint32_t *)(addr) = (uint32_t)(val))
#define getreg32(addr)			(*(volatile uint32_t *)(addr))
#define setreg16(addr, val)		(*(volatile uint16_t *)(addr) = (uint16_t)(val))
#define getreg16(addr)			(*(volatile uint16_t *)(addr))
#define setreg8(addr, val)		(*(volatile uint8_t *)(addr) = (uint8_t)(val))
#define getreg8(addr)			(*(volatile uint8_t *)(addr))

static inline void setreg32_mask(uint32_t addr, uint32_t val, uint32_t mask)
{
	uint32_t v = getreg32(addr);
	v = (v & ~mask) | (val & mask);
	setreg32(addr, v);
}

static inline void setreg16_mask(uint32_t addr, uint16_t val, uint16_t mask)
{
	uint16_t v = getreg16(addr);
	v = (v & ~mask) | (val & mask);
	setreg16(addr, v);
}

static inline void setreg8_mask(uint32_t addr, uint8_t val, uint8_t mask)
{
	uint8_t v = getreg8(addr);
	v = (v & ~mask) | (val & mask);
	setreg8(addr, v);
}

#define set_bit(n, bit, x)	(n = (n & ~(1 << bit)) | ((x << bit)))

typedef enum {
	SRC_SENSOR = 0,
	SRC_MEM,
	SRC_COLORBAR
} input_t;

typedef struct {
	bool port_en;
	uint16_t width;
	uint16_t height;
} port_fmt_t;

typedef struct {
	uint8_t pipe_id;
	input_t input_type;
	bool str_on_off_flag;
	uint16_t pipe_width;
	uint16_t pipe_height;
	port_fmt_t port_fmt[PORT_NUM];
	port_fmt_t rd_port_fmt;
	uint32_t *mem_src_addr;
} stream_fmt_t;

#endif
