/****************************************************************************
 * apps/external/isp/isp_firmware/defines.h
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

extern sem_t sem_cmd_listener, sem_irq_listener, sem_mainloop;

#define PIPE_NUM	4
#define PORT_NUM	4

#define FW_CMD_BUFFER_BASE	0x1b000
#define FW_ISP_SETTING_BASE	0x1c000
	#define FW_ISP_SETTING_OFFSET	0x1000

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
	#define ISPCTL_DEINIT_FW	7
	#define ISPCTL_INIT_FW_DONE	8
	#define ISPCTL_SETFMT_DONE	9
	#define ISPCTL_BRACKET_CAP_DONE	10
	#define ISPCTL_11	11
	#define ISPCTL_12	12
	#define ISPCTL_13	13
	#define ISPCTL_14	14
	#define ISPCTL_DEINIT_FW_DONE	15

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

#define REG_LENC_COEF_ATTR	0xb4
#define REG_LENC_COEF_RD_ATTR	0xb8

#define REG_AWB_R_GAIN		0xc4
#define REG_AWB_G_GAIN		0xc8
#define REG_AWB_B_GAIN		0xcc

//#define REG_AWB_STA_INTR_WIRE	0x172
#define REG_AWB_RAM_ADDR	0x170
#define REG_AWB_RAM_R_DOUT		0x174
#define REG_AWB_RAM_G_DOUT		0x178
#define REG_AWB_RAM_B_DOUT		0x17c

#define REG_AE_STAT_H_START	0x1a8
#define REG_AE_STAT_V_START	0x1ac
#define REG_AE_STAT_H_END	0x1b0
#define REG_AE_STAT_V_END	0x1b4
#define REG_AE_MESH_X1	0x1b8
#define REG_AE_MESH_X2	0x1bc
#define REG_AE_MESH_X3	0x1c0
#define REG_AE_MESH_Y1	0x1c4
#define REG_AE_MESH_Y2	0x1c8
#define REG_AE_MESH_Y3	0x1cc

#define REG_AE_R_HIST_BASE	0x1d0
#define REG_AE_GR_HIST_BASE	0x228
#define REG_AE_GB_HIST_BASE	0x280
#define REG_AE_B_HIST_BASE	0x2d8
#define REG_AE_R_BRI_BASE	0x330
#define REG_AE_GR_BRI_BASE	0x370
#define REG_AE_GB_BRI_BASE	0x3b0
#define REG_AE_B_BRI_BASE	0x3f0

#define REG_CC_EN		0x4e0
#define REG_CC_R_OFST	0x4e4
#define REG_CC_G_OFST	0x4e8
#define REG_CC_B_OFST	0x4ec
#define REG_CC_00		0x4f0
#define REG_CC_01		0x4f4
#define REG_CC_02		0x4f8
#define REG_CC_10		0x4fc
#define REG_CC_11		0x500
#define REG_CC_12		0x504
#define REG_CC_20		0x508
#define REG_CC_21		0x50c
#define REG_CC_22		0x510
#define REG_CC_R_OFFSET	0x514
#define REG_CC_G_OFFSET	0x518
#define REG_CC_B_OFFSET	0x51c

#define REG_GAMMA_EN	0x520
#define REG_GAMMA_WRITE_ADDR	0x524
#define REG_GAMMA_QOUT_ADDR		0x528

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

#define min(a, b)	((a > b) ? b : a)
#define max(a, b)	((a > b) ? a : b)
#define interp(x, x1, x2, y1, y2)	((y1) + ((x) - (x1)) * ((y2) - (y1)) / ((x2) - (x1)))

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

/* setformat cmd buffer */
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
} stream_fmt_t;

/* global control */
typedef struct {
	uint8_t manua_exp_en;
	uint32_t exp;
	uint32_t vts;
	uint16_t a_gain;
	uint16_t d_gain;
	uint16_t gain;
	uint16_t b_gain;
	uint16_t g_gain;
	uint16_t r_gain;

	uint8_t lsc_r_coef[256];
	uint8_t lsc_g_coef[256];
	uint8_t lsc_b_coef[256];

	uint8_t awb_muti_map_en;
	uint32_t awb_map_th0;
	uint32_t awb_map_th1;
	uint8_t awb_map_trans_band0; // 0x20 means 1X awb_map_th0
	uint8_t awb_map_trans_band1; // 0x20 means 1X awb_map_th1
	uint8_t awb_map_low[256];
	uint8_t awb_map_middle_mask[32];
	uint8_t awb_map_high_mask[32];
	uint16_t awb_mesh_thr_x[17];
	uint16_t awb_mesh_thr_y[17];
	uint8_t awb_shift_en;
	uint32_t awb_shift_br_th[4];
	uint16_t awb_shift_ct_th[6];
	uint8_t awb_shift_r_coef[4][6];
	uint8_t awb_shift_b_coef[4][6];

	uint16_t ae_h_start;
	uint16_t ae_h_end;
	uint16_t ae_v_start;
	uint16_t ae_v_end;
	uint16_t ae_h_th1;
	uint16_t ae_h_th2;
	uint16_t ae_h_th3;
	uint16_t ae_v_th1;
	uint16_t ae_v_th2;
	uint16_t ae_v_th3;
	uint8_t ae_weight[16]; // 0x10 mean base 1x
	uint32_t ae_target;
	uint32_t ae_stable_range_in;
	uint32_t ae_stable_range_out;
	uint32_t ae_coarse_step;
	uint32_t ae_fine_step;

	uint8_t cc_en;
	uint8_t cc_level_0; // 0x80 means base 1x
	uint8_t cc_level_1;
	uint32_t cc_bri_th0;
	uint32_t cc_bri_th1;
	uint16_t cc_ct_th_0;
	uint16_t cc_ct_th_1;
	uint16_t cc_ct_th_2;
	int16_t cc_coef[3][9]; // 0x100 means base 1x

	uint8_t gamma_en;
	uint32_t gamma_bri_th0;
	uint32_t gamma_bri_th1;
	uint8_t gamma_bri_trans_band0;
	uint8_t gamma_bri_trans_band1;
	uint16_t gamma_index[46]; // 46 segment curve
	uint16_t gamma_val_low[46];
	uint16_t gamma_val_mid[46];
	uint16_t gamma_val_high[46];

} global_control_t;

/* middle group */
typedef struct {
	uint32_t exp;
	uint32_t vts;
	uint16_t a_gain;
	uint16_t d_gain;
	uint16_t gain;
	uint32_t exp_idx;
	uint16_t current_ct;
	uint16_t b_gain;
	uint16_t g_gain;
	uint16_t r_gain;
	uint16_t b_gain_before_shift;
	uint16_t g_gain_before_shift;
	uint16_t r_gain_before_shift;
	uint8_t awb_sta_intr_wire;
	uint8_t awb_ram_addr;
	uint32_t awb_ram_r_dout[256];
	uint32_t awb_ram_g_dout[256];
	uint32_t awb_ram_b_dout[256];
	uint8_t awb_weight_map[256];

	uint8_t lsc_r_coef[256];
	uint8_t lsc_g_coef[256];
	uint8_t lsc_b_coef[256];

	uint32_t meany;
	uint32_t new_exp_idx_flag;
	uint32_t new_exp_idx;
	uint32_t r_hist[22];
	uint32_t gr_hist[22];
	uint32_t gb_hist[22];
	uint32_t b_hist[22];
	uint32_t r_bri[16];
	uint32_t gr_bri[16];
	uint32_t gb_bri[16];
	uint32_t b_bri[16];
	uint32_t ae_stable_flag;

} middle_group_t;

#endif
