/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

#define __BTSTACK_FILE__ "ceva_diag.c"

/*
 */

// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <mqueue.h>
#include <message.h>

#include "bt_manager.h"
#include "btstack.h"
#include "hci.h"

/*
 * @section Main Application Setup
 *
 * @text As with the packet and the heartbeat handlers, the combined app setup contains the code from the individual example setups.
 */

void test_pin_cfg(uint32_t addr)
{
   uint32_t val;

   val = *((volatile uint32_t*)addr);
   val &= 0xFFFFFFF0;  val |= 3;
   *((volatile uint32_t*)addr) = val;
}

/* LISTING_START(MainConfiguration): Init L2CAP RFCOMM SDO SM ATT Server and start heartbeat timer */

void rf_debug_enable(void)
{
   /* enable RF debug pin*/
   uint32_t reg_val;

   printf("RF Debug Pin Enable\n");

   // release RFIP reset
   reg_val = *((uint32_t *)0xa00e0180);
   *((uint32_t *)0xa00e0180) = reg_val & (~(1<<23));

   usleep(10000);
   /* supply rfip vdda and vddpa */
   // ... ...
   *((uint8_t *)0xa0300000 + (0x10a9 << 2)) = 0xF8;

   // debug pin
   *((uint32_t *)0xa00d00d4) = 0x13;
   *((uint32_t *)0xa00d00d8) = 0x13;
   *((uint32_t *)0xa00d00dc) = 0x13;
   *((uint32_t *)0xa00d00e0) = 0x13;
   *((uint32_t *)0xa00d00e4) = 0x13;
   *((uint32_t *)0xa00d00e8) = 0x13;
   *((uint32_t *)0xa00d0024) = 0x13;
   *((uint32_t *)0xa00d0028) = 0x13;
   *((uint32_t *)0xa00d0070) = 0x13;
   *((uint32_t *)0xa00d0074) = 0x13;
   *((uint32_t *)0xa00d0078) = 0x13;

   *((uint32_t *)0xa00d0194) = 2;

   reg_val = *((uint32_t *)0xa00e0500);
   reg_val &= (~(1<<20));
   reg_val |= (1<<21);
   reg_val |= (1<<23);
   *((uint32_t *)0xa00e0500) = reg_val;
}

void ceva_debug_pin_init(void)
{
    /* Enable CEVA DIAG pins */
    uint32_t val;
    val = *((volatile uint32_t*)0xa00d0194);
    val &= 0xFFFFFFF0;  val |= 1;
    *((volatile uint32_t*)0xa00d0194) = val;

    test_pin_cfg(0xa00d00D4);
    test_pin_cfg(0xa00d00D8);
    test_pin_cfg(0xa00d00DC);
    test_pin_cfg(0xa00d00E0);
    test_pin_cfg(0xa00d00E4);
    test_pin_cfg(0xa00d00E8);
    test_pin_cfg(0xa00d0024);
    test_pin_cfg(0xa00d0028);
    test_pin_cfg(0xa00d0070);
    test_pin_cfg(0xa00d0074);
    test_pin_cfg(0xa00d0078);
    test_pin_cfg(0xa00d00B0);
    test_pin_cfg(0xa00d00B4);
    test_pin_cfg(0xa00d00B8);
    test_pin_cfg(0xa00d00BC);
    test_pin_cfg(0xa00d00C0);

    val = *((volatile uint32_t*)0xa00e0500);
    val &= 0xFFFFFFF0;  val |= 0;
    *((volatile uint32_t*)0xa00e0500) = val;

    *((volatile uint32_t*)0xa0020450) = 0x0000BEA5;
}

int ceva_diag_main(int argc, const char * argv[]);
int ceva_diag_main(int argc, const char * argv[])
{
    UNUSED(argc);
    (void)argv;

    ceva_debug_pin_init();
    return 0;
}

/* LISTING_END */
/* EXAMPLE_END */
