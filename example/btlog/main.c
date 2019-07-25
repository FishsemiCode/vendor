/* -----------------------------------------------------------------------------
 * Copyright (c) 2018 Pinecone Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * -------------------------------------------------------------------------- */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>

/*
 * image layout
 * ---------------------
 *  magic
 *  img_addr
 *  img_size(exclude padding and signature)
 *  reset_handler
 *  vector 0
 *  vector 1
 *  ......
 *  vector n
 *  ......
 *  .......
 *  .......
 *  .......
 * ---------------------
 */

typedef struct _UART_TYPE {
        union {
                volatile uint32_t RBR;
                volatile uint32_t THR;
                volatile uint32_t DLL;
        };
        union {
                volatile uint32_t IER;
                volatile uint32_t DLH;
        };
        union {
                volatile uint32_t IIR;
                volatile uint32_t FCR;
        };
        volatile uint32_t TCR;
        volatile uint32_t RESERVE0;
        volatile uint32_t TSR;
        volatile uint32_t RESERVE1[26];
        volatile uint32_t USR;
} UART_TYPE;

#define UART_FCR_FIFO_EN      (1 << 0)
#define UART_FCR_RCVR_RE      (1 << 1)
#define UART_FCR_XMIT_RE      (1 << 2)

#define UART_TCR_DLAB         (1 << 7)

#define UART_TCR_PAR_NONE     (0 << 3)
#define UART_TCR_PAR_ODD      (1 << 3)
#define UART_TCR_PAR_EVEN     (3 << 3)
#define UART_TCR_PAR_MASK     (3 << 3)

#define UART_TCR_STOP_1BIT    (0 << 2)
#define UART_TCR_STOP_2BIT    (1 << 2)
#define UART_TCR_STOP_MASK    (1 << 2)

#define UART_TCR_CLS_8BITS    (3 << 0)
#define UART_TCR_CLS_7BITS    (2 << 0)
#define UART_TCR_CLS_6BITS    (1 << 0)
#define UART_TCR_CLS_5BITS    (0 << 0)
#define UART_TCR_CLS_MASK     (3 << 0)

#define UART_USR_BUSY         (1 << 0)

#define UART_TSR_THRE         (1 << 5)
#define UART_TSR_BI           (1 << 4)
#define UART_TSR_FE           (1 << 3)
#define UART_TSR_PE           (1 << 2)
#define UART_TSR_OE           (1 << 1)
#define UART_TSR_DR           (1 << 0)

#define PACK_HEAD_CHAR        0xFE
#define CMD_READ_MEM          0x01 
#define CMD_WRITE_MEM         0x02 
typedef struct {
  uint8_t  cmd;
  uint16_t size;
  uint32_t addr;
} _MEM_DUMP_CMD_;

#define CMD_PACK_SIZE         7

/*
 * Package format to read memory: Package Size is 8 bytes
 * 0xFE 0x01 ss zz(2bytes size, little endian)  aa bb cc dd (4-bytes start address, little endian)
 * When btlog is enabled, it will send out "OK" firstly
 */

UART_TYPE *uart_reg;
void sendout_char(unsigned char c)
{
    while((uart_reg->TSR & UART_TSR_THRE) == 0);
    uart_reg->THR = c;
}

int main (int argc, char *argv[])
{
    // UART1 921600
    _MEM_DUMP_CMD_ dump_cmd;
    unsigned char *cptr, *dcptr;
    unsigned char index, head_flag, c;
    uint32_t i;

    uart_reg = (UART_TYPE *)0xa0160000;
    /* initially disable the interrupt */
    __asm volatile ("csrw mie, zero");
    *((uint32_t *)0xa00e0044) = 0x00f00010; /* top_bus_mclk 147.4M */
    // uart1 clock
    *((uint32_t *)0xa00e0080)= *((uint32_t *)0xa00e007c);

    while (uart_reg->USR & UART_USR_BUSY);
    uart_reg->TCR |= UART_TCR_DLAB;
    uart_reg->DLL = 1;
    uart_reg->DLH = 0;
    uart_reg->TCR  &= ~UART_TCR_DLAB;

    uart_reg->TCR  &= ~UART_TCR_CLS_MASK;
    uart_reg->TCR |= UART_TCR_CLS_8BITS;
    uart_reg->TCR  &= ~UART_TCR_STOP_MASK;
    uart_reg->TCR |= UART_TCR_STOP_1BIT;
    uart_reg->TCR  &= ~UART_TCR_PAR_MASK;
    uart_reg->TCR |= UART_TCR_PAR_NONE;
    uart_reg->FCR = UART_FCR_RCVR_RE | UART_FCR_XMIT_RE | UART_FCR_FIFO_EN;
    uart_reg->IER = 0;

    index = 0;
    head_flag = 0;
    cptr = (unsigned char *)&dump_cmd;

    sendout_char('O');
    sendout_char('K');
    while(1) 
    {
       if (uart_reg->TSR & UART_TSR_DR)
       {
          c = uart_reg->THR;
          if ( head_flag == 0)
          {
              if (c == PACK_HEAD_CHAR)
              {
                 head_flag = 1;
              }
              continue;
          }
          cptr[index ++] = uart_reg->THR;
          if (index >= CMD_PACK_SIZE) 
          { 
             if (dump_cmd.cmd == CMD_READ_MEM) 
             {
                 dcptr = (unsigned char *)(dump_cmd.addr);
                 for (i = 0; i < dump_cmd.size; i ++) 
                 {
                     while((uart_reg->TSR & UART_TSR_THRE) == 0);
                     uart_reg->THR = dcptr[i];
                 }
             }
             index = 0;
             head_flag = 0;
          } 
       }
    }    
}

