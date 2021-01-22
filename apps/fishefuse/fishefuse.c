/****************************************************************************
 * vendor/apps/fishefuse/fishefuse.c
 *
 *   Copyright (C) 2020 Fishsemi Inc. All rights reserved.
 *   Author: Fishsemi <fishsemi@fishsemi.com>
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

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

/****************************************************************************
* Private Variables
****************************************************************************/

enum EFUSE_STATE
{
  read_address = 0,
  write_address,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* PMICFSM */
#define PMICFSM_BASE        (0xb2010000)
#define PMICFSM_LDO_EXT1    ((volatile uint32_t *)(PMICFSM_BASE + 0x58))
#define EFUSEMAXNUMBER      (128)

/****************************************************************************
 * fishefuse_main
 ****************************************************************************/

static void efuse_usage(void)
{
  printf("Input parameter error\n");
  printf("fishefuse read : fishefuse address length(default 1)\n");
  printf("fishefuse write: fishefuse address=value\n");
}

int fishefuse_main(int argc, char *argv[])
{
  int fd0, ret;
  uint8_t size;
  uint8_t efuse_state;
  char *pcvalue;
  int address = 0;
  uint8_t length;
  uint8_t i;
  uint32_t write_value;
  uint8_t value[EFUSEMAXNUMBER];

  /* Read all Register */

  if (argc == 1)
    {
      efuse_state = read_address;
      address = 0;
      length = EFUSEMAXNUMBER;
    }

  /* Read specify Register address */

  else if(argc == 2)
    {
      pcvalue = strchr(argv[1], '=');
      if (pcvalue)
        {
          *pcvalue = '\0';
          pcvalue++;
          write_value = strtoul(pcvalue, NULL, 16);
          address = ((uintptr_t)strtoul(argv[1], NULL, 16));
          efuse_state = write_address;
          length = 2;
        }
      else
        {
          efuse_state = read_address;
          address = strtoul(argv[1], NULL, 16);
          length = 1;
        }
    }

  else if(argc == 3)
    {
      efuse_state = read_address;
      address = strtoul(argv[1], NULL, 16);
      length = strtoul(argv[2], NULL, 10);
    }

  else
    {
      efuse_usage();
      return 0;
    }

  fd0 = open("/dev/efuse0", O_RDWR);
  if (fd0 < 0)
    printf("Unable to open file /dev/efuse\n");

  /*
   *ioctl parameter
   *cmd :
   *    : 2 write
   *
   *arg : 32bit
   *    : 32-24 :  address
   *    : 24-16 :  length
   *    : 16-8  :  data1
   *    : 8 -0  :  data0
   *    notify: Address and data length must be written through ioctl before reading
   *
   *  write address 0x53,
   *  write length  1
   *  data1 :0x00
   *  data0 :0x20
   * */

  switch(efuse_state)
    {
      case read_address:

              /* Over Efuse max number */

              if (address + length > EFUSEMAXNUMBER)
                length = EFUSEMAXNUMBER - address;

              lseek(fd0, address, SEEK_SET);
              size = read(fd0, value, length);

              for (ret = 0; ret < size; ret++)
                {
                  if (ret == 0)
                    printf("%04x: ", ret + address);

                  if (ret % 16 == 0 && ret != 0)
                    {
                      printf("\n");
                      printf("%04x: ", ret + address);
                    }
                  printf("%02x ", value[ret]);
                }

              printf("\n");
              break;

      case write_address:

              /* Rise to 3.3V when write efuse */

              *PMICFSM_LDO_EXT1 = 0x00001f01;

              /* Waiting for LDO stabilize */

              usleep(1000);

              for (i = 0; i < length; i++)
                {
                  write_value = write_value >> (16 * i);
                  ret = ioctl(fd0, 2, (address + 2 * i) << 24 | 2 << 16 | (write_value & 0x0000ffff));
                }

              /* Restore the origin value */

              *PMICFSM_LDO_EXT1 = 0x00001f00;
              break;
    }

  close(fd0);
  return 0;
}
