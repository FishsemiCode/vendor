/****************************************************************************
 * apps/external/example/vbattest/vbattest.c
 *
 *   Copyright (C) 2020 FishSemi Inc. All rights reserved.
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

#include <sys/stat.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/fs/fs.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "at_api.h"

/****************************************************************************
 * Static Data
 ****************************************************************************/


/****************************************************************************
 * Private Function
 ****************************************************************************/


/****************************************************************************
 * rtc_test
 ****************************************************************************/

static int gpio_port_register(int port, enum gpio_pintype_e type)
{
  char dev_name[16];
  struct stat buf;
  int ret;

  if (type == GPIO_INPUT_PIN)
    {
      snprintf(dev_name, 16, "/dev/gpin%u", (unsigned int)port);
      IOEXP_SETDIRECTION(g_ioe[0], port, IOEXPANDER_DIRECTION_IN);
    }
  else if (type == GPIO_OUTPUT_PIN)
    {
      snprintf(dev_name, 16, "/dev/gpout%u", (unsigned int)port);
      IOEXP_SETDIRECTION(g_ioe[0], port, IOEXPANDER_DIRECTION_OUT);
    }

  ret = stat(dev_name, &buf);
  if (ret == 0)
    {
      return ret;
    }

  return gpio_lower_half(g_ioe[0], (unsigned int)port, type, port);
}

static int gpio_port_write(int port, bool write_val)
{
  int fd, ret;
  char dev_name[16];

  snprintf(dev_name, 16, "/dev/gpout%u", (unsigned int)port);

  fd = open(dev_name, O_RDONLY);
  if (fd < 0)
    {
      printf("open %s failed\n", dev_name);
      return -ENXIO;
    }

  ret = ioctl(fd, GPIOC_WRITE, (unsigned long)write_val);
  if (ret)
    {
      printf("%s:P%u failed\n", __func__, (unsigned int)port);
    }

  close(fd);
  return ret;
}

static int vbat_test(int port, int times)
{
  int ret;
  int clientfd;
  int vbat;
  int i, j;
  char * pboardid;
  bool issetGPIO;

  clientfd = at_client_open();
  if(clientfd < 0)
    {
      syslog(LOG_ERR, "Open client error\n");
      return -1;
    }

  pboardid = getenv_global("board-id");
  if(pboardid)
    {
      if((strncmp(pboardid, "U1BX", 4) == 0) || (strncmp(pboardid, "U1TK", 4) == 0))
        {
          issetGPIO = true;
        }
    }
  if(issetGPIO)
    {
      gpio_port_register(39, GPIO_OUTPUT_PIN);
      gpio_port_write(39, true);
    }

  for (i = 0; i < times; i++)
    {
      int sum = 0;
      int cnt = 0;

      for (j = 0; j < 10; j++)
        {
          ret = get_vbat(clientfd, port, &vbat);
          if (ret >= 0)
            {
              sum += vbat;
              cnt++;
            }
        }

      if (cnt == 0)
        {
          syslog(LOG_ERR, "get_vbat fail\n");
          goto clean;
        }
      else
        {
          vbat = sum / cnt;
          syslog(LOG_INFO, "VBAT: %d, CNT: %d\n", vbat, i);
        }
      sleep(1);
    }

clean:
  if(issetGPIO)
    {
      gpio_port_register(39, GPIO_OUTPUT_PIN);
      gpio_port_write(39, false);
    }
  at_client_close(clientfd);
  return ret;
}


/****************************************************************************
 * Public function
 ****************************************************************************/

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int vbattest_main(int argc, char *argv[])
#endif
{
  if (argc != 3) {
    printf("[usage]: vbattest [test port(0-2)] [test times]\n");
    return EXIT_SUCCESS;
  }

  vbat_test(atoi(argv[1]), atoi(argv[2]));

  return EXIT_SUCCESS;
}
