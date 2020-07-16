/****************************************************************************
 * apps/external/example/gpiocfg/gpiocfg.c
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

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/ioexpander/gpio.h>

/****************************************************************************
 * Macro definition
 ****************************************************************************/

#define GPIO_DS_LEVEL0  (0)
#define GPIO_DS_LEVEL1  (1)
#define GPIO_DS_LEVEL2  (2)
#define GPIO_DS_LEVEL3  (3)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpio_port_register(int port, enum gpio_pintype_e type);
static int gpio_port_read(int port, bool *read_val);
static int gpio_port_write(int port, bool write_val);

/****************************************************************************
 * Private Funtions
 ****************************************************************************/

static int gpio_port_register(int port, enum gpio_pintype_e type)
{
  char dev_name[16];
  struct stat buf;
  int ret;

  if (type == GPIO_INPUT_PIN)
    {
      snprintf(dev_name, 16, "/dev/gpin%u", (unsigned int)port);
    }
  else if (type == GPIO_OUTPUT_PIN)
    {
      snprintf(dev_name, 16, "/dev/gpout%u", (unsigned int)port);
    }

  ret = stat(dev_name, &buf);
  if (ret == 0)
    {
      return ret;
    }

  return gpio_lower_half(g_ioe[0], (unsigned int)port, type, port);
}

static int gpio_port_read(int port, bool *read_val)
{
  int fd, ret;
  char dev_name[16];

  snprintf(dev_name, 16, "/dev/gpin%u", (unsigned int)port);
  fd = open(dev_name, 0);
  if (fd < 0)
    {
      printf("open %s failed\n", dev_name);
      return -ENXIO;
    }

  ret = ioctl(fd, GPIOC_READ, (unsigned long)read_val);
  if (ret)
    {
      printf("%s:P%u failed\n", __func__, (unsigned int)port);
    }

  close(fd);
  return ret;
}

static int gpio_port_write(int port, bool write_val)
{
  int fd, ret;
  char dev_name[16];

  snprintf(dev_name, 16, "/dev/gpout%u", (unsigned int)port);
  fd = open(dev_name, 0);
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

/****************************************************************************
 * Public function
 ****************************************************************************/

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int gpiocfg_main(int argc, char *argv[])
#endif
{
  int ret, gpio, dir, val = 0;
  bool gpio_val = false;

  if (argc != 4 && argc != 3)
    {
      printf("usage: %s [gpio] 0(out) [val]\n", argv[0]);
      printf("usage: %s [gpio] 1(in)\n", argv[0]);
      printf("Pin number range 4--41\n");
      goto end;
    }

  gpio = atoi(argv[1]);
  dir = atoi(argv[2]);
  if (argc == 4)
    val = !!atoi(argv[3]);

  gpio_port_register(gpio, GPIO_INPUT_PIN);
  gpio_port_register(gpio, GPIO_OUTPUT_PIN);

  if (dir)
    {
      gpio_port_read(gpio, &gpio_val);
      printf("gpio %d read val = %d\n", gpio, gpio_val);
    }
  else
    {
      gpio_val = (bool)val;
      gpio_port_write(gpio, gpio_val);
      printf("gpio %d write val = %d\n", gpio, gpio_val);
      gpio_val = false;
      gpio_port_read(gpio, &gpio_val);
      printf("gpio %d read val = %d\n", gpio, gpio_val);
    }
end:
  return EXIT_SUCCESS;
}
