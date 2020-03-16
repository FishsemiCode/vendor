/****************************************************************************
 * apps/external/example/gpiodemo/gpiodemo.c
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
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/pinctrl/pinctrl.h>
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

static int muxpin_select_gpio(int port);
static int muxpin_set_driver_type(int port, enum pinctrl_drivertype_e type);
static int muxpin_set_driver_strength(int port, int level);
static int gpio_port_register(int port, enum gpio_pintype_e type);
static int gpio_port_read(int port, bool *read_val);
static int gpio_port_write(int port, bool write_val);
static void gpio_loopback_test(int gpin, int gpout);

/****************************************************************************
 * Private Funtions
 ****************************************************************************/

static int muxpin_select_gpio(int port)
{
  int fd, ret;

  fd = open("/dev/pinctrl0", 0);
  if (fd < 0)
    {
      printf("open /dev/pinctrl0 failed\n");
      return -ENXIO;
    }

  ret = ioctl(fd, PINCTRLC_SELGPIO, (unsigned long)port);
  if (0 != ret)
    {
      printf("%s:ioctl port%d failed\n", __func__, port);
    }

  close(fd);
  return ret;
}

static int muxpin_set_driver_type(int port, enum pinctrl_drivertype_e type)
{
  int fd, ret;
  struct pinctrl_iotrans_s trans;

  fd = open("/dev/pinctrl0", 0);
  if (fd < 0)
    {
      printf("open /dev/pinctrl0 failed\n");
      return -ENXIO;
    }

  trans.pin = (uint32_t)port;
  trans.para.type = type;

  ret = ioctl(fd, PINCTRLC_SETDT, (unsigned long)&trans);
  if (0 != ret)
    {
      printf("%s:ioctl port%d failed\n", __func__, port);
    }

  close(fd);
  return ret;
}

static int muxpin_set_driver_strength(int port, int level)
{
  int fd, ret;
  struct pinctrl_iotrans_s trans;

  fd = open("/dev/pinctrl0", 0);
  if (fd < 0)
    {
      printf("open /dev/pinctrl0 failed\n");
      return -ENXIO;
    }

  trans.pin = (uint32_t)port;
  trans.para.level = (uint32_t)level;

  ret = ioctl(fd, PINCTRLC_SETDS, (unsigned long)&trans);
  if (0 != ret)
    {
      printf("%s:ioctl port%d failed\n", __func__, port);
    }

  close(fd);
  return ret;
}

static int gpio_port_register(int port, enum gpio_pintype_e type)
{
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
  if ( 0 != ret)
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
  if ( 0 != ret)
    {
      printf("%s:P%u failed\n", __func__, (unsigned int)port);
    }

  close(fd);
  return ret;
}

static void gpio_loopback_test(int gpin, int gpout)
{
  int i = 0;
  bool gpin_state;
  int ret;

  while (++i <= 5)
    {
      ret = gpio_port_write(gpout, (bool)(i & 0x1));
      sleep(1);
      ret |= gpio_port_read(gpin, &gpin_state);
      if ((0 != ret) || ((int)gpin_state != (i & 0x1)))
        {
          break;
        }
    }

  if (i > 5)
    {
      printf("%s:success\n", __func__);
    }
  else
    {
      printf("%s:failure\n", __func__);
    }
}

/****************************************************************************
 * Public function
 ****************************************************************************/

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int gpiodemo_main(int argc, char *argv[])
#endif
{
  int ret, gpin, gpout;
  bool gpin_state;

  if (3 != argc)
    {
      printf("usage: %s pin1 pin2\nPin number range 4--41\n", argv[0]);
      goto end;
    }

  gpin = atoi(argv[1]);
  gpout = atoi(argv[2]);

  ret = muxpin_select_gpio(gpin);
  ret |= muxpin_set_driver_type(gpin, BIAS_DISABLE);
  ret |= gpio_port_register(gpin, GPIO_INPUT_PIN);
  if (0 != ret)
    {
      goto end;
    }

  ret = muxpin_select_gpio(gpout);
  ret |= muxpin_set_driver_type(gpout, BIAS_DISABLE);
  ret |= muxpin_set_driver_strength(gpout, GPIO_DS_LEVEL0);
  ret |= gpio_port_register(gpout, GPIO_OUTPUT_PIN);
  if (0 != ret)
    {
      goto end;
    }

  if (0 == gpio_port_read(gpin, &gpin_state))
    {
      printf("read gpio%d:%d\n", gpin, gpin_state);
    }

  if (0 == gpio_port_write(gpout, (bool)1))
    {
      printf("write gpio%d:1\n", gpout);
    }

  gpio_loopback_test(gpin, gpout);

end:
  return EXIT_SUCCESS;
}
