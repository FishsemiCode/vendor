/****************************************************************************
 * vendor/apps/fishled/fishled.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
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
#include <nuttx/leds/fishled.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * fishled_main
 ****************************************************************************/

#ifndef CONFIG_U1_APFACTORY
int fishled_main(int argc, char *argv[])
{
  bool set, get;
  int ret;
  int fd;

  fd = open("/dev/fishled", 0);
  if (fd < 0)
    {
      printf("open device failed %d\n", errno);
      return errno;
    }

  set = true;

  while (1)
    {
      ret = ioctl(fd, FISHLED_SET, set);
      if (ret)
        {
          break;
        }

      ret = ioctl(fd, FISHLED_GET, (unsigned long)&get);
      if (ret)
        {
          break;
        }

      printf("current led: %s\n", get ? "ON" : "OFF");

      set = !set;
      sleep(2);
    }

  close(fd);
  return ret;
}
#else
int fishled_main(int argc, char *argv[])
{
  bool set;
  int ret;
  int fd;

  fd = open("/dev/fishled", 0);
  if (fd < 0)
    {
      printf("open device failed %d\n", errno);
      return errno;
    }

  set = true;

  ret = ioctl(fd, FISHLED_SET, set);
  if (ret)
    {
      printf("fishled set failed\n");
    }

  close(fd);
  return ret;
}
#endif
