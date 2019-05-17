/****************************************************************************
 * apps/external/example/rtctest/rtctest.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Bo Zhang <zhangbo@fishsemi.com>
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

#include <nuttx/timers/rtc.h>


/****************************************************************************
 * Private Function
 ****************************************************************************/

/****************************************************************************
 * rtc_test
 ****************************************************************************/

# define putreg32(v,a)        (*(volatile uint32_t *)(a) = (v))

static void mask_sleep(void)
{
  putreg32(0x70007, 0xb004035c);
  putreg32(0x3000300, 0xb0040354);
}

static void rtc_test(int sec)
{
  int fd;

  struct rtc_setalarm_s alarminfo =
  {
    .id  = 0,
    .pid = 0,
    .event =
    {
      .sigev_notify            = SIGEV_SIGNAL,
      .sigev_signo             = SIGALRM,
      .sigev_value.sival_int   = 0,
#ifdef CONFIG_SIG_EVTHREAD
      .sigev_notify_function   = NULL,
      .sigev_notify_attributes = NULL,
#endif
    }
  };
  struct rtc_time curtime;

  fd = open("/dev/rtc0", 0);
  if (fd < 0)
  {
    printf("%s, error, line %d\n", __func__, __LINE__);
    return;
  }

  ioctl(fd, RTC_RD_TIME, (unsigned long)&curtime);
  printf("Curtime is %d\n", curtime.tm_sec);
  curtime.tm_sec += sec;
  memcpy(&alarminfo.time, &curtime, sizeof(struct rtc_time));
  printf("Should be wake at %d\n", alarminfo.time.tm_sec);
  ioctl(fd, RTC_SET_ALARM, (unsigned long)&alarminfo);
  sleep(100000);
  ioctl(fd, RTC_RD_TIME, (unsigned long)&curtime);
  printf("Actually curtime at %d\n", curtime.tm_sec);

  close(fd);
}

/****************************************************************************
 * Public function
 ****************************************************************************/
#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int rtctest_main(int argc, char *argv[])
#endif
{
  if (argc != 2) {
    printf("[usage]: rtctest [sec]\n");
    return EXIT_SUCCESS;
  }

  mask_sleep();
  rtc_test(atoi(argv[1]));

  return EXIT_SUCCESS;
}