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

#define SIGVALUE_INT  42

/****************************************************************************
 * Static Data
 ****************************************************************************/

static sem_t g_rtc_sem;
static int g_rtc_received;

/****************************************************************************
 * Private Function
 ****************************************************************************/

static void rtc_callbck(union sigval value)
{
  int sival_int = value.sival_int;

  printf("%s: Received value %d, post semaphore\n", __func__, sival_int);

  g_rtc_received = sival_int;
  sem_post(&g_rtc_sem);
}

/****************************************************************************
 * rtc_test
 ****************************************************************************/

static void rtc_test(int sec)
{
  struct rtc_time curtime;
  time_t nxtime;
  int status;
  int fd;

  struct rtc_setalarm_s alarminfo =
  {
    .id  = 0,
    .pid = 0,
    .event =
    {
      .sigev_notify            = SIGEV_THREAD,
      .sigev_signo             = SIGALRM,
      .sigev_value.sival_int   = SIGVALUE_INT,
#ifdef CONFIG_SIG_EVTHREAD
      .sigev_notify_function   = rtc_callbck,
      .sigev_notify_attributes = NULL,
#endif
    }
  };

  sem_init(&g_rtc_sem, 0, 0);

  fd = open("/dev/rtc0", 0);
  if (fd < 0)
  {
    printf("%s, error, line %d\n", __func__, __LINE__);
    return;
  }

  ioctl(fd, RTC_RD_TIME, (unsigned long)&curtime);
  nxtime = mktime((FAR struct tm *)&curtime) + sec;
  gmtime_r(&nxtime, (FAR struct tm *)&alarminfo.time);
  ioctl(fd, RTC_SET_ALARM, (unsigned long)&alarminfo);

  printf("Curtime: [%02d:%02d:%02d], should wakeup at [%02d:%02d:%02d]\n",
          curtime.tm_hour, curtime.tm_min, curtime.tm_sec,
          alarminfo.time.tm_hour, alarminfo.time.tm_min, alarminfo.time.tm_sec);

  printf("%s: Waiting on semaphore\n", __func__);

  do
    {
      status = sem_wait(&g_rtc_sem);
      if (status < 0)
        {
          int error = errno;
          if (error == EINTR)
            {
              printf("%s: sem_wait() interrupted by signal\n", __func__);
            }
          else
            {
              printf("%s: ERROR sem_wait failed, errno=%d\n", __func__, error);
              goto errorout;
            }
        }
    }
  while (status < 0);

  printf("%s: Awakened with no error!\n", __func__);

  /* Check sigval */

  if (g_rtc_received != SIGVALUE_INT)
    {
      printf("rtc_callbck: ERROR sival_int=%d expected %d\n",
             g_rtc_received, SIGVALUE_INT);
    }

  ioctl(fd, RTC_RD_TIME, (unsigned long)&curtime);
  printf("Curtime: [%02d:%02d:%02d], ACT\n", curtime.tm_hour, curtime.tm_min, curtime.tm_sec);

errorout:
  sem_destroy(&g_rtc_sem);
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

  rtc_test(atoi(argv[1]));

  return EXIT_SUCCESS;
}
