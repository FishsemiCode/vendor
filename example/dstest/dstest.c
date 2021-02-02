/****************************************************************************
 * apps/external/example/dstest/dstest.c
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
#include <nuttx/timers/rtc.h>
#include <nuttx/environ.h>
#include <nuttx/misc/misc_rpmsg.h>
#include <nuttx/power/pm.h>

#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "at_api.h"
#include "cis_internals.h"

/****************************************************************************
 * Macro definition
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Funtions
 ****************************************************************************/

/****************************************************************************
 * Public function
 ****************************************************************************/
static void rtc_timer_set(int seconds)
{
  struct rtc_time curtime;
  time_t nxtime;
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

  fd = open("/dev/rtc0", 0);
  if (fd < 0)
  {
    syslog(0, "%s, error, line %d\n", __func__, __LINE__);
    return;
  }

  ioctl(fd, RTC_RD_TIME, (unsigned long)&curtime);
  syslog(0, "Curtime: h:m:s [%02d:%02d:%02d], Y:M:D [%04d:%02d:%02d]\n",
          curtime.tm_hour, curtime.tm_min, curtime.tm_sec,
          curtime.tm_year, curtime.tm_mon, curtime.tm_mday);
  syslog(0, "func:%s line:%d seconds:%d\n", __func__, __LINE__, seconds);

  nxtime = mktime((FAR struct tm *)&curtime) + seconds;
  gmtime_r(&nxtime, (FAR struct tm *)&alarminfo.time);

  ioctl(fd, RTC_SET_ALARM, (unsigned long)&alarminfo);
  close(fd);
}

static void wakeup_cp(void)
{
  int fd;
  fd = open("/dev/miscsp", 0);
  if (fd >= 0)
    {
      struct misc_remote_boot_s remote =
      {
        .name = "cp",
      };

      ioctl(fd, MISC_REMOTE_BOOT, (unsigned long)&remote);

      close(fd);
    }
}

extern uintptr_t _sreserdata;
static int gps_ok;
pthread_mutex_t gps_mutex;

static void handle_gprmc(const char *s)
{
  int ret;
  FAR char *p;
  FAR char *ignore;
  bool ready = false;
  FAR char *line = (char *)s;
  FAR char *p_longitude, *p_latitude, *p_time, *p_date, *p_speed;

  printf("s = %s\n", s);

  ret = at_tok_nextstr(&line, &p);
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextstr(&line, &p_time);
  if (ret < 0)
    {
      return;
    }

  ret = at_tok_nextstr(&line, &p);
  if (ret < 0)
    {
      return;
    }

  printf("*P = %c\n", *p);
  if (*p == 'A') // If outside, just check p = 'A'
    {
      ready = true;
    }

  ret = at_tok_nextstr(&line, &p_latitude);
  if (ret < 0)
    {
      return;
    }

  ret = at_tok_nextstr(&line, &p);
  if (ret < 0)
    {
      return;
    }

  ret = at_tok_nextstr(&line, &p_longitude);
  if (ret < 0)
    {
      return;
    }

  ret = at_tok_nextstr(&line, &ignore);
  if (ret < 0)
    {
      return;
    }

  ret = at_tok_nextstr(&line, &p_speed);
  if (ret < 0)
    {
      return;
    }

  ret = at_tok_nextstr(&line, &ignore);
  if (ret < 0)
    {
      return;
    }

  ret = at_tok_nextstr(&line, &p_date);
  if (ret < 0)
    {
      return;
    }

  if (ready)
    {
      struct tm t;
      time_t seconds;
      uint64_t mSeconds;
      char buf[4] = {0};

      strncpy(buf, p_time + 4, 2);
      t.tm_sec = atoi(buf);
      strncpy(buf, p_time + 2, 2);
      t.tm_min = atoi(buf);
      strncpy(buf, p_time, 2);
      t.tm_hour = atoi(buf);
      strncpy(buf, p_date, 2);
      t.tm_mday = atoi(buf);
      strncpy(buf, p_date + 2, 2);
      t.tm_mon = atoi(buf);
      t.tm_mon -= 1;
      strncpy(buf, p_date + 4, 2);
      t.tm_year = atoi(buf) + 100;
      seconds = mktime(&t);
      strncpy(buf, p_time + 7, 2);
      mSeconds = (uint64_t)(seconds) * 1000 + atoi(buf);
      printf("\n\n\ngps: %s,%s,%s,%s,%013llu,%u\n\n\n", p_time, p_latitude, p_longitude,
             p_date, mSeconds, seconds);
      pthread_mutex_lock(&gps_mutex);
      gps_ok = 1;
      pthread_mutex_unlock(&gps_mutex);
    }
}

static void gps_init(void)
{
  int ret;
  int at_fd;

  ret = ciscom_gps_initialize();
  if (ret < 0)
    {
      printf("failed to init gps!\n");
      return;
    }
  at_fd = ciscom_getATHandle();
  printf("at_fd = %d\n", at_fd);
  register_indication(at_fd, "$GPRMC", handle_gprmc);
  start_gps(at_fd, false);
}

static void gps_deinit(void)
{
  int at_fd;

  at_fd = ciscom_getATHandle();
  stop_gps(at_fd);
  deregister_indication(at_fd, "$GPRMC");
}

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int dstest_main(int argc, char *argv[])
#endif
{
  char bootnb_str[3] = {0};
  unsigned int *flag = (unsigned int *)&_sreserdata;
  unsigned int *bootflag = &flag[1];

  if (argc != 2)
    {
      syslog(0, "[usage]: dstest [rtc_time]\n");
      return 0;
    }

  if (*flag != 0xaa55aa55)
    {
      syslog(0, "This is first boot, set flag!\n");
      *flag = 0xaa55aa55;
      *bootflag = 0x2;
    }
  else if (*bootflag == 0x2)
    {
      syslog(0, "Only boot with nb!\n");
      sprintf(bootnb_str, "%d", *bootflag);
      setenv_global("bootnb", bootnb_str, 2);
      wakeup_cp();
      *bootflag = 0x1;
    }
  else if (*bootflag == 0x1)
    {
      pm_stay(PM_IDLE_DOMAIN, PM_STANDBY);
      syslog(0, "Only boot with gps!\n");
      sprintf(bootnb_str, "%d", *bootflag);
      setenv_global("bootnb", bootnb_str, 2);
      wakeup_cp();
      *bootflag = 0x3;
      pthread_mutex_init(&gps_mutex, NULL);
      printf("Start gps init!\n");
      gps_init();
      while (1)
        {
          pthread_mutex_lock(&gps_mutex);
          if (gps_ok)
            {
              pthread_mutex_unlock(&gps_mutex);
              break;
            }
          pthread_mutex_unlock(&gps_mutex);
          usleep(100000);
        }
      gps_deinit();
      pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
    }
  else if (*bootflag == 0x3)
    {
      syslog(0, "boot with gps and nb!\n");
      sprintf(bootnb_str, "%d", *bootflag);
      setenv_global("bootnb", bootnb_str, 2);
      wakeup_cp();
      *bootflag = 0x2;
    }

  rtc_timer_set(atoi(argv[1]));

  return 0;
}
