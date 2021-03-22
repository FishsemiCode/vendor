/****************************************************************************
 * apps/external/example/api_dcellsysinfo/api_dcellsysinfo_main.c
 *
 *     Copyright (C) 2019 FishSemi Inc. All rights reserved.
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
#include <stdio.h>
#include <fcntl.h>
#include <sys/time.h>

#include "at_api.h"
/****************************************************************************
 * Public Funtions
 ****************************************************************************/

static pthread_mutex_t g_mutex;
static pthread_cond_t g_cond;

static void handle_cellsysinfo(const char *s)
{
  int frequency;
  int pci;
  char *str = (char *)s;
  if(str == NULL)
    {
      syslog(LOG_INFO, "%s:str is null\n", __func__);
      return;
    }
  syslog(LOG_INFO, "%s: %s\n", __func__, s);

  // ^DCellInfo:948600000,173
  int ret = at_tok_start(&str);
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextint(&str, &frequency);
  if (ret < 0)
    {
      return;
    }
  syslog(LOG_INFO, "%s: frequency:%d\n", __func__, frequency);
  ret = at_tok_nextint(&str, &pci);
  if (ret < 0)
    {
      return;
    }
  syslog(LOG_INFO, "%s: pci:%d\n", __func__, pci);

  pthread_mutex_lock(&g_mutex);
  pthread_cond_signal(&g_cond);
  pthread_mutex_unlock(&g_mutex);

  return;
}

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int api_dcellsysinfo_main(int argc, char *argv[])
#endif
{
  int fd = -1;
  int ret = 0;
  struct timeval now;
  struct timespec time;
  syslog(LOG_INFO, "api dcellsysinfo test...\n");

  pthread_mutex_init(&g_mutex, NULL);
  pthread_cond_init(&g_cond, NULL);

  fd = at_client_open();
  if(fd < 0)
    {
      syslog(LOG_ERR, "%s: Open client error\n", __func__);
      return -1;
    }

  ret = register_indication(fd, "^DCellSysInfo", handle_cellsysinfo);
  if(ret < 0)
    {
      syslog(LOG_ERR, "%s: register_indication fail\n", __func__);
      goto clean;
    }

  ret = set_radiopower(fd, false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: set_radiopower false fail\n", __func__);
      goto clean;
    }
  sleep(5);

  pthread_mutex_lock(&g_mutex);

  ret = set_radiopower(fd, true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: set_radiopower false true fail\n", __func__);
      goto clean;
    }

  gettimeofday(&now, NULL);
  time.tv_sec = now.tv_sec + 80;
  time.tv_nsec = now.tv_usec * 1000;
  ret = pthread_cond_timedwait(&g_cond, &g_mutex, &time);
  if(ret != 0)
  {
    syslog(LOG_ERR, "%s: handle_cellsysinfo fail\n", __func__);
  }

  pthread_mutex_unlock(&g_mutex);

  syslog(LOG_INFO, "api register end\n");

clean:
  if(fd >= 0)
    {
      at_client_close(fd);
    }
  pthread_mutex_destroy(&g_mutex);
  pthread_cond_destroy(&g_cond);
  syslog(LOG_INFO, "%s:quit\n", __func__);
  return ret;
}
