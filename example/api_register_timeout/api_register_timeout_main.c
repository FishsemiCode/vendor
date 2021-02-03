/****************************************************************************
 * apps/external/example/api_register_timeout/api_register_timeout_main.c
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

static int g_reg_staus = -1;
static pthread_mutex_t g_nb_mutex;
static pthread_cond_t g_nb_cond;
struct timeval now;
struct timespec times;

static bool is_registered(int reg_Status)
{
  return reg_Status == 1 || reg_Status == 5;
}

static void handle_cereg(const char *s)
{
  int ret = -1;
  int value = -1;
  char *line = (char *)s;
  syslog(LOG_INFO, "%s: %s\n", __func__, s);
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextint(&line, &value);
  if (ret < 0)
    {
      return;
    }
  pthread_mutex_lock(&g_nb_mutex);
  //Only the timer is released, after the registration is successful
  if(is_registered(value))
    {
      pthread_cond_signal(&g_nb_cond);
    }
  g_reg_staus = value;
  pthread_mutex_unlock(&g_nb_mutex);
}

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int api_register_timeout_main(int argc, char *argv[])
#endif
{
  int fd = -1;
  int ret = -1;
  int regErrCnt = 0;
  syslog(LOG_INFO, "api register timeout test...\n");
  pthread_mutex_init(&g_nb_mutex, NULL);
  pthread_cond_init(&g_nb_cond, NULL);

  fd = at_client_open();
  if(fd < 0)
    {
      syslog(LOG_ERR, "%s: Open client error\n", __func__);
      return -1;
    }
  syslog(LOG_INFO, "[%s]fd:%d\n", __func__, fd);

  ret = register_indication(fd, "+CEREG", handle_cereg);
  if(ret < 0)
    {
      syslog(LOG_ERR, "%s: register_indication fail\n", __func__);
      return -1;
    }
  syslog(LOG_INFO, "[%s]ret:%d\n", __func__, ret);

  ret = set_ceregindicationstatus(fd, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "set_ceregindicationstatus fail\n");
      return -1;
    }

  sleep(5);

  while (!is_registered(g_reg_staus))
    {
      syslog(LOG_INFO, "api register while\n");
      pthread_mutex_lock(&g_nb_mutex);
      gettimeofday(&now, NULL);
      times.tv_sec = now.tv_sec + 60;
      times.tv_nsec = now.tv_usec * 1000;
      ret = pthread_cond_timedwait(&g_nb_cond, &g_nb_mutex, &times);
      if (ret != 0)
        {
          pthread_mutex_unlock(&g_nb_mutex);
          regErrCnt++;
          syslog(LOG_ERR, "reg fail, regErrCnt:%d\n", regErrCnt);
          if(regErrCnt > 2)
            {
              break;
            }
        }
    }

  if (is_registered(g_reg_staus))
    {
      pthread_mutex_unlock(&g_nb_mutex);
    }
  syslog(LOG_INFO, "api register end\n");

  return 0;
}
