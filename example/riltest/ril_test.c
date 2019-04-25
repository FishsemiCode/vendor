/****************************************************************************
 * apps/external/example/riltest/ril_test.c
 *
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Pinecone <pinecone@pinecone.net>
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

#include <nuttx/config.h>

#include <nuttx/serial/pty.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/un.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <sched.h>
#include <string.h>
#include <strings.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "at_api.h"


static void handle_cereg(const char *s)
{
  syslog(LOG_INFO, "%s: %s\n", __func__, s);
}

static int riltest_daemon(int argc, char *argv[])
{
  syslog(LOG_INFO, "%s: riltest_daemon running\n", __func__);
  int ret;
  int clientfd;

  clientfd = at_client_open();
  if(clientfd < 0)
    {
      syslog(LOG_ERR, "Open client error\n");
      return -1;
    }

  ret = set_radiopower(clientfd, true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "set_radiopower fail\n");
      goto clean;
    }

  ret = register_indication(clientfd, "+CEREG", handle_cereg);
  if(ret < 0)
    {
      syslog(LOG_ERR, "register_indication fail\n");
      goto clean;
    }

  ret = set_ceregindicationstatus(clientfd, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "set_ceregindicationstatus fail\n");
      goto clean;
    }
  at_spi_imei imei;
  ret = get_imei(clientfd, &imei);
  if (ret < 0)
    {
      syslog(LOG_ERR, "get_imei fail\n");
      goto clean;
    }
  syslog(LOG_ERR, "getImei :%s\n", imei.imei);

  at_api_imsi imsi;
  ret = get_imsi(clientfd, &imsi);
  if (ret < 0)
    {
      syslog(LOG_ERR, "get_imsi fail\n");
      goto clean;
    }
  syslog(LOG_ERR, "getImsi :%s\n", imsi.imsi);

  at_api_cellinfo cellinfo;
  ret = get_cellinfo(clientfd, &cellinfo);
  if (ret < 0)
    {
      syslog(LOG_ERR, "get_cellinfo fail\n");
      goto clean;
    }
  syslog(LOG_INFO, "cell info: mcc:%s mnc:%s, curr_mod:%d, duplex_mode:%d, \
    ue_category:%d, cellId:%x, lacId:%x, rsrp:%d, rsrq:%d, snr:%d, band:%d, \
    arfcn:%u, pci:%d\n", cellinfo.mcc, cellinfo.mnc, cellinfo.curr_mod, cellinfo.duplex_mode,
    cellinfo.ue_category, cellinfo.cellId, cellinfo.lacId, cellinfo.rsrp, cellinfo.rsrq, cellinfo.snr,
    cellinfo.band, cellinfo.arfcn, cellinfo.pci);

  syslog(LOG_ERR, "riltest_daemon running successfully\n");
clean:
  at_client_close(clientfd);
  return ret;
}
/****************************************************************************
 * Public Funtions
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, char *argv[])
#else
int riltest_main(int argc, char *argv[])
#endif
{
  int ret;
  ret = task_create(argv[0],
          CONFIG_RILTEST_PRIORITY,
          CONFIG_RILTEST_STACKSIZE,
          riltest_daemon,
          argv + 1);
  return ret > 0 ? 0 : ret;
}

