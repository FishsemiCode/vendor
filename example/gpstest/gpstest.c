/****************************************************************************
 * apps/external/example/gpstest/gpstest.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/ascii.h>
#include <nuttx/serial/pty.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include <fcntl.h>
#include <poll.h>
#include <sched.h>
#include <string.h>
#include <strings.h>

#include <sys/socket.h>
#include <arpa/inet.h>

#include <sys/time.h>
#include <sys/boardctl.h>
#include <nuttx/board.h>
#include "at_api.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

// TO-DO:
#define REMOTE_IPADDR           "101.200.151.218"   // Server IP Address
#define REMOTE_TCPPORT          9589           // Server TCP port number

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Funtions
 ****************************************************************************/
static char g_gps_data[512];
static int g_gps_flag;
static int g_reg_staus = -1;
static unsigned char g_gps_enable_flag = 1;

static pthread_mutex_t g_gps_mutex;
static pthread_cond_t g_gps_cond;
static unsigned char g_imei_value[IMEI_LENGTH + 1];

static char g_gps_decoder[3][32];

static unsigned char g_gps_timeout = 60;

int get_gps_info(int fd)
{
  int ret;
  struct timespec time;
  struct timeval now;

  syslog(LOG_INFO, "%s: Stop NB by CFUN=0\n", __func__);
  ret = set_radiopower(fd, false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: set_radiopower fail\n", __func__);
      return -1;
    }

  ret = start_gps(fd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: start_gps fail\n", __func__);
      return -1;
    }

  // wait for GPS position data
  gettimeofday(&now, NULL);
  time.tv_sec = now.tv_sec + g_gps_timeout;
  time.tv_nsec = now.tv_usec * 1000;
  syslog(LOG_INFO, "%s: Wait for GPS data:%ld\n", __func__, time.tv_sec);
  pthread_mutex_lock(&g_gps_mutex);
  ret = pthread_cond_timedwait(&g_gps_cond, &g_gps_mutex, &time);
  syslog(LOG_INFO, "%s: pthread_cond_timedwait:%d\n", __func__, ret);
  pthread_mutex_unlock(&g_gps_mutex);

  if (g_gps_flag == 1)
  {
    syslog(LOG_INFO, "%s: wait 30s to save GPS parameters\n", __func__);
    usleep(30000000);
    g_gps_flag = 2;
  }

  // stop GPS

  ret = stop_gps(fd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: stop_gps fail\n", __func__);
      return -1;
    }

  // start NB
  syslog(LOG_INFO, "%s: Start NB CFUN=1\n", __func__);
  ret = set_radiopower(fd, true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: set_radiopower fail\n", __func__);
      return -1;
    }
  return 0;
}

int get_nb_imei(int fd)
{
  int ret;
  at_spi_imei imei;
  ret = get_imei(fd, &imei);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: get_imei fail\n", __func__);
      return -1;
    }
  memcpy(g_imei_value, imei.imei, sizeof(g_imei_value));
  syslog(LOG_INFO, "%s: Get IMEI %s\n", __func__, g_imei_value);
  return 0;
}

void nb_send2server(int fd)
{
  int sock_fd;
  struct sockaddr_in remote;
  int ret;
  int cnt = 0;
  struct timeval tv;
  at_api_cellinfo cellinfo;

retry:
  cnt ++;
  if (cnt >= 5)
    {
      syslog(LOG_ERR, "%s: send NB data failed after repeat 5 times\n", __func__);
      return;
    }
  do
    {
      sock_fd = socket(AF_INET, SOCK_STREAM, 0);
      if (sock_fd < 0)
        {
          syslog(LOG_ERR, "%s: create sock failed %d Err\r\n", __func__, errno);
        }
      else
        {
          syslog(LOG_INFO, "%s: create sock successful\r\n", __func__);
        }
      } while (sock_fd < 0);

  tv.tv_sec = 2;
  tv.tv_usec = 0;
  setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  remote.sin_family = AF_INET;
  remote.sin_port = HTONS(REMOTE_TCPPORT);
  ret = inet_pton(AF_INET, REMOTE_IPADDR, &remote.sin_addr.s_addr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: error remot IP addr %d\r\n", __func__, ret);
    }

  // connect
  do
    {
      ret = connect(sock_fd, (FAR const struct sockaddr *)&remote, sizeof(remote));
      if (ret < 0)
        {
          close(sock_fd);
          syslog(LOG_ERR, "%s: connect failed %d\r\n", __func__, errno);
          goto retry;
        }
      } while (ret < 0);
  syslog(LOG_INFO, "%s: connect success\r\n", __func__);

  ret = get_cellinfo(fd, &cellinfo);
  if (ret < 0)
    {
      close(sock_fd);
      syslog(LOG_ERR, "%s: get_cellinfo fail\n", __func__);
      goto retry;
    }

  sprintf(g_gps_data, "IMEI:%s,GPS=%s,%s,%s,\ncellinfo=MCC:%s MNC:%s CELL_ID:%x \
    LAC_ID:%x RSRP:%d RSRQ:%d SNR:%d BAND:%d ARFCN:%d PCI:%d\n", g_imei_value, g_gps_decoder[0], g_gps_decoder[1], g_gps_decoder[2],
    cellinfo.mcc, cellinfo.mnc, cellinfo.cellId, cellinfo.lacId, cellinfo.rsrp, cellinfo.rsrq, cellinfo.snr,
    cellinfo.band, cellinfo.arfcn, cellinfo.pci);

  // socket to send GPS info to server
  ret = sendto(sock_fd, g_gps_data, strlen((char *)g_gps_data), 0, NULL, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: sock send to failed %d\r\n", __func__, errno);
      close(sock_fd);
      goto retry;
    }
  syslog(LOG_INFO, "%s: send data to server successfully, and try to recv the ack\r\n", __func__);

  // Using Server ACK to trigger GPS enable/disable
  ret = recvfrom(sock_fd, g_gps_data, sizeof(g_gps_data) - 1, 0, NULL, 0);
  if (ret > 0)
    {
      g_gps_data[ret] = 0;
      ret = memcmp(g_gps_data, "GPS=0", 5);
      if (ret == 0)
        {
          g_gps_enable_flag = 0;
        }
      ret = memcmp(g_gps_data, "GPS=1", 5);
      if (ret == 0)
        {
          g_gps_timeout = g_gps_data[6] - 0x30;
          g_gps_timeout = (g_gps_timeout * 10) + (g_gps_data[7] - 0x30);
          g_gps_enable_flag = 1;
        }
      syslog(LOG_INFO, "%s: NB received %s \r\n", __func__, g_gps_data);
    }
  else
    {
      syslog(LOG_INFO, "%s: Nothing Recvied from Server\r\n", __func__);
    }
  close(sock_fd);
}

static bool is_registered(int reg_Status)
{
  return g_reg_staus == 1 || g_reg_staus == 5;
}

static void handle_cereg(const char *s)
{
  int ret;
  int value;
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
  pthread_mutex_lock(&g_gps_mutex);
  g_reg_staus = value;
  pthread_cond_signal(&g_gps_cond);
  pthread_mutex_unlock(&g_gps_mutex);
}

static void handle_gprmc(const char *s)
{
  int ret;
  char *line = (char *)s;
  char *p;
  bool ready = false;
  char *p_longitude, *p_latitude, *p_time;
  syslog(LOG_INFO, "%s: %s\n", __func__, s);
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

  if (*p == 'A')
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

  syslog(LOG_ERR, "%s: %d,%s,%s,%s\n", __func__, ready, p_time, p_latitude, p_longitude);
  if (ready)
    {
      pthread_mutex_lock(&g_gps_mutex);
      if (g_gps_flag == 0)
        {
          g_gps_flag = 1;
        }
      strcpy(g_gps_decoder[0], p_time);
      strcpy(g_gps_decoder[1], p_latitude);
      strcpy(g_gps_decoder[2], p_longitude);
      pthread_cond_signal(&g_gps_cond);
      pthread_mutex_unlock(&g_gps_mutex);
    }
}


#ifdef BUILD_MODULE
int main(int argc, char *argv[])
#else
int gpstest_main(int argc, char *argv[])
#endif
{
  int ret;
  int clientfd;
  int errCnt = 0;
  syslog(LOG_INFO, "%s: gpstest running\n", __func__);

  pthread_mutex_init(&g_gps_mutex, NULL);
  pthread_cond_init(&g_gps_cond, NULL);

  clientfd = at_client_open();
  if(clientfd < 0)
    {
      syslog(LOG_ERR, "%s: Open client error\n", __func__);
      goto clean;
    }

  ret = register_indication(clientfd, "+CEREG", handle_cereg);
  if(ret < 0)
    {
      syslog(LOG_ERR, "%s: register_indication fail\n", __func__);
      goto clean;
    }
  ret = register_indication(clientfd, "$GPRMC", handle_gprmc);
  if(ret < 0)
    {
      syslog(LOG_ERR, "%s: register_indication fail\n", __func__);
      goto clean;
    }

  ret = set_ceregindicationstatus(clientfd, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "set_ceregindicationstatus fail\n");
      goto clean;
    }

  get_nb_imei(clientfd);

  // default latitude and langitude
  strcpy((void *)g_gps_decoder[1], "34.0");
  strcpy((void *)g_gps_decoder[2], "16.0");
  while (1)
    {
      // get GPS data
      if (g_gps_enable_flag == 1)
        {
          if (get_gps_info(clientfd) < 0)
            {
              errCnt++;
              if (errCnt > 5)
                {
                  board_reset(3);
                }
              continue;
            }
        }
      pthread_mutex_lock(&g_gps_mutex);
      while (!is_registered(g_reg_staus))
        {
          pthread_cond_wait(&g_gps_cond, &g_gps_mutex);
        }
      pthread_mutex_unlock(&g_gps_mutex);
      // send data to NB
      nb_send2server(clientfd);
    }

clean:
  at_client_close(clientfd);
  pthread_mutex_destroy(&g_gps_mutex);
  pthread_cond_destroy(&g_gps_cond);
  syslog(LOG_ERR, "%s: quit\n", __func__);
  return ret;
}
