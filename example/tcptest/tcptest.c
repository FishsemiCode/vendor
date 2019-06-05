/****************************************************************************
 * apps/external/example/tcptest/tcptest.c
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
static int g_reg_staus = -1;
static pthread_mutex_t g_tcp_mutex;
static pthread_cond_t g_tcp_cond;
static unsigned char g_imei_value[IMEI_LENGTH + 1];
static char g_nb_data[400];


static int get_nb_imei(int fd)
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

static void nb_send2server(int fd, char *addr, int port)
{
  int sock_fd;
  struct sockaddr_in remote;
  int ret;
  int cnt = 0;
  at_api_cellinfo cellinfo;
  struct linger ling;

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

  remote.sin_family = AF_INET;
  remote.sin_port = HTONS(port);
  ret = inet_pton(AF_INET, addr, &remote.sin_addr.s_addr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: error remot IP addr %d\r\n", __func__, ret);
    }

  ling.l_onoff  = 1;
  ling.l_linger = 5;     /* timeout is seconds */

  if (setsockopt(sock_fd, SOL_SOCKET, SO_LINGER, &ling, sizeof(struct linger)) < 0)
    {
      syslog(LOG_ERR, "%s: set LINGER failed %d wait 1s to repeat\r\n", __func__, errno);
      goto retry;
    }

  // connect
  do
    {
      ret = connect(sock_fd, (FAR const struct sockaddr *)&remote, sizeof(remote));
      if (ret < 0)
        {
          close(sock_fd);
          syslog(LOG_ERR, "%s: connect failed %d wait 1s to repeat\r\n", __func__, errno);
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

  sprintf(g_nb_data, "IMEI:%s,cellinfo=MCC:%s MNC:%s CELL_ID:%x \
    LAC_ID:%x RSRP:%d RSRQ:%d SNR:%d BAND:%d ARFCN:%d PCI:%d\n", g_imei_value,
    cellinfo.mcc, cellinfo.mnc, cellinfo.cellId, cellinfo.lacId, cellinfo.rsrp, cellinfo.rsrq, cellinfo.snr,
    cellinfo.band, cellinfo.arfcn, cellinfo.pci);

  ret = sendto(sock_fd, g_nb_data, strlen((char *)g_nb_data), 0, NULL, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: sock send to failed %d\r\n", __func__, errno);
      close(sock_fd);
      goto retry;
    }
  syslog(LOG_INFO, "%s: send data to server successfully, and try to recv the ack\r\n", __func__);

  ret = recvfrom(sock_fd, g_nb_data, sizeof(g_nb_data) - 1, 0, NULL, 0);
  if (ret > 0)
    {
      syslog(LOG_INFO, "%s: NB received %s \r\n", __func__, g_nb_data);
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
  pthread_mutex_lock(&g_tcp_mutex);
  g_reg_staus = value;
  pthread_cond_signal(&g_tcp_cond);
  pthread_mutex_unlock(&g_tcp_mutex);
}

static void usage(void)
{
  syslog(LOG_ERR, "Usage: tcptest [-a addr] [-p port]\n"
      "\t-a addr\t\tThe remote ip address\n"
      "\t-p port\t\tThe remote port\n");
}


#ifdef BUILD_MODULE
int main(int argc, char *argv[])
#else
int tcptest_main(int argc, char *argv[])
#endif
{
  int ret;
  int clientfd;
  int opt;
  int port = 0;
  char addr[INET6_ADDRSTRLEN];
  memset(addr, 0x0, sizeof(addr));

  if (argc < 3)
    {
      usage();
      exit(1);
    }
  while ((opt = getopt(argc, argv, "a:p:")) != -1)
    {
      switch (opt)
        {
          case 'a':
            strncpy(addr, optarg, sizeof(addr) - 1);
            break;
          case 'p':
            port = atoi(optarg);
            break;
          default:
            usage();
            exit(1);
          }
    }
  if (port == 0 || addr[0] == '\0')
    {
      usage();
      exit(1);
    }
  syslog(LOG_INFO, "%s: tcptest running\n", __func__);

  pthread_mutex_init(&g_tcp_mutex, NULL);
  pthread_cond_init(&g_tcp_cond, NULL);

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

  ret = set_ceregindicationstatus(clientfd, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "set_ceregindicationstatus fail\n");
      goto clean;
    }

  get_nb_imei(clientfd);

  while (1)
    {
      pthread_mutex_lock(&g_tcp_mutex);
      while (!is_registered(g_reg_staus))
        {
          pthread_cond_wait(&g_tcp_cond, &g_tcp_mutex);
        }
      pthread_mutex_unlock(&g_tcp_mutex);
      // send data to NB
      nb_send2server(clientfd, addr, port);
      usleep(5000000);
    }

clean:
  at_client_close(clientfd);
  pthread_mutex_destroy(&g_tcp_mutex);
  pthread_cond_destroy(&g_tcp_cond);
  syslog(LOG_ERR, "%s: quit\n", __func__);
  return ret;
}
