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
#include <sys/ioctl.h>

#include <arpa/inet.h>

#include <sys/time.h>
#include <sys/boardctl.h>
#include <nuttx/board.h>
#include <nuttx/timers/rtc.h>
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

#define TCP_PAR_MAX_LENGTH 20

static int g_reg_staus = -1;
static pthread_mutex_t g_tcp_mutex;
static pthread_cond_t g_tcp_cond;
static unsigned char g_imei_value[IMEI_LENGTH + 1];
static char g_nb_data[400];

static char *g_tcp_statistics_path = "/data/tcpStatistics";

typedef enum
{
  TCP_STATISTICS_PAR_ADDR,
  TCP_STATISTICS_PAR_PORT,
  TCP_STATISTICS_PAR_TCPREMAINTESTCOUNT,
  TCP_STATISTICS_PAR_TCPINTERVAL,
  TCP_STATISTICS_PAR_ENTERDS,
  TCP_STATISTICS_PAR_ACTIONAFTERNBSENT,
  TCP_STATISTICS_COUNT,
}GPS_STATISTICS;

static char g_tcp_statistics_array[TCP_STATISTICS_COUNT][TCP_PAR_MAX_LENGTH];

ssize_t tcp_safe_write(int fd, const void *buf, size_t len)
{
  ssize_t once = 0, written = 0;

  while (len > 0)
    {
      once = write(fd, buf, len);
      if (once <= 0)
        {
          break;
        }
      len -= once;
      buf += once;
      written += once;
    }

  return written ? written : once;
}

ssize_t tcp_safe_read(int fd, void *buf, size_t len)
{
  ssize_t once = 0, readn = 0;

  while (len > 0)
    {
      once = read(fd, buf, len);
      if (once <= 0)
        {
          break;
        }
      len -= once;
      buf += once;
      readn += once;
    }

  return readn ? readn : once;
}

static int tcp_get_statistics(char *filePath)
{
  int fd;
  int size = 0;
  char *buf;
  char *p;
  int i;
  fd = open(filePath, O_RDWR | O_CREAT);
  if (fd < 0)
    {
      syslog(LOG_ERR, "%s: open %s error:%d, %s\n",
        __func__, filePath, errno, strerror(errno));
      return -1;
    }
  buf = malloc((TCP_PAR_MAX_LENGTH + 1) * TCP_STATISTICS_COUNT + 1);
  if (buf == NULL)
    {
      syslog(LOG_ERR, "%s: malloc error\n", __func__);
      close(fd);
      return -1;
    }
  buf[(TCP_PAR_MAX_LENGTH + 1) * TCP_STATISTICS_COUNT] = '\0';
  if ((size = tcp_safe_read(fd, buf, (TCP_PAR_MAX_LENGTH + 1) * TCP_STATISTICS_COUNT)) < 0)
    {
      syslog(LOG_ERR, "%s: read %s error:%d, %s\n",
        __func__, filePath, errno, strerror(errno));
      close(fd);
      free(buf);
      return -1;
    }

  if (size == 0)
    {
      for (i = 0; i < TCP_STATISTICS_COUNT; i++)
        {
          memcpy(buf + i * (TCP_PAR_MAX_LENGTH + 1), "0000000000;", TCP_PAR_MAX_LENGTH + 1);
        }
      tcp_safe_write(fd, buf, (TCP_PAR_MAX_LENGTH + 1) * TCP_STATISTICS_COUNT);
    }
  else
    {
      char *bufBak = buf;
      if ((size % (TCP_PAR_MAX_LENGTH + 1)) != 0 ||
        size > (TCP_PAR_MAX_LENGTH + 1) * TCP_STATISTICS_COUNT)
        {
          syslog(LOG_ERR, "%s: read error:%d\n", __func__, size);
          close(fd);
          free(buf);
          return -1;
        }
      i = 0;
      while ((p = strsep(&bufBak, ";")) && i < TCP_STATISTICS_COUNT)
        {
          //syslog(LOG_INFO, "%s: %s %d\n", __func__, p, i);
          strcpy(g_tcp_statistics_array[i], p);
          i++;
        }
    }
  close(fd);
  free(buf);
  return 0;
}

static int tcp_update_statistics(char *filePath, GPS_STATISTICS num, char* value)
{
  int fd;
  char buf[TCP_PAR_MAX_LENGTH + 2] = {0};

  fd = open(filePath, O_WRONLY);
  if (fd < 0)
    {
      syslog(LOG_ERR, "%s: open %s error:%d, %s\n",
        __func__, filePath, errno, strerror(errno));
      return -1;
    }

  snprintf(buf, sizeof(buf), "%020s;", value);
  lseek(fd, num * (TCP_PAR_MAX_LENGTH + 1), SEEK_SET);
  if (tcp_safe_write(fd, buf, strlen(buf)) != strlen(buf))
    {
      syslog(LOG_ERR, "%s: write %s error:%d, %s\n",
        __func__, filePath, errno, strerror(errno));
      close(fd);
      return -1;
    }
  strcpy(g_tcp_statistics_array[num], value);
  syslog(LOG_INFO, "%s: %d, %s\n",__func__, num, value);
  close(fd);

  return 0;
}

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

static void rtc_sleep(int fd, int seconds)
{
  struct rtc_time curtime;
  time_t nxtime;

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

  ioctl(fd, RTC_RD_TIME, (unsigned long)&curtime);
  nxtime = mktime((FAR struct tm *)&curtime) + seconds;
  gmtime_r(&nxtime, (FAR struct tm *)&alarminfo.time);
  ioctl(fd, RTC_SET_ALARM, (unsigned long)&alarminfo);
  syslog(LOG_INFO, "Curtime: [%02d:%02d:%02d], should wakeup at [%02d:%02d:%02d]\n",
          curtime.tm_hour, curtime.tm_min, curtime.tm_sec,
          alarminfo.time.tm_hour, alarminfo.time.tm_min, alarminfo.time.tm_sec);

  sleep(seconds);

  ioctl(fd, RTC_RD_TIME, (unsigned long)&curtime);
  syslog(LOG_INFO, "Curtime: [%02d:%02d:%02d], ACT\n",
          curtime.tm_hour, curtime.tm_min, curtime.tm_sec);
}

static void nb_send2server(int fd, char *addr, int port)
{
  int sock_fd;
  struct sockaddr_in remote;
  int ret;
  int cnt = 0;
  struct timeval tv;
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

  tv.tv_sec = 30;
  tv.tv_usec = 0;
  setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

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
      close(sock_fd);
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
  syslog(LOG_INFO, "%s: NB send:%s\r\n", __func__, g_nb_data);
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
  if(is_registered(g_reg_staus))
  {
    pthread_cond_signal(&g_tcp_cond);
  }
  pthread_mutex_unlock(&g_tcp_mutex);
}

static void usage(void)
{
  syslog(LOG_ERR, "Usage: tcptest [-a addr] [-p port] [-c count] [-i interval] [-m DSmode] [-n Action After Nb Sent]\n"
      "\t-a addr\t\tThe remote ip address\n"
      "\t-p port\t\tThe remote port\n"
      "\t-c count\t\tTest count\n"
      "\t-i interval\t\tTest interval\n"
      "\t-m DSmode\t\tDS mode\n"
      "\t-n action\t\tAction After Nb Sent\n");
}

#ifdef BUILD_MODULE
int main(int argc, char *argv[])
#else
int tcptest_main(int argc, char *argv[])
#endif
{
  pm_stay(PM_IDLE_DOMAIN, PM_STANDBY);

  int ret;
  int clientfd;
  int opt;
  int port = 0;
  char addr[INET6_ADDRSTRLEN];
  uint32_t testCount = UINT32_MAX;
  uint32_t testInterval = UINT32_MAX;          //test time interval, unit:second
  uint32_t dsmode = UINT32_MAX;                //DS mode
  uint32_t actionafternbsent = UINT32_MAX;     //Action After Nb Sent
  uint32_t index = 1;
  struct timeval now;
  struct timespec time;

  int rtcfd = open("/dev/rtc0", 0);
  if(rtcfd < 0)
    {
      syslog(LOG_ERR, "%s: Open rtc error\n", __func__);
      pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
      return -1;
    }

  memset(addr, 0x0, sizeof(addr));

  const char *start_reason = getenv("START_REASON");
  if (strcmp(start_reason, "first_pon") == 0 || strcmp(start_reason, "button_rstn") == 0 || strcmp(start_reason, "soft_rstn") == 0)
    {
      syslog(LOG_INFO, "%s: clear g_tcp_statistics_array\n", __func__);
      tcp_update_statistics(g_tcp_statistics_path, TCP_STATISTICS_PAR_ADDR, 0);
      tcp_update_statistics(g_tcp_statistics_path, TCP_STATISTICS_PAR_PORT, 0);
      tcp_update_statistics(g_tcp_statistics_path, TCP_STATISTICS_PAR_TCPREMAINTESTCOUNT, 0);
      tcp_update_statistics(g_tcp_statistics_path, TCP_STATISTICS_PAR_TCPINTERVAL, 0);
      tcp_update_statistics(g_tcp_statistics_path, TCP_STATISTICS_PAR_ENTERDS, 0);
      tcp_update_statistics(g_tcp_statistics_path, TCP_STATISTICS_PAR_ACTIONAFTERNBSENT, 0);
    }

  if (tcp_get_statistics(g_tcp_statistics_path) != 0)
  {
    syslog(LOG_INFO, "%s: get tcp stattistics error\n", __func__);
    pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
    return -1;
  }

  while ((opt = getopt(argc, argv, "a:p:c:i:m:n:")) != -1)
    {
      switch (opt)
        {
          case 'a':
            strncpy(addr, optarg, sizeof(addr) - 1);
            break;
          case 'p':
            port = atoi(optarg);
            break;
          case 'c':
            testCount = atoi(optarg);
            break;
          case 'i':
            testInterval = atoi(optarg);
            break;
          case 'm':
            dsmode = atoi(optarg);
            break;
          case 'n':
            actionafternbsent = atoi(optarg);
            break;
          default:
            usage();
            pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
            exit(1);
          }
    }

  if(atoi(g_tcp_statistics_array[TCP_STATISTICS_PAR_TCPREMAINTESTCOUNT]) > 0)
  {
    //remove extra spaces in addr
    int j = 0;
    char addrtemp[INET6_ADDRSTRLEN];
    memset(addr, 0x0, sizeof(addr));
    memset(addrtemp, 0x0, sizeof(addrtemp));
    strncpy(addrtemp, g_tcp_statistics_array[TCP_STATISTICS_PAR_ADDR], 20);
    for(int i=0; i < INET6_ADDRSTRLEN; i++)
    {
      if(addrtemp[i] != ' ')
      {
        addr[j++] = addrtemp[i];
      }
    }
    addr[j] = '\0';

    port = atoi(g_tcp_statistics_array[TCP_STATISTICS_PAR_PORT]);
    testCount = atoi(g_tcp_statistics_array[TCP_STATISTICS_PAR_TCPREMAINTESTCOUNT]);
    testInterval = atoi(g_tcp_statistics_array[TCP_STATISTICS_PAR_TCPINTERVAL]);
    dsmode = atoi(g_tcp_statistics_array[TCP_STATISTICS_PAR_ENTERDS]);
    actionafternbsent = atoi(g_tcp_statistics_array[TCP_STATISTICS_PAR_ACTIONAFTERNBSENT]);

    syslog(LOG_INFO, "tcp statistics addr:%s\n", addr);
    syslog(LOG_INFO, "tcp statistics port:%d\n", port);
    syslog(LOG_INFO, "tcp statistics testCount:%d\n", testCount);
    syslog(LOG_INFO, "tcp statistics testInterval:%d\n", testInterval);
    syslog(LOG_INFO, "tcp statistics dsmode:%d\n", dsmode);
    syslog(LOG_INFO, "tcp statistics actionafternbsent:%d\n", actionafternbsent);
  }
  else
  {
    if (port == 0 || addr[0] == '\0' || testCount < 1)
    {
      //usage();
      //syslog(LOG_INFO, "%s: par error \n", __func__);
      pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
      exit(1);
    }
    else
    {
      char strtemp[20];
      tcp_update_statistics(g_tcp_statistics_path, TCP_STATISTICS_PAR_ADDR, addr);
      itoa(port, strtemp, 10);
      tcp_update_statistics(g_tcp_statistics_path, TCP_STATISTICS_PAR_PORT, strtemp);
      itoa(testCount, strtemp, 10);
      tcp_update_statistics(g_tcp_statistics_path, TCP_STATISTICS_PAR_TCPREMAINTESTCOUNT, strtemp);
      itoa(testInterval, strtemp, 10);
      tcp_update_statistics(g_tcp_statistics_path, TCP_STATISTICS_PAR_TCPINTERVAL, strtemp);
      itoa(dsmode, strtemp, 10);
      tcp_update_statistics(g_tcp_statistics_path, TCP_STATISTICS_PAR_ENTERDS, strtemp);
      itoa(actionafternbsent, strtemp, 10);
      tcp_update_statistics(g_tcp_statistics_path, TCP_STATISTICS_PAR_ACTIONAFTERNBSENT, strtemp);
    }
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

  while (index++ <= testCount)
    {
      char strtemp[20];
      itoa(testCount - index + 1, strtemp, 10);
      tcp_update_statistics(g_tcp_statistics_path, TCP_STATISTICS_PAR_TCPREMAINTESTCOUNT, strtemp);
      NBRADIO_STATUS nbRadioStatus;
      ret = get_nbradiostatus(clientfd, &nbRadioStatus);
      if (ret < 0)
      {
        syslog(LOG_ERR, "get_nbradiostatus fail\n");
        goto clean;
      }
      syslog(LOG_INFO, "%s: nbRadioStatus=%d\n", __func__, nbRadioStatus);
      if(nbRadioStatus != NBRADIO_STATUS_ON)
      {
        if (set_radiopower(clientfd, true) < 0)
        {
          syslog(LOG_ERR, "%s: set_radiopower true fail\n", __func__);
          pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
          return -1;
        }
      }

      pthread_mutex_lock(&g_tcp_mutex);
      while (!is_registered(g_reg_staus))
        {
          gettimeofday(&now, NULL);
          time.tv_sec = now.tv_sec + 80;
          time.tv_nsec = now.tv_usec * 1000;
          ret = pthread_cond_timedwait(&g_tcp_cond, &g_tcp_mutex, &time);
          if (ret != 0)
            {
              syslog(LOG_ERR, "reg fail, timeout\n");
              break;
            }
        }
      pthread_mutex_unlock(&g_tcp_mutex);

      if(is_registered(g_reg_staus))
      {
        // send data to NB
        nb_send2server(clientfd, addr, port);

        if(actionafternbsent == 0)
        {
          if (set_radiopower(clientfd, false) < 0)
          {
            syslog(LOG_ERR, "%s: set_radiopower false fail\n", __func__);
            pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
            return ret;
          }
        }
        else if(actionafternbsent == 1)
        {
          if (release_signalconnection(clientfd) < 0)
          {
            syslog(LOG_ERR, "%s: release_signalconnection fail\n", __func__);
          }
        }

        if(testCount == 1)
        {
          goto clean;
        }
      }
      else
      {
          syslog(LOG_INFO, "%s: reg fail, set_radiopower false\n", __func__);
          if (set_radiopower(clientfd, false) < 0)
          {
            syslog(LOG_ERR, "%s: set_radiopower false fail\n", __func__);
            pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
            return ret;
          }
      }

      if (dsmode)
      {
        pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
      }
      rtc_sleep(rtcfd, testInterval);
      if (dsmode)
      {
        pm_stay(PM_IDLE_DOMAIN, PM_STANDBY);
      }
      syslog(LOG_INFO, "%s: sleep over\n", __func__);
    }

clean:
  pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
  at_client_close(clientfd);
  pthread_mutex_destroy(&g_tcp_mutex);
  pthread_cond_destroy(&g_tcp_cond);
  syslog(LOG_ERR, "%s: quit\n", __func__);
  return ret;
}
