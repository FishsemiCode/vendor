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
#include <time.h>

#include <sys/socket.h>
#include <arpa/inet.h>

#include <sys/time.h>
#include <sys/boardctl.h>
#include <sys/ioctl.h>
#include <nuttx/board.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/power/pm.h>

#include "at_api.h"
#include "gpstest_callback.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

// TO-DO:
#define REMOTE_IPADDR           "101.200.151.218"   // Server IP Address
#define REMOTE_TCPPORT          5678           // Server TCP port number

#define MSS 1400
#define GPS_RECORD_SIZE 49
#define UINT_MAX_LENGTH 10


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

typedef enum
{
  GPS_STATISTICS_POS_FAIL,
  GPS_STATISTICS_SENT_FAIL,
  GPS_STATISTICS_CONNECT_FAIL,
  GPS_STATISTICS_ACK_FAIL,
  GPS_STATISTICS_START_GPS_FAIL,
  GPS_STATISTICS_STOP_GPS_FAIL,
  GPS_STATISTICS_START_NB_FAIL,
  GPS_STATISTICS_STOP_NB_FAIL,
  GPS_STATISTICS_NB_REG_FAIL,
  GPS_STATISTICS_POS_SUCCESS,
  GPS_STATISTICS_NB_SUCCESS,
  GPS_STATISTICS_TEST_CASE_NUM,
  GPS_STATISTICS_TEST_CASE_START_TIME,
  GPS_STATISTICS_NB_SENT_COUNT,
  GPS_STATISTICS_NB_SENT_INTERVAL,
  GPS_STATISTICS_NB_START_TIME,
  GPS_STATISTICS_GPS_INFO_SIZE,
  GPS_STATISTICS_GPS_INFO_OFFSET,
  GPS_STATISTICS_GPS_EPHEMERIS_SAVE_TIME,
  GPS_STATISTICS_PAR_SECONDS,
  GPS_STATISTICS_PAR_NBSENDINTERVAL,
  GPS_STATISTICS_PAR_ENTERDS,
  GPS_STATISTICS_PAR_ACTIONAFTERNBSENT,
  GPS_STATISTICS_PAR_GPSREMAINTESTCOUNT,
  GPS_STATISTICS_PAR_NBREMAINTTESTCOUNT,
  GPS_STATISTICS_COUNT,
}GPS_STATISTICS;

typedef enum
{
  GPS_ACTION_AFTER_NB_SENT_RADIO_POWEROFF,
  GPS_ACTION_AFTER_NB_SENT_RRC_RELEASE,
}GPS_ACTION_AFTER_NB_SENT;

static char g_gps_data[MSS + 1];
static int g_gps_flag;
static int g_reg_staus = -1;
static unsigned char g_gps_enable_flag = 1;

static pthread_mutex_t g_gps_mutex;
static pthread_cond_t g_gps_cond;
static pthread_mutex_t g_nb_mutex;
static pthread_cond_t g_nb_cond;

static unsigned char g_imei_value[IMEI_LENGTH + 1];

static unsigned char g_gps_timeout = 60;

static bool g_nbPowered = true;
static bool g_gpsPowered = false;

static unsigned char g_warm_start_location_max_time = 15;
static char *g_gpsInfo_file_path = "/data/gpsInfo";

static uint32_t g_gps_position_start_time = 0;
static uint32_t g_gps_position_end_time = 0;

static char *g_gps_statistics_path = "/data/gpsStatistics";
static uint32_t g_gps_statistics_array[GPS_STATISTICS_COUNT];

static GpsTestCallBack g_callback;
static bool g_startReasonSent;

static pthread_mutex_t g_statistics_mutex;
static int g_rtcfd = -1;

bool bNeedSaveEphemerics = true;

static struct timeval g_positon_end_time;

static char *g_longitude;
static char *g_latitude;
static uint64_t g_rtc_Seconds;
static uint32_t g_position_time;

int g_positionDelaySeconds = 1;

static void fillGpsInfo(GpsInfo *gpsInfo, uint32_t positionTime, char *time,
  char *longitude, char *latitude, char *date)
{
  memset(gpsInfo, 0x0, sizeof(GpsInfo));
  strcpy(gpsInfo->time, time);
  strcpy(gpsInfo->longitude, longitude);
  strcpy(gpsInfo->latitude, latitude);
  strcpy(gpsInfo->date, date);
  gpsInfo->positionTime = positionTime;
}

static uint32_t gettime(void)
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  syslog(LOG_INFO, "%s:%d\n", __func__, t.tv_sec);
  return t.tv_sec;
}

static uint32_t getEclipseTime(uint32_t startTime, uint32_t endTime)
{
  if (endTime > startTime)
    {
      return endTime - startTime;
    }
  else
    {
      return 0;
    }
}


ssize_t gps_safe_write(int fd, const void *buf, size_t len)
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

ssize_t gps_safe_read(int fd, void *buf, size_t len)
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

static int gps_get_statistics(char *filePath)
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
  buf = malloc((UINT_MAX_LENGTH + 1) * GPS_STATISTICS_COUNT + 1);
  if (buf == NULL)
    {
      syslog(LOG_ERR, "%s: malloc error\n", __func__);
      close(fd);
      return -1;
    }
  buf[(UINT_MAX_LENGTH + 1) * GPS_STATISTICS_COUNT] = '\0';
  if ((size = gps_safe_read(fd, buf, (UINT_MAX_LENGTH + 1) * GPS_STATISTICS_COUNT)) < 0)
    {
      syslog(LOG_ERR, "%s: read %s error:%d, %s\n",
        __func__, filePath, errno, strerror(errno));
      close(fd);
      free(buf);
      return -1;
    }

  if (size == 0)
    {
      for (i = 0; i < GPS_STATISTICS_COUNT; i++)
        {
          memcpy(buf + i * (UINT_MAX_LENGTH + 1), "0000000000;", UINT_MAX_LENGTH + 1);
        }
      gps_safe_write(fd, buf, (UINT_MAX_LENGTH + 1) * GPS_STATISTICS_COUNT);
    }
  else
    {
      char *bufBak = buf;
      if ((size % (UINT_MAX_LENGTH + 1)) != 0 ||
        size > (UINT_MAX_LENGTH + 1) * GPS_STATISTICS_COUNT)
        {
          syslog(LOG_ERR, "%s: read error:%d\n", __func__, size);
          close(fd);
          free(buf);
          return -1;
        }
      i = 0;
      while ((p = strsep(&bufBak, ";")) && i < GPS_STATISTICS_COUNT)
        {
          //syslog(LOG_INFO, "%s: %s %d\n", __func__, p, i);
          g_gps_statistics_array[i] = atoi(p);
          i++;
        }
    }
  close(fd);
  free(buf);
  return 0;
}

static int gps_update_statistics(char *filePath, GPS_STATISTICS num, uint32_t value)
{
  int fd;
  char buf[UINT_MAX_LENGTH + 2] = {0};

  fd = open(filePath, O_WRONLY);
  if (fd < 0)
    {
      syslog(LOG_ERR, "%s: open %s error:%d, %s\n",
        __func__, filePath, errno, strerror(errno));
      return -1;
    }

  snprintf(buf, sizeof(buf), "%010d;", value);
  lseek(fd, num * (UINT_MAX_LENGTH + 1), SEEK_SET);
  if (gps_safe_write(fd, buf, strlen(buf)) != strlen(buf))
    {
      syslog(LOG_ERR, "%s: write %s error:%d, %s\n",
        __func__, filePath, errno, strerror(errno));
      close(fd);
      return -1;
    }
  g_gps_statistics_array[num] = value;
  syslog(LOG_INFO, "%s: %d, %d\n",
        __func__, num, value);
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
      if (g_callback.imeiInfo)
        {
          g_callback.imeiInfo("");
        }
      return -1;
    }
  memcpy(g_imei_value, imei.imei, sizeof(g_imei_value));
  syslog(LOG_INFO, "%s: Get IMEI %s\n", __func__, g_imei_value);
  if (g_callback.imeiInfo)
    {
      g_callback.imeiInfo((char *)g_imei_value);
    }
  return 0;
}

#ifdef CONFIG_GPSTEST_GUI
static int get_curr_oper_info(int fd)
{
  int ret;
  at_api_curroper curr_oper;
  ret = get_currentoper(fd, &curr_oper);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: get_currentoper fail\n", __func__);
      return -1;
    }
  syslog(LOG_INFO, "%s: after\n", __func__);
  if (g_callback.currOperInfo)
    {
      g_callback.currOperInfo(&curr_oper);
    }
  return 0;
}
#endif

static ssize_t read_gps_info(void *buf, size_t len, int offset)
{
  ssize_t once = 0, readen = 0;
  int fd;
  fd = open(g_gpsInfo_file_path, O_RDONLY);
  if (fd < 0)
    {
      syslog(LOG_ERR, "%s: open %s error:%d, %s\n",
        __func__, g_gpsInfo_file_path, errno, strerror(errno));
      return -1;
    }
  lseek(fd, offset, SEEK_SET);
  while (len > 0)
    {
      once = read(fd, buf, len);
      if (once <= 0)
        {
          break;
        }
      len -= once;
      buf += once;
      readen += once;
    }
  close(fd);

  return readen ? readen : once;
}

static int save_gps_info_to_file(const char *p_longitude, const char *p_latitude, uint64_t mSeconds, uint32_t positionTime)
{
  int fd;
  char line[GPS_RECORD_SIZE + 1] = {0};
  int len;

  fd = open(g_gpsInfo_file_path, O_APPEND | O_CREAT | O_WRONLY);
  if (fd < 0)
    {
      syslog(LOG_ERR, "%s: open %s error:%d, %s\n",
        __func__, g_gpsInfo_file_path, errno, strerror(errno));
      return -1;
    }
  snprintf(line, sizeof(line), "GPS:%05d:%013llu:%s:%s,", positionTime, mSeconds, p_latitude, p_longitude);
  len = strlen(line);

  if (gps_safe_write(fd, line, len) != len)
    {
      syslog(LOG_ERR, "%s: write %s error:%d, %s\n",
        __func__, g_gpsInfo_file_path, errno, strerror(errno));
      close(fd);
      return -1;
    }
  close(fd);

  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_GPS_INFO_SIZE, len + g_gps_statistics_array[GPS_STATISTICS_GPS_INFO_SIZE]);

  return 0;
}

static void notify_statistics(void)
{
  if (g_callback.statisticsInfo)
    {
      StatisticsInfo statisticsInfo;
      statisticsInfo.position_fail = g_gps_statistics_array[GPS_STATISTICS_POS_FAIL];
      statisticsInfo.sent_fail = g_gps_statistics_array[GPS_STATISTICS_SENT_FAIL];
      statisticsInfo.connect_fail = g_gps_statistics_array[GPS_STATISTICS_CONNECT_FAIL];
      statisticsInfo.ack_fail = g_gps_statistics_array[GPS_STATISTICS_ACK_FAIL];
      statisticsInfo.startGps_fail = g_gps_statistics_array[GPS_STATISTICS_START_GPS_FAIL];
      statisticsInfo.stopGps_fail = g_gps_statistics_array[GPS_STATISTICS_STOP_GPS_FAIL];
      statisticsInfo.startNb_fail = g_gps_statistics_array[GPS_STATISTICS_START_NB_FAIL];
      statisticsInfo.stopNb_fail = g_gps_statistics_array[GPS_STATISTICS_STOP_NB_FAIL];
      statisticsInfo.regNb_fail = g_gps_statistics_array[GPS_STATISTICS_NB_REG_FAIL];
      statisticsInfo.gps_success = g_gps_statistics_array[GPS_STATISTICS_POS_SUCCESS];
      statisticsInfo.nb_success = g_gps_statistics_array[GPS_STATISTICS_NB_SUCCESS];
      g_callback.statisticsInfo(&statisticsInfo);
    }
}

static void increase_statistics(GPS_STATISTICS num)
{
#ifdef CONFIG_GPSTEST_GUI
  pthread_mutex_lock(&g_statistics_mutex);
  g_gps_statistics_array[num]++;
  gps_update_statistics(g_gps_statistics_path, num, g_gps_statistics_array[num]);

  notify_statistics();
  pthread_mutex_unlock(&g_statistics_mutex);
#endif
}


static int get_gps_info(int fd, int seconds, int nbSendInterval)
{
  int ret;
  struct timespec time;
  struct timeval now;
  uint32_t currTime;
  bool bColdStart = false;
  currTime = gettime();
  g_gps_position_start_time = gettime();
  g_gps_position_end_time = 0;

  if (g_nbPowered)
    {
      syslog(LOG_INFO, "%s: Stop NB by CFUN=0\n", __func__);
      ret = set_radiopower(fd, false);
      if (ret < 0)
        {
          increase_statistics(GPS_STATISTICS_STOP_NB_FAIL);
          syslog(LOG_ERR, "%s: set_radiopower fail\n", __func__);
          return -1;
        }
      pthread_mutex_lock(&g_nb_mutex);
      g_reg_staus = 0;
      pthread_mutex_unlock(&g_nb_mutex);
      if (g_callback.serviceStateChanged)
        {
          ServiceState serviceState;
          serviceState.regState = 0;
          g_callback.serviceStateChanged(&serviceState);
        }
      g_nbPowered = false;
    }

  if (g_gps_flag == 2 &&
    getEclipseTime(g_gps_statistics_array[GPS_STATISTICS_GPS_EPHEMERIS_SAVE_TIME], currTime) > 60 * 60)
    {
      g_gps_flag = 0;
      bColdStart = true;
    }

  syslog(LOG_INFO, "%s: start gps\n", __func__);
  if (!g_gpsPowered)
    {
      ret = start_gps(fd, bColdStart);
      if (ret < 0)
        {
          increase_statistics(GPS_STATISTICS_START_GPS_FAIL);
          syslog(LOG_ERR, "%s: start_gps fail\n", __func__);
          return -1;
        }
    }
  g_gpsPowered = true;
  if (g_callback.gpsActivity)
    {
      g_callback.gpsActivity(GPS_ACTIVITY_INOUT);
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

  if (ret == 0)
    {
      increase_statistics(GPS_STATISTICS_POS_SUCCESS);

      if (g_positon_end_time.tv_sec + g_gps_timeout - time.tv_sec > g_warm_start_location_max_time)
        {
          syslog(LOG_INFO, "%s: warm start position fail!!!\n", __func__);
          g_gps_flag = 1;
          g_gps_statistics_array[GPS_STATISTICS_GPS_EPHEMERIS_SAVE_TIME] = 0;
        }

      if (g_gps_flag == 1 && (g_gps_statistics_array[GPS_STATISTICS_GPS_EPHEMERIS_SAVE_TIME] == 0 ||
        getEclipseTime(g_gps_statistics_array[GPS_STATISTICS_GPS_EPHEMERIS_SAVE_TIME], currTime) > 60 * 60) &&
        bNeedSaveEphemerics)
        {
          uint32_t sleepSeconds = getEclipseTime(g_gps_position_start_time, g_gps_position_end_time);

          if (sleepSeconds >= 45)
            {
              sleepSeconds = 15;
            }
          else
            {
              sleepSeconds = 45 - sleepSeconds < 15 ? 15 : 45 - sleepSeconds;
            }
          syslog(LOG_INFO, "%s: wait %ds to save GPS parameters\n", __func__, sleepSeconds);
          sleep(sleepSeconds);
          g_gps_flag = 2;
          gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_GPS_EPHEMERIS_SAVE_TIME, currTime);
        }
      else if (g_positionDelaySeconds > 0)
        {
          //sleep(g_positionDelaySeconds);
        }
    }
  else
    {
      GpsInfo gpsInfo;
      syslog(LOG_INFO, "%s: position fail!!!\n", __func__);
      increase_statistics(GPS_STATISTICS_POS_FAIL);
      fillGpsInfo(&gpsInfo, 0, "000000.000", "00000.000000", "0000.000000", "000000");
      if (g_callback.gpsInfo)
        {
          g_callback.gpsInfo(&gpsInfo);
        }
    }

  // stop GPS
 // if (currTime - g_gps_statistics_array[GPS_STATISTICS_NB_START_TIME] >= nbSendInterval || seconds == nbSendInterval)
    {
      if (stop_gps(fd) < 0)
        {
          increase_statistics(GPS_STATISTICS_STOP_GPS_FAIL);
          if (g_callback.gpsActivity)
            {
              g_callback.gpsActivity(GPS_ACTIVITY_NONE);
            }
          syslog(LOG_ERR, "%s: stop_gps fail\n", __func__);
          return -1;
        }
      g_gpsPowered = false;
    }
  if (ret == 0)
    {
      pthread_mutex_lock(&g_gps_mutex);
      save_gps_info_to_file(g_longitude, g_latitude, g_rtc_Seconds, g_position_time);
      syslog(LOG_INFO, "%s: save_gps_info_to_file:%d\n", __func__, g_position_time);
      if (g_longitude)
        {
          free(g_longitude);
          g_longitude = NULL;
        }
      if (g_latitude)
        {
          free(g_latitude);
          g_latitude = NULL;
        }
      pthread_mutex_unlock(&g_gps_mutex);
    }
  else
    {
      save_gps_info_to_file("00000.000000", "0000.000000", 0, 0);
      syslog(LOG_INFO, "%s: save_gps_info_to_file:%d\n", __func__, 0);
    }
  if (g_callback.gpsActivity)
    {
      g_callback.gpsActivity(GPS_ACTIVITY_NONE);
    }

  currTime = gettime();
  if (currTime - g_gps_statistics_array[GPS_STATISTICS_NB_START_TIME] >= nbSendInterval || seconds == nbSendInterval)
    {
      // start NB
      syslog(LOG_INFO, "%s: Start NB CFUN=1\n", __func__);
      gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_NB_START_TIME, currTime);
      ret = set_radiopower(fd, true);
      if (ret < 0)
        {
          increase_statistics(GPS_STATISTICS_START_NB_FAIL);
          syslog(LOG_ERR, "%s: set_radiopower fail\n", __func__);
          return -1;
        }
      g_nbPowered = true;
    }
  return 0;
}


static int nb_send2server(int fd, const char *start_reason, int actionAfterNbSent)
{
  int sock_fd;
  struct sockaddr_in remote;
  int ret;
  int cnt = 0;
  struct timeval tv;
  int len;
  int size = -1;
  int readSize;
  int sentSize = 0;
  at_api_cellinfo cellinfo;
  uint32_t nb_sent_time = 0;
  uint32_t nb_ack_recv_time = 0;
  uint32_t nb_sent_interval = 0;
  uint32_t nb_sent_count = 0;
  uint32_t gps_info_offset;

  nb_sent_count = g_gps_statistics_array[GPS_STATISTICS_NB_SENT_COUNT];
  nb_sent_interval = g_gps_statistics_array[GPS_STATISTICS_NB_SENT_INTERVAL];
  gps_info_offset = g_gps_statistics_array[GPS_STATISTICS_GPS_INFO_OFFSET];
retry:
  if (g_callback.dataActivity)
    {
      g_callback.dataActivity(DATA_ACTIVITY_NONE);
    }
  if (nb_sent_time == 0)
    {
      nb_sent_time = gettime();
    }
  len = 0;
  cnt++;
  if (cnt >= 3)
    {
      syslog(LOG_ERR, "%s: send NB data failed after repeat 3 times\n", __func__);
      if (sentSize > 0)
        {
          gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_GPS_INFO_OFFSET, sentSize + gps_info_offset);
          gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_GPS_INFO_SIZE, size);
          gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_NB_SENT_COUNT, nb_sent_count);
        }
      ret = -1;
      goto clean;
    }

  sock_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_fd < 0)
    {
      syslog(LOG_ERR, "%s: create sock failed %d Err\r\n", __func__, errno);
      goto retry;
    }
  else
    {
      syslog(LOG_INFO, "%s: create sock successful\r\n", __func__);
    }

  tv.tv_sec = 30;
  tv.tv_usec = 0;
  setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  remote.sin_family = AF_INET;
  remote.sin_port = HTONS(REMOTE_TCPPORT);
  ret = inet_pton(AF_INET, REMOTE_IPADDR, &remote.sin_addr.s_addr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: error remot IP addr %d\r\n", __func__, ret);
      close(sock_fd);
      goto retry;
    }
  ret = get_cellinfo(fd, &cellinfo);
  if (ret < 0)
    {
      close(sock_fd);
      syslog(LOG_ERR, "%s: get_cellinfo fail\n", __func__);
      goto retry;
    }
  if (g_callback.cellInfoChanged)
    {
      g_callback.cellInfoChanged(&cellinfo);
    }
  if (g_callback.dataActivity)
    {
      g_callback.dataActivity(DATA_ACTIVITY_OUT);
    }
  // connect
  do
    {
      ret = connect(sock_fd, (FAR const struct sockaddr *)&remote, sizeof(remote));
      if (ret < 0)
        {
          close(sock_fd);
          syslog(LOG_ERR, "%s: connect failed %d\r\n", __func__, errno);
          increase_statistics(GPS_STATISTICS_CONNECT_FAIL);
          goto retry;
        }
      } while (ret < 0);
  syslog(LOG_INFO, "%s: connect success\r\n", __func__);

  size = g_gps_statistics_array[GPS_STATISTICS_GPS_INFO_SIZE];

  syslog(LOG_INFO, "%s: gps info size:%d\r\n", __func__, size);

  if (size < 0)
    {
      close(sock_fd);
      gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_NB_SENT_COUNT, nb_sent_count);
      if (g_callback.dataActivity)
        {
          g_callback.dataActivity(DATA_ACTIVITY_NONE);
        }
      syslog(LOG_ERR, "%s: gps info < 0 should not happen!!!\r\n", __func__);
      ret = -1;
      goto clean;
    }

  while (size > 0)
    {
      memset(g_gps_data, 0x0, sizeof(g_gps_data));
      if (!g_startReasonSent)
        {
          len = snprintf(g_gps_data, sizeof(g_gps_data), "NUM:%d,START:%s,TIME:%d,IMEI:%s,cellinfo=MCC:%s:MNC:%s:CELL_ID:%x:LAC_ID:%x:RSRP:%d:RSRQ:%d:SNR:%d:BAND:%d:ARFCN:%d:PCI:%d,", nb_sent_count, start_reason, nb_sent_interval, g_imei_value, cellinfo.mcc, cellinfo.mnc, cellinfo.cellId, cellinfo.lacId, cellinfo.rsrp, cellinfo.rsrq, cellinfo.snr,
            cellinfo.band, cellinfo.arfcn, cellinfo.pci);
        }
      else
        {
          len = snprintf(g_gps_data, sizeof(g_gps_data), "NUM:%d,START:,TIME:%d,IMEI:%s,cellinfo=MCC:%s:MNC:%s:CELL_ID:%x:LAC_ID:%x:RSRP:%d:RSRQ:%d:SNR:%d:BAND:%d:ARFCN:%d:PCI:%d,", nb_sent_count, nb_sent_interval, g_imei_value, cellinfo.mcc, cellinfo.mnc, cellinfo.cellId, cellinfo.lacId, cellinfo.rsrp, cellinfo.rsrq, cellinfo.snr,
            cellinfo.band, cellinfo.arfcn, cellinfo.pci);
        }
      if (len + size <= MSS)
        {
          readSize = size;
        }
      else
        {
          readSize = (MSS - len) - (MSS - len) % GPS_RECORD_SIZE;
        }
      if (read_gps_info(g_gps_data + len, readSize, sentSize + gps_info_offset) < readSize)
        {
          close(sock_fd);
          if (sentSize > 0)
            {
              gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_GPS_INFO_OFFSET, sentSize + gps_info_offset);
              gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_GPS_INFO_SIZE, size);
              gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_NB_SENT_COUNT, nb_sent_count);
            }
          if (g_callback.dataActivity)
            {
              g_callback.dataActivity(DATA_ACTIVITY_NONE);
            }
          syslog(LOG_ERR, "%s: read gps info error:%d,%d,%d\r\n", __func__, readSize, sentSize, gps_info_offset);
          goto clean;
        }
      syslog(LOG_INFO, "%s: sent:%d,%s\r\n", __func__, readSize, g_gps_data);

      // socket to send GPS info to server
      ret = sendto(sock_fd, g_gps_data, strlen((char *)g_gps_data), 0, NULL, 0);
      if (ret < 0)
        {
          syslog(LOG_ERR, "%s: sock send to failed %d\r\n", __func__, errno);
          increase_statistics(GPS_STATISTICS_SENT_FAIL);
          if (sentSize > 0)
            {
              gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_GPS_INFO_OFFSET, sentSize + gps_info_offset);
              gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_GPS_INFO_SIZE, size);
              gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_NB_SENT_COUNT, nb_sent_count);
            }
          close(sock_fd);
          goto retry;
        }
      syslog(LOG_INFO, "%s: send data to server successfully, and try to recv the ack:%d\r\n", __func__, nb_sent_count);
      size -= readSize;
      sentSize += readSize;
      nb_sent_count++;
      nb_ack_recv_time = gettime();
      nb_sent_interval = nb_ack_recv_time - nb_sent_time;
      gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_NB_SENT_INTERVAL, nb_sent_interval);

      if (g_callback.dataActivity)
        {
          g_callback.dataActivity(DATA_ACTIVITY_IN);
        }
      // Using Server ACK to trigger GPS enable/disable
      ret = recvfrom(sock_fd, g_gps_data, sizeof(g_gps_data) - 1, 0, NULL, 0);
      if (ret > 0)
        {
          nb_ack_recv_time = gettime();
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
          increase_statistics(GPS_STATISTICS_NB_SUCCESS);
          g_startReasonSent = true;
        }
      else
        {
          syslog(LOG_INFO, "%s: Nothing Recvied from Server\r\n", __func__);
          increase_statistics(GPS_STATISTICS_ACK_FAIL);
        }
    }
  if (g_callback.dataActivity)
    {
      g_callback.dataActivity(DATA_ACTIVITY_NONE);
    }
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_NB_SENT_COUNT, nb_sent_count);
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_GPS_INFO_OFFSET, 0);
  unlink(g_gpsInfo_file_path);
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_GPS_INFO_SIZE, 0);
  close(sock_fd);
  ret = 0;
clean:
  if (actionAfterNbSent == GPS_ACTION_AFTER_NB_SENT_RRC_RELEASE)
    {
      if (release_signalconnection(fd) < 0)
        {
          syslog(LOG_ERR, "%s: release_signalconnection fail\n", __func__);
        }
    }
  else if (actionAfterNbSent == GPS_ACTION_AFTER_NB_SENT_RADIO_POWEROFF)
    {
      if (set_radiopower(fd, false) < 0)
        {
          syslog(LOG_ERR, "%s: set_radiopower fail\n", __func__);
          increase_statistics(GPS_STATISTICS_STOP_NB_FAIL);
          return ret;
        }
      pthread_mutex_lock(&g_nb_mutex);
      g_reg_staus = 0;
      pthread_mutex_unlock(&g_nb_mutex);
      if (g_callback.serviceStateChanged)
        {
          ServiceState serviceState;
          serviceState.regState = 0;
          g_callback.serviceStateChanged(&serviceState);
        }
      g_nbPowered = false;
    }
  return ret;
}

static bool is_registered(int reg_Status)
{
  return reg_Status == 1 || reg_Status == 5;
}

static void handle_cereg(const char *s)
{
  int ret;
  ServiceState serviceState;
  char *line = (char *)s;
  syslog(LOG_INFO, "%s: %s\n", __func__, s);
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextint(&line, &(serviceState.regState));
  if (ret < 0)
    {
      return;
    }
  if (at_tok_hasmore(&line))
    {
      at_tok_nexthexint(&line, &(serviceState.lac));
      if (ret < 0)
        {
          return;
        }
      at_tok_nexthexint(&line, &(serviceState.cid));
      if (ret < 0)
        {
          return;
        }
    }
  pthread_mutex_lock(&g_nb_mutex);
  //Only the timer is released, after the registration is successful
  if(is_registered(serviceState.regState))
    {
      pthread_cond_signal(&g_nb_cond);
    }

  g_reg_staus = serviceState.regState;
  pthread_mutex_unlock(&g_nb_mutex);
  if (g_callback.serviceStateChanged)
    {
      g_callback.serviceStateChanged(&serviceState);
    }
}
#ifdef CONFIG_GPSTEST_GUI
static void handle_cesq(const char *s)
{
  int ret;
  SignalStrength singalStrength;
  int ignore;
  char *line = (char *)s;
  syslog(LOG_INFO, "%s: %s\n", __func__, s);
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextint(&line, &ignore);
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextint(&line, &ignore);
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextint(&line, &ignore);
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextint(&line, &ignore);
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextint(&line, &(singalStrength.rsrq));
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextint(&line, &(singalStrength.rsrp));
  if (ret < 0)
    {
      return;
    }
  if (g_callback.signalStrengthChanged)
    {
      g_callback.signalStrengthChanged(&singalStrength);
    }
}
#endif

static void handle_gprmc(const char *s)
{
  int ret;
  char *line = (char *)s;
  char *p;
  bool ready = false;
  char *p_longitude, *p_latitude, *p_time, *date;
  char *ignore;
  struct tm t;
  time_t seconds;
  uint64_t mSeconds;
  char buf[4] = {0};
  bool bFirstReady = true;
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

  ret = at_tok_nextstr(&line, &ignore);
  if (ret < 0)
    {
      return;
    }

  ret = at_tok_nextstr(&line, &ignore);
  if (ret < 0)
    {
      return;
    }

  ret = at_tok_nextstr(&line, &ignore);
  if (ret < 0)
    {
      return;
    }

  ret = at_tok_nextstr(&line, &date);
  if (ret < 0)
    {
      return;
    }

  if (ready && (bFirstReady || g_positionDelaySeconds > 0))
    {
      uint32_t position_time = 0;
      GpsInfo gpsInfo;

      gettimeofday(&g_positon_end_time, NULL);
      memset(&t, 0x0, sizeof(t));
      strncpy(buf, p_time + 4, 2);
      t.tm_sec = atoi(buf);
      strncpy(buf, p_time + 2, 2);
      t.tm_min = atoi(buf);
      strncpy(buf, p_time, 2);
      t.tm_hour = atoi(buf);
      strncpy(buf, date, 2);
      t.tm_mday = atoi(buf);
      strncpy(buf, date + 2, 2);
      t.tm_mon = atoi(buf);
      t.tm_mon -= 1;
      strncpy(buf, date + 4, 2);
      t.tm_year = atoi(buf) + 100;
      seconds = mktime(&t);
      strncpy(buf, p_time + 7, 2);
      mSeconds = (uint64_t)(seconds) * 1000 + atoi(buf);
      syslog(LOG_INFO, "%s: %d,%s,%s,%s,%s,%013llu,%u\n", __func__, ready, p_time, p_latitude, p_longitude, date, mSeconds, seconds);
      pthread_mutex_lock(&g_gps_mutex);
      if (g_gps_flag == 0)
        {
          g_gps_flag = 1;
        }
      g_gps_position_end_time = gettime();
      position_time = g_gps_position_end_time - g_gps_position_start_time;
      fillGpsInfo(&gpsInfo, position_time, p_time, p_longitude, p_latitude, date);
      if (g_callback.gpsInfo)
        {
          g_callback.gpsInfo(&gpsInfo);
        }
      if (g_longitude)
        {
          free(g_longitude);
        }
      if (g_latitude)
        {
          free(g_latitude);
        }
      g_longitude = strdup(p_longitude);
      g_latitude = strdup(p_latitude);
      g_rtc_Seconds = mSeconds;
      if (bFirstReady)
        {
          g_position_time = position_time;
        }
      pthread_cond_signal(&g_gps_cond);
      pthread_mutex_unlock(&g_gps_mutex);
      bFirstReady = false;
    }
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

void gps_clear_statistics(void)
{
  pthread_mutex_lock(&g_statistics_mutex);
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_POS_FAIL, 0);
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_SENT_FAIL, 0);
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_CONNECT_FAIL, 0);
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_ACK_FAIL, 0);
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_START_GPS_FAIL, 0);
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_STOP_GPS_FAIL, 0);
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_START_NB_FAIL, 0);
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_STOP_NB_FAIL, 0);
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_NB_REG_FAIL, 0);
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_POS_SUCCESS, 0);
  gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_NB_SUCCESS, 0);
  notify_statistics();
  pthread_mutex_unlock(&g_statistics_mutex);
}

static void gps_var_init(void)
{
  g_reg_staus = -1;
  g_nbPowered = true;
  g_gpsPowered = false;
  g_gps_flag = 0;
}

static int gps_service(int argc, char *argv[])
{
  int ret;
  int clientfd = -1;
  int errCnt = 0;
  int regErrCnt = 0;
  int nbSentErrCnt = 0;
  int seconds = 0;
  int nbSendInterval = 0;
  uint32_t curTestCase;
  uint32_t testCaseStartTime;
  uint32_t eclipseTime = 0;
  uint32_t currTime;
  struct timeval now;
  struct timespec time;
  int success = -1;
#ifdef CONFIG_GPSTEST_GUI
  SIM_STATUS simStatus;
  bool bHasGetOperInfo = false;
#endif
  int enterDs = 0;
  int actionAfterNbSent = GPS_ACTION_AFTER_NB_SENT_RADIO_POWEROFF;
  uint32_t gpsTestCount = UINT32_MAX;
  int32_t nbTestCount = INT32_MAX;
  uint32_t gpsTestIndex = 1;
  uint32_t nbTestIndex = 1;

  gps_var_init();

  const char *start_reason = getenv("START_REASON");

  g_rtcfd = open("/dev/rtc0", 0);
  if(g_rtcfd < 0)
    {
      syslog(LOG_ERR, "%s: Open rtc error\n", __func__);
      return -1;
    }

  currTime = gettime();

  if (gps_get_statistics(g_gps_statistics_path) != 0)
    {
      syslog(LOG_INFO, "%s: get stattistics error\n", __func__);
      close(g_rtcfd);
      return -1;
    }
  notify_statistics();

  if (strcmp(start_reason, "rtc_rstn") != 0 && strcmp(start_reason, "uart_rstn") != 0)
    {
      g_gps_statistics_array[GPS_STATISTICS_NB_SENT_COUNT] = 0;
      g_gps_statistics_array[GPS_STATISTICS_NB_START_TIME] = 0;
      g_gps_statistics_array[GPS_STATISTICS_GPS_EPHEMERIS_SAVE_TIME] = 0;
    }
  if (strcmp(start_reason, "first_pon") == 0 || strcmp(start_reason, "button_rstn") == 0)
    {
      gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_GPS_INFO_SIZE, 0);
      gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_GPS_INFO_OFFSET, 0);
      unlink(g_gpsInfo_file_path);
    }
  if (strcmp(start_reason, "first_pon") == 0 || strcmp(start_reason, "button_rstn") == 0 || strcmp(start_reason, "soft_rstn") == 0)
    {
      gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_SECONDS, 0);
      gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_NBSENDINTERVAL, 0);
      gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_ENTERDS, 0);
      gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_ACTIONAFTERNBSENT, 0);
      gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_GPSREMAINTESTCOUNT, 0);
      gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_NBREMAINTTESTCOUNT, 0);
    }

  curTestCase = g_gps_statistics_array[GPS_STATISTICS_TEST_CASE_NUM];
  testCaseStartTime = g_gps_statistics_array[GPS_STATISTICS_TEST_CASE_START_TIME];

  if (testCaseStartTime > 0)
    {
      eclipseTime = getEclipseTime(testCaseStartTime, currTime);
    }

  syslog(LOG_INFO, "%s: gpstest running:%u,%u,%u,%s\n", __func__, curTestCase, testCaseStartTime, eclipseTime, start_reason);
  syslog(LOG_INFO, "%s: gpstest cur case:%u\n", __func__, curTestCase);

  if (g_gps_statistics_array[GPS_STATISTICS_NB_START_TIME] == 0)
    {
      gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_NB_START_TIME, currTime);
    }

  if (argc >= 2)
    {
      seconds = nbSendInterval = atoi(argv[1]);
      if (argc >= 3)
        {
          nbSendInterval = atoi(argv[2]);
          if (argc >= 5)
            {
              enterDs = atoi(argv[3]);
              actionAfterNbSent = atoi(argv[4]);
              if (argc >= 6)
                {
                  gpsTestCount = atoi(argv[5]);
                  if (argc >= 7)
                    {
                      nbTestCount = atoi(argv[6]);
                    }
                }
            }
        }
    }

  if(g_gps_statistics_array[GPS_STATISTICS_PAR_GPSREMAINTESTCOUNT] > 0)
    {
      seconds           = g_gps_statistics_array[GPS_STATISTICS_PAR_SECONDS];
      nbSendInterval    = g_gps_statistics_array[GPS_STATISTICS_PAR_NBSENDINTERVAL];
      enterDs           = g_gps_statistics_array[GPS_STATISTICS_PAR_ENTERDS];
      actionAfterNbSent = g_gps_statistics_array[GPS_STATISTICS_PAR_ACTIONAFTERNBSENT];
      gpsTestCount      = g_gps_statistics_array[GPS_STATISTICS_PAR_GPSREMAINTESTCOUNT];
      nbTestCount       = g_gps_statistics_array[GPS_STATISTICS_PAR_NBREMAINTTESTCOUNT];

      syslog(LOG_INFO, "statistics seconds:%d\n", seconds);
      syslog(LOG_INFO, "statistics nbSendInterval:%d\n", nbSendInterval);
      syslog(LOG_INFO, "statistics enterDs:%d\n", enterDs);
      syslog(LOG_INFO, "statistics actionAfterNbSent:%d\n", actionAfterNbSent);
      syslog(LOG_INFO, "statistics gpsTestCount:%d\n", gpsTestCount);
      syslog(LOG_INFO, "statistics nbTestCount:%d\n", nbTestCount);
    }
  else
    {
      if(seconds <= 0)
        {
          syslog(LOG_INFO, "%s: seconds <= 0, goto clean\n", __func__);
          close(g_rtcfd);
          return -1;
        }
      else
        {
          gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_SECONDS, seconds);
          gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_NBSENDINTERVAL, nbSendInterval);
          gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_ENTERDS, enterDs);
          gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_ACTIONAFTERNBSENT, actionAfterNbSent);
          gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_GPSREMAINTESTCOUNT, gpsTestCount);
          gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_NBREMAINTTESTCOUNT, nbTestCount);
        }
    }

  syslog(LOG_INFO, "gps interval:%d, nb sent interval:%d, gpsTestCount:%d, nbTestCount:%d\n", seconds, nbSendInterval, gpsTestCount, nbTestCount);

  pthread_mutex_init(&g_gps_mutex, NULL);
  pthread_cond_init(&g_gps_cond, NULL);
  pthread_mutex_init(&g_nb_mutex, NULL);
  pthread_cond_init(&g_nb_cond, NULL);
  pthread_mutex_init(&g_statistics_mutex, NULL);

  clientfd = at_client_open();
  if(clientfd < 0)
    {
      syslog(LOG_ERR, "%s: Open client error\n", __func__);
      goto clean;
    }
#ifdef CONFIG_GPSTEST_GUI
  ret = get_simstatus(clientfd, &simStatus);
  if (ret < 0)
    {
      syslog(LOG_ERR, "get_simstatus fail\n");
      goto clean;
    }
  if (g_callback.simStatus)
    {
      g_callback.simStatus(simStatus);
    }
  if (simStatus != SIM_STATUS_EXIST)
    {
      syslog(LOG_ERR, "sim not exist\n");
      goto clean;
    }
#endif
  ret = register_indication(clientfd, "+CEREG", handle_cereg);
  if(ret < 0)
    {
      syslog(LOG_ERR, "%s: register_indication fail\n", __func__);
      goto clean;
    }
#ifdef CONFIG_GPSTEST_GUI
  ret = register_indication(clientfd, "+CESQ", handle_cesq);
  if(ret < 0)
    {
      syslog(LOG_ERR, "%s: register_indication fail\n", __func__);
      goto clean;
    }
#endif
  ret = register_indication(clientfd, "$GPRMC", handle_gprmc);
  if(ret < 0)
    {
      syslog(LOG_ERR, "%s: register_indication fail\n", __func__);
      goto clean;
    }

  ret = register_indication(clientfd, "$BDRMC", handle_gprmc);
  if(ret < 0)
    {
      syslog(LOG_ERR, "%s: register_indication fail\n", __func__);
      goto clean;
    }

  ret = set_ceregindicationstatus(clientfd, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "set_ceregindicationstatus fail\n");
      goto clean;
    }
#ifdef CONFIG_GPSTEST_GUI
  ret = set_singalstrengthindicationstatus(clientfd, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "set_singalstrengthindicationstatus fail\n");
      goto clean;
    }
#endif
  get_nb_imei(clientfd);
  while (1)
    {
      success = -1;
      // get GPS data
      if (g_gps_enable_flag == 1)
        {
          if (gpsTestIndex++ > gpsTestCount)
            {
              break;
            }
          gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_GPSREMAINTESTCOUNT, gpsTestCount - gpsTestIndex + 1);
          syslog(LOG_INFO, "gps remain count:%d\n", gpsTestCount - gpsTestIndex + 1);
          if ((success = get_gps_info(clientfd, seconds, nbSendInterval)) < 0)
            {
              errCnt++;
              if (errCnt > 5)
                {
                  board_reset(0);
                }
              continue;
            }
          errCnt = 0;
        }
      if (g_nbPowered && (nbTestCount >= 0))
        {
          if (nbTestIndex++ > nbTestCount)
            {
              break;
            }
          gps_update_statistics(g_gps_statistics_path, GPS_STATISTICS_PAR_NBREMAINTTESTCOUNT, nbTestCount - nbTestIndex + 1);
          syslog(LOG_INFO, "nb remain count:%d\n", nbTestCount - nbTestIndex + 1 );
          pthread_mutex_lock(&g_nb_mutex);
          while (!is_registered(g_reg_staus))
            {
              gettimeofday(&now, NULL);
              time.tv_sec = now.tv_sec + 80;
              time.tv_nsec = now.tv_usec * 1000;
              ret = pthread_cond_timedwait(&g_nb_cond, &g_nb_mutex, &time);
              if (ret != 0)
                {
                  pthread_mutex_unlock(&g_nb_mutex);
                  syslog(LOG_ERR, "reg fail\n");
                  increase_statistics(GPS_STATISTICS_NB_REG_FAIL);
                  regErrCnt++;
                  if (regErrCnt > 5)
                    {
                      board_reset(0);
                    }
                  break;
                }
            }
          if (is_registered(g_reg_staus))
            {
            #ifdef CONFIG_GPSTEST_GUI
              if (!bHasGetOperInfo)
                {
                  get_curr_oper_info(clientfd);
                  bHasGetOperInfo = true;
                }
            #endif
              regErrCnt = 0;
              pthread_mutex_unlock(&g_nb_mutex);
              // send data to NB
              if ((success = nb_send2server(clientfd, start_reason, actionAfterNbSent)) < 0)
                {
                  nbSentErrCnt++;
                  if (nbSentErrCnt > 5)
                    {
                      syslog(LOG_ERR, "nb send fail 5 times, cfun 0\n");
                      if (set_radiopower(clientfd, false) < 0)
                        {
                          syslog(LOG_ERR, "%s: set_radiopower fail\n", __func__);
                          increase_statistics(GPS_STATISTICS_STOP_NB_FAIL);
                        }
                    }
                }
              else
                {
                  nbSentErrCnt = 0;
                }
            }
          else
            {
              if (set_radiopower(clientfd, false) < 0)
                {
                  syslog(LOG_ERR, "%s: set_radiopower fail\n", __func__);
                  increase_statistics(GPS_STATISTICS_STOP_NB_FAIL);
                }
              else
                {
                  g_nbPowered = false;
                }
            }
        }

      eclipseTime = getEclipseTime(currTime, gettime());
      syslog(LOG_INFO, "gps nb cost time:%d\n", eclipseTime);
      if (eclipseTime <= seconds)
        {
        if (g_callback.sleeping)
          {
            g_callback.sleeping(true);
          }
        #ifdef CONFIG_GPSTEST_GUI
          usleep(50000);
          gui_destory();
        #endif
          if (eclipseTime <= seconds - 10)
          {
            if (enterDs)
            {
              pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
            }
            rtc_sleep(g_rtcfd, seconds - eclipseTime);
            if (enterDs)
            {
              pm_stay(PM_IDLE_DOMAIN, PM_STANDBY);
            }
          }
          if (g_callback.sleeping)
          {
            g_callback.sleeping(false);
          }
        }
      currTime = gettime();
    }
clean:
  if (g_rtcfd >= 0)
    {
      close(g_rtcfd);
    }
  if (clientfd >= 0)
    {
      at_client_close(clientfd);
    }
  pthread_mutex_destroy(&g_gps_mutex);
  pthread_cond_destroy(&g_gps_cond);
  pthread_mutex_destroy(&g_nb_mutex);
  pthread_cond_destroy(&g_nb_cond);
  pthread_mutex_destroy(&g_statistics_mutex);
  syslog(LOG_INFO, "%s: quit\n", __func__);
  return -1;
}

int gpstest_main(int argc, char *argv[])
{
  pm_stay(PM_IDLE_DOMAIN, PM_STANDBY);
#ifdef CONFIG_GPSTEST_GUI
  gui_create(&g_callback);
#endif
  gps_service(argc, argv);
  pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
  return EXIT_SUCCESS;
}

