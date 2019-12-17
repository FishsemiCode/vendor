/****************************************************************************
 * apps/external/example/gpstest/gpstest_callback.h
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
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


#ifndef __APPS_EXAMPLES_GPSTEST_CALLBACK_H
#define __APPS_EXAMPLES_GPSTEST_CALLBACK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
typedef struct
{
  int regState;
  int lac;
  int cid;
}ServiceState;

typedef struct
{
  int rsrp;
  int rsrq;
  int snr;
}SignalStrength;

typedef enum
{
  SIGNAL_STRENGTH_NONE_OR_UNKNOWN,
  SIGNAL_STRENGTH_POOR,
  SIGNAL_STRENGTH_MODERATE,
  SIGNAL_STRENGTH_GOOD,
  SIGNAL_STRENGTH_GREAT,
  SIGNAL_STRENGTH_EXCELLENT,
}SINGAL_STRENGTH_LEVEL;


typedef enum
{
  DATA_ACTIVITY_NONE,
  DATA_ACTIVITY_IN,
  DATA_ACTIVITY_OUT,
  DATA_ACTIVITY_INOUT,
}DATA_ACTIVITY;

typedef enum
{
  GPS_ACTIVITY_NONE,
  GPS_ACTIVITY_INOUT,
}GPS_ACTIVITY;

typedef struct
{
  uint32_t positionTime;
  char time[11];
  char longitude[13];
  char latitude[12];
  char date[7];
}GpsInfo;

typedef struct
{
  int position_fail;
  int sent_fail;
  int connect_fail;
  int ack_fail;
  int startGps_fail;
  int stopGps_fail;
  int startNb_fail;
  int stopNb_fail;
  int regNb_fail;
  int gps_success;
  int nb_success;
}StatisticsInfo;


typedef void (*serviceStateChanged_callback)(ServiceState *state);
typedef void (*signalStrengthChanged_callback)(SignalStrength *signalStrength);
typedef void (*cellInfoChanged_callback)(at_api_cellinfo *cellInfo);
typedef void (*dataActivity_callback)(DATA_ACTIVITY state);
typedef void (*gpsActivity_callback)(GPS_ACTIVITY state);
typedef void (*gpsInfo_callback)(GpsInfo *gpsInfo);
typedef void (*statisticsInfo_callback)(StatisticsInfo *statisticsInfo);
typedef void (*imeiInfo_callback)(char *imei);
typedef void (*currOperInfo_callback)(at_api_curroper *currOper);
typedef void (*simStatus_callback)(SIM_STATUS simStatus);
typedef void (*sleep_callback)(bool sleeping);


typedef struct
{
  serviceStateChanged_callback serviceStateChanged;
  signalStrengthChanged_callback signalStrengthChanged;
  cellInfoChanged_callback cellInfoChanged;
  dataActivity_callback dataActivity;
  gpsActivity_callback gpsActivity;
  gpsInfo_callback gpsInfo;
  statisticsInfo_callback statisticsInfo;
  imeiInfo_callback imeiInfo;
  currOperInfo_callback currOperInfo;
  simStatus_callback simStatus;
  sleep_callback sleeping;
}GpsTestCallBack;


int gui_create(GpsTestCallBack *callback);
void gui_destory(void);
void gui_restory(void);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  /*__APPS_EXAMPLES_GPSTEST_CALLBACK_H*/

