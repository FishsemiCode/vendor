/****************************************************************************
 * apps/external/example/gpstest/gpstest_gui.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/boardctl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>

#include <graphics/lvgl.h>

#include "fbdev.h"
#include "tp.h"
#include "at_api.h"
#include "gpstest_callback.h"
#include "tp_cal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
LV_IMG_DECLARE(no_sim);
LV_IMG_DECLARE(singal_0);
LV_IMG_DECLARE(singal_1);
LV_IMG_DECLARE(singal_2);
LV_IMG_DECLARE(singal_3);
LV_IMG_DECLARE(singal_4);
LV_IMG_DECLARE(singal_5);
LV_IMG_DECLARE(singal_null);
LV_IMG_DECLARE(dataActivity_in);
LV_IMG_DECLARE(dataActivity_out);
LV_IMG_DECLARE(dataActivity_none);


LV_IMG_DECLARE(statistics);
LV_IMG_DECLARE(nb);
LV_IMG_DECLARE(gps);

LV_IMG_DECLARE(gps_on);
LV_IMG_DECLARE(sleeping);



/****************************************************************************
 * Private Functions
 ****************************************************************************/
lv_obj_t *img_singal;
lv_obj_t *img_data_activity;
lv_obj_t *img_gps_on;
lv_obj_t *curr_oper_label;
lv_obj_t *img_sleep;


/*nb info*/
lv_obj_t *nb_cont;
lv_obj_t *img_nb;
lv_obj_t *imei_label;
lv_obj_t *lac_label;
lv_obj_t *ci_label;
lv_obj_t *rsrp_label;
lv_obj_t *rsrq_label;
lv_obj_t *snr_label;
lv_obj_t *band_label;
lv_obj_t *arfcn_label;
lv_obj_t *pci_label;

/*gps info*/
lv_obj_t *gps_cont;
lv_obj_t *img_gps;
lv_obj_t *lon_label;
lv_obj_t *lat_label;
lv_obj_t *utc_label;
lv_obj_t *date_label;
lv_obj_t *costTime_label;

/*statistics info*/
lv_obj_t *statistics_cont;
lv_obj_t *img_statistics;
lv_obj_t *gps_success_label;
lv_obj_t *nb_success_label;
lv_obj_t *position_fail_label;
lv_obj_t *regNb_fail_label;
lv_obj_t *connect_fail_label;
lv_obj_t *sent_fail_label;
lv_obj_t *ack_fail_label;
lv_obj_t *startNb_fail_label;
lv_obj_t *stopNb_fail_label;

static lv_style_t button_style;

static int RSRP_THRESH_LENIENT[] = {-140, -140, -125, -115, -110, -102, -44};

static int g_rsrpDbm = INT_MAX;
static int g_reg_staus = -1;

static pthread_t g_tick_thread;
static pthread_t g_task_thread;

bool g_tick_thread_quit = false;
bool g_task_thread_quit = false;

static bool is_registered(int reg_Status)
{
  return reg_Status == 1 || reg_Status == 5;
}


/****************************************************************************
 * Name: tick_func
 *
 * Description:
 *
 * Input Parameters:
 *   data
 *
 * Returned Value:
 *   NULL
 *
 ****************************************************************************/

static FAR void *tick_func(void *data)
{
  static long last_ms;
  long    ms;
  struct timespec spec;

  while (!g_tick_thread_quit)
    {
      long diff;

      /* Calculate how much time elapsed */

      clock_gettime(CLOCK_REALTIME, &spec);
      ms = (long)spec.tv_nsec / 1000000;
      diff = ms - last_ms;

      /* Handle overflow */

      if (diff < 0)
        {
          diff = 1000 + diff;
        }

      lv_tick_inc(diff);
      usleep(5000);

      last_ms = ms;
    }

  /* Never will reach here */

  return NULL;
}

static void *gui_task(void *data)
{
  /* Handle LittlevGL tasks */
  while (!g_task_thread_quit)
    {
      lv_task_handler();
      usleep(10000);
    }
  return NULL;
}


static SINGAL_STRENGTH_LEVEL getSingalStrengthLevel(int rsrpDbm)
{
  int *threshRsrp = RSRP_THRESH_LENIENT;
  SINGAL_STRENGTH_LEVEL level = SIGNAL_STRENGTH_NONE_OR_UNKNOWN;
  if (rsrpDbm > threshRsrp[6])
    {
      level = SIGNAL_STRENGTH_NONE_OR_UNKNOWN;
    }
  else if (rsrpDbm >= threshRsrp[5])
    {
      level = SIGNAL_STRENGTH_EXCELLENT;
    }
  else if (rsrpDbm >= threshRsrp[4])
    {
      level = SIGNAL_STRENGTH_GREAT;
    }
  else if (rsrpDbm >= threshRsrp[3])
    {
      level = SIGNAL_STRENGTH_GOOD;
    }
  else if (rsrpDbm >= threshRsrp[2])
    {
      level = SIGNAL_STRENGTH_MODERATE;
    }
  else if (rsrpDbm >= threshRsrp[1])
    {
      level = SIGNAL_STRENGTH_POOR;
    }
  else if (rsrpDbm >= threshRsrp[0])
    {
      level = SIGNAL_STRENGTH_NONE_OR_UNKNOWN;
    }
  return level;
}


void set_singal_img(int rsrp)
{
  SINGAL_STRENGTH_LEVEL level = getSingalStrengthLevel(rsrp);
  if (!is_registered(g_reg_staus))
    {
      lv_img_set_src(img_singal, &singal_null);
      lv_obj_set_hidden(img_data_activity, true);
    }
  else
    {
      lv_obj_set_hidden(img_data_activity, false);
      lv_img_set_src(img_data_activity, &dataActivity_none);
      switch (level)
        {
          case SIGNAL_STRENGTH_POOR:
            lv_img_set_src(img_singal, &singal_1);
            break;
          case SIGNAL_STRENGTH_MODERATE:
            lv_img_set_src(img_singal, &singal_2);
            break;
          case SIGNAL_STRENGTH_GOOD:
            lv_img_set_src(img_singal, &singal_3);
            break;
          case SIGNAL_STRENGTH_GREAT:
            lv_img_set_src(img_singal, &singal_4);
            break;
          case SIGNAL_STRENGTH_EXCELLENT:
            lv_img_set_src(img_singal, &singal_5);
            break;
          default:
            lv_img_set_src(img_singal, &singal_0);
            break;
        }
    }
}

void onServiceStateChanged(ServiceState *state)
{
  char buf[12] = {0};
  g_reg_staus = state->regState;
  if (is_registered(g_reg_staus))
    {
      snprintf(buf, sizeof(buf), "LAC:%x", state->lac);
      lv_label_set_text(lac_label, buf);
      snprintf(buf, sizeof(buf), "CI:%x", state->cid);
      lv_label_set_text(ci_label, buf);
    }
  else
    {
      lv_label_set_text(lac_label, "LAC:");
      lv_label_set_text(ci_label, "CI:");
    }
  set_singal_img(g_rsrpDbm);
}


static int convertRsrpToDbm(int n)
{
  if( n >= 0 && n <= 97)
    {
      return 141 - n;
    }
  else
    {
      return INT_MAX;
    }
}

static int convertRsrqToDbm(int n)
{
  if( n >= 0 && n <= 34)
    {
      return 20 - n / 2;
    }
  else
    {
      return INT_MAX;
    }
}


void onSignalStrengthChanged(SignalStrength *signalStrength)
{
  int rsrpDbm = convertRsrpToDbm(signalStrength->rsrp);
  int rsrqDb = convertRsrqToDbm(signalStrength->rsrq);
  char buf[18] = {0};
  g_rsrpDbm = -rsrpDbm;
  set_singal_img(-rsrpDbm);
  snprintf(buf, sizeof(buf), "RSRP:%d", -rsrpDbm);
  lv_label_set_text(rsrp_label, buf);
  snprintf(buf, sizeof(buf), "RSRQ:%d", -rsrqDb);
  lv_label_set_text(rsrq_label, buf);
}
void onCellInfoChanged(at_api_cellinfo *cellInfo)
{
  char buf[18] = {0};
  set_singal_img(cellInfo->rsrp);
  snprintf(buf, sizeof(buf), "SNR:%d", cellInfo->snr);
  lv_label_set_text(snr_label, buf);
  snprintf(buf, sizeof(buf), "BAND:%d", cellInfo->band);
  lv_label_set_text(band_label, buf);
  snprintf(buf, sizeof(buf), "RSRP:%d", cellInfo->rsrp);
  lv_label_set_text(rsrp_label, buf);
  snprintf(buf, sizeof(buf), "RSRQ:%d", cellInfo->rsrq);
  lv_label_set_text(rsrq_label, buf);
  snprintf(buf, sizeof(buf), "PCI:%d", cellInfo->pci);
  lv_label_set_text(pci_label, buf);
  snprintf(buf, sizeof(buf), "ARFCN:%d", cellInfo->arfcn);
  lv_label_set_text(arfcn_label, buf);
}

void onDataActivity(DATA_ACTIVITY state)
{
  if (state == DATA_ACTIVITY_IN)
    {
      lv_img_set_src(img_data_activity, &dataActivity_in);
    }
  else if (state == DATA_ACTIVITY_OUT)
    {
      lv_img_set_src(img_data_activity, &dataActivity_out);
    }
  else if (state == DATA_ACTIVITY_NONE)
    {
      lv_img_set_src(img_data_activity, &dataActivity_none);
    }
}

void onGpsActivity(GPS_ACTIVITY state)
{
  if (state == GPS_ACTIVITY_INOUT)
    {
      lv_obj_set_hidden(img_gps_on, false);
    }
  else
    {
      lv_obj_set_hidden(img_gps_on, true);
    }
}
void onGpsInfo(GpsInfo *pGpsInfo)
{
  char buf[20] = {0};
  snprintf(buf, sizeof(buf), "LON:%s", pGpsInfo->longitude);
  lv_label_set_text(lon_label, buf);
  snprintf(buf, sizeof(buf), "LAT:%s", pGpsInfo->latitude);
  lv_label_set_text(lat_label, buf);
  snprintf(buf, sizeof(buf), "UTC TIME:%s", pGpsInfo->time);
  lv_label_set_text(utc_label, buf);
  snprintf(buf, sizeof(buf), "UTC DATE:%s", pGpsInfo->date);
  lv_label_set_text(date_label, buf);
  snprintf(buf, sizeof(buf), "TIME:%d", pGpsInfo->positionTime);
  lv_label_set_text(costTime_label, buf);
}

void onStatisticsInfo(StatisticsInfo *statisticsInfo)
{
  char buf[18] = {0};
  snprintf(buf, sizeof(buf), "GPS_S:%d", statisticsInfo->gps_success);
  lv_label_set_text(gps_success_label, buf);
  snprintf(buf, sizeof(buf), "NB_S:%d", statisticsInfo->nb_success);
  lv_label_set_text(nb_success_label, buf);
  snprintf(buf, sizeof(buf), "POS_F:%d", statisticsInfo->position_fail);
  lv_label_set_text(position_fail_label, buf);
  snprintf(buf, sizeof(buf), "REG_F:%d", statisticsInfo->regNb_fail);
  lv_label_set_text(regNb_fail_label, buf);
  snprintf(buf, sizeof(buf), "CON_F:%d", statisticsInfo->connect_fail);
  lv_label_set_text(connect_fail_label, buf);
  snprintf(buf, sizeof(buf), "SET_F:%d", statisticsInfo->sent_fail);
  lv_label_set_text(sent_fail_label, buf);
  snprintf(buf, sizeof(buf), "ACK_F:%d", statisticsInfo->ack_fail);
  lv_label_set_text(ack_fail_label, buf);
  snprintf(buf, sizeof(buf), "STN_F:%d", statisticsInfo->startNb_fail);
  lv_label_set_text(startNb_fail_label, buf);
  snprintf(buf, sizeof(buf), "SPN_F:%d", statisticsInfo->stopNb_fail);
  lv_label_set_text(stopNb_fail_label, buf);
}

void onImeiInfo(char *imei)
{
  char buf[IMEI_LENGTH + 5 + 1] = {0};
  snprintf(buf, sizeof(buf), "IMEI:%s", imei);
  lv_label_set_text(imei_label, buf);
}

void onCurrOperInfo(at_api_curroper *currOper)
{
  lv_obj_set_hidden(curr_oper_label, false);
  lv_label_set_text(curr_oper_label, currOper->shortName);
}


void onSimStatus(SIM_STATUS simStatus)
{
  if (simStatus == SIM_STATUS_EXIST)
    {
      lv_img_set_src(img_singal, &singal_null);
    }
}

void onSleeping(bool bSleep)
{
  if (bSleep)
    {
      lv_obj_set_hidden(img_sleep, false);
    }
  else
    {
      lv_obj_set_hidden(img_sleep, true);
    }
}

lv_obj_t *createTextCont(char *str, lv_obj_t *parenet)
{
  lv_obj_t *text_btn = lv_btn_create(parenet, NULL);
  lv_btn_set_style(text_btn, LV_BTN_STYLE_REL, &button_style);
  lv_btn_set_style(text_btn, LV_BTN_STYLE_PR, &button_style);
  lv_btn_set_toggle(text_btn, false);
  lv_cont_set_fit(text_btn, LV_FIT_TIGHT);
  lv_obj_t *text_label = lv_label_create(text_btn, NULL);
  lv_label_set_text(text_label, str);
  return text_label;
}

static void btn_event_handler(lv_obj_t * btn, lv_event_t event)
{
    if(event == LV_EVENT_PRESSED)
      {
        gps_clear_statistics();
        syslog(LOG_INFO, "*****btxbtx button pressed\n");
      }
}


void demo_create(void)
{
  lv_theme_t *th;
  lv_theme_set_current(lv_theme_default_init(0, NULL));
  th = lv_theme_get_current();
  static lv_style_t h_style;
  lv_style_copy(&h_style, &lv_style_transp);
  h_style.body.padding.inner = LV_DPI / 10;
  h_style.body.padding.left = LV_DPI / 4;
  h_style.body.padding.right = LV_DPI / 4;
  h_style.body.padding.top = LV_DPI / 10;
  h_style.body.padding.bottom = LV_DPI / 10;
  lv_obj_t *cont = lv_cont_create(lv_disp_get_scr_act(NULL), NULL);
  lv_obj_set_size(cont, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
  lv_obj_set_click(cont, false);
  lv_cont_set_fit(cont, LV_FIT_NONE);

  lv_style_copy(&button_style, th->style.btn.rel);
  button_style.body.padding.top = 0;
  button_style.body.padding.bottom = 0;
  button_style.body.padding.left = 0;
  button_style.body.padding.right = 0;


  /*label info*/
  lv_obj_t *cont_label = lv_cont_create(cont, NULL);
  lv_obj_set_click(cont_label, false);
  lv_obj_set_size(cont_label, lv_disp_get_hor_res(NULL), 40);
  lv_cont_set_fit(cont, LV_FIT_TIGHT);
  lv_cont_set_layout(cont_label, LV_LAYOUT_ROW_T);

  img_singal = lv_img_create(cont_label, NULL);
  lv_img_set_src(img_singal, &no_sim);

  img_data_activity = lv_img_create(cont_label, NULL);
  lv_img_set_src(img_data_activity, &dataActivity_none);
  lv_obj_set_hidden(img_data_activity, true);

  img_gps_on = lv_img_create(cont_label, NULL);
  lv_img_set_src(img_gps_on, &gps_on);
  lv_obj_set_hidden(img_gps_on, true);

  img_sleep = lv_img_create(cont_label, NULL);
  lv_img_set_src(img_sleep, &sleeping);
  lv_obj_set_hidden(img_sleep, true);

  curr_oper_label = lv_label_create(cont_label, NULL);
  lv_obj_set_hidden(curr_oper_label, true);
  lv_obj_align(curr_oper_label, cont_label, LV_ALIGN_IN_RIGHT_MID, -0.2*LV_DPI, 0);
  lv_obj_set_protect(curr_oper_label, LV_PROTECT_POS);
  /*NB info*/
  nb_cont = lv_cont_create(cont, NULL);
  lv_obj_set_click(nb_cont, false);
  lv_cont_set_fit(nb_cont, LV_FIT_TIGHT);
  lv_cont_set_layout(nb_cont, LV_LAYOUT_COL_L);
  lv_obj_align(nb_cont, cont_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);

  img_nb = lv_img_create(nb_cont, NULL);
  lv_img_set_src(img_nb, &nb);
  lv_obj_align(img_nb, nb_cont, LV_ALIGN_IN_TOP_MID, 0, 0);

  imei_label = createTextCont("IMEI:", nb_cont);
  lac_label = createTextCont("LAC:", nb_cont);
  ci_label = createTextCont("CI:", nb_cont);
  rsrp_label = createTextCont("RSRP:", nb_cont);
  rsrq_label = createTextCont("RSRQ:", nb_cont);
  snr_label = createTextCont("SNR:", nb_cont);
  band_label = createTextCont("BAND:", nb_cont);
  pci_label = createTextCont("PCI:", nb_cont);
  arfcn_label = createTextCont("ARFCN:", nb_cont);

  /*GPS info*/
  gps_cont = lv_cont_create(cont, NULL);
  lv_obj_set_click(gps_cont, false);
  lv_cont_set_fit(gps_cont, LV_FIT_TIGHT);
  lv_cont_set_layout(gps_cont, LV_LAYOUT_COL_L);
  lv_obj_align(gps_cont, nb_cont, LV_ALIGN_OUT_RIGHT_TOP, LV_DPI * 1.15, 0);

  img_gps = lv_img_create(gps_cont, NULL);
  lv_img_set_src(img_gps, &gps);
  lv_obj_align(img_gps, gps_cont, LV_ALIGN_IN_TOP_MID, 0, 0);

  lon_label = createTextCont("LON:", gps_cont);
  lat_label = createTextCont("LAT:", gps_cont);
  utc_label = createTextCont("UTC TIME:", gps_cont);
  date_label = createTextCont("UTC DATE:", gps_cont);
  costTime_label = createTextCont("TIME:", gps_cont);

  /*statistics info*/
  statistics_cont = lv_cont_create(cont, NULL);
  lv_obj_set_click(statistics_cont, false);
  lv_cont_set_fit(statistics_cont, LV_FIT_TIGHT);
  lv_cont_set_layout(statistics_cont, LV_LAYOUT_COL_L);
  lv_obj_align(statistics_cont, gps_cont, LV_ALIGN_OUT_RIGHT_TOP, LV_DPI * 1.0, 0);

  lv_obj_t *statistics_sub_cont = lv_cont_create(statistics_cont, NULL);
  lv_obj_set_click(statistics_sub_cont, false);
  lv_cont_set_fit(statistics_sub_cont, LV_FIT_TIGHT);
  lv_cont_set_layout(statistics_sub_cont, LV_LAYOUT_ROW_M);

  img_statistics = lv_img_create(statistics_sub_cont, NULL);
  lv_img_set_src(img_statistics, &statistics);

  static lv_style_t clear_button_style_release;
  lv_style_copy(&clear_button_style_release, th->style.btn.rel);
  clear_button_style_release.body.padding.top = 5;
  clear_button_style_release.body.padding.bottom = 5;
  clear_button_style_release.body.padding.left = 2;
  clear_button_style_release.body.padding.right = 2;
  clear_button_style_release.body.main_color = LV_COLOR_RED;
  static lv_style_t clear_button_style_press;
  lv_style_copy(&clear_button_style_press, th->style.btn.pr);
  clear_button_style_press.body.padding.top = 5;
  clear_button_style_press.body.padding.bottom = 5;
  clear_button_style_press.body.padding.left = 2;
  clear_button_style_press.body.padding.right = 2;
  clear_button_style_press.body.main_color = LV_COLOR_RED;
  lv_obj_t *clear_btn = lv_btn_create(statistics_sub_cont, NULL);
  lv_obj_set_event_cb(clear_btn, btn_event_handler);
  lv_cont_set_fit(clear_btn, LV_FIT_TIGHT);
  lv_btn_set_style(clear_btn, LV_BTN_STYLE_REL, &clear_button_style_release);
  lv_btn_set_style(clear_btn, LV_BTN_STYLE_PR, &clear_button_style_press);
  lv_obj_t *clear_label = lv_label_create(clear_btn, NULL);
  lv_label_set_text(clear_label, "clear");

  gps_success_label = createTextCont("GPS_S:", statistics_cont);
  nb_success_label = createTextCont("NB_S:", statistics_cont);
  position_fail_label = createTextCont("POS_F:", statistics_cont);
  regNb_fail_label = createTextCont("REG_F:", statistics_cont);
  connect_fail_label = createTextCont("CON_F:", statistics_cont);
  sent_fail_label = createTextCont("SET_F:", statistics_cont);
  ack_fail_label = createTextCont("ACK_F:", statistics_cont);
  startNb_fail_label = createTextCont("STN_F:", statistics_cont);
  stopNb_fail_label = createTextCont("SPN_F:", statistics_cont);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fishguidemo_main
 *
 * Description:
 *
 * Input Parameters:
 *   Standard argc and argv
 *
 * Returned Value:
 *   Zero on success; a positive, non-zero value on failure.
 *
 ****************************************************************************/

int gui_create(GpsTestCallBack *callback)
{
  lv_disp_drv_t disp_drv;

  /* LittlevGL initialization */

  lv_init();


  /* Display interface initialization */

  fbdev_init();

  /* Basic LittlevGL display driver initialization */

  static lv_disp_buf_t disp_buf_1;
  static lv_color_t buf1_1[LV_HOR_RES_MAX * 50];                      /*A buffer for 10 rows*/
  lv_disp_buf_init(&disp_buf_1, buf1_1, NULL, LV_HOR_RES_MAX * 50);   /*Initialize the display buffer*/

  lv_disp_drv_init(&disp_drv);
  disp_drv.flush_cb = fbdev_flush;
  disp_drv.buffer = &disp_buf_1;
  lv_disp_drv_register(&disp_drv);

  /* Tick interface initialization */

  pthread_create(&g_tick_thread, NULL, tick_func, NULL);

  tp_init();
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;

  /* This function will be called periodically (by the library) to get the
   * mouse position and state.
   */

  indev_drv.read_cb = tp_read;
  lv_indev_drv_register(&indev_drv);

  demo_create();
  tp_cal_create();

  pthread_create(&g_task_thread, NULL, gui_task, NULL);
  callback->serviceStateChanged = onServiceStateChanged;
  callback->signalStrengthChanged = onSignalStrengthChanged;
  callback->cellInfoChanged = onCellInfoChanged;
  callback->dataActivity = onDataActivity;
  callback->gpsActivity = onGpsActivity;
  callback->gpsInfo = onGpsInfo;
  callback->statisticsInfo = onStatisticsInfo;
  callback->imeiInfo = onImeiInfo;
  callback->currOperInfo = onCurrOperInfo;
  callback->simStatus = onSimStatus;
  callback->sleeping = onSleeping;
  return EXIT_SUCCESS;
}

void gui_destory(void)
{
  g_task_thread_quit = true;
  pthread_kill(g_task_thread, SIGALRM);
  pthread_join(g_task_thread, NULL);
  g_task_thread = -1;
  g_tick_thread_quit = true;
  pthread_kill(g_tick_thread, SIGALRM);
  pthread_join(g_tick_thread, NULL);
  g_tick_thread = -1;
}

void gui_restory(void)
{
  if (g_tick_thread == -1)
    {
      g_tick_thread_quit = false;
      pthread_create(&g_tick_thread, NULL, tick_func, NULL);
    }
  if (g_task_thread == -1)
    {
      g_task_thread_quit = false;
      pthread_create(&g_task_thread, NULL, gui_task, NULL);
    }
}
