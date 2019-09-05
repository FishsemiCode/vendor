/****************************************************************************
 * vendor/apps/fishsnshub/fishsnshub.c
 *
 *   Copyright (C) 2019, FishSemi Inc. All rights reserved.
 *   Author: Dong Jiuzhu <dongjiuzhu@fishsemi>
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
#include <nuttx/semaphore.h>
#include <stdio.h>
#include <errno.h>

#include "client_manager.h"
#include "circ_buffer.h"
#include "sensor.h"

/****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define SNSHUB_ASENSOR_RATE     200
#define SNSHUB_GSENSOR_RATE     200

#define SNSHUB_ASENSOR_DELAY    (1000000 / SNSHUB_ASENSOR_RATE)
#define SNSHUB_GSENSOR_DELAY    (1000000 / SNSHUB_GSENSOR_RATE)

/****************************************************************************
 * Private Function Prototypes
 *****************************************************************************/

static void fishsnshub_client_sevent_cb(struct sensor_event *event, size_t num);
static void fishsnshub_client_saccuracy_cb(struct snshub_sensor_t *sensor, int accuracy);

/****************************************************************************
 * Private Data
 *****************************************************************************/

static struct snshub_client    *g_snshub_client;
static struct cmgr_circ_buffer *g_snshub_event_buf;
static struct snshub_sensor_t  *g_asensor;
static struct snshub_sensor_t  *g_gsensor;
static sem_t                    g_snshub_sem;
static snshub_data_mode         g_snshub_data_mode;
static bool                     g_ctrl_c_exit;

static struct cmgr_callback fishsnshub_client_cb =
{
  .event_update     = fishsnshub_client_sevent_cb,
  .accuracy_changed = fishsnshub_client_saccuracy_cb,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void fishsnshub_client_sevent_cb(struct sensor_event *event, size_t num)
{
  int err;

  /* the callback to handle the sensor events, in client_manager thread context */

  err = cmgr_circ_buffer_push(g_snshub_event_buf, event, num);
  if (err)
    {
      printf("failed to push event to g_snshub buffer\n");
    }
  else
    {
      sem_post(&g_snshub_sem);
    }
}

static void fishsnshub_client_saccuracy_cb(struct snshub_sensor_t *sensor, int accuracy)
{
  /* the callback to handle the sensor accuracy change */
}

static int tclient_event_handler(void *data)
{
  struct sensor_event *event = data;
  static uint32_t cnt_a = 0;
  static uint32_t cnt_g = 0;

  switch (event->type)
    {
    case SENSOR_TYPE_ACCELEROMETER:
      if (!(cnt_a++ % SNSHUB_ASENSOR_RATE))
        {
          printf("accel: %f, %f, %f\n", event->accel_t.x, event->accel_t.y,event->accel_t.z);
        }
      break;
    case SENSOR_TYPE_GYROSCOPE:
      if (!(cnt_g++ % SNSHUB_GSENSOR_RATE))
        {
          printf("gyro:  %f, %f, %f\n", event->gyro_t.x, event->gyro_t.y, event->gyro_t.z);
        }
      break;
    default:
      break;
    }

  return 0;
}

static void fishsnshub_release_res(void)
{
  if (g_asensor)
    {
      cmgr_activate_sensor_one(g_snshub_client, g_asensor, 0, false, g_snshub_data_mode);
    }

  if (g_gsensor)
    {
      cmgr_activate_sensor_one(g_snshub_client, g_gsensor, 0, false, g_snshub_data_mode);
    }

  if (g_snshub_data_mode == SNSHUB_INTERRUPT)
    {
      sem_destroy(&g_snshub_sem);
      cmgr_circ_buffer_deinit(g_snshub_event_buf);
    }

  cmgr_client_release(g_snshub_client);
  _exit(0);
}

static int fishsnshub_interrupt(void)
{
  while (1)
    {
      sem_wait(&g_snshub_sem);

      if (g_ctrl_c_exit)
        break;

      cmgr_circ_buffer_for_each(g_snshub_event_buf, tclient_event_handler);
    }

  return 0;
}

static int fishsnshub_polling(void)
{
  struct sensor_event event;

  while (1)
    {
      cmgr_read_data(g_snshub_client, g_asensor, &event);
      tclient_event_handler(&event);

      cmgr_read_data(g_snshub_client, g_gsensor, &event);
      tclient_event_handler(&event);

      usleep(SNSHUB_ASENSOR_DELAY);

      if (g_ctrl_c_exit)
        break;
    }

  return 0;
}

/* Anything related to sleep doesn't work in this callback */
static void fishsnshub_exit(int signo)
{
  if (g_snshub_data_mode == SNSHUB_INTERRUPT)
    sem_post(&g_snshub_sem);

  g_ctrl_c_exit = true;
}

static int fishsnshub_usage(void)
{
  printf("fishsnshub: the parameter is not enough!\n");
  printf("Usage:\n");
  printf("fishsnshub <arg>\n");
  printf("i: Using interrupt mode to read sensor data, default polling\n");
  printf("Ctrl-C to exit.\n");
  return 0;
}

/****************************************************************************
 * fishsnshub_main
 ****************************************************************************/

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int fishsnshub_main(int argc, char *argv[])
#endif
{
  int ret = 0;

  g_ctrl_c_exit = false;
  g_snshub_data_mode = SNSHUB_POLLING;

  if (argc == 2)
    {
      g_snshub_data_mode = SNSHUB_INTERRUPT;
    }
  else
    {
      fishsnshub_usage();
    }

  /* Install signal handlers */

  signal(SIGINT, fishsnshub_exit);

  /* Register a snshub client to snshub client manager */

  g_snshub_client = cmgr_client_request("test", &fishsnshub_client_cb);
  if (!g_snshub_client)
    {
      printf("fishsnshub:failed to request sensor client\n");
      return -EBUSY;
    }

  /* Init some resource for Interrupt Mode */

  if (g_snshub_data_mode == SNSHUB_INTERRUPT)
    {
      /* initialize the circular event buffer */

      ret = cmgr_circ_buffer_init(&g_snshub_event_buf, "fishsnshub event", sizeof(struct sensor_event), 32);
      if (ret)
        {
          printf("fishsnshub event buffer init failed:%d\n", ret);
          return -EINVAL;
        }

      sem_init(&g_snshub_sem, 0, 0);
      sem_setprotocol(&g_snshub_sem, SEM_PRIO_NONE);
    }

  /* Wait snshub ready */

  while(!cmgr_system_ready());

  /* Get A & G sensor handle */

  g_asensor = cmgr_get_sensor_by_type(SENSOR_TYPE_ACCELEROMETER);
  if (g_asensor)
    {
      ret = cmgr_activate_sensor_one(g_snshub_client, g_asensor,
                                     SNSHUB_ASENSOR_DELAY,
                                     true, g_snshub_data_mode);
      if (ret < 0)
        {
          printf("fishsnshub:failed to ativate acc-sensor\n");
          goto fail;
        }
    }
  else
    {
      printf("fishsnshub:failed to get acc-sensor\n");
      goto fail;
    }

  g_gsensor = cmgr_get_sensor_by_type(SENSOR_TYPE_GYROSCOPE);
  if (g_gsensor)
    {
      ret = cmgr_activate_sensor_one(g_snshub_client, g_gsensor,
                                     SNSHUB_GSENSOR_DELAY,
                                     true, g_snshub_data_mode);
      if (ret < 0)
        {
          printf("fishsnshub:failed to ativate gyro-sensor\n");
          goto fail;
        }
    }
  else
    {
      printf("fishsnshub:failed to get gyro-sensor\n\n");
      goto fail;
    }

  /* Get sensor data from snshub */

  if (g_snshub_data_mode == SNSHUB_INTERRUPT)
    {
      ret = fishsnshub_interrupt();
    }
  else
    {
      ret = fishsnshub_polling();
    }

fail:
  fishsnshub_release_res();

  return ret;
}
