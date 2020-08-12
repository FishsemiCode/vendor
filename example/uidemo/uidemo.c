/****************************************************************************
 * apps/external/example/uidemo/uidemo.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Copyright (C) 2020 FishSemi Inc. All rights reserved.
 *   Author: fishsemi <fishsemi@fishsemi.net>
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

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>

#include <graphics/lvgl.h>
#include "fbdev.h"
#include "demo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tick_func
 *
 * Description:
 *   Calls lv_tick_inc(...) every 5ms.
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

  while (1)
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main or uidemo_main
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

int main(int argc, FAR char *argv[])
{
  lv_disp_drv_t disp_drv;
  pthread_t tick_thread;

  lv_disp_buf_t disp_buf;
  static lv_color_t buf[CONFIG_LV_VDB_SIZE];

  /* LittlevGL initialization */

  lv_init();

  /* Display interface initialization */

  fbdev_init();

  /* Basic LittlevGL display driver initialization */

  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);
  lv_disp_drv_init(&disp_drv);
  disp_drv.flush_cb = fbdev_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  /* Tick interface initialization */

  pthread_create(&tick_thread, NULL, tick_func, NULL);

  /* Demo initialization */

  demo_create();

  /* Handle LittlevGL tasks */

  while (1)
    {
      lv_task_handler();
      usleep(10000);
    }

  return EXIT_SUCCESS;
}
