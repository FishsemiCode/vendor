/****************************************************************************
 * apps/external/example/uidemo/fbdev.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gábor Kiss-Vámosi <kisvegabor@gmail.com>
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

#include "fbdev.h"

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/rgbcolors.h>

#include <nuttx/lcd/ili9486_lcd_ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef FBDEV_PATH
#  define FBDEV_PATH  "/dev/ili9486"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fb_state_s
{
  int fd;
  struct ili9486_lcd_attributes_s attr;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct fb_state_s state;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fbdev_init
 *
 * Description:
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int fbdev_init(void)
{
  FAR const char *fbdev = FBDEV_PATH;
  int ret;

  /* Open the framebuffer driver */

  state.fd = open(fbdev, O_RDWR);
  if (state.fd < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", fbdev, errcode);
      return EXIT_FAILURE;
    }

  /* Get the characteristics of the framebuffer */

  ret = ioctl(state.fd, ILI9486_LCDIOC_GETATTRIBUTES,
              (unsigned long)((uintptr_t)&state.attr));
  if (ret < 0)
    {
      int errcode = errno;

      fprintf(stderr, "ERROR: ioctl(ILI9486_LCDIOC_GETATTRIBUTES) failed: %d\n",
              errcode);
      close(state.fd);
      return EXIT_FAILURE;
    }

  printf("LCD Attributes:\n");
  printf("lcd width : %u\n", state.attr.lcd_width);
  printf("lcd height: %u\n", state.attr.lcd_height);

  return EXIT_SUCCESS;
}

static int ili9486_lcd_flush(int fd, int len, uint16_t *matrix)
{
  ssize_t nwritten;

  nwritten = write(fd, (char *)matrix, len);
  if (nwritten < 0)
    {
      int errcode = errno;
      printf("write failed: %d\n", errcode);

      if (errcode != EINTR)
        {
          return nwritten;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: fbdev_flush
 *
 * Description:
 *   Flush a buffer to the marked area
 *
 * Input Parameters:
 *   x1      - Left coordinate
 *   y1      - Top coordinate
 *   x2      - Right coordinate
 *   y2      - Bottom coordinate
 *   color_p - A n array of colors
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void fbdev_flush(struct _disp_drv_t *disp_drv, const lv_area_t *area,
                 lv_color_t *color_p)
{
  struct ili9486_lcd_matrix_s matrix;
  int32_t x1 = area->x1;
  int32_t y1 = area->y1;
  int32_t x2 = area->x2;
  int32_t y2 = area->y2;
  int32_t act_x1;
  int32_t act_y1;
  int32_t act_x2;
  int32_t act_y2;
  int ret = 0;
  uint16_t color;

  /* Return if the area is out the screen */

  if (x2 < 0)
    {
      return;
    }

  if (y2 < 0)
    {
      return;
    }

  if (x1 > state.attr.lcd_width - 1)
    {
      return;
    }

  if (y1 > state.attr.lcd_height - 1)
    {
      return;
    }

  /* Truncate the area to the screen */

  act_x1 = x1 < 0 ? 0 : x1;
  act_y1 = y1 < 0 ? 0 : y1;
  act_x2 = x2 > state.attr.lcd_width - 1 ? state.attr.lcd_width - 1 : x2;
  act_y2 = y2 > state.attr.lcd_height - 1 ? state.attr.lcd_height - 1 : y2;

  matrix.x = act_x1;
  matrix.y = act_y1;
  matrix.w = act_x2 - act_x1 + 1;
  matrix.h = act_y2 - act_y1 + 1;

  ret = ioctl(state.fd, ILI9486_LCDIOC_SET_MATRIX, (unsigned long)&matrix);
  if (ret < 0)
    {
      printf("flush: ioctl(ILI9486_LCDIOC_SET_MATRIX) failed: %d\n", errno);
      goto end;
    }

  ili9486_lcd_flush(state.fd, matrix.w * matrix.h, (uint16_t *)&color_p->full);

end:
  /* Tell the flushing is ready */
  lv_disp_flush_ready(disp_drv);
}

/****************************************************************************
 * Name: fbdev_fill
 *
 * Description:
 *   Fill an area with a color
 *
 * Input Parameters:
 *   x1    - Left coordinate
 *   y1    - Top coordinate
 *   x2    - Right coordinate
 *   y2    - Bottom coordinate
 *   color - The fill color
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void fbdev_fill(struct _disp_drv_t *disp_drv,
                int32_t x1, int32_t y1,
                int32_t x2, int32_t y2,
                lv_color_t color)
{
  struct ili9486_lcd_matrix_s matrix;
  int32_t act_x1;
  int32_t act_y1;
  int32_t act_x2;
  int32_t act_y2;
  int ret = 0;

  /* Return if the area is out the screen */

  if (x2 < 0)
    {
      return;
    }

  if (y2 < 0)
    {
      return;
    }

  if (x1 > state.attr.lcd_width - 1)
    {
      return;
    }

  if (y1 > state.attr.lcd_height - 1)
    {
      return;
    }

  /* Truncate the area to the screen */

  act_x1 = x1 < 0 ? 0 : x1;
  act_y1 = y1 < 0 ? 0 : y1;
  act_x2 = x2 > state.attr.lcd_width - 1 ? state.attr.lcd_width - 1 : x2;
  act_y2 = y2 > state.attr.lcd_height - 1 ? state.attr.lcd_height - 1 : y2;

  matrix.x = act_x1;
  matrix.y = act_y1;
  matrix.w = act_x2 - act_x1 + 1;
  matrix.h = act_y2 - act_y1 + 1;

  ret = ioctl(state.fd, ILI9486_LCDIOC_SET_MATRIX, (unsigned long)&matrix);
  if (ret < 0)
    {
      printf("fill: ioctl(ILI9486_LCDIOC_SET_MATRIX) failed: %d\n", errno);
      goto end;
    }

  ili9486_lcd_flush(state.fd, matrix.w * matrix.h, (uint16_t *)&color);

end:
  /* Tell the flushing is ready */
  lv_disp_flush_ready(disp_drv);
}

/****************************************************************************
 * Name: fbdev_map
 *
 * Description:
 *   Write an array of pixels (like an image) to the marked area
 *
 * Input Parameters:
 *   x1      - Left coordinate
 *   y1      - Top coordinate
 *   x2      - Right coordinate
 *   y2      - Bottom coordinate
 *   color_p - An array of colors
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void fbdev_map(struct _disp_drv_t *disp_drv,
               int32_t x1, int32_t y1,
               int32_t x2, int32_t y2,
               FAR const lv_color_t *color_p)
{
  fbdev_fill(disp_drv, x1, y1, x2, y2, *color_p);
}
