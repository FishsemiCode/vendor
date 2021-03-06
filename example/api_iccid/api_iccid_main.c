/****************************************************************************
 * apps/external/example/api_iccid/api_iccid_main.c
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
#include <stdio.h>
#include <fcntl.h>

#include "at_api.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * api_iccid_main
 ****************************************************************************/

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int api_iccid_main(int argc, char *argv[])
#endif
{
  int fd = -1;
  int ret = 0;
  at_spi_iccid iccid;
  syslog(LOG_INFO, "api iccid test...\n");

  fd = at_client_open();
  if(fd < 0)
    {
      syslog(LOG_ERR, "%s: Open client error\n", __func__);
      return -1;
    }
  ret = get_iccid(fd, &iccid);
  syslog(LOG_INFO, "[%s]fd:%d, ret:%d\n", __func__, fd, ret);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: get_iccid fail\n", __func__);
      goto clean;
    }
  syslog(LOG_INFO, "[%s]iccid:%s\n", __func__, iccid.iccid);

clean:
  if(fd >= 0)
    {
      at_client_close(fd);
    }
  syslog(LOG_INFO, "%s:quit\n", __func__);
  return ret;
}
