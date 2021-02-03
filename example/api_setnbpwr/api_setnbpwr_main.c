/****************************************************************************
 * apps/external/example/api_setnbpwr/api_setnbpwr_main.c
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
 * api_setnbpwr_main
 ****************************************************************************/

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int api_setnbpwr_main(int argc, char *argv[])
#endif
{
  int fd = -1;
  int ret = -1;
  int pwr = -1;
  syslog(LOG_INFO, "api setnbpwr test...\n");

  fd = at_client_open();
  if(fd < 0)
    {
      syslog(LOG_ERR, "%s: Open client error\n", __func__);
      return -1;
    }

  if (set_radiopower(fd, false) < 0)
    {
      syslog(LOG_ERR, "%s: set_radiopower fail\n", __func__);
      return -1;
    }

  ret = set_pwrmaxvalue(fd, 20);
  if(ret < 0)
    {
      syslog(LOG_ERR, "%s: set pwrmaxvalue fail\n", __func__);
      return -1;
    }

  get_pwrmaxvalue(fd, &pwr);
  syslog(LOG_INFO, "%s: get nb power:%d\n", __func__, pwr);

  return 0;
}
