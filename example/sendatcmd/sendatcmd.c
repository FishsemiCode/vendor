/****************************************************************************
 * apps/external/example/sendatcmd/sendatcmd.c
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

#include <nuttx/config.h>

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "at_client.h"


static int sendatcmd_daemon(int argc, char *argv[])
{
  int ret = -1;
  int clientfd;

  if (argc < 2)
    {
      syslog(LOG_ERR, "too few arguments!\n");
      return -1;
    }

  clientfd = at_client_open();
  if(clientfd < 0)
    {
      syslog(LOG_ERR, "open client error\n");
      return -1;
    }

  char* ptr = argv[1];
  for (int i = 0; i < strlen(argv[1]); i++, ptr++)
    {
      *ptr = toupper(*ptr);
    }

  ATResponse *response = NULL;
  ret = sendATRequest(clientfd, argv[1], &response);
  if (ret < 0 || response->error != NONE_ERROR)
    {
      syslog(LOG_ERR, "send at failed!\n");
      goto clean;
    }
  for(int i = 0; i < response->lineNumber; i++)
    {
      printf("\n%s\n", response->lines[i]);
    }
  printf("\nOK\n");
  at_c_response_free(response);

  ret = 0;

clean:
  at_client_close(clientfd);
  return ret;
}
/****************************************************************************
 * Public Funtions
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, char *argv[])
#else
int sendat_main(int argc, char *argv[])
#endif
{
  int ret;
  ret = task_create(argv[0],
          CONFIG_SENDATCMD_PRIORITY,
          CONFIG_SENDATCMD_STACKSIZE,
          sendatcmd_daemon,
          argv + 1);
  return ret > 0 ? 0 : ret;
}

