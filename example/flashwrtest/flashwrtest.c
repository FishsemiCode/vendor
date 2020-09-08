/****************************************************************************
 * apps/external/example/flashwrtest/flashwrtest.c
 *
 *   Copyright (C) 2020 FishSemi Inc. All rights reserved.
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

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "at_api.h"

#define FLASHWR_SIZE 1024
#define FLASHWR_FILE "/onchip/flashwr.test"

static unsigned char flashwr_content[FLASHWR_SIZE];
static unsigned int success_write;
static unsigned int blocked_write;

/****************************************************************************
 * Private Function
 ****************************************************************************/

static int flashwr_test(unsigned int minimal_cost, int times)
{
  int ret = 0;
  int clientfd;
  FILE *fp = NULL;
  int i;
  unsigned int remain;
  unsigned int start, end;
  unsigned int sum = 0;

  clientfd = at_client_open();
  if(clientfd < 0)
    {
      printf("Open client error\n");
      return -1;
    }

  for(i = 0; i < times; i++)
    {
      if(get_rfsleep_remain_time(clientfd, &remain))
        {
          printf("get_rfsleep_remain_time failed\n");
          goto clean;
        }

      if(remain >= minimal_cost)
        {
          start = TICK2MSEC(clock_systimer());

          fp = fopen(FLASHWR_FILE, "w");
          if(!fp)
            {
              printf("Open file error\n");
              goto clean;
            }

          ret = fwrite(flashwr_content, 1, FLASHWR_SIZE, fp);
          if(ret != FLASHWR_SIZE)
            {
              printf("fwrite failed\n");
              goto clean;
            }

          if(fflush(fp))
            {
              printf("fflush failed\n");
              goto clean;
            }

          fclose(fp);
          fp = NULL;
          success_write++;

          end = TICK2MSEC(clock_systimer());
          if(end > start)
            {
              sum += (end - start);
            }
          else
            {
              printf("time wrong\n");
              goto clean;
            }
        }
      else
        {
          blocked_write++;
        }
    }

  printf("Total write:%d, succeed:%u, blocked:%u\n",
    times, success_write, blocked_write);

  if(success_write)
    {
      printf("Write %dbytes %u times, average:%ums\n",
        FLASHWR_SIZE, success_write, (unsigned int)(sum / success_write));
    }

clean:
  if(fp) fclose(fp);
  unlink(FLASHWR_FILE);
  at_client_close(clientfd);
  return ret;
}

/****************************************************************************
 * Public function
 ****************************************************************************/

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int flashwrtest_main(int argc, char *argv[])
#endif
{
  unsigned int i;

  if (argc != 3)
    {
      printf("[usage]: flashwrtest [minimal cost(ms)] [write times]\n");
      return EXIT_SUCCESS;
    }

  success_write = 0;
  blocked_write = 0;

  for(i = 0; i < FLASHWR_SIZE; i++)
    {
      flashwr_content[i] = (unsigned char)(i & 0xff);
    }

  flashwr_test((unsigned int)atol(argv[1]), atoi(argv[2]));

  return EXIT_SUCCESS;
}
