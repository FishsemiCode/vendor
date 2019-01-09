/****************************************************************************
 * apps/external/example/api_gpio/api_gpio_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <mqueue.h>

#include <message.h>
#include <gpio.h>

static void *g_rcvr;
static gpio_handle_t g_handle;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * api_gpio_main
 ****************************************************************************/

static int message_handler(message_t *msg, uint16_t len)
{
  msg_gpio_t *m = (msg_gpio_t *)msg;
  switch (m->id)
    {
      case MSG_GPIO_CHANGED:
          printf("rcv pid:%d pin:%d state:%d time:%u!!\n",
                  getpid(),
                  m->changed.pin, m->changed.state,
                  m->changed.time);
          break;
      case MSG_EXIT_LOOP:
          return MESSAGE_EXIT_LOOP;
      default:
          break;
    }

  return MESSAGE_HANDLED;
}

static int api_gpio_task(int argc, char *argv[])
{
  g_rcvr = message_init(message_handler, 0, 0);
  if (g_rcvr == NULL)
    return -1;

  g_handle = gpio_register(g_rcvr, 0x31, 0);

  message_loop(g_rcvr);

  gpio_deregister(g_handle);

  message_fini(g_rcvr);

  return 0;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int api_gpio_main(int argc, char *argv[])
#endif
{
  int ret;

  printf("api gpio test...\n");

  ret = task_create(argv[0],
          CONFIG_EXAMPLES_API_GPIO_PRIORITY,
          CONFIG_EXAMPLES_API_GPIO_STACKSIZE,
          api_gpio_task,
          argv + 1);

  return ret > 0 ? 0 : ret;
}
