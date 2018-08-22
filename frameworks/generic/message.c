/* -----------------------------------------------------------------------------
 * Copyright (c) 2018 Pinecone Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * -------------------------------------------------------------------------- */

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <mqueue.h>
#include <debug.h>

#include <message.h>

typedef struct message_task
{
  struct mq_attr        mqattr;
  mqd_t                 mqfd;
  message_handler_t     handler;
} message_task_t;

void *message_init(message_handler_t handler, uint16_t max_cnt, uint16_t max_size)
{
  char mq_name[12];
  message_task_t *mt = malloc(sizeof(message_task_t));
  if (mt == NULL)
    return NULL;

  sprintf(mq_name, "mq-%p", mt);

  mt->mqattr.mq_maxmsg  = max_cnt ? max_cnt : MESSAGE_MAX_MSG_CNT;
  mt->mqattr.mq_msgsize = max_size ? max_size : MESSAGE_MAX_MSG_SIZE;
  mt->mqattr.mq_flags   = 0;

  mt->mqfd = mq_open(mq_name, O_RDWR|O_CREAT, 0666, &mt->mqattr);
  if (mt->mqfd == (mqd_t)ERROR)
    {
      _err("message - mq open failed:%d\n", errno);
      free(mt);
      return NULL;
    }
  mt->handler = handler;

  return (void *)mt;
}

void message_fini(void *task)
{
  message_task_t *mt;
  char mq_name[12];

  DEBUGASSERT(task != NULL);
  mt = (message_task_t *)task;

  sprintf(mq_name, "mq-%p", mt);

  mq_close(mt->mqfd);
  mq_unlink(mq_name);
  free(mt);
}

int message_send(void *task, const message_t *msg, uint16_t len)
{
  message_task_t *mt;

  DEBUGASSERT(task != NULL);
  mt = (message_task_t *)task;

  return mq_send(mt->mqfd, (FAR const char *)msg, len, 42);
}

int message_loop(void *task)
{
  int nbytes;
  message_task_t *mt;
  char *msg_buffer;
  message_t *message;

  DEBUGASSERT(task != NULL);
  mt = (message_task_t *)task;

  msg_buffer = malloc(mt->mqattr.mq_msgsize);
  if (msg_buffer == NULL)
    return -ENOMEM;

  message = (message_t *)&msg_buffer[0];

  while(1)
  {
    nbytes = mq_receive(mt->mqfd, msg_buffer,
            mt->mqattr.mq_msgsize, 0);
    if (nbytes < 0)
      {
        _err("message - rcv error:%d, %d!!\n", errno, nbytes);
      }
    else
      {
        mt->handler(message, nbytes);
      }
  }

  free(msg_buffer);

  return 0;
}
