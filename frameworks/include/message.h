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

#ifndef MESSAGE_H_
#define MESSAGE_H_

/****************************************************************************
 * Public MACRO definition
 ***************************************************************************/

#define MESSAGE_MAX_MSG_CNT     10
#define MESSAGE_MAX_MSG_SIZE    CONFIG_MQ_MAXMSGSIZE

/****************************************************************************
 * Public API definition
 ***************************************************************************/

typedef struct message
{
  uint16_t              id;
  char                  msg[0];
} message_t;

typedef void (*message_handler_t)(message_t *msg, uint16_t len);

void *message_init(message_handler_t handler, uint16_t max_cnt, uint16_t max_size);
void message_fini(void *task);
int message_send(void *task, const message_t *msg, uint16_t len);
int message_loop(void *task);

#endif /* MESSAGE_H_ */
