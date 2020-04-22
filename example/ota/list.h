/****************************************************************************
 * apps/external/example/ota/list.h
 *
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


#ifndef __DLIST__H__
#define __DLIST__H__

#include <stdlib.h>

struct dlist
{
  struct dlist *next;
  struct dlist *prev;
};

#define INIT_DLIST(name) { .next = &name, .prev = &name }

#define DECLARE_DLIST(name)			\
	struct dlist name = INIT_DLIST(name)

static inline void dlist_init(struct dlist *list)
{
  list->next = list->prev = list;
}

static inline void dlist_add_before(struct dlist *node,
					 struct dlist *new_node)
{
  new_node->prev = node->prev;
  new_node->next = node;
  new_node->next->prev = new_node;
  new_node->prev->next = new_node;
}

static inline void dlist_add_after(struct dlist *node,
					struct dlist *new_node)
{
  new_node->prev = node;
  new_node->next = node->next;
  new_node->next->prev = new_node;
  new_node->prev->next = new_node;
}

static inline void dlist_add_head(struct dlist *list,
				       struct dlist *node)
{
  dlist_add_after(list, node);
}

static inline void dlist_add_tail(struct dlist *list,
				       struct dlist *node)
{
  dlist_add_before(list, node);
}

static inline int dlist_is_empty(struct dlist *list)
{
  return list->next == list;
}

static inline void dlist_del(struct dlist *node)
{
  node->next->prev = node->prev;
  node->prev->next = node->next;
  node->next = node->prev = node;
}

static inline struct dlist *dlist_first(struct dlist *list)
{
  return dlist_is_empty(list) ? NULL : list->next;
}

#define dlist_for_each(list, node)		\
	for ((node) = (list)->next;		\
	     (node) != (list);			\
	     (node) = (node)->next)
/** @} */

#define dlist_offset_of(structure, member)      \
    ((intptr_t) &(((structure *) 0)->member))

#define dlist_container_of(ptr, structure, member)  \
    (void *)((intptr_t)(ptr) - dlist_offset_of(structure, member))

#endif /* __DLIST__H__ */
