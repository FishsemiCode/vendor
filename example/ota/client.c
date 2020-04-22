/****************************************************************************
 * apps/external/example/ota/client.c
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


#include "client.h"

struct client_s *client_setup(const char *address, int port)
{
  struct client_s *c = (struct client_s *)malloc(sizeof(struct client_s));

  if (c == NULL)
    {
      printf("Failed to create client_s!\n");
      return NULL;
    }

  c->sock = socket(AF_INET, SOCK_STREAM, 0);
  if (c->sock < 0)
    {
      printf("Failed to create client socket!\n");
      free(c);
      return NULL;
    }

  if ((int)inet_addr(address) < 0)
    {
      int i;
      struct hostent *he;
      struct in_addr **addr_list;
      if ((he = gethostbyname(address)) == NULL)
        {
          printf("Failed to get host!\n");
          close(c->sock);
          free(c);
          return NULL;
        }
      addr_list = (struct in_addr **)he->h_addr_list;
      for (i = 0; addr_list[i] != NULL; i++)
        {
          c->server.sin_addr = *addr_list[i];
          break;
        }
    }
  else
    {
      c->server.sin_addr.s_addr = inet_addr(address);
    }
  c->server.sin_family = AF_INET;
  c->server.sin_port = htons(port);

  if (connect(c->sock, (struct sockaddr *)&c->server, sizeof(c->server)) < 0)
    {
      printf("Failed to connect to server!\n");
      return NULL;
    }

  return c;
}

ssize_t client_send(struct client_s *c, const void *data, size_t len)
{
  ssize_t ret = 0;

  if (c->sock > 0)
    {
      ret = send(c->sock, data, len, 0);
      if (ret < 0)
        {
          printf("Send failed ret = %zd\n", ret);
        }
      return ret;
    }

  return 0;
}

ssize_t client_recv(struct client_s *c, void *data, size_t len)
{
  ssize_t ret = 0;

  if (c->sock > 0)
    {
      ret = recv(c->sock, data, len, 0);
      if (ret < 0)
        {
          printf("Recv failed ret = %zd\n", ret);
        }
      return ret;
    }

  return 0;
}

void client_exit(struct client_s *c)
{
  if (c && c->sock)
    {
      close(c->sock);
    }
}
