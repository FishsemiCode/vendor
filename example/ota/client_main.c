/****************************************************************************
 * apps/external/example/ota/client_main.c
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


#include <stdbool.h>
#include "client.h"
#include "ota_utils.h"
#include <signal.h>
#include "at_api.h"
#include <sys/utsname.h>
#include  <sys/stat.h>

#define MAX_PACKET_SIZE 1500
#define MAX_LINE    256

#define MAX_RETRY_NUM 5


#define OTA_DIR "/data/ota"
#define PATCH_NAME "ap.patch"


static struct client_s *g_client;
static struct patch_context g_patch;

extern void up_reset(int status);

struct save_patch_context
{
  FILE *fd;
  uint32_t pos;
  uint32_t received;
};


static struct save_patch_context g_save;

static int g_reg_staus = -1;
static pthread_mutex_t g_ota_reg_mutex;
static pthread_cond_t g_ota_reg_cond;

static uint8_t g_buffer[MAX_PACKET_SIZE];

static void sig_exit(int s)
{
  client_exit(g_client);
  exit(0);
}

static int set_get_saved_length(const char *path, struct patch_context *patch,
                                struct save_patch_context *save, bool is_set)
{
  FILE *fd;
  char *str;
  char content[MAX_LINE];
  char fullpath[MAX_PATH];

  if (!is_set)
    {
      save->pos = 0;
    }

  sprintf(fullpath, "%s/history.txt", path);

  fd = fopen(fullpath, "r+");
  if (fd == NULL)
    {
      if (!is_set)
        {
          sprintf(content, "%s/%s", path, PATCH_NAME);
          remove(content);
          return 0;
        }
      else
        {
          fd = fopen(fullpath, "w+");
          if (fd == NULL)
            {
              return 0;
            }
          else
            {
              fprintf(fd, "%s_%s: 0",  patch->old_version, patch->new_version);
              fclose(fd);
              return 0;
            }
        }
    }

  str = fgets(content, MAX_LINE, fd);
  if (str == NULL)
    {
      goto end;
    }
  sprintf(fullpath, "%s_%s: ", patch->old_version, patch->new_version);
  if (strncmp(fullpath, content, strlen(fullpath)))
    {
      goto end;
    }
  else
    {
      if (!is_set)
        {
          str = content + strlen(fullpath);
          save->pos = atoi(str);
          fclose(fd);
          return save->pos;
        }
      else
        {
          fseek(fd, 0, SEEK_SET);
          sprintf(content, "%s%d", fullpath, save->pos);
          fwrite(content, sizeof(char), strlen(content), fd);
        }
    }
end:
  fclose(fd);
  return save->pos;
}


static int save_patch_to_file(const uint8_t *imei, const uint8_t *packet,
                              uint32_t len, struct save_patch_context *save)
{
  char fullpath[MAX_PATH];
  uint32_t msg_len = 0;
  uint8_t *msg = NULL;

  if (save->fd == NULL)
    {
      return 0;
    }

  fwrite(packet, sizeof(uint8_t), len, save->fd);
  save->pos += len;
  save->received += len;

  set_get_saved_length(OTA_DIR, &g_patch, save, true);

  if (save->received == g_patch.patch_length)
    {
      printf("Save end!\n");
      fclose(save->fd);
      sprintf(fullpath, "%s/history.txt", OTA_DIR);
      remove(fullpath);
      save->fd = NULL;
      save->pos = 0;
      save->received = 0;
      msg = create_ota_data_end_req_resp(imei, true);
      msg_len = get_req_resp_len((struct common_header *)msg);
      client_send(g_client, msg, msg_len);
      free(msg);
    }
  return 1;
}


static int ota_get_version(char *buf)
{
  struct utsname name;
  char *p;
  if (uname(&name) == -1)
    {
      printf("uname fail\n");
      return -1;
    }
  p = strchr(name.version, ' ');
  if (p)
    {
      *p = '\0';
    }

  strcpy(buf, name.version);
  return 0;
}

static int handle_ota_cmd(const char *msg, ssize_t len)
{
  uint32_t packet_len = 0;
  uint8_t *packet = NULL;
  const uint8_t *imei;
  uint8_t type;
  char new_patch_name[128];
  char oldVersion[VERSION_NAMELEN];

  if (strncmp(msg, MAGIC_CODE, strlen(MAGIC_CODE)))
    {
      printf("magic code mismatch %02x %02x %02x %02x, return\n", msg[0], msg[1], msg[2], msg[3]);
      return -1;
    }

  imei = (const uint8_t *)(msg + strlen(MAGIC_CODE));

  type = *(msg + strlen(MAGIC_CODE) + IMEI_LEN);

  switch (type)
    {
      case OTA_UPDATE_RSP:
        if (ota_get_version(oldVersion))
          {
            return -1;
          }
        printf("OTA_UPDATE_RSP received, send OTA_VERSION_REQ:%s\n", oldVersion);
        packet = create_ota_version_req_resp(imei, oldVersion, true);
        strcpy(g_patch.old_version, oldVersion);
        break;

      case OTA_VERSION_RSP:
        printf("OTA_VERSION_RSP received, send OTA_DATA_HEADER_REQ\n");
        get_version_from_req_resp(msg, g_patch.new_version);
        printf("New version: %s\n", g_patch.new_version);
        if (strcmp(g_patch.new_version, "INVALIDVER") == 0)
          {
            return 2;
          }
        packet = create_ota_data_header_req_resp(imei,
            set_get_saved_length(OTA_DIR, &g_patch, &g_save, false), true);
        printf("current pos: %d\n", g_save.pos);
        break;

      case OTA_DATA_HEADER_RSP:
        get_req_resp_patch_info((struct common_header *)msg, &g_patch);
        if (g_patch.patch_length == 0)
          {
            printf("OTA_DATA_HEADER_RSP received, no remain send OTA_DATA_END_REQ\n");
            packet = create_ota_data_end_req_resp(imei, true);
          }
        else
          {
            printf("OTA_DATA_HEADER_RSP received, send OTA_DATA_REQ\n");
            packet = create_ota_data_send_req_resp(imei, NULL, 0, true);
            sprintf(new_patch_name, "%s/%s", OTA_DIR, PATCH_NAME);
            g_save.fd = fopen(new_patch_name, "ab+");
            if (g_save.fd == NULL)
              {
                printf("Failed to create patch file:%s!\n", new_patch_name);
              }
            else
              {
                fseek(g_save.fd, (long)g_save.pos, SEEK_SET);
              }
          }
        break;
      case OTA_DATA_END_RSP:
        printf("OTA_DATA_END_RSP received, teminate\n");
        return 1;
      default:
        break;
    }

  packet_len = get_req_resp_len((struct common_header *)packet);
  client_send(g_client, packet, packet_len);
  free(packet);

  return 0;
}


static bool is_registered(int reg_Status)
{
  return g_reg_staus == 1 || g_reg_staus == 5;
}


static void handle_cereg(const char *s)
{
  int ret;
  int value;
  char *line = (char *)s;
  syslog(LOG_INFO, "%s: %s\n", __func__, s);
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextint(&line, &value);
  if (ret < 0)
    {
      return;
    }
  pthread_mutex_lock(&g_ota_reg_mutex);
  g_reg_staus = value;
  pthread_cond_signal(&g_ota_reg_cond);
  pthread_mutex_unlock(&g_ota_reg_mutex);
}


int otaclient_main(int argc, char *argv[])
{
  ssize_t ret = 0;
  uint8_t *packet;
  uint32_t packet_len;
  int clientfd = -1;
  at_spi_imei imei;
  int retry_num = 0;

  if (argc != 3)
    {
      printf("Usage: ./client ip port!\n");
      return -1;
    }

  signal(SIGINT, sig_exit);

  ret = mkdir(OTA_DIR, 0644);
  if (ret)
    {
      printf("mkdir error:%d\n", errno);
    }

retry:

  pthread_mutex_init(&g_ota_reg_mutex, NULL);
  pthread_cond_init(&g_ota_reg_cond, NULL);

  clientfd = at_client_open();
  if(clientfd < 0)
    {
      printf("%s: Open client error\n", __func__);
      return -1;
    }

  ret = register_indication(clientfd, "+CEREG", handle_cereg);
  if(ret < 0)
    {
      printf("%s: register_indication fail\n", __func__);
      goto clean;
    }

  ret = set_ceregindicationstatus(clientfd, 1);
  if (ret < 0)
    {
      printf("set_ceregindicationstatus fail\n");
      goto clean;
    }

  pthread_mutex_lock(&g_ota_reg_mutex);
  while (!is_registered(g_reg_staus))
    {
      pthread_cond_wait(&g_ota_reg_cond, &g_ota_reg_mutex);
    }
  pthread_mutex_unlock(&g_ota_reg_mutex);

  ret = get_imei(clientfd, &imei);
  if (ret < 0)
    {
      printf("%s: get_imei fail\n", __func__);
      goto clean;
    }

  g_client = client_setup(argv[1], atoi(argv[2]));
  if (!g_client)
    {
      ret = -1;
      printf("client_setup fail\n");
      goto clean;
    }

  packet = create_ota_update_req_resp((const uint8_t *)imei.imei, true);
  printf("packet [0] [1] [2] [3] = %02x, %02x, %02x, %02x\n", packet[0], packet[1], packet[2], packet[3]);
  packet_len = get_req_resp_len((struct common_header *)packet);

  client_send(g_client, packet, packet_len);

  while(1)
    {
      ret = client_recv(g_client, g_buffer, sizeof(g_buffer));
      if (ret > 0 && !save_patch_to_file((const uint8_t *)imei.imei, g_buffer, ret, &g_save))
        {
          ret = handle_ota_cmd((const char *)g_buffer, ret);
          if (ret)
            {
              break;
            }
        }
      else if (ret <= 0)
      {
        printf("client_recv fail:%d,%d,%s\n", ret, errno, strerror(errno));
        break;
      }
    }

clean:
  g_reg_staus = -1;
  memset(&g_save, 0x0, sizeof(g_save));
  memset(&g_patch, 0x0, sizeof(g_patch));
  client_exit(g_client);
  if (g_client)
    {
      free(g_client);
    }
  at_client_close(clientfd);
  pthread_mutex_destroy(&g_ota_reg_mutex);
  pthread_cond_destroy(&g_ota_reg_cond);

  if (ret == 1)
    {
      printf("*****download successfully*******\n");
      up_reset(3);
    }
  else if (ret != 2 && retry_num++ < MAX_RETRY_NUM)
    {
        printf("ota fail, retry:%d\n", retry_num);
        goto retry;
    }

  return ret;
}
