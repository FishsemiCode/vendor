/****************************************************************************
 * apps/external/example/ota/ota_utils.c
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
#include "ota_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void fill_common_header(struct common_header *header,
                               const uint8_t *imei, uint8_t type)
{
  strcpy(header->magic, MAGIC_CODE);
  memcpy(header->imei, imei, IMEI_LEN);
  header->packet_type = type;
}

void get_req_resp_patch_info(struct common_header *header, struct patch_context *patch)
{

  patch->patch_length = *(uint32_t*)header->payload;
  printf("@@@@@@@@@@@@ patch_length = %d @@@@@@@@@@@@@@@@\n", patch->patch_length);

  sprintf(patch->patch_name, "%s_%s.patch", patch->old_version, patch->new_version);
  printf("@@@@@@@@@@@@ patch_name = %s @@@@@@@@@@@@@@@@\n", patch->patch_name);
}

int get_req_resp_len(struct common_header *header)
{
  if (header != NULL)
    {
      return sizeof(struct common_header) + header->payload_len;
    }
  return 0;
}

static uint8_t * create_no_payload_req_resp(const uint8_t *imei, uint8_t type)
{
  struct common_header *header;
  uint8_t *req;

  req = (uint8_t *)malloc(sizeof(struct common_header));
  if (req == NULL)
    {
      printf("Failed to alloc update req!\n");
      return NULL;
    }

  header = (struct common_header *)req;
  fill_common_header(header, imei, type);
  header->payload_len = 0;

  return req;
}

void get_version_from_req_resp(const char *packet, char *version)
{
  const struct common_header *header = (const struct common_header *)packet;
  uint16_t len = header->payload_len;

  version[len] = 0;

  memcpy(version, header->payload, len);
}

uint8_t * create_ota_update_req_resp(const uint8_t *imei, bool is_req)
{
  return create_no_payload_req_resp(imei, is_req ? OTA_UPDATE_REQ : OTA_UPDATE_RSP);
}

uint8_t * create_ota_data_end_req_resp(const uint8_t *imei, bool is_req)
{
  return create_no_payload_req_resp(imei, is_req ? OTA_DATA_END_REQ : OTA_DATA_END_RSP);
}

uint8_t * create_ota_version_req_resp(const uint8_t *imei,
                                      const char *version, bool is_req)
{
  struct common_header *header;
  uint8_t *req;

  req = (uint8_t *)malloc(sizeof(struct common_header) + strlen(version));
  if (req == NULL)
    {
      printf("Failed to alloc version req!\n");
      return NULL;
    }

  header = (struct common_header *)req;
  fill_common_header(header, imei, is_req ? OTA_VERSION_REQ : OTA_VERSION_RSP);
  header->payload_len = strlen(version);
  memcpy(header->payload, version, strlen(version));

  return req;
}

uint8_t * create_ota_data_header_req_resp(const uint8_t *imei,
                                          uint32_t length, bool is_req)
{
  struct common_header *header;
  uint32_t *p = NULL;
  uint8_t *req;

  req = (uint8_t *)malloc(sizeof(struct common_header) + sizeof(uint32_t));
  if (req == NULL)
    {
      printf("Failed to alloc data header req!\n");
      return NULL;
    }

  header = (struct common_header *)req;
  fill_common_header(header, imei, is_req ? OTA_DATA_HEADER_REQ : OTA_DATA_HEADER_RSP);
  header->payload_len = sizeof(uint32_t);
  p = (uint32_t *)header->payload;
  *p = length;

  return req;
}

uint8_t * create_ota_data_send_req_resp(const uint8_t *imei, uint8_t *data,
                                       uint16_t len, bool is_req)
{
  if (!is_req)
    {
      struct common_header *header;
      uint8_t *req;

      req = (uint8_t *)malloc(sizeof(struct common_header) + len);
      if (req == NULL)
        {
          printf("Failed to alloc data req!\n");
          return NULL;
        }

      header = (struct common_header *)req;
      fill_common_header(header, imei, OTA_DATA_SEND_RSP);
      header->payload_len = len;
      memcpy(header->payload, data, len);
      return req;
    }
  else
    {
      return create_no_payload_req_resp(imei, OTA_DATA_SEND_REQ);
    }
}
