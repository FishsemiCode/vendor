/****************************************************************************
 * apps/external/example/ota/ota_utils.h
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


#ifndef OTA_UTILS_H
#define OTA_UTILS_H

#include <stdint.h>

#define MAGIC_CODE  "FISH"
#define IMEI_LEN    16
#define VERSION_LEN 16
#define MAX_PATH    64
#define MAX_CMD_LEN 256

struct common_header
{
  char magic[4];
  uint8_t imei[IMEI_LEN];
  uint8_t packet_type;
  uint16_t payload_len;
  uint8_t payload[0];
};

enum request_packet_type
{
  OTA_UPDATE_REQ,
  OTA_VERSION_REQ,
  OTA_DATA_HEADER_REQ,
  OTA_DATA_SEND_REQ,
  OTA_DATA_END_REQ,
};

enum reponse_packet_type
{
  OTA_UPDATE_RSP = 0x80,
  OTA_VERSION_RSP,
  OTA_DATA_HEADER_RSP,
  OTA_DATA_SEND_RSP,
  OTA_DATA_END_RSP,
};

struct patch_context
{
  char old_version[VERSION_LEN];
  char new_version[VERSION_LEN];
  char patch_name[MAX_PATH];
  uint32_t patch_length;
  uint32_t patch_pos;
};

void get_req_resp_patch_info(struct common_header *header, struct patch_context *patch);
int get_req_resp_len(struct common_header *header);
uint8_t *create_ota_update_req_resp(const uint8_t *imei, bool is_req);
uint8_t *create_ota_version_req_resp(const uint8_t *imei,
                                      const char *version, bool is_req);
uint8_t *create_ota_data_header_req_resp(const uint8_t *imei,
                                          uint32_t length, bool is_req);
uint8_t *create_ota_data_send_req_resp(const uint8_t *imei, uint8_t *data,
                                        uint16_t len, bool is_req);
uint8_t *create_ota_data_end_req_resp(const uint8_t *imei, bool is_req);

void get_version_from_req_resp(const char *packet, char *version);
#endif

