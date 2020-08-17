/****************************************************************************
 * apps/external/example/uidemo/fsdev.h
 *
 *   Copyright (C) 2020 Bo Zhang. All rights reserved.
 *   Author: Bo Zhang <zhangbo@fishsemi.com>
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

#ifndef __APPS_EXAMPLES_UIDEMO_FSDEV_H
#define __APPS_EXAMPLES_UIDEMO_FSDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <graphics/lvgl.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

bool fsdev_ready(struct _lv_fs_drv_t *drv);
lv_fs_res_t fsdev_open(struct _lv_fs_drv_t * drv, void * file_p, const char * path,
                       lv_fs_mode_t mode);
lv_fs_res_t fsdev_close(struct _lv_fs_drv_t * drv, void * file_p);
lv_fs_res_t fsdev_remove(struct _lv_fs_drv_t * drv, const char * fn);
lv_fs_res_t fsdev_read(struct _lv_fs_drv_t * drv, void * file_p, void * buf,
                       uint32_t btr, uint32_t * br);
lv_fs_res_t fsdev_write(struct _lv_fs_drv_t * drv, void * file_p, const void * buf,
                        uint32_t btw, uint32_t * bw);
lv_fs_res_t fsdev_seek(struct _lv_fs_drv_t * drv, void * file_p, uint32_t pos);
lv_fs_res_t fsdev_tell(struct _lv_fs_drv_t * drv, void * file_p, uint32_t * pos_p);
lv_fs_res_t fsdev_trunc(struct _lv_fs_drv_t * drv, void * file_p);
lv_fs_res_t fsdev_size(struct _lv_fs_drv_t * drv, void * file_p, uint32_t * size_p);
lv_fs_res_t fsdev_rename(struct _lv_fs_drv_t * drv, const char * oldname, const char * newname);
lv_fs_res_t fsdev_free_space(struct _lv_fs_drv_t * drv, uint32_t * total_p, uint32_t * free_p);

lv_fs_res_t fsdev_dir_open(struct _lv_fs_drv_t * drv, void * rddir_p, const char * path);
lv_fs_res_t fsdev_dir_read(struct _lv_fs_drv_t * drv, void * rddir_p, char * fn);
lv_fs_res_t fsdev_dir_close(struct _lv_fs_drv_t * drv, void * rddir_p);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_EXAMPLES_UIDEMO_FSDEV_H */
