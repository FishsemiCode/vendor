/****************************************************************************
 * apps/external/example/uidemo/fsdev.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "fsdev.h"

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

#ifndef MAX_PATH
#define MAX_PATH 64
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fsdev_ready
 *
 * Description:
 *   Tell if file system is ready
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv
 *
 * Returned Value:
 *   bool
 *
 ****************************************************************************/
bool fsdev_ready(struct _lv_fs_drv_t * drv)
{
  return true;  
}

/****************************************************************************
 * Name: fsdev_open
 *
 * Description:
 *   Open an exist file
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv (fs driver structure)
 *   void * file_p             (private file_t structure)
 *   const char *path          (file path)
 *   lv_fs_mode_t mode         (open permission)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fsdev_open (lv_fs_drv_t * drv, void * file_p, const char * path,
                        lv_fs_mode_t mode)
{
  int oflags = 0;
  int *fd_p = (int *)file_p;
  int fd;
  char abs_path[MAX_PATH];

  if (file_p == NULL || path == NULL)
    {
      return LV_FS_RES_INV_PARAM;
    }

  if (mode == LV_FS_MODE_WR)
    {
      oflags = O_WRONLY;
    }
  else if (mode == LV_FS_MODE_RD)
    {
      oflags = O_RDONLY;
    }
  else if (mode == (LV_FS_MODE_WR | LV_FS_MODE_RD))
    {
      oflags = O_RDWR;
    }

  strcpy(abs_path, "/");
  strcat(abs_path, path);
  fd = open(abs_path, oflags);
  if (fd < 0)
    {
      printf("Failed to open %s ret = %d\n", abs_path, fd);
      return LV_FS_RES_NOT_EX;
    }

  *fd_p = fd;

  return LV_FS_RES_OK;
}

/****************************************************************************
 * Name: fsdev_close
 *
 * Description:
 *   Close an opened file
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv (fs driver structure)
 *   void * file_p             (private file_t structure)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fsdev_close (lv_fs_drv_t * drv, void * file_p)
{
  if (file_p == NULL)
    {
      return LV_FS_RES_INV_PARAM;
    }

  close(*(int *)file_p);

  return LV_FS_RES_OK;
}

/****************************************************************************
 * Name: fsdev_read
 *
 * Description:
 *   Read data from an opened file
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv (fs driver structure)
 *   void * file_p             (private file_t structure)
 *   void * buf                (buf pointer to a memory block where to store the read data)
 *   uint32_t btr              (number of bytes to read)
 *   uint32_t * br             (the real number of read bytes)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fsdev_read(lv_fs_drv_t * drv, void * file_p, void * buf,
                       uint32_t btr, uint32_t * br)
{
  int ret, fd;

  if (file_p == NULL || br == NULL)
    {
      return LV_FS_RES_INV_PARAM;
    }

  fd = *(int *)file_p;

  ret = read(fd, buf, btr);
  if (ret < 0)
    {
      printf("Failed to read file, ret = %d\n", ret);
      return LV_FS_RES_FS_ERR;
    }

  *br = (uint32_t)ret;

  return LV_FS_RES_OK;
}

/****************************************************************************
 * Name: fsdev_write
 *
 * Description:
 *   Write into a file
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv (fs driver structure)
 *   void * file_p             (private file_t structure)
 *   void * buf                (buf pointer to a buffer with the bytes to write)
 *   uint32_t btr              (number of bytes to write)
 *   uint32_t * br             (the real number of write bytes, NULL if unused)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fsdev_write(lv_fs_drv_t * drv, void * file_p, const void * buf,
                        uint32_t btw, uint32_t * bw)
{
  int ret, fd;

  if (file_p == NULL)
    {
      return LV_FS_RES_INV_PARAM;
    }

  fd = *(int *)file_p;

  ret = write(fd, buf, btw);
  if (ret < 0)
    {
      printf("Failed to write file, ret = %d\n", ret);
      return LV_FS_RES_FS_ERR;
    }

  if (bw != NULL)
    {
      *bw = fd;
    }

  return LV_FS_RES_OK;
}

/****************************************************************************
 * Name: fsdev_seek
 *
 * Description:
 *   Set the read write pointer. Also expand the file size if necessary.
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv (fs driver structure)
 *   void * file_p             (private file_t structure)
 *   uint32_t pos              (the new position of read write pointer)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fsdev_seek(lv_fs_drv_t * drv, void * file_p, uint32_t pos)
{
  off_t ret;

  if (file_p == NULL)
    {
      return LV_FS_RES_INV_PARAM;
    }

  ret = lseek(*(int *)file_p, pos, SEEK_SET);
  if (ret < 0)
    {
      printf("Failed to seek file, ret = %d\n", ret);
      return LV_FS_RES_FS_ERR;
    }

  return LV_FS_RES_OK;
}

/****************************************************************************
 * Name: fsdev_size
 *
 * Description:
 *   Give the size of a file bytes
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv (fs driver structure)
 *   void * file_p             (private file_t structure)
 *   uint32_t * size_p         (pointer to a variable to store the size)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fsdev_size(lv_fs_drv_t * drv, void * file_p, uint32_t * size_p)
{
  off_t ret;

  if (file_p == NULL || size_p == NULL)
    {
      return LV_FS_RES_INV_PARAM;
    }

  ret = lseek(*(int *)file_p, 0, SEEK_END);
  if (ret < 0)
    {
      printf("Failed to get file size, ret = %d\n", ret);
      return LV_FS_RES_FS_ERR;
    }

  *size_p = (uint32_t)ret;

  lseek(*(int *)file_p, 0, SEEK_SET);

  return LV_FS_RES_OK;
}

/****************************************************************************
 * Name: fsdev_tell
 *
 * Description:
 *   Give the position of the read write pointer
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv (fs driver structure)
 *   void * file_p             (private file_t structure)
 *   uint32_t * pos_p          (pointer to to store the result)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fsdev_tell(lv_fs_drv_t * drv, void * file_p, uint32_t * pos_p)
{
   off_t ret;

  if (file_p == NULL || pos_p == NULL)
    {
      return LV_FS_RES_INV_PARAM;
    }

  ret = lseek(*(int *)file_p, 0, SEEK_CUR);
  if (ret < 0)
    {
      printf("Failed to get position, ret = %d\n", ret);
      return LV_FS_RES_FS_ERR;
    }

  *pos_p = (uint32_t)ret;

  return LV_FS_RES_OK; 
}

/****************************************************************************
 * Name: fsdev_remove
 *
 * Description:
 *   Delete a file
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv (fs driver structure)
 *   const char * path         (path of the file to delete)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fs_remove(lv_fs_drv_t * drv, const char *path)
{
  if (path == NULL)
    {
      return LV_FS_RES_INV_PARAM;
    }

  if (unlink(path))
    {
      printf("Failed to delete file %s\n", path);
      return LV_FS_RES_NOT_EX;
    }

  return LV_FS_RES_OK;
}

/****************************************************************************
 * Name: fsdev_trunc
 *
 * Description:
 *   Truncate the file size to the current position of the read write pointer
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv (fs driver structure)
 *   void * file_p             (private file_t structure)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fsdev_trunc(lv_fs_drv_t * drv, void * file_p)
{
  off_t cur;

  if (file_p == NULL)
    {
      return LV_FS_RES_INV_PARAM;
    }

  cur = lseek(*(int *)file_p, 0, SEEK_CUR);
  if (cur < 0)
    {
      printf("Failed to find current pos, ret = %d\n", cur);
      return LV_FS_RES_FS_ERR;
    }

  if (ftruncate(*(int *)file_p, cur))
    {
      printf("Failed to trunc file!\n");
      return LV_FS_RES_FS_ERR;
    }

  return LV_FS_RES_OK;
}

/****************************************************************************
 * Name: fsdev_rename
 *
 * Description:
 *   Rename a file
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv (fs driver structure)
 *   const char * oldname      (oldname path to the file)
 *   const char * newname      (newname path with the new name)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fsdev_rename(lv_fs_drv_t * drv, const char * oldname, const char * newname)
{
  if (oldname == NULL || newname == NULL)
    {
      return LV_FS_RES_INV_PARAM;
    }

  if (rename(oldname, newname))
    {
      printf("Failed to rename from %s to %s\n", oldname, newname);
      return LV_FS_RES_FS_ERR;
    }

  return LV_FS_RES_OK;
}

/****************************************************************************
 * Name: fsdev_free_space
 *
 * Description:
 *   Get the free and total size of a driver in kB
 *
 * Input Parameters:
 *   uint32_t * total_p        (pointer to store the total size KB)
 *   uint32_t * free_p         (pointer to store the free size KB)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fsdev_free_space(struct _lv_fs_drv_t * drv, uint32_t * total_p, uint32_t * free_p)
{
  return LV_FS_RES_NOT_IMP; 
}

/****************************************************************************
 * Name: fsdev_dir_open
 *
 * Description:
 *   Initialize a 'fs_read_dir_t' variable for directory reading
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv (fs driver structure)
 *   void * rddir_p            (pointer to a 'fs_read_dir_t' variable)
 *   const char * path         (path to a directory)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fsdev_dir_open(lv_fs_drv_t * drv, void * rddir_p, const char *path)
{
  return LV_FS_RES_NOT_IMP;
}

/****************************************************************************
 * Name: fsdev_dir_read
 *
 * Description:
 *   Read the next filename form a directory
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv (fs driver structure)
 *   void * rddir_p            (pointer to a 'fs_read_dir_t' variable)
 *   char * fn                 (pointer to a buffer to store the filename)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fsdev_dir_read(lv_fs_drv_t * drv, void * rddir_p, char *fn)
{
  return LV_FS_RES_NOT_IMP;
}

/****************************************************************************
 * Name: fsdev_dir_close
 *
 * Description:
 *   Close the directory reading
 *
 * Input Parameters:
 *   struct _lv_fs_drv_t * drv (fs driver structure)
 *   void * rddir_p            (pointer to a 'fs_read_dir_t' variable)
 *
 * Returned Value:
 *   lv_fs_res_t
 *
 ****************************************************************************/
lv_fs_res_t fsdev_dir_close(lv_fs_drv_t * drv, void * rddir_p)
{
  return LV_FS_RES_NOT_IMP;
}
