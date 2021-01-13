/****************************************************************************
 * vendor/apps/fishpm/fishpm
 *
 *   Copyright (C) 2021 Fishsemi Inc. All rights reserved.
 *   Author: Fishsemi <fishsemi@fishsemi.com>
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
#include <string.h>
#include <stdlib.h>

#include "nuttx/power/pm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * fishpm_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, char *argv[])
#else
int fishpm_main(int argc, char *argv[])
{
  if (argc == 1)
    {
      pm_stay(PM_IDLE_DOMAIN, PM_NORMAL);
    }
  else if (argc == 3)
    {
      if (strcmp(argv[1], "stay") == 0)
        {
          if (strcmp(argv[2], "normal") == 0)
          {
            pm_stay(PM_IDLE_DOMAIN, PM_NORMAL);
          }
        else if (strcmp(argv[2], "idle") == 0)
          {
            pm_stay(PM_IDLE_DOMAIN, PM_IDLE);
          }
        else if (strcmp(argv[2], "standby") == 0)
          {
            pm_stay(PM_IDLE_DOMAIN, PM_STANDBY);
          }
        else if (strcmp(argv[2], "sleep") == 0)
          {
            pm_stay(PM_IDLE_DOMAIN, PM_SLEEP);
          }
        }
      else if(strcmp(argv[1], "relax") == 0)
        {
          if (strcmp(argv[2], "normal") == 0)
            {
              pm_relax(PM_IDLE_DOMAIN, PM_NORMAL);
            }
          else if (strcmp(argv[2], "idle") == 0)
            {
              pm_relax(PM_IDLE_DOMAIN, PM_IDLE);
            }
          else if (strcmp(argv[2], "standby") == 0)
            {
              pm_relax(PM_IDLE_DOMAIN, PM_STANDBY);
            }
          else if (strcmp(argv[2], "sleep") == 0)
            {
              pm_relax(PM_IDLE_DOMAIN, PM_SLEEP);
            }
        }
    }
  else
    {
      printf("ERROR: Missing required arguments\n");
      printf("Input: fishpm relax/stay normal/idle/standby/sleep\n");
      printf("NO argument: fishpm stay normal\n");
      return EXIT_FAILURE;
    }

  return 0;
}
#endif
