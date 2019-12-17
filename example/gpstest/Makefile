############################################################################
# apps/external/example/gpstest/Makefile
#
#   Copyright (C) 2018 Pinecone Inc. All rights reserved.
#   Author: Pinecone <pinecone@pinecone.net>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs

APPNAME = gpstest
PRIORITY = CONFIG_GPSTEST_PRIORITY
STACKSIZE = CONFIG_GPSTEST_STACKSIZE

MAINSRC = gpstest.c
ifeq ($(CONFIG_GPSTEST_GUI),y)
CSRCS += fbdev.c gpstest_gui.c tp_cal.c tp.c
CSRCS += img_singal_0.c img_singal_1.c img_singal_2.c img_singal_3.c img_singal_4.c img_singal_5.c img_singal_null.c
CSRCS += img_nb.c img_gps.c img_statistics.c img_no_sim.c img_sleep.c
CSRCS += img_data_activity_in.c img_data_activity_out.c img_data_activity_null.c img_gps_activity.c
CFLAGS += -I$(APPDIR)/graphics/littlevgl/lvgl
endif

CFLAGS += -I $(TOPDIR)/../vendor/services/ril/at_client
PROGNAME = gpstest$(EXEEXT)

include $(APPDIR)/Application.mk
