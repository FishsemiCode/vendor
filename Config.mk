############################################################################
# external/Config.mk
# Common make definitions provided to external applications
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

EFLAGS =
EXXFLAGS =

ifneq ($(CONFIG_FRAMEWORKS),)
    EFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/frameworks/include}
endif

ifeq ($(CONFIG_BT_CONTROLLER),y)
    EFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/bt/controller/plf/song/src/arch/main}
endif

ifneq ($(CONFIG_BT_HOST),)
    EFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/bt/host/src/classic}
    EFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/bt/host/src/ble}
    EFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/bt/host/src}
    EFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/bt/host/platform/song}
    EFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/bt/host/port/song}
endif

ifneq ($(CONFIG_CMSIS),)
    EFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/cmsis/CMSIS/Core/Include}
    EFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/cmsis/CMSIS/DSP/Include}

  ifeq ($(CONFIG_ARCH_ARM_SONG),y)
    EFLAGS += ${shell $(TOPDIR)/tools/define.sh "$(CC)" __NVIC_PRIO_BITS=3}
  endif

  ifeq ($(CONFIG_ARCH_CORTEXM4),y)
    EFLAGS += ${shell $(TOPDIR)/tools/define.sh "$(CC)" ARM_MATH_CM4=1}
  endif

  ifeq ($(CONFIG_ARCH_FPU),y)
    EFLAGS += ${shell $(TOPDIR)/tools/define.sh "$(CC)" __FPU_PRESENT=1}
  endif

  ifeq ($(CONFIG_ARM_MPU),y)
    EFLAGS += ${shell $(TOPDIR)/tools/define.sh "$(CC)" __MPU_PRESENT=1}
  endif
endif

ifneq ($(CONFIG_NETUTILS_LIBCOAP),)
    EFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/libcoap/include}
endif

ifneq ($(CONFIG_CODEC_HELIXAAC),)
    EFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/codec/libhelix-aac}
endif

ifneq ($(CONFIG_CODEC_FAAD2),)
    EFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/codec/faad2/include}
endif

ifneq ($(CONFIG_CODEC_FDKAAC),)
    EXXFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/codec/fdk-aac/libAACdec/include}
    EXXFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/codec/fdk-aac/libFDK/include}
    EXXFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/codec/fdk-aac/libSYS/include}
    EXXFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/codec/fdk-aac/libMpegTPDec/include}
    EXXFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/codec/fdk-aac/libSBRdec/include}
    EXXFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/codec/fdk-aac/libPCMutils/include}
endif

ifneq ($(CONFIG_CODEC_BLUEDROID),)
    EFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(APPDIR)/external/system/codec/bluedroid/decoder/include}
endif

CFLAGS   += $(EFLAGS)
CXXFLAGS += $(EFLAGS) $(EXXFLAGS)
