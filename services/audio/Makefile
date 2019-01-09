-include $(TOPDIR)/Make.defs

CSRCS := topology.c audio_track.c

ifeq ($(CONFIG_AUDIO_DAEMON),y)
APPNAME = aservice
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = 4096

MAINSRC := aservice.c

PROGNAME = aservice$(EXEEXT)

MODULE = CONFIG_AUDIO_SERVICE
else
CSRCS += aservice.c
endif

include $(APPDIR)/Application.mk
