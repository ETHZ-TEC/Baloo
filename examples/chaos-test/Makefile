CONTIKI_PROJECT = chaos-test
CONTIKI = ../..
DESCRIPTION ?= Chaos test

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
PROJECT_SOURCEFILES += rtimer-ext.c chaos.c
#CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_)

all: $(CONTIKI_PROJECT)
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)

upload: $(CONTIKI_PROJECT).upload

#include ../../tools/flocklab/Makefile.flocklab
include $(CONTIKI)/Makefile.include
