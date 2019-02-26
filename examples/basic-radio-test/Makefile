CONTIKI_PROJECT = radio-test
CONTIKI = ../..
DESCRIPTION ?= Basic radio test

# for convenience only
ifeq ($(TARGET), dpp)
  override TARGET = dpp-cc430
endif

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
PROJECT_SOURCEFILES += basic-radio.c
CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_)

all: $(CONTIKI_PROJECT)
	$(info compiled for target platform $(TARGET) $(BOARD))
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)

upload: $(CONTIKI_PROJECT).upload

ifeq ($(TARGET), dpp)
reset: $(CONTIKI_PROJECT).reset
endif

include ../../tools/flocklab/Makefile.flocklab
include $(CONTIKI)/Makefile.include
