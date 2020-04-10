CONTIKI_PROJECT = hello-world
MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
CONTIKI = ../..

TARGET ?= sky
# for convenience only
ifeq ($(TARGET), dpp)
  override TARGET = dpp-cc430
endif

CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_)

all: $(CONTIKI_PROJECT)
	$(info compiled for target platform $(TARGET) $(BOARD))
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)

upload: $(CONTIKI_PROJECT).upload

ifeq (TARGET, dpp-cc430)
login: $(CONTIKI_PROJECT).login
endif

reset: $(CONTIKI_PROJECT).reset

include ../../tools/flocklab/Makefile.flocklab
include $(CONTIKI)/Makefile.include
