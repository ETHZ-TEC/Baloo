CONTIKI_PROJECT = baloo-test-static
CONTIKI = ../..

TARGET ?= dpp

# for convenience only
ifeq ($(TARGET), dpp)
  override TARGET = dpp-cc430
endif

# select the number of slots per round
CFLAGS += -DTTW_MAX_SLOTS_PER_ROUND=30
# select the payload size
CFLAGS += -DTTW_MAX_PAYLOAD_LEN=8
# select the random seed
CFLAGS += -DRANDOM_SEED=354

# Mark as a Baloo project
CFLAGS += -DBALOO
# Enable custom configuration variable
CFLAGS += -DCUSTOM

# Add Bolt source
PROJECT_SOURCEFILES += bolt.c

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
MODULES += os/net/mac/gmw
PROJECT_SOURCEFILES += gmw-platform.c rtimer-ext.c glossy.c
#PROJECT_CONF_PATH = project-conf.h
CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_) -DGMW_PLATFORM_CONF_PATH=\"gmw-conf-$(TARGET).h\"

all: $(CONTIKI_PROJECT)
	$(info compiled for target platform $(TARGET) $(BOARD))
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)

upload: $(CONTIKI_PROJECT).upload

include ../../tools/flocklab/Makefile.flocklab
include $(CONTIKI)/Makefile.include
