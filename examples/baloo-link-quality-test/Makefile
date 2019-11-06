CONTIKI_PROJECT = baloo-link-quality-test
CONTIKI = ../..
DESCRIPTION ?= Link quality estimation

ifeq ($(TARGET), dpp)
  override TARGET = dpp-cc430
  # select the RF channel
  CFLAGS += -DRF_CHANNEL=8
endif

ifeq ($(TARGET), sky)
	# select the RF channel
  CFLAGS += -DRF_CHANNEL=26
endif

# select the payload size
CFLAGS += -DPAYLOAD_LEN=8
# select the random seed
CFLAGS += -DRANDOM_SEED=123
# select the host
CFLAGS += -DHOST_ID_MAKEFILE=10

# Mark as a Baloo project
CFLAGS += -DBALOO
# Enable custom configuration variable
CFLAGS += -DCUSTOM

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
MODULES += os/net/mac/gmw
PROJECT_SOURCEFILES += gmw-platform.c rtimer-ext.c glossy.c
PROJECT_SOURCEFILES += strobing.c
#PROJECT_CONF_PATH = project-conf.h
CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_) -DGMW_PLATFORM_CONF_PATH=\"gmw-conf-$(TARGET).h\"

all: $(CONTIKI_PROJECT)
	$(info compiled for target platform $(TARGET) $(BOARD))
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)

upload: $(CONTIKI_PROJECT).upload

include ../../tools/flocklab/Makefile.flocklab
include $(CONTIKI)/Makefile.include
