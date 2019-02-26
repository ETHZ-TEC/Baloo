CONTIKI_PROJECT = baloo-get-connectivity
CONTIKI = ../..
TESTDURATION ?= 480

# Configure payload size
TEST_PAYLOAD_SIZE = 8
CFLAGS += -DTEST_PAYLOAD_SIZE=$(TEST_PAYLOAD_SIZE)

# Configure RF channel
GMW_CONF_RF_TX_CHANNEL=26
CFLAGS += -DGMW_CONF_RF_TX_CHANNEL=$(GMW_CONF_RF_TX_CHANNEL)

# Set test description for flocklab
DESCRIPTION ?= $(TEST_PAYLOAD_SIZE)B - ch$(GMW_CONF_RF_TX_CHANNEL)

#CFLAGS += -DFLOCKLAB
CFLAGS += -DGRAZ

# for convenience only
ifeq ($(TARGET), dpp)
  override TARGET = dpp-cc430
endif

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
MODULES += os/net/mac/gmw
PROJECT_SOURCEFILES += gmw-platform.c rtimer-ext.c glossy.c strobing.c
CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_) -DGMW_PLATFORM_CONF_PATH=\"gmw-conf-$(TARGET).h\"

all: $(CONTIKI_PROJECT)
	$(info compiled for target platform $(TARGET) $(BOARD))
	@msp430-objcopy $(CONTIKI_PROJECT).$(TARGET) -O ihex $(CONTIKI_PROJECT)-$(TARGET).ihex
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)
	
upload: $(CONTIKI_PROJECT).upload

include ../../tools/flocklab/Makefile.flocklab
include $(CONTIKI)/Makefile.include
