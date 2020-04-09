CONTIKI_PROJECT = baloo-ttnet
CONTIKI = ../..


TARGET ?= dpp
#TARGET ?= sky

# for convenience only
ifeq ($(TARGET), dpp)
  override TARGET = dpp-cc430
endif

# Mark as a Baloo project
CFLAGS += -DBALOO

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
MODULES += os/net/mac/gmw
PROJECT_SOURCEFILES += gmw-platform.c rtimer-ext.c glossy.c
CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_) -DGMW_PLATFORM_CONF_PATH=\"gmw-conf-$(TARGET).h\"

LDFLAGS += -Wl,--section-start -Wl,.testbedConfigSection=0xd400

all: $(CONTIKI_PROJECT)
	$(info compiled for target platform $(TARGET) $(BOARD))
	@msp430-objcopy $(CONTIKI_PROJECT).$(TARGET) -O ihex $(CONTIKI_PROJECT)-$(TARGET).ihex
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)

sections: $(CONTIKI_PROJECT)
	@msp430-objcopy -I ihex --output-target=elf32-msp430 $(CONTIKI_PROJECT)-$(TARGET).ihex $(CONTIKI_PROJECT)-$(TARGET).elf
	@msp430-readelf -a $(CONTIKI_PROJECT)-$(TARGET).elf


upload: $(CONTIKI_PROJECT).upload

include ../../tools/flocklab/Makefile.flocklab
include $(CONTIKI)/Makefile.include
