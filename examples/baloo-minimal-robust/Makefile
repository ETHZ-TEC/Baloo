CONTIKI_PROJECT = baloo-minimal-robust
CONTIKI = ../..
DESCRIPTION ?= Baloo minimal test with robust Glossy

TARGET ?= sky

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
MODULES += os/net/mac/gmw
PROJECT_SOURCEFILES += gmw-platform.c rtimer-ext.c robust-glossy.c
#PROJECT_CONF_PATH = project-conf.h
CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_) -DGMW_PLATFORM_CONF_PATH=\"gmw-conf-$(TARGET).h\"
CLEAN += $(CONTIKI_PROJECT)-$(TARGET).hex

all: $(CONTIKI_PROJECT)
	@msp430-objcopy $(CONTIKI_PROJECT).$(TARGET) -O ihex $(CONTIKI_PROJECT)-$(TARGET).hex
	$(info compiled for target platform $(TARGET) $(BOARD))
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)

upload: $(CONTIKI_PROJECT).upload

include ../../tools/flocklab/Makefile.flocklab
include $(CONTIKI)/Makefile.include
