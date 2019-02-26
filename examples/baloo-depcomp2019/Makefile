CONTIKI_PROJECT = baloo-depcomp2019
CONTIKI = ../..
TESTDURATION ?= 480

TARGET=sky

#CFLAGS += -DFLOCKLAB
CFLAGS += -DGRAZ

# Set flag for competition-specific includes, regardless of the test environment
CFLAGS += -DDEPCOMP

# Mark as a Baloo project
CFLAGS += -DBALOO

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
MODULES += os/net/mac/gmw
PROJECT_SOURCEFILES += gmw-platform.c rtimer-ext.c glossy.c my2c.c
CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_) -DGMW_PLATFORM_CONF_PATH=\"gmw-conf-$(TARGET).h\"

LDFLAGS += -Wl,--section-start -Wl,.testbedConfigSection=0xd400

all: $(CONTIKI_PROJECT)
	$(info compiled for target platform $(TARGET) $(BOARD))
	@msp430-objcopy $(CONTIKI_PROJECT).$(TARGET) -O ihex $(CONTIKI_PROJECT)-$(TARGET).ihex
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)
	#@msp430-objcopy -I ihex --output-target=elf32-msp430 $(CONTIKI_PROJECT)-$(TARGET).ihex $(CONTIKI_PROJECT).elf
	
upload: $(CONTIKI_PROJECT).upload

include ../../tools/flocklab/Makefile.flocklab
include $(CONTIKI)/Makefile.include
