CONTIKI_PROJECT = group-project

ifdef COOJA
TARGET ?= sky
CFLAGS += -DCOOJA
else
TARGET ?= dpp-cc430
endif


# select the data rate
CFLAGS += -DDATARATE=1
# select the sink address
CFLAGS += -DSINK_ADDRESS=22
# select the sink address
CFLAGS += -DRANDOM_SEED=123

PROJECT_SOURCEFILES += data-generator.c

# --- Here come my additions --- 
# Mark as a Baloo project
CFLAGS += -DBALOO -DGMW_PLATFORM_CONF_PATH=\"gmw-conf-$(TARGET).h\"
MODULES += os/net/mac/gmw
PROJECT_SOURCEFILES += gmw-platform.c rtimer-ext.c glossy.c
# ------------------------------ 

CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_)

all: $(CONTIKI_PROJECT)
	$(info compiled for target platform $(TARGET) $(BOARD))
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)

upload: $(CONTIKI_PROJECT).upload

reset: $(CONTIKI_PROJECT).reset

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
CONTIKI = ../..
include ../../tools/flocklab/Makefile.flocklab
include $(CONTIKI)/Makefile.include


XMLTEMPLATE = flocklab-$(TARGET)-LPSD.xml

flocklab_test_lpsd:
	@base64 $(CONTIKI_PROJECT).$(TARGET) > $(CONTIKI_PROJECT).$(TARGET).b64
	@sed -n '1h;1!H;$${ g;s/<data>[^<]*<\/data>/<data_new>\n<\/data>/;p}' $(FLOCKLABTOOLS)$(XMLTEMPLATE) > $(XMLFILE)
	@sed -i '/<data_new>/r $(CONTIKI_PROJECT).$(TARGET).b64' $(XMLFILE)
	@sed -i 's/<data_new>/<data>/' $(XMLFILE)
	@sed -i 's/<name>Name<\/name>/<name>$(CONTIKI_PROJECT)<\/name>/' $(XMLFILE)
	@sed -i 's/<description>Description<\/description>/<description>$(DESCRIPTION) \($(TARGET), $(GITSHA)\)<\/description>/' $(XMLFILE)
	@sed -i 's/<durationSecs>[^<]*<\/durationSecs>/<durationSecs>$(TESTDURATION)<\/durationSecs>/' $(XMLFILE)
	@rm $(CONTIKI_PROJECT).$(TARGET).b64
	$(FLOCKLABTOOLS)flocklab -c $(XMLFILE)
	
flocklab_test: flocklab_xml
	$(FLOCKLABTOOLS)flocklab -c $(XMLFILE)