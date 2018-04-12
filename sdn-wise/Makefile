CONTIKI_PROJECT = sdn-wise
all: $(CONTIKI_PROJECT)

CONTIKI = ../contiki
CONTIKI_WITH_RIME = 1

# DEFINES=NETSTACK_CONF_RDC=contikimac_driver,NETSTACK_CONF_MAC=csma_driver
DEFINES=NETSTACK_CONF_RDC=nullrdc_driver,NETSTACK_CONF_MAC=csma_driver

PROJECT_SOURCEFILES = flowtable.c packet-buffer.c address.c neighbor-table.c packet-handler.c node-conf.c packet-creator.c
TARGETDIRS += ../targets
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

include $(CONTIKI)/Makefile.include
