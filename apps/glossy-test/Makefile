CONTIKI_PROJECT = glossy-test
all: $(CONTIKI_PROJECT)

CONTIKI = ../..
include $(CONTIKI)/Makefile.include

OBJDUMP = msp430-objdump

bytes: 
	@$(OBJDUMP) -h $(CONTIKI_PROJECT).sky | perl -ne '$$b{$$1}=hex $$2 if /^\s*\d+\s*\.(text|data|bss)\s+(\S+)/; END { printf("%16d bytes in ROM\n%16d bytes in RAM\n",$$b{text}+$$b{data},$$b{data}+$$b{bss}); }'

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++ Other Stuff +++
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

make_assembly:
	@$(OBJDUMP) -d $(CONTIKI_PROJECT).sky > $(CONTIKI_PROJECT).dis

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++ LOCAL +++
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

local_nodes: CFLAGS += -DINITIATOR_NODE_ID=1 \
			-DUSE_TINYOS_NODEIDS=0 -DTINYOS_SERIAL_FRAMES=0 \
			-DFLOCKLAB=0 -DCOOJA=0 -DLOCAL_NODES=1 -DBURN_NODE_ID=0 \
			-DGLOSSY_DEBUG=1 -DGLOSSY_DEBUG_PINS=1 -DGLOSSY_CLOCK_DRIFT_COMP=1
local_nodes: $(CONTIKI_PROJECT).upload

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++ FLOCKLAB +++
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

flocklab_debug_pins: CFLAGS += -DINITIATOR_NODE_ID=1 \
				-DUSE_TINYOS_NODEIDS=1 -DTINYOS_SERIAL_FRAMES=0 \
				-DFLOCKLAB=1 -DCOOJA=0 -DLOCAL_NODES=0 -DBURN_NODE_ID=0 \
				-DGLOSSY_DEBUG=0 -DGLOSSY_DEBUG_PINS=1 -DGLOSSY_CLOCK_DRIFT_COMP=0
flocklab_debug_pins: $(CONTIKI_PROJECT) bytes
	cp $(CONTIKI_PROJECT).sky $(CONTIKI_PROJECT).flocklab_debug_pins.sky
	
flocklab_debug_all: CFLAGS += -DINITIATOR_NODE_ID=1 \
				-DUSE_TINYOS_NODEIDS=1 -DTINYOS_SERIAL_FRAMES=0 \
				-DFLOCKLAB=1 -DCOOJA=0 -DLOCAL_NODES=0 -DBURN_NODE_ID=0 \
				-DGLOSSY_DEBUG=1 -DGLOSSY_DEBUG_PINS=1 -DGLOSSY_CLOCK_DRIFT_COMP=0
flocklab_debug_all: $(CONTIKI_PROJECT) bytes
	cp $(CONTIKI_PROJECT).sky $(CONTIKI_PROJECT).flocklab_debug_all.sky
	
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++ COOJA +++
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	
cooja_debug_pins: CFLAGS += -DINITIATOR_NODE_ID=1 \
				-DUSE_TINYOS_NODEIDS=0 -DTINYOS_SERIAL_FRAMES=0 \
				-DFLOCKLAB=0 -DCOOJA=1 -DLOCAL_NODES=0 -DBURN_NODE_ID=0 \
				-DGLOSSY_DEBUG=1 -DGLOSSY_DEBUG_PINS=1 -DGLOSSY_CLOCK_DRIFT_COMP=0
cooja_debug_pins: $(CONTIKI_PROJECT) bytes
	cp $(CONTIKI_PROJECT).sky $(CONTIKI_PROJECT).cooja_debug_pins.sky
	
cooja_debug_all: CFLAGS += -DINITIATOR_NODE_ID=1 \
				-DUSE_TINYOS_NODEIDS=0 -DTINYOS_SERIAL_FRAMES=0 \
				-DFLOCKLAB=0 -DCOOJA=1 -DLOCAL_NODES=0 -DBURN_NODE_ID=0 \
				-DGLOSSY_DEBUG=1 -DGLOSSY_DEBUG_PINS=1 -DGLOSSY_CLOCK_DRIFT_COMP=0
cooja_debug_all: $(CONTIKI_PROJECT) bytes
	cp $(CONTIKI_PROJECT).sky $(CONTIKI_PROJECT).cooja_debug_all.sky