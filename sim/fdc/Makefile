# Verilator test of the whole fdc1772 with MFMT (STX) images
#
#   make MAIN=/path/to/Main_MiSTer test STX="a.stx b.stx"
#   make CLKCPU_KHZ=4000 ...   disk 8x faster than real, for many tracks

MAIN       ?= ../../../main_mister_physical_floppy
ST          = $(abspath $(MAIN)/support/st)
RTL         = ../../rtl/fdc1772
CLKCPU_KHZ ?= 32000
OBJ         = obj_$(CLKCPU_KHZ)

VFLAGS = --cc --exe --build -O2 --public-flat-rw -Wno-fatal -Wno-WIDTH -Wno-CASEINCOMPLETE -Wno-UNUSED \
         -GIMG_TYPE=1 -GCLKCPU_KHZ=$(CLKCPU_KHZ) -GDEBUG_EVENTS=1 --top-module fdc1772 -Mdir $(OBJ) \
         -CFLAGS "-O2 -std=gnu++17 -I$(ST) -I$(ST)/test -DTB_CLKCPU_KHZ=$(CLKCPU_KHZ)"

SRC = $(RTL)/fdc1772.sv $(RTL)/floppy.v $(RTL)/mfmt_track.sv $(RTL)/scp_track.sv $(RTL)/mfm_dec.sv tb_fdc.cpp \
      $(ST)/st_stx.cpp $(ST)/st_trackbuild.cpp $(ST)/test/wdsim.cpp

all: $(OBJ)/Vfdc1772

$(OBJ)/Vfdc1772: $(SRC)
	verilator $(VFLAGS) $(SRC) -o Vfdc1772

test: all
	$(OBJ)/Vfdc1772 $(STX)

clean:
	rm -rf obj_*
