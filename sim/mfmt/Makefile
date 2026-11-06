# Verilator test of the MFMT drive and MFM decoder
#
#   make MAIN=/path/to/Main_MiSTer test STX="a.stx b.stx"
#
# MAIN is a Main_MiSTer checkout with support/st (STX converter and test model)

MAIN     ?= ../../../main_mister_physical_floppy
ST        = $(MAIN)/support/st
RTL       = ../../rtl/fdc1772
ACC_STEP ?= 4000
OBJ       = obj_$(ACC_STEP)

VFLAGS = --cc --exe --build -O2 -Wno-fatal -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND \
         -GACC_STEP=$(ACC_STEP) --top-module tb_top -Mdir $(OBJ) \
         -CFLAGS "-O2 -std=gnu++17 -I$(abspath $(ST)) -I$(abspath $(ST))/test -DACC_STEP=$(ACC_STEP)"

SRC = tb_top.sv $(RTL)/mfmt_track.sv $(RTL)/mfm_dec.sv tb.cpp \
      $(abspath $(ST))/st_stx.cpp $(abspath $(ST))/st_trackbuild.cpp $(abspath $(ST))/test/wdsim.cpp

all: $(OBJ)/Vtb_top

$(OBJ)/Vtb_top: $(SRC) tb_top.sv
	verilator $(VFLAGS) $(SRC) -o Vtb_top

test: all
	$(OBJ)/Vtb_top -s $(STX)

clean:
	rm -rf obj_*
