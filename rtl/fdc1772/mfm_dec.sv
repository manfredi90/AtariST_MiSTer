//
// mfm_dec.sv - WD1772 style MFM byte assembly from a bit-cell stream
//
// Copyright (c) 2026 Manfredi
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// The drive (mfmt_track) delivers cells already separated, so there is no
// PLL: data bits are the odd cells of each 16-cell byte.
//
// mode SEARCH: looking for an address mark. Every A1 sync (0x4489) restarts
//              the byte boundary and is output at once with sync = 1.
// mode FIELD:  inside an ID or data field. Bytes every 16 cells, no resync.
// mode TRACK:  read track. The boundary follows every A1 (0x4489) and C2
//              (0x5224) pattern, false ones too; the byte in progress is
//              output only if more than 8 cells went by since the previous
//              boundary (behaviour of the real chip, see MAME wd_fdc.cpp).

module mfm_dec
(
	input            clk,
	input            restart,    // forget the current byte boundary (a cell on the
	                             // same clock starts the new byte)
	input      [1:0] mode,
	input            cell_stb,
	input            cell_bit,

	output reg       byte_stb,
	output reg [7:0] byte_val,
	output reg       byte_sync,  // the 16 cells were an A1 sync mark
	output reg       byte_c2     // the 16 cells were a C2 sync mark
);

localparam MODE_SEARCH = 2'd0;
localparam MODE_FIELD  = 2'd1;
localparam MODE_TRACK  = 2'd2;

reg [15:0] shift;
reg  [4:0] count;

wire [15:0] sh = {shift[14:0], cell_bit};
wire        a1 = sh == 16'h4489;
wire        c2 = sh == 16'h5224;
// a restart together with a cell makes that cell the first of a new byte
wire  [4:0] cn = restart ? 5'd1 : count + 1'd1;

function [7:0] data_bits(input [15:0] raw);
	data_bits = {raw[14], raw[12], raw[10], raw[8], raw[6], raw[4], raw[2], raw[0]};
endfunction

always @(posedge clk) begin
	byte_stb <= 0;
	if (restart && !cell_stb) count <= 0;
	else if (cell_stb) begin
		shift <= sh;
		count <= cn;

		case (mode)
		MODE_SEARCH:
			if (a1 || cn == 16) begin
				byte_stb <= 1;
				count <= 0;
			end

		MODE_TRACK:
			if ((a1 || c2) && cn != 16) begin
				if (cn > 8) byte_stb <= 1;
				count <= 0;
			end
			else if (cn == 16) begin
				byte_stb <= 1;
				count <= 0;
			end

		default: // MODE_FIELD
			if (cn == 16) begin
				byte_stb <= 1;
				count <= 0;
			end
		endcase

		byte_val <= data_bits(sh);
		byte_sync <= a1;
		byte_c2 <= c2;
	end
end

endmodule
