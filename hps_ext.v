//
// hps_ext for Atari ST
//
// Copyright (c) 2020 Alexey Melnikov
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
///////////////////////////////////////////////////////////////////////

module hps_ext
(
	input             clk_sys,
	inout      [35:0] EXT_BUS,
	
	output reg        dio_in_strobe,
	output reg [15:0] dio_in,

	output reg        dio_out_strobe,
	input      [15:0] dio_out,

	output reg        dma_ack,
	output reg  [7:0] dma_status,
	output reg        dma_nak,

	input       [7:0] dio_status,
	output      [3:0] dio_status_idx,

	// FDC debug events (stx.log)
	input      [95:0] fdc_dbg_q,
	input             fdc_dbg_valid,
	output reg        fdc_dbg_pop
);

assign EXT_BUS[15:0] = io_dout;
wire [15:0] io_din = EXT_BUS[31:16];
assign EXT_BUS[32] = dout_en;
wire io_strobe = EXT_BUS[33];
wire io_enable = EXT_BUS[34];

localparam EXT_CMD_MIN     = ST_WRITE_MEMORY;
localparam EXT_CMD_MAX     = ST_GET_FDCDBG;

localparam ST_WRITE_MEMORY = 8;
localparam ST_READ_MEMORY  = 9;
localparam ST_ACK_DMA      = 10;
localparam ST_NAK_DMA      = 11;
localparam ST_GET_DMASTATE = 12;
localparam ST_GET_FDCDBG   = 13;

reg [15:0] io_dout;
reg        dout_en = 0;
reg  [9:0] byte_cnt;

assign dio_status_idx = byte_cnt[3:0] - 1'd1;

reg [95:0] dbg;
reg        dbg_v;

always@(posedge clk_sys) begin
	reg [15:0] cmd;

	if(~io_enable) begin
		dout_en <= 0;
		io_dout <= 0;
		byte_cnt <= 0;
	end
	else if(io_strobe) begin

		io_dout <= 0;
		if(~&byte_cnt) byte_cnt <= byte_cnt + 1'd1;

		if(byte_cnt == 0) begin
			cmd <= io_din;
			dout_en <= (io_din >= EXT_CMD_MIN && io_din <= EXT_CMD_MAX);
			if(io_din == ST_NAK_DMA) dma_nak <= ~dma_nak;
			if(io_din == ST_GET_FDCDBG) begin
				dbg <= fdc_dbg_q;
				dbg_v <= fdc_dbg_valid;
				if (fdc_dbg_valid) fdc_dbg_pop <= ~fdc_dbg_pop;
			end
		end else begin
			case(cmd)
				ST_WRITE_MEMORY:
					begin
						dio_in <= {io_din[7:0], io_din[15:8]};
						dio_in_strobe <= ~dio_in_strobe;
					end

				ST_READ_MEMORY:
					begin
						io_dout <= {dio_out[7:0], dio_out[15:8]};
						dio_out_strobe <= ~dio_out_strobe;
					end

				ST_ACK_DMA:
					begin
						dma_ack <= ~dma_ack;
						dma_status <= io_din[7:0];
					end

				ST_GET_DMASTATE:
					begin
						io_dout <= dio_status;
					end

				ST_GET_FDCDBG:
					case(byte_cnt[2:0])
						1: io_dout <= {15'h6D80, dbg_v};   // 0xDB00 | valid
						2: io_dout <= dbg[95:80];
						3: io_dout <= dbg[79:64];
						4: io_dout <= dbg[63:48];
						5: io_dout <= dbg[47:32];
						6: io_dout <= dbg[31:16];
						7: io_dout <= dbg[15:0];
						default: io_dout <= 0;
					endcase
			endcase
		end
	end
end

endmodule
