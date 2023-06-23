//
// data_io_pce.v
//
// data_io for the MiST board (PCE extensions)
// https://github.com/mist-devel/mist-board
//
// Copyright (c) 2014 Till Harbaum <till@harbaum.org>
// Copyright (c) 2015-2017 Sorgelig
// Copyright (c) 2019-2021 György Szombathelyi
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

module data_io_pce
(
	input             clk_sys,

	// Global SPI clock from ARM. 24MHz
	input             SPI_SCK,
	input             SPI_SS2,
	input             SPI_DI,
	output reg        SPI_DO,

	// CD IO
	output reg [15:0] cd_stat,
	output reg        cd_stat_strobe,
	input      [95:0] cd_command,
	input             cd_command_strobe,
	input      [79:0] cd_data,
	input             cd_data_strobe,
	output reg  [7:0] cd_data_out,
	output reg        cd_dm,
	output reg        cd_data_out_strobe,
	output reg        cd_dataout_req,
	input             cd_dat_req,
	input             cd_reset_req,
	input             cd_fifo_halffull
);

///////////////////////////////   DOWNLOADING   ///////////////////////////////

localparam CD_STAT_GET      = 8'h60;
localparam CD_STAT_SEND     = 8'h61;
localparam CD_COMMAND_GET   = 8'h62;
localparam CD_DATA_GET      = 8'h63;
localparam CD_DATA_SEND     = 8'h64;
localparam CD_DATAOUT_REQ   = 8'h65;
localparam CD_ACK           = 8'h66;

// SPI receiver IO -> FPGA

reg       spi_receiver_strobe_r = 0;
reg       spi_transfer_end_r = 1;
reg [7:0] spi_byte_in;
reg [7:0] cmd;
reg [2:0] bit_cnt;
reg [7:0] byte_cnt;

// data_io has its own SPI interface to the io controller
// Read at spi_sck clock domain, assemble bytes for transferring to clk_sys
always@(posedge SPI_SCK or posedge SPI_SS2) begin : data_input

	reg  [6:0] sbuf;

	if(SPI_SS2) begin
		spi_transfer_end_r <= 1;
		bit_cnt <= 0;
		cmd <= 0;
		byte_cnt <= 0;
	end else begin
		spi_transfer_end_r <= 0;
		
		bit_cnt <= bit_cnt + 1'd1;

		if(bit_cnt != 7)
			sbuf[6:0] <= { sbuf[5:0], SPI_DI };

		// finished reading a byte, prepare to transfer to clk_sys
		if(bit_cnt == 7) begin
			if (~&byte_cnt) byte_cnt <= byte_cnt + 1'd1;
			if (cmd == 0) cmd <= { sbuf, SPI_DI};
			spi_byte_in <= { sbuf, SPI_DI};
			spi_receiver_strobe_r <= ~spi_receiver_strobe_r;
		end
	end
end

reg cd_command_pending;
reg cd_data_pending;
reg cd_dat_req_pending;
reg cd_reset_pending;

wire [7:0] cd_status = {3'd0, cd_fifo_halffull, cd_reset_pending, cd_dat_req_pending, cd_data_pending, cd_command_pending};

// SPI transmitter FPGA -> IO
always@(negedge SPI_SCK or posedge SPI_SS2) begin : data_output

	if(SPI_SS2) begin
		SPI_DO <= 1'bZ;
	end else begin
		if(cmd == CD_COMMAND_GET)
			SPI_DO <= cd_command[{ byte_cnt - 1'd1, ~bit_cnt }];
		else if (cmd == CD_DATA_GET)
			SPI_DO <= cd_data[{ byte_cnt - 1'd1, ~bit_cnt }];
		else
			SPI_DO <= cd_status[~bit_cnt];
	end
end

always @(posedge clk_sys) begin

	reg        spi_receiver_strobe;
	reg        spi_transfer_end;
	reg        spi_receiver_strobeD;
	reg        spi_transfer_endD;
	reg  [7:0] acmd;
	reg  [2:0] abyte_cnt;   // counts bytes

	if (cd_command_strobe) cd_command_pending <= 1;
	if (cd_data_strobe) cd_data_pending <= 1;
	if (cd_dat_req) cd_dat_req_pending <= 1;
	if (cd_reset_req) cd_reset_pending <= 1;
	cd_stat_strobe <= 0;
	cd_data_out_strobe <= 0;
	cd_dataout_req <= 0;

	//synchronize between SPI and sys clock domains
	spi_receiver_strobeD <= spi_receiver_strobe_r;
	spi_receiver_strobe <= spi_receiver_strobeD;
	spi_transfer_endD <= spi_transfer_end_r;
	spi_transfer_end <= spi_transfer_endD;

	if (spi_transfer_end) begin
		abyte_cnt <= 3'd0;
	end else if (spi_receiver_strobeD ^ spi_receiver_strobe) begin
		if(~&abyte_cnt) abyte_cnt <= abyte_cnt + 1'd1;

		if(abyte_cnt == 0) begin
			acmd <= spi_byte_in;
		end else begin
			case (acmd)
				CD_STAT_SEND:
					if (abyte_cnt == 1) cd_stat[7:0] <= spi_byte_in; else
					if (abyte_cnt == 2) begin
						cd_stat[15:8] <= spi_byte_in;
						cd_stat_strobe <= 1;
					end

				CD_COMMAND_GET: cd_command_pending <= 0;

				CD_DATA_GET: cd_data_pending <= 0;

				CD_DATA_SEND:
					if (abyte_cnt == 0 || abyte_cnt == 1)
						// skip command + header 1st byte
						cd_dat_req_pending <= 0;
					else if (abyte_cnt == 2) 
						cd_dm = spi_byte_in[7];
					else begin
						cd_data_out <= spi_byte_in;
						cd_data_out_strobe <= 1;
					end

				CD_DATAOUT_REQ: cd_dataout_req <= 1;

				CD_ACK: cd_reset_pending <= 0;

			endcase
		end
	end

end

endmodule
