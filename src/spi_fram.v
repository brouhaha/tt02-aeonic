// SPI FRAM simulation model, minimal subset of FM25V20A
// Copyright 2022 Eric Smith <spacewar@gmail.com>
// SPDX-License-Identifier: CERN-OHL-W-2.0

`default_nettype none

module spi_fram
  #( parameter L2_SIZE_BYTES = 16,
     parameter DEBUG = 0,
     parameter LOAD = 1,
     parameter FILENAME = "fram.memh"
  )
(
   input  n_ss,  // select
   input  sclk,  // clock
   input  mosi,  // data in
   output miso   // data out
);

   localparam [7:0]
     cmd_write    = 8'h02,
     cmd_read     = 8'h03,
     cmd_write_en = 8'h06;

   localparam [2:0]
     state_idle      = 3'b000,
     state_cmd       = 3'b001,
     state_addr_high = 3'b010,
     state_addr_med  = 3'b011,
     state_addr_low  = 3'b100,
     state_data      = 3'b101;

   reg [2:0]  state;

   reg [2:0]  bit_cnt;
   
   reg [7:0]  sr_in;
   reg [8:0]  sr_out;

   reg [7:0]  cmd;
   reg [23:0] addr;
   reg [23:0] addr_mask = (1 << L2_SIZE_BYTES) - 1;
   
   reg 	      write_enable;

   reg [7:0]  memory[0:((1 << L2_SIZE_BYTES) - 1)];

   assign miso = sr_out[8];

   initial
     begin
	if (LOAD)
	  begin
	     if (DEBUG >= 1) $display("fram: loading from file");
	     $readmemh(FILENAME, memory);
	  end
	write_enable = 0;
     end

   always @(negedge n_ss)
     begin
	state <= state_cmd;
	bit_cnt <= 0;
	sr_in  <= 8'h00;
	sr_out <= 8'h00;
     end

   always @(posedge n_ss)
     begin
	if (bit_cnt != 0)
	  begin
	     if (DEBUG >= 1) $display("fram: %d dribble bits\n", bit_cnt);
	  end
	if (cmd == cmd_write)
	  write_enable <= 0;
     end
   
   always @(posedge sclk)
     begin
	if (DEBUG >= 2) $display("fram: shifted in bit %b, sr_in will be %08b, bit count will be %d", mosi, { sr_in[6:0], mosi }, bit_cnt + 1);
	sr_in <= { sr_in[6:0], mosi };
	if (n_ss == 0)
	  begin
	     if (bit_cnt == 7)
	       case (state)
		 state_idle:
		   begin
		      // used when CS inactive, and to absorb unrecognized commands
		   end
		 state_cmd:
		   begin
		      cmd <= { sr_in[6:0], mosi };
		      case ({ sr_in[7:0], mosi })
			cmd_write_en:
			  begin
			     if (DEBUG >= 1) $display("fram: cmd write enable");
			     write_enable = 1;
			     state <= state_idle;  // ignore any additional bytes
			  end
			cmd_write:
			  begin
			     if (DEBUG >= 1) $display("fram: cmd write");
			     if (write_enable)
			       state <= state_addr_high;
			     else
			       begin
				  if (DEBUG >= 1) $display("fram: write without write enable");
				  state <= state_idle;
			       end
			  end
			cmd_read:
			  begin
			     if (DEBUG >= 1) $display("fram: cmd read");
			     state <= state_addr_high;
			  end
			default:
			  begin
			     if (DEBUG >= 1) $display("fram: unrecognized command %08b", { sr_in[6:0], mosi});
			     state <= state_idle;
			  end
		      endcase
		   end
		 state_addr_high:
		   begin
		      addr[23:16] <= { sr_in[6:0], mosi };
		      state <= state_addr_med;
		   end
		 state_addr_med:
		   begin
		      addr[15:8] <= { sr_in[6:0], mosi };
		      state <= state_addr_low;
		   end
		 state_addr_low:
		   begin
		      addr[7:0] <= { sr_in[6:0], mosi };
		      state <= state_data;
		      if (cmd == cmd_read)
			begin
			   sr_out <= memory[{ addr[23:8], sr_in[6:0], mosi } & addr_mask];
			end
		   end
		 state_data:
		   begin
		      case (cmd)
			cmd_write:
			  begin
			     if (DEBUG >= 1) $display("fram: addr %06x write data %02x", addr & addr_mask, { sr_in[6:0], mosi });
			     memory[addr & addr_mask] = { sr_in[6:0], mosi };
			     addr <= addr + 1;
			  end
			cmd_read:
			  begin
			     if (DEBUG >= 1) $display("fram: addr %06x read data %02x", addr & addr_mask, memory[addr & addr_mask]);
			     addr <= addr + 1;
			     sr_out <= memory[(addr + 1) & addr_mask];
			  end
			default:
			  begin
			     if (DEBUG >= 1) $display("fram: internal error");
			  end
		      endcase
		   end
	       endcase
	     bit_cnt <= bit_cnt + 1;
	  end
     end

   always @(negedge sclk)
     begin
	if (n_ss == 0)
	  sr_out = sr_out << 1;
     end

endmodule

