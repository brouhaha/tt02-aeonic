// SPI FRAM controller
// Copyright 2022 Eric Smith <spacewar@gmail.com>
// SPDX-License-Identifier: CERN-OHL-W-2.0

`default_nettype none

module spi_fram_cont
  #(
    parameter DEBUG = 0
  )
(
   input 	 clk,        // everything is synchronous to rising edge
   input 	 rst,        // 1 to reset
   input 	 start,      // pulse high to start a transfer
   input 	 write,      // 1 for write, 0 for read
   input 	 word,       // 1 for 16-bit xfer, 0 for 8-bit xfer
   output 	 ready,      // 1 when ready to start transfer (or transfer done)
                             // 0 while busy,
   input [23:0]  addr,       // RAM address to start transfer
   input [15:0]  write_data, // data to write
   output [15:0] read_data,  // data that has been read (registered)

   output 	 n_ss,       // select to SPI FRAM, active low
   output 	 sclk,       // clock to SPI FRAM
   output 	 mosi,       // data to SPI FRAM
   input 	 miso        // data from SPI FRAM
);
   localparam [2:0]
     major_state_idle      = 3'b000,
     major_state_cmd_wren  = 3'b001,
     major_state_cmd_rdwr  = 3'b010,
     major_state_addr_high = 3'b011,
     major_state_addr_mid  = 3'b100,
     major_state_addr_low  = 3'b101,
     major_state_byte0     = 3'b110,
     major_state_byte1     = 3'b111;

   localparam [1:0]
     minor_state_enter  = 2'b00,
     minor_state_start  = 2'b01,
     minor_state_run    = 2'b10,
     minor_state_exit   = 2'b11;

   reg          r_write;
   reg          r_word;
   reg [23:0] 	r_addr;
   reg [15:0] 	r_write_data;

   reg [2:0] 	major_state;
   reg [2:0] 	minor_state;

   wire		byte_start;
   reg		hold_n_ss;
   reg [7:0] 	tx_byte;

   wire 	byte_ready;
   wire [7:0] 	rx_byte;

   reg [15:0]   read_data_reg;

   assign byte_start = (major_state != major_state_idle) && (minor_state == minor_state_start);

   assign ready = (major_state == major_state_idle) && (minor_state == minor_state_run);

   assign read_data = read_data_reg;

   initial
     begin
	major_state <= major_state_idle;
	minor_state <= minor_state_run;
     end

   always @(posedge clk)
     begin
	if (rst)
	  begin
	     major_state <= major_state_idle;
	     minor_state <= minor_state_enter;
	  end
	else
	  if (DEBUG >= 2) $display("fram_cont: major state %d, minor state %d, byte_ready %d", major_state, minor_state, byte_ready);	
	  case (minor_state)
	    minor_state_enter:
	      begin
		 case (major_state)
		   major_state_idle:
		     begin
			tx_byte = 8'h00;
			hold_n_ss = 0;
		     end
	  	   major_state_cmd_wren:
		     begin
			tx_byte = 8'h06;
			hold_n_ss = 0;
		     end
		   major_state_cmd_rdwr:
		     begin
			if (r_write)
			  tx_byte = 8'h02;
			else
			  tx_byte = 8'h03;
			hold_n_ss = 1;
		     end
		   major_state_addr_high:
		     begin
			tx_byte = r_addr[23:16];
			hold_n_ss = 1;
		     end
		   major_state_addr_mid:
		     begin
			tx_byte = r_addr[15:8];
			hold_n_ss = 1;
		     end
		   major_state_addr_low:
		     begin
			tx_byte = r_addr[7:0];
			hold_n_ss = 1;
		     end
		   major_state_byte0:
		     begin
			if (r_write)
			  tx_byte = r_write_data[7:0];
			else
			  tx_byte = 8'b00000000;
			hold_n_ss = r_word;
		     end
		   major_state_byte1:
		     begin
			if (r_write)
			  tx_byte = r_write_data[15:8];
			else
			  tx_byte = 8'b00000000;
			hold_n_ss = 0;
		     end
		 endcase
		 // can't issue start yet because the tx_data mux
		 // has to have setup time before the start
		 if (! rst)
		   minor_state <= minor_state_start;
	      end
	    minor_state_start:
	      begin
		 minor_state <= minor_state_run;
	      end
	    minor_state_run:
	      begin
		 if ((major_state == major_state_idle) && start)
		   begin
		      if (write)
			if (word)
			  begin
			     if (DEBUG >= 1) $display("fram cont: start write word addr=%06x data=%04x", addr, write_data);
			  end
			else
			  begin
			     if (DEBUG >= 1) $display("fram cont: start write byte addr=%06x data=%02x", addr, write_data & 8'hff);
			  end
		      else
			if (word)
			  begin
			     if (DEBUG >= 1) $display("fram cont: start read word addr=%06x", addr);
			  end
			else
			  begin
			     if (DEBUG >= 1) $display("fram cont: start read byte addr=%06x", addr);
			  end
		      r_write <= write;
		      r_word <= word;
		      r_addr <= addr;
		      r_write_data <= write_data;
		      minor_state <= minor_state_exit;
		   end
		 if ((major_state != major_state_idle) && byte_ready)
		   begin
		     if (DEBUG >= 2) $display("fram cont: rx byte: %02x", rx_byte);
		     minor_state <= minor_state_exit;
		   end
	      end
	    minor_state_exit:
	      begin
		 case (major_state)
		   major_state_idle:
		     begin
			if (r_write)
			   major_state <= major_state_cmd_wren;
			 else
			   major_state <= major_state_cmd_rdwr;
		     end
                   major_state_cmd_wren, major_state_cmd_rdwr, major_state_addr_high, major_state_addr_mid, major_state_addr_low:
		     begin
			// a command or address byte has been sent
			major_state <= major_state + 1;
		     end
		   major_state_byte0:
		     begin
			// a data byte has been sent or received
			if (! r_write)
			  begin
			     if (DEBUG >= 1) $display("fram_cont: rx byte 0: %02x", rx_byte);
			     read_data_reg[7:0] = rx_byte;
			  end
			if (r_word)
			  major_state <= major_state_byte1;
			else
			  major_state <= major_state_idle;
		     end
		   major_state_byte1:
		     begin
			// a data byte has been sent or received
			if (! r_write)
			  begin
			     if (DEBUG >= 1) $display("fram_cont: rx byte 1: %02x", rx_byte);
			     read_data_reg[15:8] = rx_byte;
			  end
			major_state <= major_state_idle;
		     end
		 endcase
		 minor_state <= minor_state_enter;
	      end
	  endcase // case (minor_state)
     end

   spi_cont spi_cont0(.clk(clk),
		      .rst(rst),
		      .start(byte_start),
		      .hold_n_ss(hold_n_ss),
		      .tx_byte(tx_byte),
		      .ready(byte_ready),
		      .rx_byte(rx_byte),

		      .n_ss(n_ss),
		      .sclk(sclk),
		      .mosi(mosi),
		      .miso(miso));

endmodule

