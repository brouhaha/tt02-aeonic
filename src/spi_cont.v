// SPI controller
// Copyright 2022 Eric Smith <spacewar@gmail.com>
// SPDX-License-Identifier: CERN-OHL-W-2.0

`default_nettype none

module spi_cont
(
   input 	clk,      // everything is synchronous to rising edge
   input 	rst,      // 1 to reset
   input 	start,    // pulse high to start a transfer
   input        hold_n_ss,  // 1 to leave select asserted at end of tranfers
   input [7:0] 	tx_byte,  // data to send
   output 	ready,    // 1 when ready to start transfer (or transfer done)
   output [7:0] rx_byte,  // data received (registered)

   output 	n_ss,     // select to SPI device, active low
   output 	sclk,     // clock to SPI device
   output 	mosi,     // data to SPI device
   input 	miso      // data from SPI device
);

   logic [7:0] 	shift_reg;
   logic [3:0] 	counter;
   
   logic        busy;
   logic [7:0] 	rx_byte_int;
   logic        spi_n_ss;
   logic 	spi_sclk;
   logic        spi_mosi;

   assign ready = ! busy;
   assign rx_byte = rx_byte_int;
   assign n_ss = spi_n_ss;
   assign sclk = spi_sclk;
   assign mosi = spi_mosi;
   

   initial
     begin
	busy = 0;
     end

   always @(posedge clk)
     begin
	if (rst)
	  begin
	     spi_n_ss <= 1;
	     spi_sclk <= 0;
	     spi_mosi <= 0;
	     busy <= 0;
	     counter <= 4'b0000;
	  end
	else if (busy)
	  begin
	     if (counter != 0)
	       begin
		  if (spi_sclk == 0)
		    begin
		       spi_sclk <= 1;  // generate rising edge
		       shift_reg[7:1] <= shift_reg[6:0];
		       shift_reg[0] <= miso;
		    end
		  else
		    begin
		       spi_sclk <= 0;  // generate falling edge
		       spi_mosi <= shift_reg[7];
		       counter <= counter - 1;
		    end // else: !if(spi_clk == 0)
	       end // if (counter != 0)
	     else
	       begin
		  rx_byte_int <= shift_reg;
		  if (! hold_n_ss)
		    spi_n_ss <= 1;
		  busy <= 0;
	       end
	  end
	else if (start)
	  begin
	     shift_reg <= tx_byte;
	     spi_n_ss <= 0;
	     spi_sclk <= 0;
	     spi_mosi <= tx_byte[7];
	     counter <= 4'b1000;
	     busy <= 1;
	  end
     end // always @ (posedge clk)

endmodule
