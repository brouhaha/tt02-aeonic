// Aeonic wrapper for TinyTapeout 2

// Copyright 2022 Eric Smith <spacewar@gmail.com>
// SPDX-License-Identifier: CERN-OHL-W-2.0

`default_nettype none

module aeonic_tt
  (input [7:0] io_in,
   output [7:0]	io_out
   );
   
   wire	      clk;
   wire	      rst;

   wire	      spi_nss;
   wire	      spi_clk;
   wire	      spi_mosi;
   wire	      spi_miso;

   wire [7:0] gp_in;
   wire [7:0] gp_out;
   wire	      gp_out_stb;
   

   assign clk = io_in[0];
   assign rst = io_in[1];
   assign spi_miso = io_in[2];

   assign gp_in[4:0] = io_in[7:3];
   assign gp_in[7:5] = 3'b000;

   assign io_out[0] = spi_nss;
   assign io_out[1] = spi_clk;
   assign io_out[2] = spi_mosi;
   assign io_out[6:3] = gp_out[3:0];
   assign io_out[7] = gp_out_stb;

   aeonic aeonic0(.clk(clk),
		  .rst(rst),
		  .gp_in(gp_in),
		  .gp_out(gp_out),
		  .gp_out_stb(gp_out_stb),
		  .n_ss(spi_nss),
		  .sclk(spi_clk),
		  .mosi(spi_mosi),
		  .miso(spi_miso));

endmodule

