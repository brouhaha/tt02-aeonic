`default_nettype none
`timescale 1ns/1ps

/*
this testbench just instantiates the module and makes some convenient wires
that can be driven / tested by the cocotb test.py
*/

module tb (
    // testbench is controlled by test.py
    input	 clk,
    input	 rst,
    input [4:0]	 gp_in,
    output [3:0] gp_out,
    output	 gp_out_stb
   );

   // this part dumps the trace to a vcd file that can be viewed with GTKWave
   initial begin
      $dumpfile ("tb.vcd");
      $dumpvars (0, tb);
      #1;
   end

   // wire up the inputs and outputs
   wire [7:0] inputs = { gp_in[4:0], spi_miso, rst, clk};
   wire [7:0] outputs;

   wire	      spi_nss;
   wire	      spi_clk;
   wire	      spi_mosi;
   wire	      spi_miso;

   assign gp_out_stb = outputs[7];
   assign gp_out[3:0] = outputs[6:3];
   assign spi_mosi = outputs[2];
   assign spi_clk = outputs[1];
   assign spi_nss = outputs[0];

   // instantiate the DUT
   aeonic_tt aeonic_tt0(.io_in(inputs),
			.io_out(outputs)
			);
   
   // instantiate the FRAM functional model
   spi_fram #(.FILENAME("utest4.memh")) spi_fram0 (.n_ss(spi_nss),
						   .sclk(spi_clk),
						   .mosi(spi_mosi),
						   .miso(spi_miso));

endmodule
