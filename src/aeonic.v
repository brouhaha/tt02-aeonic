// Aeonic microengine - mostly compatible with Glacial microcode,
// but uses SPI FRAM memory

// Copyright 2022 Eric Smith <spacewar@gmail.com>
// SPDX-License-Identifier: CERN-OHL-W-2.0

// Aeonic differences from Glacial:

// * Uses SPI FRAM memory instead of byte parallel memroy

// * Instructions take a variable number of cycles, due to the SPI memory
//  interface, vs. fixed four cycles per instruction in Glacial

// * UART output is not implemented due to variable instruction timing
//   (bit 8 of operate instructions has no effect)
//   use of gp_out port is suggested as an alternative

// * Interrupt and clock tick support removed
//   (bit 9 of operat instructions has no effect)

// * 8-bit GPIO input port gp_in provided
//   operate instruction with bit 10 set reads gp_into to AC
//   (For TinyTapout 2, gp_in[7:5] are unavailable, and read as zero.)

// * 8-bit GPIO output port gp_out provided
//   operate instruction with bit 11 set writes AC to gp_out
//   (For TinyTapout 2, gp_out[7:5] are unavailable, and are driven as sero.)

// * branch conditions for external interrupt and clock tick use
//   gp_in bits 0 and 1 as the conditions instead

`default_nettype none

module aeonic
  #( parameter DEBUG = 0
  )
  (input            clk,
   input	    rst,

   input [7:0]	    gp_in,
   output reg [7:0] gp_out,
   output reg	    gp_out_stb,

   output	    n_ss,
   output	    sclk,
   output	    mosi,
   input	    miso
   );

   localparam [3:0]
     state_init_1  = 3'b110,  // these init states are workarounds for a bug
     state_init_2  = 3'b111,  // (architectural flaw) in spi_fram_cont.v
		   
     state_fetch_1 = 3'b000,  // issues fram start for instruction fetch
     state_fetch_2 = 3'b001,  // waits for fetch complete, loads ir

     state_mem_1   = 3'b010,  // starts a memory cycle if needed
     state_mem_2   = 3'b011,  // waits for memory cycle complete

     state_exec    = 3'b100,
     state_opr_2   = 3'b101;

   localparam [1:0]
     mem_addr_sel_pc = 2'b00,
     mem_addr_sel_ir = 2'b01,
     mem_addr_sel_x  = 2'b10,
     mem_addr_sel_y  = 2'b11;

   localparam
     ir_sel_hold = 1'b0,
     ir_sel_load = 1'b1;

   localparam [2:0]
     pc_sel_hold   = 3'b000,
     pc_sel_zero   = 3'b001,
     pc_sel_inc    = 3'b010,
     pc_sel_add_a  = 3'b011,
     pc_sel_return = 3'b100,
     pc_sel_branch = 3'b101;

   localparam [0:0]
     ret_sel_hold  = 1'b0,
     ret_sel_pc    = 1'b1;

   localparam [1:0]
     x_sel_hold    = 2'b00,
     x_sel_ac      = 2'b01,
     x_sel_inc     = 2'b10;

   localparam [1:0]
     y_sel_hold    = 2'b10,
     y_sel_ac      = 2'b00,
     y_sel_inc     = 2'b01;

   localparam [1:0]
     b_sel_hold = 2'b00,
     b_sel_imm  = 2'b01,
     b_sel_mem  = 2'b10;

   localparam [1:0]
     ac_sel_hold   = 2'b00,
     ac_sel_alu    = 2'b01,
     ac_sel_rotate = 2'b10,
     ac_sel_gp_in  = 2'b11;

   localparam [1:0]
     cy_sel_hold   = 2'b00,
     cy_sel_alu    = 2'b01,
     cy_sel_rotate = 2'b10,
     cy_sel_ir2    = 2'b11;

   localparam
     gp_out_sel_hold = 1'b0,
     gp_out_sel_ac   = 1'b1;

   localparam [1:0]
     alu_op_sel_mem = 2'b00,
     alu_op_sel_and = 2'b01,
     alu_op_sel_xor = 2'b10,
     alu_op_sel_adc = 2'b11;

   // SPI FRAM controller interface
   reg 		     fram_start;
   reg 		     fram_write;
   reg 		     fram_word;
   wire		     fram_ready;
   wire [23:0] 	     fram_address;
   wire [15:0] 	     fram_write_data;
   wire [15:0] 	     fram_read_data;

   // architectural registers
   reg [11:1] 	     pc;
   reg [11:1] 	     ret;

   reg [7:0] 	     x;
   reg [15:0] 	     y;

   reg [7:0] 	     ac;  // accumulator
   reg 		     cy;  // carry

   // non-architectural registers
   reg [2:0] 	     state;
   reg [15:0] 	     ir;
   reg [7:0] 	     b;  // operand register, either immediate or from memory read

   // combinatorial signals
   reg [2:0] 	     next_state;
   reg [15:0] 	     mem_addr;

   // combinatorial signals for control (output by microinstruction decode)
   reg [1:0] 	     mem_addr_sel;
   reg [2:0] 	     pc_sel;
   reg 		     ir_sel;
   reg [0:0] 	     ret_sel;
   reg [1:0] 	     x_sel;
   reg [1:0] 	     y_sel;
   reg [1:0]         b_sel;
   reg [1:0] 	     ac_sel;
   reg [1:0] 	     cy_sel;
   reg		     gp_out_sel;

   // combinatorial signals in data path
   reg [7:0] 	     rotate_out;
   reg 		     rotate_cout;

   reg [7:0] 	     alu_out;
   reg 		     alu_cy_out;

   reg 		     branch_cond;

   // combinatorial instruction decode
   wire		immediate_inst;
   assign immediate_inst = ir[15:12] == 4'b0100;

   wire 	mem_read_inst;
   assign mem_read_inst = (((ir[15:14] == 2'b01) || (ir[15:14] == 2'b10)) &&
			   (ir[13:12] != 2'b00));

   wire 	store_inst;
   assign store_inst = (ir[15:14] == 2'b00) && (ir[13:12] != 2'b00);

   // combinatorial FRAM signals
   assign fram_write_data = { 8'h00, ac };

   assign fram_address = { 8'h00, mem_addr };


   always @(negedge clk)
     begin
	if (DEBUG >= 1)
	  begin
	     if (state == state_fetch_1)
	       begin
		  $display("state %d: pc=%03x ac=%02x cy=%d x=%02x y=%04x", state, { pc, 1'b0 }, ac, cy, x, y);
	       end
	     if (state == state_mem_1)
	       begin
		  $display("state %d: ir=%04x" , state, ir);
		  if (store_inst)
		    $display("writing addr %04x data %02x", mem_addr, ac);
		  else if (mem_read_inst)
		    $display("reading addr %04x", mem_addr);
	       end
	     if (state == state_exec)
	       begin
		  if (mem_read_inst)
		    $display("read data %02x", b);
	       end
	  end
     end

   // main state machine, synchronous
   always @(posedge clk)
     begin
	state <= next_state;
     end

   // main state machine, combinatorial decode
   always @(*)
     begin
	next_state = state;
	
	mem_addr_sel = mem_addr_sel_pc;
	pc_sel = pc_sel_hold;
	ir_sel = ir_sel_hold;
	ret_sel = ret_sel_hold;
	x_sel = x_sel_hold;
	y_sel = y_sel_hold;
	b_sel = b_sel_hold;
	ac_sel = ac_sel_hold;
	cy_sel = cy_sel_hold;
	gp_out_sel = gp_out_sel_hold;
	fram_start = 0;
	fram_write = 0;
	fram_word  = 0;

	if (rst)
	  begin
	     pc_sel = pc_sel_zero;
	     next_state = state_init_1; // state_fetch_1;
	  end
	else
	  case (state)
	    state_init_1:
	      begin
		 next_state = state_init_2;
	      end
	    state_init_2:
	      begin
		 next_state = state_fetch_1;
	      end
	    state_fetch_1:
	      begin
		 fram_start = 1;
		 fram_word = 1;
		 pc_sel = pc_sel_inc;
		 next_state = state_fetch_2;
	      end
	    state_fetch_2:
	      begin
		 if (fram_ready)
		   begin
		      ir_sel = ir_sel_load;
		      next_state = state_mem_1;
		   end
	      end
	    state_mem_1:
	      begin
		 next_state = state_exec;  // if not a memory inst
		 if (ir[13] == 1'b0)
		   mem_addr_sel = mem_addr_sel_ir;
		 else if (ir[0] == 0)
		   mem_addr_sel = mem_addr_sel_x;
		 else
		   mem_addr_sel = mem_addr_sel_y;
		 if (store_inst)
		   begin
		      fram_start = 1;
		      fram_write = 1;
		      next_state = state_mem_2;
		   end
		 else if (mem_read_inst)
		   begin
		      fram_start = 1;
		      next_state = state_mem_2;
		   end
		 else if (immediate_inst)
		   begin
		      b_sel = b_sel_imm;
		  end
	      end
	    state_mem_2:
	      begin
		 if (fram_ready)
		   begin
		      if (ir[13:12] == 2'b11)
			begin
			   // indexed postincrement
			   if (ir[0] == 1'b0)
			     x_sel = x_sel_inc;
			   else
			     y_sel = y_sel_inc;
			end
		      if (mem_read_inst)
			begin
			   b_sel = b_sel_mem;
			   next_state = state_exec;
			end
		      else
			begin
			   next_state = state_fetch_1;
			end
		   end
	      end
	    state_exec:
	      begin
		 next_state = state_fetch_1;  // unless opr
		 case (ir[15:12])
		   4'b0000: // opr
		     begin
			if (ir[3])
			  cy_sel = cy_sel_ir2;     // clc, sec
			if (ir[6])
			  pc_sel = pc_sel_return;  // ret
			if (ir[10])
			  ac_sel = ac_sel_gp_in;
			next_state = state_opr_2;
		     end
		   4'b0001,
		   4'b0010,
		   4'b0011: // store
		     ; // won't get here, state_exec is skipped
		   4'b0100,
		   4'b0101,
		   4'b0110,
		   4'b0111: // load and ALU
		       begin
			  ac_sel = ac_sel_alu;
			  cy_sel = cy_sel_alu;
		       end
		   4'b1000: // jump
		     begin
			pc_sel = pc_sel_branch;
			if (ir[11])
			  ret_sel = ret_sel_pc;
		     end
		   4'b1001, // skb direct
		   4'b1010, // skb indirect
		   4'b1011: // skb postinc
		       begin
			  if (b[ir[10:8]] == ir[11])
			    pc_sel = pc_sel_inc;
		       end
		   4'b1100, // br
		   4'b1101,
		   4'b1110,
		   4'b1111:
		       begin
			  if (branch_cond == ir[11])
			    pc_sel = pc_sel_branch;
		       end
		 endcase // case (ir[15:12])
	      end
	    state_opr_2:
	      begin
		 // state_opr_2 only used for operate instructions
		 if (ir[1] == 1'b1)
		   begin
		      if (ir[0] == 1'b0)
			x_sel = x_sel_ac;  // tax
		      else
			y_sel = y_sel_ac;  // tay
		   end
		 if (ir[5] == 1'b1)  // rlc, rrc
		   begin
		      ac_sel = ac_sel_rotate;
		      cy_sel = cy_sel_rotate;
		   end
		 if (ir[7] == 1'b1)
		   pc_sel = pc_sel_add_a;  // addapc
		 if (ir[11])
		   gp_out_sel = gp_out_sel_ac;
	         next_state = state_fetch_1;
	      end
	  endcase
     end

   // memory address selection, combinatorial
   always @(*)
     case (mem_addr_sel)
       mem_addr_sel_pc:
	 mem_addr = { 4'b0000, pc, 1'b0 };
       mem_addr_sel_ir:
	 mem_addr = { 8'b00000000, ir[7:0] };
       mem_addr_sel_x:
	 mem_addr = { 8'b00000000, x };
       mem_addr_sel_y:
	 mem_addr = y;
     endcase // case (mem_addr_sel)

   // ALU, combinatorial
   always @(*)
     case (ir[9:8])
       alu_op_sel_mem: { alu_cy_out, alu_out } = { cy, b };
       alu_op_sel_and: { alu_cy_out, alu_out } = { cy, ac & b };
       alu_op_sel_xor: { alu_cy_out, alu_out } = { cy, ac ^ b };
       alu_op_sel_adc: { alu_cy_out, alu_out } = { 1'b0, ac } + { 1'b0, b } + { 8'b0000000, cy };
     endcase

   // branch cond, combinatorial
   always @(*)
     case (ir[13:12])
       2'b00: branch_cond = (ac == 8'b00000000);
       2'b01: branch_cond = cy;
       2'b10: branch_cond = gp_in[0];  // was xint in Glacial
       2'b11: branch_cond = gp_in[1];  // was tick in Glacial
     endcase

   // instruction register (IR), synchronous
   always @(posedge clk)
     case (ir_sel)
       ir_sel_hold:   ;
       ir_sel_load:
	 begin
	    // Glacial was designed with instruction words stored in
	    // big-endian byte order, but the SPI FRAM controller has
	    // a little-endian interface, so we have to swap the
	    // instruction bytes as they are loaded.
	    ir[15:8] <= fram_read_data[7:0];
	    ir[7:0]  <= fram_read_data[15:8];
	 end
     endcase

   // program counter (PC), synchronous
   always @(posedge clk)
     case (pc_sel)
       pc_sel_hold:   ;
       pc_sel_zero:   pc <= 0;
       pc_sel_inc:    pc <= pc + 1;
       pc_sel_add_a:  pc <= pc + { 3'b000, ac };
       pc_sel_return: pc <= ret;
       pc_sel_branch: pc <= ir [10:0];
     endcase

   // return register, synchronous
   always @(posedge clk)
     case (ret_sel)
       ret_sel_hold:  ;
       ret_sel_pc:    ret <= pc;
     endcase

   // X register, synchronous
  always @(posedge clk)
    case (x_sel)
      x_sel_hold:  ;
      x_sel_ac:    x <= ac;
      x_sel_inc:   x <= x + 1;
    endcase

   // Y register, synchronous
   always @(posedge clk)
     case (y_sel)
       y_sel_hold:  ;
       y_sel_ac:    y <= { ac, y [15:8] };
       y_sel_inc:   y <= y + 1;
     endcase

   // rotate unit, combinatorial
  always @(*)
    begin
       if (ir [4]) begin
	  rotate_out  = { cy, ac [7:1] };
	  rotate_cout = ac [0];
       end
       else
	 begin
	    rotate_out  = { ac [6:0], cy };
	    rotate_cout = ac [7];
	 end
    end

   // b register (operand), synchronous
   always @(posedge clk)
     case (b_sel)
       b_sel_hold:  ;
       b_sel_imm:   b <= ir[7:0];
       b_sel_mem:   b <= fram_read_data[7:0];
     endcase

   // accumulator, synchronous
   always @(posedge clk)
     case (ac_sel)
       ac_sel_hold:   ;
       ac_sel_alu:    ac <= alu_out;
       ac_sel_rotate: ac <= rotate_out;
       ac_sel_gp_in:  ac <= gp_in;
     endcase

   // carry, synchronous
   always @(posedge clk)
     case (cy_sel)
       cy_sel_hold:   ;
       cy_sel_alu:    cy <= alu_cy_out;
       cy_sel_rotate: cy <= rotate_cout;
       cy_sel_ir2:    cy <= ir[2];
     endcase

   // gp out, synchronous
   always @(posedge clk)
     case (gp_out_sel)
       gp_out_sel_hold:
	 gp_out_stb <= 0;
       gp_out_sel_ac:
	 begin
	    gp_out <= ac;
	    gp_out_stb <= 1;
	 end
     endcase

   // SPI FRAM controller
   spi_fram_cont spi_fram_cont0(.clk(clk),
				.rst(rst),
				.start(fram_start),
				.write(fram_write),
				.word(fram_word),
				.ready(fram_ready),
				.addr(fram_address),
				.write_data(fram_write_data),
				.read_data(fram_read_data),
				.n_ss(n_ss),
				.sclk(sclk),
				.mosi(mosi),
				.miso(miso));

endmodule

