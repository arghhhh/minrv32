
/*  minrv32 -- A smaller RISC-V (RV32I) Processor Core
 *
 *  Copyright (C) 2019 David Hossack
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */





`timescale 1 ns / 1 ps
// `default_nettype none
// `define DEBUGNETS
// `define DEBUGREGS
// `define DEBUGASM
// `define DEBUG

`ifdef DEBUG
  `define debug(debug_command) debug_command
`else
  `define debug(debug_command)
`endif

`ifdef FORMAL
  `define FORMAL_KEEP (* keep *)
  `define assert(assert_expr) assert(assert_expr)
`else
  `ifdef DEBUGNETS
    `define FORMAL_KEEP (* keep *)
  `else
    `define FORMAL_KEEP
  `endif
  `define assert(assert_expr) empty_statement
`endif

module minrv32 
(

	  input clk
	, input resetn
	, output  trap


//	, input      [31:0] pc
//	, output     [31:0] pc_next
//	, output            pc_next_valid

	, output     [31:0] insn_addr

	, input [31:0] insn
	, input        insn_valid
/*

	, output     [ 4:0] rs1_addr 
	, output     [ 4:0] rs2_addr 
	, output     [ 4:0] rd_addr  
	, output            rs1_addr_valid
	, output            rs2_addr_valid
	, output            rd_addr_valid

	, input      [31:0] rs1_rdata
	, input      [31:0] rs2_rdata
	, output     [31:0] rd_wdata
*/

	, output            mem_valid
	, output            mem_instr
	, input             mem_ready

	, output     [31:0] mem_addr
	, output     [31:0] mem_wdata
	, output     [ 3:0] mem_wstrb
	, output     [ 3:0] mem_rmask
	, input      [31:0] mem_rdata

//	, input      [63:0] csr_time
//	, input      [63:0] csr_cycle
//	, input      [63:0] csr_instret
//
	// IRQ Interface
//	input      [31:0] irq,
//	output reg [31:0] eoi,

`ifdef RISCV_FORMAL
	, output         rvfi_valid
	, output  [63:0] rvfi_order
	, output  [31:0] rvfi_insn
	, output         rvfi_trap
	, output         rvfi_halt
	, output         rvfi_intr
	, output  [ 1:0] rvfi_mode
	, output  [ 1:0] rvfi_ixl
	, output  [ 4:0] rvfi_rs1_addr
	, output  [ 4:0] rvfi_rs2_addr
	, output  [31:0] rvfi_rs1_rdata
	, output  [31:0] rvfi_rs2_rdata
	, output  [ 4:0] rvfi_rd_addr
	, output  [31:0] rvfi_rd_wdata
	, output  [31:0] rvfi_pc_rdata
	, output  [31:0] rvfi_pc_wdata

	, output  [31:0] rvfi_mem_addr
	, output  [ 3:0] rvfi_mem_rmask
	, output  [ 3:0] rvfi_mem_wmask
	, output  [31:0] rvfi_mem_rdata
	, output  [31:0] rvfi_mem_wdata

	, output  [63:0] rvfi_csr_mcycle_rmask
	, output  [63:0] rvfi_csr_mcycle_wmask
	, output  [63:0] rvfi_csr_mcycle_rdata
	, output  [63:0] rvfi_csr_mcycle_wdata

	, output  [63:0] rvfi_csr_minstret_rmask
	, output  [63:0] rvfi_csr_minstret_wmask
	, output  [63:0] rvfi_csr_minstret_rdata
	, output  [63:0] rvfi_csr_minstret_wdata
`endif


);


`ifdef RVFI_MONITOR

rvfi_monitor rvfi_monitor(
    /*  input           */  .clk                      ( clk                     )  ,
    /*  input           */  .reset                    ( !resetn                   )  ,
	/*  input           */  .rvfi_valid               ( rvfi_valid              )  ,
	/*  input   [63:0]  */  .rvfi_order               ( rvfi_order              )  ,
	/*  input   [31:0]  */  .rvfi_insn                ( rvfi_insn               )  ,
	/*  input           */  .rvfi_trap                ( rvfi_trap               )  ,
	/*  input           */  .rvfi_halt                ( rvfi_halt               )  ,
	/*  input           */  .rvfi_intr                ( rvfi_intr               )  ,
	/*  input   [ 1:0]  */  .rvfi_mode                ( rvfi_mode               )  ,
	/*  input   [ 1:0]  */  .rvfi_ixl                 ( rvfi_ixl                )  ,
	/*  input   [ 4:0]  */  .rvfi_rs1_addr            ( rvfi_rs1_addr           )  ,
	/*  input   [ 4:0]  */  .rvfi_rs2_addr            ( rvfi_rs2_addr           )  ,
	/*  input   [31:0]  */  .rvfi_rs1_rdata           ( rvfi_rs1_rdata          )  ,
	/*  input   [31:0]  */  .rvfi_rs2_rdata           ( rvfi_rs2_rdata          )  ,
	/*  input   [ 4:0]  */  .rvfi_rd_addr             ( rvfi_rd_addr            )  ,
	/*  input   [31:0]  */  .rvfi_rd_wdata            ( rvfi_rd_wdata           )  ,
	/*  input   [31:0]  */  .rvfi_pc_rdata            ( rvfi_pc_rdata           )  ,
	/*  input   [31:0]  */  .rvfi_pc_wdata            ( rvfi_pc_wdata           )  ,
	/*  input   [31:0]  */  .rvfi_mem_addr            ( rvfi_mem_addr           )  ,
	/*  input   [ 3:0]  */  .rvfi_mem_rmask           ( rvfi_mem_rmask          )  ,
	/*  input   [ 3:0]  */  .rvfi_mem_wmask           ( rvfi_mem_wmask          )  ,
	/*  input   [31:0]  */  .rvfi_mem_rdata           ( rvfi_mem_rdata          )  ,
	/*  input   [31:0]  */  .rvfi_mem_wdata           ( rvfi_mem_wdata          )  ,
	/*  input   [63:0]  */  .rvfi_csr_mcycle_rmask    ( rvfi_csr_mcycle_rmask   )  ,
	/*  input   [63:0]  */  .rvfi_csr_mcycle_wmask    ( rvfi_csr_mcycle_wmask   )  ,
	/*  input   [63:0]  */  .rvfi_csr_mcycle_rdata    ( rvfi_csr_mcycle_rdata   )  ,
	/*  input   [63:0]  */  .rvfi_csr_mcycle_wdata    ( rvfi_csr_mcycle_wdata   )  ,
	/*  input   [63:0]  */  .rvfi_csr_minstret_rmask  ( rvfi_csr_minstret_rmask )  ,
	/*  input   [63:0]  */  .rvfi_csr_minstret_wmask  ( rvfi_csr_minstret_wmask )  ,
	/*  input   [63:0]  */  .rvfi_csr_minstret_rdata  ( rvfi_csr_minstret_rdata )  ,
	/*  input   [63:0]  */  .rvfi_csr_minstret_wdata  ( rvfi_csr_minstret_wdata )  
);

`endif

//	wire  insn_valid = 1 ;  // for now, insn is always valid
	wire  rs1_ready   ;
	wire  rs2_ready   ;
	wire  rd_ready  = 1  ;  // for now, write destination is always ready




reg [63:0]  csr_cycle;
reg [63:0]  csr_time;
reg [63:0]  csr_instret;

wire insn_complete;

always @(posedge clk) begin
	if ( !resetn ) begin
		csr_cycle   <= 0 ;
		csr_time    <= 0 ; 
		csr_instret <= 0 ;
	end else begin
		csr_cycle   <= csr_cycle   + 1 ;
		csr_time    <= csr_time    + 1 ; 
		if ( insn_complete ) csr_instret <= csr_instret + 1 ;
	end
end

localparam PROGADDR_RESET = 'h10000;
localparam STACKADDR      = 'h10000;



reg [31:0] pc;
wire [31:0] pc_next;

initial pc = PROGADDR_RESET;
always @(posedge clk) begin
	pc <= resetn ? ( insn_complete ? pc_next : pc ) : PROGADDR_RESET;
end



wire  [ 4:0] rs1_addr         ;
wire  [ 4:0] rs2_addr         ;
wire  [ 4:0] rd_addr          ;
wire         rs1_addr_valid   ;
wire         rs2_addr_valid   ;
wire         rd_addr_valid    ;
wire  [31:0] rd_wdata         ;
wire  [31:0] rs1_rdata        ;
wire  [31:0] rs2_rdata        ;

wire         rs1_request      ;
wire         rs2_request      ;
wire         rd_request       ;


reg [31:0] registers [ 0:31 ];
always @(posedge clk) begin
	if ( !resetn ) begin
		registers[ 0] <= 0;

// use USE_MYSTDLIB in Makefile to avoid needing the stack pointer to be set in hardware
// I think the bootloader would deal with setting the stack pointer otherwise...
// .. but there is no bootloader.
//		registers[ 2] <= STACKADDR;
	end
	if ( rd_request && insn_complete ) begin
		registers[ rd_addr ] <= rd_wdata;
	end
end

wire [31:0] reg_x0  = registers[ 0];
wire [31:0] reg_x1  = registers[ 1];
wire [31:0] reg_x2  = registers[ 2];
wire [31:0] reg_x3  = registers[ 3];
wire [31:0] reg_x4  = registers[ 4];
wire [31:0] reg_x5  = registers[ 5];
wire [31:0] reg_x6  = registers[ 6];
wire [31:0] reg_x7  = registers[ 7];
wire [31:0] reg_x8  = registers[ 8];
wire [31:0] reg_x9  = registers[ 9];
wire [31:0] reg_x10 = registers[10];
wire [31:0] reg_x11 = registers[11];
wire [31:0] reg_x12 = registers[12];
wire [31:0] reg_x13 = registers[13];
wire [31:0] reg_x14 = registers[14];
wire [31:0] reg_x15 = registers[15];
wire [31:0] reg_x16 = registers[16];
wire [31:0] reg_x17 = registers[17];
wire [31:0] reg_x18 = registers[18];
wire [31:0] reg_x19 = registers[19];
wire [31:0] reg_x20 = registers[20];
wire [31:0] reg_x21 = registers[21];
wire [31:0] reg_x22 = registers[22];
wire [31:0] reg_x23 = registers[23];
wire [31:0] reg_x24 = registers[24];
wire [31:0] reg_x25 = registers[25];
wire [31:0] reg_x26 = registers[26];
wire [31:0] reg_x27 = registers[27];
wire [31:0] reg_x28 = registers[28];
wire [31:0] reg_x29 = registers[29];
wire [31:0] reg_x30 = registers[30];
wire [31:0] reg_x31 = registers[31];





// Add trace using the RVFI interface
// dump out the first few instructions in using cache and not, and compare...

`define USECACHE2


`ifdef USECACHE0
// combinatorial version:

assign rs1_rdata = registers[ rs1_addr ];
assign rs2_rdata = registers[ rs2_addr ];
assign rs1_ready = 1;
assign rs2_ready = 1;
`endif


`ifdef USECACHE1


// very simple cache to begin with
// just the two register reads for now.


reg [31:0] cache_rs1;
reg [31:0] cache_rs2;
reg [31:0] cache_rs1_next;
reg [31:0] cache_rs2_next;



reg [4:0]  rs_addr;
wire [31:0] rs_rdata;

assign cache_clear_next = insn_complete;

// reg  cache_clear;
wire rs_ready = 1;  // a register read can always take place ... for now

reg        cache_rs1_valid;
reg        cache_rs2_valid;
reg        cache_rs1_valid_next ;
reg        cache_rs2_valid_next ;

always @(*) begin
	cache_rs1_valid_next = cache_rs1_valid;
	cache_rs2_valid_next = cache_rs2_valid;
	cache_rs1_next = cache_rs1;
	cache_rs2_next = cache_rs2;
	rs_addr  = 0;

	if ( cache_clear_next ) begin
		cache_rs1_valid_next = 0;
		cache_rs2_valid_next = 0;
		cache_rs1_next = 0;
		cache_rs2_next = 0;
	end 
	//else
	 begin
		if ( rs1_addr_valid && !cache_rs1_valid ) begin
			rs_addr = rs1_addr;
			cache_rs1_next = rs_rdata;
			cache_rs1_valid_next = rs_ready;
		end else if ( rs2_addr_valid && !cache_rs2_valid ) begin
			rs_addr = rs2_addr;
			cache_rs2_next = rs_rdata;
			cache_rs2_valid_next = rs_ready;
		end
	end
end


assign rs1_rdata = cache_rs1;
assign rs2_rdata = cache_rs2;

always @(posedge clk  )
if ( !resetn ) begin
	cache_rs1_valid <= 0;
	cache_rs2_valid <= 0;
//	cache_clear     <= 1;
	cache_rs1       <= 0;
	cache_rs2       <= 0;
end else begin
	cache_rs1_valid <= cache_rs1_valid_next;
	cache_rs2_valid <= cache_rs2_valid_next;
//	cache_clear <= cache_clear_next;
	cache_rs1 <= cache_rs1_next;
	cache_rs2 <= cache_rs2_next;
end


assign  rs_rdata = registers[ rs_addr ];


assign cache_clear_next = insn_complete;

assign rs1_ready = cache_rs1_valid;
assign rs2_ready = cache_rs2_valid;

`endif

`ifdef USECACHE2




wire [2:0] rs1_state_readyIn;
wire [2:0] rs2_state_readyIn;
wire [2:0] rs1_state_readyOut;
wire [2:0] rs2_state_readyOut;

wire [1:0] arb_readyIn;
wire [1:0] arb_readyOut;

sequencer #( .N(3), .sticky( 3'b110 ) ) rs1_state( 
      .readyIn( rs1_state_readyIn )
    , .readyOut( rs1_state_readyOut )

	, .clk( clk )
	, .resetn( resetn )
);

sequencer #( .N(3), .sticky( 3'b110 ) ) rs2_state( 
      .readyIn( rs2_state_readyIn )
    , .readyOut( rs2_state_readyOut )
	, .clk( clk )
	, .resetn( resetn )
);

priority_arb #(2) arb(
      .readyIn ( arb_readyIn )
    , .readyOut( arb_readyOut )
);

assign arb_readyIn[0]       = rs1_addr_valid && rs1_state_readyOut[0];
assign rs1_state_readyIn[0] = rs1_addr_valid && arb_readyOut[0];

assign arb_readyIn[1]       = rs2_addr_valid && rs2_state_readyOut[0];
assign rs2_state_readyIn[0] = rs2_addr_valid && arb_readyOut[1];

//assign rs1_state_readyIn[1] = arb_readyOut[0];  // combinatorial memory read
//assign rs2_state_readyIn[1] = arb_readyOut[1];  // combinatorial memory read

localparam mem_delay = 3;

pipeline_resetable #( .Nbits(2), .Nstages(mem_delay) ) reg_responses_pipeline ( 
	  .in( arb_readyOut) 
	, .out( { rs2_state_readyIn[1], rs1_state_readyIn[1] } )
	, .clk(clk)
	, .resetn( resetn ) 
	);


assign rs1_state_readyIn[2] = insn_complete;
assign rs2_state_readyIn[2] = insn_complete;

assign rs1_ready = rs1_state_readyOut[2];
assign rs2_ready = rs2_state_readyOut[2];


wire [4:0] rs_addr;

one_hot_mux #( .Ninputs(2), .Nbits(5) ) mux_rs_addr (
      .ins ( { rs2_addr, rs1_addr } )
    , .select( arb_readyOut )
    , .out( rs_addr )
);

wire [31:0] rs_data;
wire [31:0] rs_data1 = registers[ rs_addr ];

pipeline #( .Nbits(32), .Nstages(mem_delay) ) registers_pipeline( .in( rs_data1) , .out( rs_data ), .clk(clk) );

wire rs1_reg_en = rs1_state_readyIn && rs1_state_readyOut;
wire rs2_reg_en = rs2_state_readyIn && rs2_state_readyOut;

register #(32) rs1( .in( rs_data ), .out( rs1_rdata ), .enable( rs1_reg_en ), .clk( clk) );
register #(32) rs2( .in( rs_data ), .out( rs2_rdata ), .enable( rs2_reg_en ), .clk( clk) );


`endif







wire [3:0] mem_wstrb1;
assign mem_wstrb = insn_complete ? mem_wstrb1 : 0;

comb_rv32 comb_rv32 (

	  .trap            ( trap           )
	, .pc              ( pc             )
	, .pc_next         ( pc_next        )
	, .pc_next_valid   ( pc_next_valid  )
	, .insn_addr       ( insn_addr      )
	, .insn            ( insn           )
	, .insn_valid      ( insn_valid && resetn    )
	, .insn_complete   ( insn_complete  )
	, .rs1_addr        ( rs1_addr       )
	, .rs2_addr        ( rs2_addr       )
	, .rd_addr         ( rd_addr        )

	, .rs1_request     ( rs1_request    )   // rs1_addr_valid && rs1_addr!=0
	, .rs2_request     ( rs2_request    )
	, .rd_request      ( rd_request     )

	, .rs1_addr_valid  ( rs1_addr_valid )
	, .rs2_addr_valid  ( rs2_addr_valid )
	, .rd_addr_valid   ( rd_addr_valid  )
	, .rs1_rdata       ( rs1_rdata      )
	, .rs2_rdata       ( rs2_rdata      )
	, .rd_wdata        ( rd_wdata       )
	, .rs1_ready       ( rs1_ready      )
	, .rs2_ready       ( rs2_ready      )
	, .rd_ready        ( rd_ready       )
	, .mem_valid       ( mem_valid      )
	, .mem_instr       ( mem_instr      )
	, .mem_ready       ( mem_ready      )
	, .mem_addr        ( mem_addr       )
	, .mem_wdata       ( mem_wdata      )
	, .mem_wstrb       ( mem_wstrb1      )
	, .mem_rmask       ( mem_rmask      )
	, .mem_rdata       ( mem_rdata      )
	, .csr_time        ( csr_time       )
	, .csr_cycle       ( csr_cycle      )
	, .csr_instret     ( csr_instret    )


`ifdef RISCV_FORMAL
	, .rvfi_valid              ( rvfi_valid                  )
	, .rvfi_order              ( rvfi_order                  )
	, .rvfi_insn               ( rvfi_insn                   )
	, .rvfi_trap               ( rvfi_trap                   )
	, .rvfi_halt               ( rvfi_halt                   )
	, .rvfi_intr               ( rvfi_intr                   )
	, .rvfi_mode               ( rvfi_mode                   )
	, .rvfi_ixl                ( rvfi_ixl                    )
	, .rvfi_rs1_addr           ( rvfi_rs1_addr               )
	, .rvfi_rs2_addr           ( rvfi_rs2_addr               )
	, .rvfi_rs1_rdata          ( rvfi_rs1_rdata              )
	, .rvfi_rs2_rdata          ( rvfi_rs2_rdata              )
	, .rvfi_rd_addr            ( rvfi_rd_addr                )
	, .rvfi_rd_wdata           ( rvfi_rd_wdata               )
	, .rvfi_pc_rdata           ( rvfi_pc_rdata               )
	, .rvfi_pc_wdata           ( rvfi_pc_wdata               )
	, .rvfi_mem_addr           ( rvfi_mem_addr               )
	, .rvfi_mem_rmask          ( rvfi_mem_rmask              )
	, .rvfi_mem_wmask          ( rvfi_mem_wmask              )
	, .rvfi_mem_rdata          ( rvfi_mem_rdata              )
	, .rvfi_mem_wdata          ( rvfi_mem_wdata              )
	, .rvfi_csr_mcycle_rmask   ( rvfi_csr_mcycle_rmask       )
	, .rvfi_csr_mcycle_wmask   ( rvfi_csr_mcycle_wmask       )
	, .rvfi_csr_mcycle_rdata   ( rvfi_csr_mcycle_rdata       )
	, .rvfi_csr_mcycle_wdata   ( rvfi_csr_mcycle_wdata       )
	, .rvfi_csr_minstret_rmask ( rvfi_csr_minstret_rmask     )
	, .rvfi_csr_minstret_wmask ( rvfi_csr_minstret_wmask     )
	, .rvfi_csr_minstret_rdata ( rvfi_csr_minstret_rdata     )
	, .rvfi_csr_minstret_wdata ( rvfi_csr_minstret_wdata     )
`endif

);

endmodule

/*
module #( parameter integer N = 1 ) priority_arbiter (
	  input      [N-1:0] req
	, output reg [N-1:0] grant
);
integer i;
	always @(*) begin
		grant = 0;
		for( i=N-1: i >= 0; i = i-1 ) begin
			if ( req[i] ) begin
				grant[i] = 1;
				break;
			end
		end
	end
endmodule

module #( parameter integer N, parameter integer W) one_hot_mux (
	  input  [N-1:0]        select
	, input  [N-1:0][W-1:0] in
	, output reg    [W-1:0] out
);
integer i;
	out = 0;
	always @(*) begin
		out = 0;
		for( i=N-1: i >= 0; i = i-1 ) begin
			if ( sel[i] ) begin
				out = out | in[i][W-1:0]
			end
		end
	end
endmodule
*/

