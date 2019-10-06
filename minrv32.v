
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


// need to make code clean when this isn't defined
// for now, just define it
`define RISCV_FORMAL




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


	wire  insn_valid = 1 ;  // for now, insn is always valid
	wire  rs1_ready   ;
	wire  rs2_ready   ;
	wire  rd_ready  = 1  ;  // for now, write destination is always ready




reg [63:0]  csr_cycle;
reg [63:0]  csr_time;
reg [63:0]  csr_instret;

always @(posedge clk) begin
	if ( !resetn ) begin
		csr_cycle   <= 0 ;
		csr_time    <= 0 ; 
		csr_instret <= 0 ;
	end else begin
		csr_cycle   <= csr_cycle   + 1 ;
		csr_time    <= csr_time    + 1 ; 
		csr_instret <= csr_instret + 1 ;
	end
end

localparam PROGADDR_RESET = 'h10000;
localparam STACKADDR      = 'h10000;

wire insn_complete;

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
		registers[ 2] <= STACKADDR;
	end
	if ( rd_request ) begin
		registers[ rd_addr ] <= rd_wdata;
	end
end



// Add trace using the RVFI interface
// dump out the first few instructions in using cache and not, and compare...

// `define USECACHE
`ifdef USECACHE


// very simple cache to begin with
// just the two register reads for now.


reg [31:0] cache_rs1;
reg [31:0] cache_rs2;
reg [31:0] cache_rs1_next;
reg [31:0] cache_rs2_next;



reg [4:0]  rs_addr;
wire [31:0] rs_rdata;

assign cache_clear_next = insn_complete;

reg  cache_clear;
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

	if ( cache_clear ) begin
		cache_rs1_valid_next = 0;
		cache_rs2_valid_next = 0;
		cache_rs1_next = 0;
		cache_rs2_next = 0;
	end else begin
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
	cache_rs1_valid <= 0;
	cache_clear     <= 1;
	cache_rs1       <= 0;
	cache_rs2       <= 0;
end else begin
	cache_rs1_valid <= cache_rs1_valid_next;
	cache_rs2_valid <= cache_rs2_valid_next;
	cache_clear <= cache_clear_next;
	cache_rs1 <= cache_rs1_next;
	cache_rs2 <= cache_rs2_next;
end


assign  rs_rdata = registers[ rs_addr ];


assign cache_clear_next = insn_complete;

assign rs1_ready = cache_rs1_valid;
assign rs2_ready = cache_rs2_valid;






`else

// combinatorial version:

assign rs1_rdata = registers[ rs1_addr ];
assign rs2_rdata = registers[ rs2_addr ];
assign rs1_ready = 1;
assign rs2_ready = 1;

`endif

wire [3:0] mem_wstrb1;
assign mem_wstrb = insn_complete ? mem_wstrb1 : 0;

minrv32_combinatorial minrv32_combinatorial (

	  .trap            ( trap           )
	, .pc              ( pc             )
	, .pc_next         ( pc_next        )
	, .pc_next_valid   ( pc_next_valid  )
	, .insn_addr       ( insn_addr      )
	, .insn            ( insn           )
	, .insn_valid      ( insn_valid     )
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


module minrv32_combinatorial #(
	parameter [ 0:0] ENABLE_COUNTERS = 1,
	parameter [ 0:0] ENABLE_COUNTERS64 = 1,
	parameter [ 0:0] ENABLE_REGS_16_31 = 1,
	parameter [ 0:0] ENABLE_REGS_DUALPORT = 1,
	parameter [ 0:0] LATCHED_MEM_RDATA = 0,
	parameter [ 0:0] TWO_STAGE_SHIFT = 1,
	parameter [ 0:0] BARREL_SHIFTER = 0,
	parameter [ 0:0] TWO_CYCLE_COMPARE = 0,
	parameter [ 0:0] TWO_CYCLE_ALU = 0,
	parameter [ 0:0] COMPRESSED_ISA = 0,
	parameter [ 0:0] CATCH_MISALIGN = 1,
	parameter [ 0:0] CATCH_ILLINSN = 1,
	parameter [ 0:0] ENABLE_PCPI = 0,
	parameter [ 0:0] ENABLE_MUL = 0,
	parameter [ 0:0] ENABLE_FAST_MUL = 0,
	parameter [ 0:0] ENABLE_DIV = 0,
	parameter [ 0:0] ENABLE_IRQ = 0,
	parameter [ 0:0] ENABLE_IRQ_QREGS = 1,
	parameter [ 0:0] ENABLE_IRQ_TIMER = 1,
	parameter [ 0:0] ENABLE_TRACE = 0,
	parameter [ 0:0] REGS_INIT_ZERO = 0,
	parameter [31:0] MASKED_IRQ = 32'h 0000_0000,
	parameter [31:0] LATCHED_IRQ = 32'h ffff_ffff,
	parameter [31:0] PROGADDR_RESET = 32'h 0000_0000,
	parameter [31:0] PROGADDR_IRQ = 32'h 0000_0010,
	parameter [31:0] STACKADDR = 32'h ffff_ffff
) (

	  output  trap

	, input      [31:0] pc
	, output reg [31:0] pc_next
	, output reg        pc_next_valid

	, output     [31:0] insn_addr

	, input [31:0] insn
	, input        insn_valid

	, output       insn_complete

	, output     [ 4:0] rs1_addr 
	, output     [ 4:0] rs2_addr 
	, output     [ 4:0] rd_addr  
	, output            rs1_addr_valid
	, output            rs2_addr_valid
	, output            rd_addr_valid

	, output            rs1_request   // rs1_addr_valid && rs1_addr!=0
	, output            rs2_request
	, output            rd_request

	, input      [31:0] rs1_rdata
	, input      [31:0] rs2_rdata
	, output reg [31:0] rd_wdata

	, input             rs1_ready
	, input             rs2_ready
	, input             rd_ready

	, output reg        mem_valid
	, output reg        mem_instr
	, input             mem_ready

	, output reg [31:0] mem_addr
	, output reg [31:0] mem_wdata
	, output reg [ 3:0] mem_wstrb
	, output reg [ 3:0] mem_rmask
	, input      [31:0] mem_rdata

	, input      [63:0] csr_time
	, input      [63:0] csr_cycle
	, input      [63:0] csr_instret

	// IRQ Interface
//	input      [31:0] irq,
//	output reg [31:0] eoi,

`ifdef RISCV_FORMAL
	, output         rvfi_valid,
	output  [63:0] rvfi_order,
	output  [31:0] rvfi_insn,
	output         rvfi_trap,
	output         rvfi_halt,
	output         rvfi_intr,
	output  [ 1:0] rvfi_mode,
	output  [ 1:0] rvfi_ixl,
	output  [ 4:0] rvfi_rs1_addr,
	output  [ 4:0] rvfi_rs2_addr,
	output  [31:0] rvfi_rs1_rdata,
	output  [31:0] rvfi_rs2_rdata,
	output  [ 4:0] rvfi_rd_addr,
	output  [31:0] rvfi_rd_wdata,
	output  [31:0] rvfi_pc_rdata,
	output  [31:0] rvfi_pc_wdata,

	output  [31:0] rvfi_mem_addr,
	output  [ 3:0] rvfi_mem_rmask,
	output  [ 3:0] rvfi_mem_wmask,
	output  [31:0] rvfi_mem_rdata,
	output  [31:0] rvfi_mem_wdata,

	output  [63:0] rvfi_csr_mcycle_rmask,
	output  [63:0] rvfi_csr_mcycle_wmask,
	output  [63:0] rvfi_csr_mcycle_rdata,
	output  [63:0] rvfi_csr_mcycle_wdata,

	output  [63:0] rvfi_csr_minstret_rmask,
	output  [63:0] rvfi_csr_minstret_wmask,
	output  [63:0] rvfi_csr_minstret_rdata,
	output  [63:0] rvfi_csr_minstret_wdata
`endif

);


	reg rs1_addr_valid;
	reg rs2_addr_valid;
	reg rd_addr_valid;


assign rs1_request = rs1_addr_valid && ( rs1_addr != 0 );
assign rs2_request = rs2_addr_valid && ( rs2_addr != 0 );
assign rd_request  = rd_addr_valid && ( rd_addr != 0 );


	assign insn_addr = { pc[31:1], 1'b0 };

	assign rvfi_order = csr_instret;

	reg insn_decode_valid;
	reg gen_trap;
	assign trap = !insn_decode_valid || gen_trap;


	wire [6:0] insn_field_opcode = insn[6:0];
	wire [2:0] insn_field_funct3 = insn[14:12];
	wire [6:0] insn_field_funct7 = insn[31:25];
	wire [4:0] insn_field_rd     = insn[11:7];
	wire [4:0] insn_field_rs1    = insn[19:15];
	wire [4:0] insn_field_rs2    = insn[24:20];


	assign rs1_addr = rs1_addr_valid ? insn_field_rs1 : 5'b0;
	assign rs2_addr = rs2_addr_valid ? insn_field_rs2 : 5'b0;
	assign rd_addr  = rd_addr_valid  ? insn_field_rd : 5'b0;

	reg [31:0] rs1_value ;
	reg [31:0] rs2_value ;

//	reg [3:0] mem_wmask
//	assign mem_wstrb = mem_wmask;


	assign rvfi_insn = insn;
	assign rvfi_pc_rdata = insn_addr;
	assign rvfi_pc_wdata = pc_next;

	assign rvfi_rs1_addr   = rs1_addr  ;
	assign rvfi_rs2_addr   = rs2_addr  ;
	assign rvfi_rd_addr    = rd_addr   ;

	assign rvfi_rs1_rdata  = ( rs1_addr_valid ) ? rs1_value : 32'b0 ;
	assign rvfi_rs2_rdata  = ( rs2_addr_valid ) ? rs2_value : 32'b0 ;
	assign rvfi_rd_wdata   = ( rd_addr_valid && ( insn_field_rd != 0 ) ) ? rd_wdata  : 32'b0 ;

	assign rvfi_valid      = 1        ;
	assign rvfi_trap       = trap     ;
	assign rvfi_halt       = 0        ;

	assign rvfi_mem_addr   = mem_addr  ;
	assign rvfi_mem_rmask  = mem_rmask ;
	assign rvfi_mem_wmask  = mem_wstrb ;
	assign rvfi_mem_rdata  = mem_rdata ;
	assign rvfi_mem_wdata  = mem_wdata ;

	assign rvfi_intr = 0;

	assign rvfi_csr_mcycle_rmask   = 0 ;
	assign rvfi_csr_mcycle_wmask   = 0 ;
	assign rvfi_csr_mcycle_rdata   = 0 ;
	assign rvfi_csr_mcycle_wdata   = 0 ;

	assign rvfi_csr_minstret_rmask = 0 ;
	assign rvfi_csr_minstret_wmask = 0 ;
	assign rvfi_csr_minstret_rdata = 0 ;
	assign rvfi_csr_minstret_wdata = 0 ;


	reg is_alu_immediate;

	wire [31:0] immediate_12bit = { {20{insn[31]}}, insn[31:20] };
	wire [31:0] immediate_12bit_for_stores = { immediate_12bit[31:5], insn[11:7] };
	wire [31:0] immediate_for_jal = { {12{insn[31]}}, insn[19:12], insn[20], insn[30:21], 1'b0 };
	wire [31:0] immediate_for_branches = { {20{insn[31]}}, insn[7], insn[ 30:25], insn[11:8], 1'b0 };

	wire [31:0] pc_next_no_branch = insn_addr + 4;
	wire [31:0] pc_next_branch = ( insn_addr + immediate_for_branches ) & 32'hFFFF_FFFE;


	wire cond_eq  = rs1_value == rs2_value ;
	wire cond_neq = rs1_value != rs2_value ;
	wire cond_lt  = ( rs1_value ^ 32'h8000_0000 ) < ( rs2_value ^ 32'h8000_0000 ) ;
	wire cond_ge  = !cond_lt;
	wire cond_ltu = rs1_value < rs2_value;
	wire cond_geu = !cond_ltu;

	wire rs1_ok = !rs1_addr_valid || rs1_ready ;
	wire rs2_ok = !rs2_addr_valid || rs2_ready ;
	wire rd_ok  = !rd_addr_valid  || rd_ready  ;
	wire mem_ok = !mem_valid || mem_ready      ;

	assign insn_complete = insn_valid && rs1_ok && rs2_ok && rd_ok && mem_ok;

	always @* begin
		insn_decode_valid = 0;
		gen_trap = 0;
		rs1_addr_valid = 0 ;
		rs2_addr_valid = 0 ;
		rd_addr_valid  = 0 ;
		rd_wdata = 32'b0;
		pc_next = pc_next_no_branch;
		pc_next_valid = 0;  // not valid until insn_valid

		is_alu_immediate = 0;

		rs1_value = ( insn_field_rs1 == 0 ) ? 0 : rs1_rdata;
		rs2_value = ( insn_field_rs2 == 0 ) ? 0 : rs2_rdata;  // may override this for immediate instructions

		mem_valid = 0;
		mem_instr = 0;
		mem_addr  = 0;
		mem_wdata = 0;
		mem_rmask = 0;
		mem_wstrb = 0;

		if ( insn_valid ) begin 
			pc_next_valid = 1;  // by default, valid, except for jumps and branches
			if (insn_field_opcode == 7'b 01_101_11) begin
				insn_decode_valid = 1; // LUI load upper immediate
				rd_addr_valid  = 1 ;
				rd_wdata = { insn[31:12], 12'b0 };
			end
			if (insn_field_opcode == 7'b 00_101_11) begin
				insn_decode_valid = 1; // AUIPC add upper immediate program counter
				rd_addr_valid  = 1 ;
				rd_wdata = { insn[31:12], 12'b0 } + insn_addr;
			end
			if (insn_field_opcode == 7'b 11_011_11) begin
				insn_decode_valid = 1; // JAL jump and link
				rd_addr_valid  = 1 ;
				rd_wdata = pc_next_no_branch;
				pc_next = insn_addr + immediate_for_jal;
				pc_next_valid = insn_complete;
				gen_trap = |pc_next[1:0];
			end
			if (insn_field_opcode == 7'b 11_001_11) begin // JALR jump and link register
				if (  insn_field_funct3 == 3'b 000 ) begin
					insn_decode_valid = 1;
					rs1_addr_valid = 1 ;
					rd_addr_valid  = 1 ;
					rd_wdata = pc_next_no_branch;
					pc_next = ( rs1_value + immediate_12bit ) & 32'hFFFF_FFFE;
					gen_trap = |pc_next[1:0];
				end
			end

			if (insn_field_opcode == 7'b 11_000_11) begin // BRANCH
				insn_decode_valid = (insn_field_funct3 != 3'b 010) && (insn_field_funct3 != 3'b 011);
				rs1_addr_valid = 1 ;
				rs2_addr_valid = 1 ;
				if (
					   ( ( insn_field_funct3 == 3'b000 ) && cond_eq  ) // BEQ branch equal
					|| ( ( insn_field_funct3 == 3'b001 ) && cond_neq ) // BNE branch not equal
					|| ( ( insn_field_funct3 == 3'b100 ) && cond_lt  ) // BLT branch less than
					|| ( ( insn_field_funct3 == 3'b101 ) && cond_ge  ) // BGE branch greater or equal
					|| ( ( insn_field_funct3 == 3'b110 ) && cond_ltu ) // BLTU branch less than unsigned
					|| ( ( insn_field_funct3 == 3'b111 ) && cond_geu ) // BGEU branch greater or equal unsigned
					) begin
						pc_next = pc_next_branch;
					end
				pc_next_valid = insn_complete;
				gen_trap = |pc_next[1:0];
			end

			if (insn_field_opcode == 7'b 00_000_11) begin // LOAD
				insn_decode_valid = (insn_field_funct3 != 3'b 011) && (insn_field_funct3 != 3'b 110) && (insn_field_funct3 != 3'b 111);
				rs1_addr_valid = 1 ;
				rd_addr_valid  = 1 ;
				mem_valid = 1;
				mem_addr = rs1_value + immediate_12bit;
				if (insn_field_funct3 == 3'b000 ) begin // LB  load byte
					mem_rmask = 4'b0001;
					rd_wdata = { {24{mem_rdata[7]}}, mem_rdata[7:0] };
				end
				if (insn_field_funct3 == 3'b100 ) begin // LBU  load byte unsigned
					mem_rmask = 4'b0001;
					rd_wdata = { 24'b0, mem_rdata[7:0] };
				end
				if (insn_field_funct3 == 3'b001 ) begin // LH  load half word
					mem_rmask = 4'b0011;
					rd_wdata = { {16{mem_rdata[15]}}, mem_rdata[15:0] };
				end
				if (insn_field_funct3 == 3'b101 ) begin // LHU load half word unsigned
					mem_rmask = 4'b0011;
					rd_wdata = { 16'b0, mem_rdata[15:0] };
				end
				if (insn_field_funct3 == 3'b010 ) begin // LW  load word
					mem_rmask = 4'b1111;
					rd_wdata = mem_rdata;
				end
			end

			if (insn_field_opcode == 7'b 01_000_11) begin // STORE
				insn_decode_valid = (insn_field_funct3 == 3'b 000) || (insn_field_funct3 == 3'b 001) || (insn_field_funct3 == 3'b 010);

				rs1_addr_valid = 1 ;
				rs2_addr_valid = 1 ;
				mem_valid = 1;
				mem_wdata = rs2_value;

				mem_addr = rs1_value + immediate_12bit_for_stores;
				if (insn_field_funct3 == 3'b000 ) begin // SB  store byte
					mem_wstrb = 4'b0001;
				end
				if (insn_field_funct3 == 3'b001 ) begin // SH  store half word
					mem_wstrb = 4'b0011;
				end
				if (insn_field_funct3 == 3'b010 ) begin // SW  store word
					mem_wstrb = 4'b1111;
				end

			end

			if ( (insn_field_opcode == 7'b 01_100_11) || (insn_field_opcode == 7'b 00_100_11) ) begin // OP or OP-IMM
				is_alu_immediate = (insn_field_opcode == 7'b 00_100_11);
				if ( is_alu_immediate ) begin
					rs2_value = immediate_12bit;
				end

				case (insn_field_funct3)
					3'b 000: begin // ADD SUB
						rs1_addr_valid = 1 ;
						rs2_addr_valid = !is_alu_immediate ;
						rd_addr_valid  = 1 ;

						if (is_alu_immediate || insn_field_funct7 == 7'b 0000000) begin
							insn_decode_valid = 1; // ADD
							rd_wdata = rs1_value + rs2_value;
						end else if ( insn_field_funct7 == 7'b 0100000 ) begin
							insn_decode_valid = 1; // SUB
							rd_wdata = rs1_value - rs2_value;
						end
					end
					3'b 101: begin // SRL SRA SRLI SRAI
						rs1_addr_valid = 1 ;
						rs2_addr_valid = !is_alu_immediate ;
						rd_addr_valid  = 1 ;

	//					if ( (!is_alu_immediate && insn_field_funct7 == 7'b 0000000 ) || ( is_alu_immediate && insn_field_funct7 == 7'b 0000000 ) ) begin
						if ( insn_field_funct7 == 7'b 0000000 ) begin
							insn_decode_valid = 1; // SRL
							rd_wdata = rs1_value >> rs2_value[4:0];
						end else if ( (!is_alu_immediate && insn_field_funct7 == 7'b 0100000 ) || ( is_alu_immediate && insn_field_funct7 == 7'b 0100000 ) ) begin
							insn_decode_valid = 1; // SRA
							rd_wdata = ( { {32{rs1_value[31]}}, rs1_value } >> rs2_value[4:0] ); //[31:0];
						end
					end
					3'b001: begin 
	//					if ( (!is_alu_immediate && insn_field_funct7 == 7'b 0000000 ) || ( is_alu_immediate && insn_field_funct7 == 7'b 0000000 ) ) begin 
						if ( insn_field_funct7 == 7'b 0000000 ) begin 
							insn_decode_valid = 1; // SLLI
							rs1_addr_valid = 1 ;
							rs2_addr_valid = !is_alu_immediate ;
							rd_addr_valid  = 1 ;
							rd_wdata = rs1_value << rs2_value[4:0] ;
						end
					end
					default: begin
						if ( is_alu_immediate || insn_field_funct7 == 7'b 0000000 ) begin
							insn_decode_valid = 1;
							rs1_addr_valid = 1 ;
							rs2_addr_valid = !is_alu_immediate ;
							rd_addr_valid  = 1 ;
							case (insn_field_funct3)
					//			3'b001: begin // SLL
					//				rd_wdata = rs1_value << rs2_value[4:0] ;
					//			end
								3'b010: begin // SLT set less than
									rd_wdata = { 31'b0, cond_lt };
								end
								3'b011: begin // SLTU set less than unsigned (with a signed extended immediate if immediate mode)
									rd_wdata = { 31'b0, cond_ltu }; 
								end
								3'b100: begin // XOR
									rd_wdata = rs1_value ^ rs2_value ;
								end
								3'b110: begin // OR
									rd_wdata = rs1_value | rs2_value ;
								end
								3'b111: begin // AND
									rd_wdata = rs1_value & rs2_value ;
								end
							endcase
						end
					end
				endcase
			end
			if (insn_field_opcode == 7'b 11_100_11) begin 
				if ( insn_field_rs1 == 0 && insn_field_funct3 == 3'b010 ) begin
					case (insn[31:20])
						12'hC00: begin
							insn_decode_valid = 1 ;
							rd_addr_valid = 1;
							rd_wdata = csr_cycle[31:0] ;
						end
						12'hC01: begin
							insn_decode_valid = 1 ;
							rd_addr_valid = 1;
							rd_wdata = csr_time[31:0] ;
						end
						12'hC02: begin
							insn_decode_valid = 1 ;
							rd_addr_valid = 1;
							rd_wdata = csr_instret[31:0] ;
						end
						12'hC80: begin
							insn_decode_valid = 1 ;
							rd_addr_valid = 1;
							rd_wdata = csr_cycle[63:32] ;
						end
						12'hC81: begin
							insn_decode_valid = 1 ;
							rd_addr_valid = 1;
							rd_wdata = csr_time[63:32] ;
						end
						12'hC82: begin
							insn_decode_valid = 1 ;
							rd_addr_valid = 1;
							rd_wdata = csr_instret[63:32] ;
						end
					endcase
					end
			end
		end
	end

endmodule
