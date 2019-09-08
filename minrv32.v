
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


module minrv32 #(
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
	input clk, resetn,
	output  trap

	, output reg [31:0] pc
	, output reg [31:0] pc_next

	, input [31:0] insn

	, output     [ 4:0] rs1_addr 
	, output     [ 4:0] rs2_addr 
	, output     [ 4:0] rd_addr  
	, output            rs1_addr_valid
	, output            rs2_addr_valid
	, output            rd_addr_valid

	, input      [31:0] rs1_rdata
	, input      [31:0] rs2_rdata
	, output reg [31:0] rd_wdata



	, output reg      mem_valid,
	output reg        mem_instr,
	input             mem_ready,

	output reg [31:0] mem_addr,
	output reg [31:0] mem_wdata,
	output     [ 3:0] mem_wstrb,
	output reg [ 3:0] mem_rmask,
	input      [31:0] mem_rdata,


	// IRQ Interface
//	input      [31:0] irq,
//	output reg [31:0] eoi,

`ifdef RISCV_FORMAL
	output         rvfi_valid,
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
	output  [63:0] rvfi_csr_minstret_wdata,
`endif

	// Trace Interface
//	output reg        trace_valid,
//	output reg [35:0] trace_data
);

initial pc = PROGADDR_RESET;
always @(posedge clk) begin
	pc <= resetn ? pc_next : PROGADDR_RESET;
end


reg [63:0] rvfi_instruction_count = 0;
assign rvfi_order =  rvfi_instruction_count;
always @(posedge clk) begin
	rvfi_instruction_count <= resetn ? rvfi_instruction_count+1 : 0;
end


	reg valid;
	reg gen_trap;
	assign trap = !valid || gen_trap;


	reg rs1_addr_valid;
	reg rs2_addr_valid;
	reg rd_addr_valid;
	

	assign rs1_addr = rs1_addr_valid ? insn[19:15] : 5'b0;
	assign rs2_addr = rs2_addr_valid ? insn[24:20] : 5'b0;
	assign rd_addr  = rd_addr_valid  ? insn[11: 7] : 5'b0;

	reg [31:0] rs1_value ;
	reg [31:0] rs2_value ;

	reg [3:0] mem_wmask;
	assign mem_wstrb = mem_wmask;


	assign rvfi_insn = insn;
	assign rvfi_pc_rdata = pc;
	assign rvfi_pc_wdata = pc_next;

	assign rvfi_rs1_addr   = rs1_addr  ;
	assign rvfi_rs2_addr   = rs2_addr  ;
	assign rvfi_rd_addr    = rd_addr   ;

	assign rvfi_rs1_rdata  = ( rs1_addr_valid ) ? rs1_value : 32'b0 ;
	assign rvfi_rs2_rdata  = ( rs2_addr_valid ) ? rs2_value : 32'b0 ;
	assign rvfi_rd_wdata   = ( rd_addr_valid && ( insn[11:7] != 0 ) ) ? rd_wdata  : 32'b0 ;

	assign rvfi_valid      = 1        ;
	assign rvfi_trap       = trap     ;

	assign rvfi_mem_addr   = mem_addr  ;
	assign rvfi_mem_rmask  = mem_rmask ;
	assign rvfi_mem_wmask  = mem_wmask ;
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

	wire [31:0] pc_next_branch = ( pc + immediate_for_branches ) & 32'hFFFF_FFFE;


	wire cond_eq  = rs1_value == rs2_value ;
	wire cond_neq = rs1_value != rs2_value ;
	wire cond_lt  = ( rs1_value ^ 32'h8000_0000 ) < ( rs2_value ^ 32'h8000_0000 ) ;
	wire cond_ge  = !cond_lt;
	wire cond_ltu = rs1_value < rs2_value;
	wire cond_geu = !cond_ltu;

	always @* begin
		valid = 0;
		gen_trap = 0;
		rs1_addr_valid = 0 ;
		rs2_addr_valid = 0 ;
		rd_addr_valid  = 0 ;
		rd_wdata = 32'b0;
		pc_next = pc + 4;

		is_alu_immediate = 0;

		rs1_value = ( insn[19:15] == 0 ) ? 0 : rs1_rdata;
		rs2_value = ( insn[24:20] == 0 ) ? 0 : rs2_rdata;  // may override this for immediate instructions

		mem_valid = 0;
		mem_instr = 0;
		mem_addr  = 0;
		mem_wdata = 0;
		mem_rmask = 0;
		mem_wmask = 0;


		if (insn[6:0] == 7'b 01_101_11) begin
			valid = 1; // LUI load upper immediate
			rd_addr_valid  = 1 ;
			rd_wdata = { insn[31:12], 12'b0 };
		end
		if (insn[6:0] == 7'b 00_101_11) begin
			valid = 1; // AUIPC add upper immediate program counter
			rd_addr_valid  = 1 ;
			rd_wdata = { insn[31:12], 12'b0 } + pc;
		end
		if (insn[6:0] == 7'b 11_011_11) begin
			valid = 1; // JAL jump and link
			rd_addr_valid  = 1 ;
			rd_wdata = pc_next;
			pc_next = pc + immediate_for_jal;
			gen_trap = |pc_next[1:0];
		end
		if (insn[6:0] == 7'b 11_001_11) begin // JALR jump and link register
			if (  insn[14:12] == 3'b 000 ) begin
				valid = 1;
				rs1_addr_valid = 1 ;
				rd_addr_valid  = 1 ;
				rd_wdata = pc_next;
				pc_next = ( rs1_value + immediate_12bit ) & 32'hFFFF_FFFE;
				gen_trap = |pc_next[1:0];
			end
		end

		if (insn[6:0] == 7'b 11_000_11) begin // BRANCH
			valid = (insn[14:12] != 3'b 010) && (insn[14:12] != 3'b 011);
			rs1_addr_valid = 1 ;
			rs2_addr_valid = 1 ;
			if (
				   ( ( insn[14:12] == 3'b000 ) && cond_eq  ) // BEQ branch equal
				|| ( ( insn[14:12] == 3'b001 ) && cond_neq ) // BNE branch not equal
				|| ( ( insn[14:12] == 3'b100 ) && cond_lt  ) // BNE branch less than
				|| ( ( insn[14:12] == 3'b101 ) && cond_ge  ) // BNE branch greater or equal
				|| ( ( insn[14:12] == 3'b110 ) && cond_ltu ) // BNE branch less than unsigned
				|| ( ( insn[14:12] == 3'b111 ) && cond_geu ) // BNE branch greater or equal unsigned
				) begin
					pc_next = pc_next_branch;
				end
			gen_trap = |pc_next[1:0];
		end

		if (insn[6:0] == 7'b 00_000_11) begin // LOAD
			valid = (insn[14:12] != 3'b 011) && (insn[14:12] != 3'b 110) && (insn[14:12] != 3'b 111);
			rs1_addr_valid = 1 ;
			rd_addr_valid  = 1 ;
			mem_valid = 1;
			mem_addr = rs1_value + immediate_12bit;
			if (insn[14:12] == 3'b000 ) begin // LB  load byte
				mem_rmask = 4'b0001;
				rd_wdata = { {24{mem_rdata[7]}}, mem_rdata[7:0] };
			end
			if (insn[14:12] == 3'b100 ) begin // LBU  load byte unsigned
				mem_rmask = 4'b0001;
				rd_wdata = { 24'b0, mem_rdata[7:0] };
			end
			if (insn[14:12] == 3'b001 ) begin // LH  load half word
				mem_rmask = 4'b0011;
				rd_wdata = { {16{mem_rdata[15]}}, mem_rdata[15:0] };
			end
			if (insn[14:12] == 3'b101 ) begin // LHU load half word unsigned
				mem_rmask = 4'b0011;
				rd_wdata = { 16'b0, mem_rdata[15:0] };
			end
			if (insn[14:12] == 3'b010 ) begin // LW  load word
				mem_rmask = 4'b1111;
				rd_wdata = mem_rdata;
			end
		end

		if (insn[6:0] == 7'b 01_000_11) begin // STORE
			valid = (insn[14:12] == 3'b 000) || (insn[14:12] == 3'b 001) || (insn[14:12] == 3'b 010);

			rs1_addr_valid = 1 ;
			rs2_addr_valid = 1 ;
			mem_valid = 1;
			mem_wdata = rs2_value;

			mem_addr = rs1_value + immediate_12bit_for_stores;
			if (insn[14:12] == 3'b000 ) begin // SB  store byte
				mem_wmask = 4'b0001;
			end
			if (insn[14:12] == 3'b001 ) begin // SH  store half word
				mem_wmask = 4'b0011;
			end
			if (insn[14:12] == 3'b010 ) begin // SW  store word
				mem_wmask = 4'b1111;
			end

		end

/*
		if (insn[6:0] == 7'b 00_100_11) begin // OP-IMM
			case (insn[14:12])
				3'b 001: begin // SLLI
					valid = insn[31:25] == 7'b 0000000;
				end
				3'b 101: begin // SRLI SRAI
					valid = (insn[31:25] == 7'b 0000000) || (insn[31:25] == 7'b 0100000);
				end
				default: begin
					valid = 1;
				end
			endcase
		end
*/

		if ( (insn[6:0] == 7'b 01_100_11) || (insn[6:0] == 7'b 00_100_11) ) begin // OP or OP-IMM
			is_alu_immediate = (insn[6:0] == 7'b 00_100_11);
			if ( is_alu_immediate ) begin
				rs2_value = immediate_12bit;
			end

			case (insn[14:12])
				3'b 000: begin // ADD SUB
					rs1_addr_valid = 1 ;
					rs2_addr_valid = !is_alu_immediate ;
					rd_addr_valid  = 1 ;

					if (is_alu_immediate || insn[31:25] == 7'b 0000000) begin
						valid = 1; // ADD
						rd_wdata = rs1_value + rs2_value;
					end else if ( insn[31:25] == 7'b 0100000 ) begin
						valid = 1; // SUB
						rd_wdata = rs1_value - rs2_value;
					end
				end
				3'b 101: begin // SRL SRA SRLI SRAI
					rs1_addr_valid = 1 ;
					rs2_addr_valid = !is_alu_immediate ;
					rd_addr_valid  = 1 ;

					if ( (!is_alu_immediate && insn[31:25] == 7'b 0000000 ) || ( is_alu_immediate && insn[31:25] == 7'b 0000000 ) ) begin
						valid = 1; // SRL
						rd_wdata = rs1_value >> rs2_value[4:0];
					end else if ( (!is_alu_immediate && insn[31:25] == 7'b 0100000 ) || ( is_alu_immediate && insn[31:25] == 7'b 0100000 ) ) begin
						valid = 1; // SRA
						rd_wdata = { {32{rs1_value[31]}}, rs1_value } >> rs2_value[4:0];
					end
				end
				default: begin
					if ( is_alu_immediate || insn[31:25] == 7'b 0000000 ) begin
						valid = 1;
						rs1_addr_valid = 1 ;
						rs2_addr_valid = !is_alu_immediate ;
						rd_addr_valid  = 1 ;
						case (insn[14:12])
							3'b001: begin // SLL
								rd_wdata = rs1_value << rs2_value[4:0] ;
							end
							3'b010: begin // SLT set less than
								rd_wdata = ( rs1_value ^ 32'h8000_0000 ) < ( rs2_value ^ 32'h8000_0000 ) ;
							end
							3'b011: begin // SLTU set less than unsigned
								rd_wdata = rs1_value < rs2_value ;
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
	end

endmodule
