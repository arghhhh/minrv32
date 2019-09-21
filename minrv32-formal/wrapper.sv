

module rvfi_wrapper (
	input         clock,
	input         reset,
	`RVFI_OUTPUTS
);
	(* keep *) wire trap;

	(* keep *) `rvformal_rand_reg mem_ready;
	(* keep *) `rvformal_rand_reg [31:0] mem_rdata;

	(* keep *) wire        mem_valid;
	(* keep *) wire        mem_instr;
	(* keep *) wire [31:0] mem_addr;
	(* keep *) wire [31:0] mem_wdata;
	(* keep *) wire [3:0]  mem_wstrb;

	reg [31:0] pc;
	wire [31:0] pc_next;

localparam PROGADDR_RESET = 'h10000;


always @(posedge clk) begin
	pc <= (!reset) ? pc_next : PROGADDR_RESET;
end


localparam STACKADDR      = 'h10000;


wire  [ 4:0] rs1_addr         ;
wire  [ 4:0] rs2_addr         ;
wire  [ 4:0] rd_addr          ;
wire         rs1_addr_valid   ;
wire         rs2_addr_valid   ;
wire         rd_addr_valid    ;
wire  [31:0] rd_wdata         ;
wire  [31:0] rs1_rdata        ;
wire  [31:0] rs2_rdata        ;


reg [31:0] registers [ 0:31 ];
always @(posedge clk) begin
	if ( reset ) begin
		registers[ 2] <= STACKADDR;
	end
	if ( rd_addr_valid ) begin
		registers[ rd_addr ] <= rd_wdata;
	end
end


// `ifndef RISCV_FORMAL_BLACKBOX_REGS
	assign rs1_rdata = registers[ rs1_addr ];
	assign rs2_rdata = registers[ rs2_addr ];
// `else
//	rs1_rdata = decoded_rs1 ? $anyseq : 0;
//	rs2_rdata = decoded_rs2 ? $anyseq : 0;
// `endif


reg [63:0]  csr_instret;

always @(posedge clk) begin
	if ( reset ) begin
		csr_instret <= 0 ;
	end else begin
		csr_instret <= csr_instret + 1 ;
	end
end


	minrv32 #(
		.COMPRESSED_ISA(0),
		.ENABLE_FAST_MUL(1),
		.ENABLE_DIV(1),
		.BARREL_SHIFTER(1)
	) uut (
//		  .clk       (clock    )
//		, .resetn    (!reset   )
		  .trap      (trap     )

		, .mem_valid (mem_valid)
		, .mem_instr (mem_instr)
		, .mem_ready (mem_ready)
		, .mem_addr  (mem_addr )
		, .mem_wdata (mem_wdata)
		, .mem_wstrb (mem_wstrb)
		, .mem_rdata (mem_rdata)

		, .pc        ( pc )
		, .pc_next   ( pc_next )

		, .rs1_addr        (  rs1_addr       )
		, .rs2_addr        (  rs2_addr       )
		, .rd_addr         (  rd_addr        )
		, .rs1_addr_valid  (  rs1_addr_valid )
		, .rs2_addr_valid  (  rs2_addr_valid )
		, .rd_addr_valid   (  rd_addr_valid  )
		, .rs1_rdata       (  rs1_rdata      )
		, .rs2_rdata       (  rs2_rdata      )
		, .rd_wdata        (  rd_wdata       )

		, .csr_cycle  (    )
		, .csr_time   (     )
		, .csr_instret( csr_instret )



		, `RVFI_CONN
	);

`ifdef PICORV32_FAIRNESS
	reg [2:0] mem_wait = 0;
	always @(posedge clock) begin
		mem_wait <= {mem_wait, mem_valid && !mem_ready};
		assume (~mem_wait || trap);
	end
`endif

`ifdef PICORV32_CSR_RESTRICT
	always @* begin
		if (rvfi_valid && rvfi_insn[6:0] == 7'b1110011) begin
			if (rvfi_insn[14:12] == 3'b010) begin
				assume (rvfi_insn[31:20] == 12'hC00 || rvfi_insn[31:20] == 12'hC01 || rvfi_insn[31:20] == 12'hC02 ||
						rvfi_insn[31:20] == 12'hC80 || rvfi_insn[31:20] == 12'hC81 || rvfi_insn[31:20] == 12'hC82);
				assume (rvfi_insn[19:15] == 0);
			end
			assume (rvfi_insn[14:12] != 3'b001);
			assume (rvfi_insn[14:12] != 3'b011);
			assume (rvfi_insn[14:12] != 3'b101);
			assume (rvfi_insn[14:12] != 3'b110);
			assume (rvfi_insn[14:12] != 3'b111);
		end
	end
`endif
endmodule

