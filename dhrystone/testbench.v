`timescale 1 ns / 1 ps

module testbench;
	reg clk = 1;
	reg resetn = 0;
	wire trap;

	always #5 clk = ~clk;

	initial begin
		repeat (100) @(posedge clk);
		resetn <= 1;
	end

	wire mem_valid;
	wire mem_instr;
	wire mem_ready;
	wire [31:0] mem_addr;
	wire [31:0] mem_wdata;
	wire [3:0] mem_wstrb;
	wire [3:0] mem_rmask;
	wire [31:0] mem_rdata;

	reg  [31:0] pc;
	wire [31:0] pc_next;
	wire [31:0] insn_addr;

	wire [31:0] insn;



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


initial pc = PROGADDR_RESET;
always @(posedge clk) begin
	pc <= resetn ? pc_next : PROGADDR_RESET;
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


reg [31:0] registers [ 0:31 ];
always @(posedge clk) begin
	if ( !resetn ) begin
		registers[ 2] <= STACKADDR;
	end
	if ( rd_addr_valid ) begin
		registers[ rd_addr ] <= rd_wdata;
	end
end

assign rs1_rdata = registers[ rs1_addr ];
assign rs2_rdata = registers[ rs2_addr ];


	minrv32 #(
		.BARREL_SHIFTER(1),
		.ENABLE_FAST_MUL(1),
		.ENABLE_DIV(1),
		.PROGADDR_RESET('h10000),
		.STACKADDR('h10000)
	) uut (
		.clk         (clk        ),
		.resetn      (resetn     ),
		.trap        (trap       ),
		.mem_valid   (mem_valid  ),
		.mem_instr   (mem_instr  ),
		.mem_ready   (mem_ready  ),
		.mem_addr    (mem_addr   ),
		.mem_wdata   (mem_wdata  ),
		.mem_rmask   (mem_rmask  ),
		.mem_wstrb   (mem_wstrb  ),
		.mem_rdata   (mem_rdata  ),

		  .pc        ( pc )
		, .pc_next   ( pc_next )
		, .insn_addr ( insn_addr )

		, .insn( insn )

		, .csr_cycle  ( csr_cycle   )
		, .csr_time   ( csr_time    )
		, .csr_instret( csr_instret )

		, .rs1_addr        (  rs1_addr       )
		, .rs2_addr        (  rs2_addr       )
		, .rd_addr         (  rd_addr        )
		, .rs1_addr_valid  (  rs1_addr_valid )
		, .rs2_addr_valid  (  rs2_addr_valid )
		, .rd_addr_valid   (  rd_addr_valid  )
		, .rs1_rdata       (  rs1_rdata      )
		, .rs2_rdata       (  rs2_rdata      )
		, .rd_wdata        (  rd_wdata       )


//		.mem_la_read (mem_la_read ),
//		.mem_la_write(mem_la_write),
//		.mem_la_addr (mem_la_addr ),
//		.mem_la_wdata(mem_la_wdata),
//		.mem_la_wstrb(mem_la_wstrb)
	);

	reg [7:0] memory [0:256*1024-1];
	initial $readmemh("dhry.hex", memory);

	assign mem_ready = 1;

// unrealistic memory model - combinatorial read, single cycle write....
// extra read port for the instruction!

	assign mem_rdata[ 7: 0] = mem_rmask[0] ? memory[mem_addr + 0] : 'bx;
	assign mem_rdata[15: 8] = mem_rmask[1] ? memory[mem_addr + 1] : 'bx;
	assign mem_rdata[23:16] = mem_rmask[2] ? memory[mem_addr + 2] : 'bx;
	assign mem_rdata[31:24] = mem_rmask[3] ? memory[mem_addr + 3] : 'bx;

	assign insn[ 7: 0] = memory[insn_addr + 0] ;
	assign insn[15: 8] = memory[insn_addr + 1] ;
	assign insn[23:16] = memory[insn_addr + 2] ;
	assign insn[31:24] = memory[insn_addr + 3] ;

	always @(posedge clk) begin
		if (mem_wstrb != 0 ) begin
			case (mem_addr)
				32'h1000_0000: begin
`ifndef TIMING
					$write("%c", mem_wdata);
					$fflush();
`endif
				end
				default: begin
					if (mem_wstrb[0]) memory[mem_addr + 0] <= mem_wdata[ 7: 0];
					if (mem_wstrb[1]) memory[mem_addr + 1] <= mem_wdata[15: 8];
					if (mem_wstrb[2]) memory[mem_addr + 2] <= mem_wdata[23:16];
					if (mem_wstrb[3]) memory[mem_addr + 3] <= mem_wdata[31:24];
				end
			endcase
		end
	end

	initial begin
		$dumpfile("testbench.vcd");
		$dumpvars(0, testbench);
	end

	always @(posedge clk) begin
		if (resetn && trap) begin
			repeat (10) @(posedge clk);
			$display("TRAP");
			$finish;
		end
	end

`ifdef TIMING
	initial begin
		repeat (100000) @(posedge clk);
		$finish;
	end
//	always @(posedge clk) begin
//		if (uut.dbg_next)
//			$display("## %-s %d", uut.dbg_ascii_instr ? uut.dbg_ascii_instr : "pcpi", uut.count_cycle);
//	end
`endif
endmodule
