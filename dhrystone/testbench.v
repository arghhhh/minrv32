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

	wire [31:0] insn_addr;
	wire [31:0] insn;

	reg insn_valid = 0;

	minrv32 #(
//		.BARREL_SHIFTER(1),
//		.ENABLE_FAST_MUL(1),
//		.ENABLE_DIV(1),
//		.PROGADDR_RESET('h10000),
//		.STACKADDR('h10000)
	) uut (
		.clk( clk )
		, .resetn( resetn )


		,  .trap        (trap       )
		, .mem_valid   (mem_valid  )
		, .mem_instr   (mem_instr  )
		, .mem_ready   (mem_ready  )
		, .mem_addr    (mem_addr   )
		, .mem_wdata   (mem_wdata  )
		, .mem_rmask   (mem_rmask  )
		, .mem_wstrb   (mem_wstrb  )
		, .mem_rdata   (mem_rdata  )

		, .insn_addr   ( insn_addr )

		, .insn( insn )
		, .insn_valid( insn_valid )

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

	always @(posedge clk ) insn_valid = 1;// $random;

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
