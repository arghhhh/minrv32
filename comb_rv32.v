

// need to make code clean when this isn't defined
// for now, just define it
`define RISCV_FORMAL

module comb_rv32 #(
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
	wire [1:0] c_insn_field_opcode   = insn[1:0];
	wire [5:0] c_insn_field_funct6   = insn[15:10];
	wire [2:0] c_insn_field_funct1_2 = {insn[12], insn[6:5]};
	wire [2:0] c_insn_field_funct3   = insn[15:13];
	wire [1:0] c_insn_field_funct2   = insn[11:10];
	wire       c_insn_field_funct    = insn[12];

    wire [4:0] c_insn_field_rs1 = (insn[1:0] == 2'b00 || ({insn[15], insn[1:0]} == 3'b101 && (insn[14:13] != 2'b01))) ? {2'b01, insn[9:7]} : insn[11:7];
    wire [4:0] c_insn_field_rs2 = (insn[1:0] == 2'b00 || {insn[15:13], insn[1:0]} == 5'b10001) ? {2'b01, insn[4:2]}: insn[6:2];
    wire [4:0] c_insn_field_rd  = (insn[1:0] == 2'b00) ? c_insn_field_rs2 : c_insn_field_rs1;

    reg  [4:0] c_rs1_addr;
    reg  [4:0] c_rd_addr;

    wire [31:0] signed_immediate_6bit    = {{27{insn[12]}}, insn[6:2]};
    wire [31:0] unsigned_immediate_6bit  = {26'b0, insn[12], insn[6:2]};
    wire [31:0] immediate_LWSP           = {insn[3:2], insn[12], insn[6:4], 2'b0};
    wire [31:0] immediate_SWSP           = {insn[8:7], insn[12:9], 2'b0};
    wire [31:0] immediate_c_LW_SW        = {25'b0, insn[5], insn[12:10], insn[6], 2'b0};
    wire [31:0] immediate_ADDI4SPN       = {22'b0, insn[10:7], insn[12:11], insn[5], insn[6], 2'b0};
    wire [31:0] immediate_ADDI16SP       = {{23{insn[12]}}, insn[4:3], insn[5], insn[2], insn[6], 4'b0};
    wire [31:0] immediate_for_branches_c = {{24{insn[12]}}, insn[6:5], insn[2], insn[11:10], insn[4:3], 1'b0};
	wire [31:0] immediate_c_J            = {{21{insn[12]}}, insn[8], insn[10], insn[9], insn[6], insn[7], insn[2], insn[11], insn[5:3], 1'b0};


	reg rs1_addr_valid;
	reg rs2_addr_valid;
	reg rd_addr_valid;


	assign rs1_request = rs1_addr_valid && ( rs1_addr != 0 );
	assign rs2_request = rs2_addr_valid && ( rs2_addr != 0 );
	assign rd_request  = rd_addr_valid  && ( rd_addr != 0  );


	assign insn_addr = { pc[31:1], 1'b0 };

	assign rvfi_order = csr_instret;

	reg insn_decode_valid;
	reg gen_trap;
	assign trap = insn_valid && ( !insn_decode_valid || gen_trap );


	wire [6:0] insn_field_opcode = insn[6:0];
	wire [2:0] insn_field_funct3 = insn[14:12];
	wire [6:0] insn_field_funct7 = insn[31:25];
	wire [4:0] insn_field_rd     = insn[11:7];
	wire [4:0] insn_field_rs1    = insn[19:15];
	wire [4:0] insn_field_rs2    = insn[24:20];


	assign rs1_addr = rs1_addr_valid ? (c_insn_field_opcode == 2'b11) ? insn_field_rs1 : c_rs1_addr : 5'b0;
	assign rs2_addr = rs2_addr_valid ? (c_insn_field_opcode == 2'b11) ? insn_field_rs2 : c_insn_field_rs2 : 5'b0;
	assign rd_addr  = rd_addr_valid  ? (c_insn_field_opcode == 2'b11) ? insn_field_rd  : c_rd_addr  : 5'b0;

	reg [31:0] rs1_value ;
	reg [31:0] rs2_value ;
	reg [31:0] c_rs1_value ;
	reg [31:0] c_rs2_value ;

//	reg [3:0] mem_wmask
//	assign mem_wstrb = mem_wmask;


	assign rvfi_insn = insn;
	assign rvfi_pc_rdata   = insn_addr;
	assign rvfi_pc_wdata   = `valid_data_or_x( rvfi_valid, pc_next );

	assign rvfi_rs1_addr   = `valid_data_or_x( rvfi_valid && rs1_addr != 0, rs1_addr  );
	assign rvfi_rs2_addr   = `valid_data_or_x( rvfi_valid && rs2_addr != 0, rs2_addr  );
	assign rvfi_rd_addr    = `valid_data_or_x( rvfi_valid, rd_addr   );

	assign rvfi_rs1_rdata  = `valid_data_or_x( rvfi_valid && rs1_addr != 0, ( rs1_addr_valid ) ? ((c_insn_field_opcode == 2'b11) ? rs1_value : c_rs1_value) : 32'b0 );
	assign rvfi_rs2_rdata  = `valid_data_or_x( rvfi_valid && rs2_addr != 0, ( rs2_addr_valid ) ? ((c_insn_field_opcode == 2'b11) ? rs2_value : c_rs2_value) : 32'b0 );
	assign rvfi_rd_wdata   = `valid_data_or_x( rvfi_valid, ( rd_addr_valid && ( rd_addr != 0 ) ) ? rd_wdata  : 32'b0 );

	// even the combo version might not complete in one cycle if mem_ready is held low...
	assign rvfi_valid      = insn_complete        ;
	assign rvfi_trap       = `valid_data_or_x( rvfi_valid, trap      );
	assign rvfi_halt       = `valid_data_or_x( rvfi_valid, trap      ); // for the liveness check.. maybe revisit if trap handling is added

	assign rvfi_mem_addr   = `valid_data_or_x( rvfi_valid, mem_addr  );
	assign rvfi_mem_rmask  = `valid_data_or_x( rvfi_valid, mem_rmask );
	assign rvfi_mem_wmask  = `valid_data_or_x( rvfi_valid, mem_wstrb );
	assign rvfi_mem_rdata  = `valid_data_or_x( rvfi_valid, mem_rdata );
	assign rvfi_mem_wdata  = `valid_data_or_x( rvfi_valid, mem_wdata );

	assign rvfi_intr       = `valid_data_or_x( rvfi_valid,  0        );

	assign rvfi_csr_mcycle_rmask   = 0 ;
	assign rvfi_csr_mcycle_wmask   = 0 ;
	assign rvfi_csr_mcycle_rdata   = 0 ;
	assign rvfi_csr_mcycle_wdata   = 0 ;

	assign rvfi_csr_minstret_rmask = 0 ;
	assign rvfi_csr_minstret_wmask = 0 ;
	assign rvfi_csr_minstret_rdata = 0 ;
	assign rvfi_csr_minstret_wdata = 0 ;


	reg is_alu_immediate;

	wire [31:0] immediate_12bit            = {{20{insn[31]}}, insn[31:20]};
	wire [31:0] immediate_12bit_for_stores = {immediate_12bit[31:5], insn[11:7]};
	wire [31:0] immediate_for_jal          = {{12{insn[31]}}, insn[19:12], insn[20], insn[30:21], 1'b0};
	wire [31:0] immediate_for_branches     = {{20{insn[31]}}, insn[7], insn[ 30:25], insn[11:8], 1'b0};

	wire [31:0] pc_next_no_branch = (c_insn_field_opcode == 2'b11) ? insn_addr + 4 : insn_addr + 2;
	wire [31:0] pc_next_branch    = (insn_addr + immediate_for_branches)   & 32'hFFFF_FFFE;
	wire [31:0] pc_next_branch_c  = (insn_addr + immediate_for_branches_c) & 32'hFFFF_FFFE;


	wire cond_eq   = rs1_value == rs2_value ;
	wire cond_neq  = rs1_value != rs2_value ;
	wire cond_eqz  = rs1_value == 0 ;
	wire cond_neqz = rs1_value != 0 ;
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

		rs1_value   = (insn_field_rs1 == 0) ? 0 : rs1_rdata;
		rs2_value   = (insn_field_rs2 == 0) ? 0 : rs2_rdata;  // may override this for immediate instructions
		c_rs1_value = (c_insn_field_rs1 == 0) ? 0 : rs1_rdata;
		c_rs2_value = (c_insn_field_rs2 == 0) ? 0 : rs2_rdata;

        c_rs1_addr  = c_insn_field_rs1;
        c_rd_addr   = c_insn_field_rd;

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
				// gen_trap = |pc_next[1:0];
			end
			if (insn_field_opcode == 7'b 11_001_11) begin // JALR jump and link register
				if (  insn_field_funct3 == 3'b 000 ) begin
					insn_decode_valid = 1;
					rs1_addr_valid = 1 ;
					rd_addr_valid  = 1 ;
					rd_wdata = pc_next_no_branch;
					pc_next = ( rs1_value + immediate_12bit ) & 32'hFFFF_FFFE;
					// gen_trap = |pc_next[1:0];
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
				// gen_trap = |pc_next[1:0];
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
            /*******************Compressed Instructions****************************/
			if (c_insn_field_opcode == 2'b00) begin // C0
				case (c_insn_field_funct3) 
					3'b000: begin // C.ADDI4SPN
                        c_rs1_addr = 5'b00010;
                        rs1_addr_valid = 1;
                        rd_addr_valid = 1;
                        insn_decode_valid = 1;
                        rd_wdata = c_rs1_value + immediate_ADDI4SPN;
					end
					3'b010: begin // C.LW
						insn_decode_valid = 1;
						mem_valid = 1;
						rs1_addr_valid = 1;
						rd_addr_valid  = 1;  
						rd_wdata = mem_rdata;
						mem_rmask = 4'b1111;
						mem_addr = c_rs1_value + immediate_c_LW_SW;
					end
					3'b110: begin // C.SW				
                        insn_decode_valid = 1;
						mem_valid = 1;
						rs1_addr_valid = 1;
						rs2_addr_valid = 1;
						mem_wdata = c_rs2_value;
						mem_addr = c_rs1_value + immediate_c_LW_SW;
						mem_wstrb = 4'b1111;
					end
					default: begin
						insn_decode_valid = 0;
						mem_valid = 0;
						rs1_addr_valid = 0;
						rd_addr_valid  = 0;  
						rd_wdata = 0;
						mem_rmask = 0;
						mem_addr = 0;						
					end
				endcase
			end
			else if (c_insn_field_opcode == 2'b01) begin // C1
				case (c_insn_field_funct3) 
					3'b000: begin // C.ADDI
                        rs1_addr_valid = 1;
                        rd_addr_valid  = 1;
                        insn_decode_valid = 1;
                        rd_wdata = c_rs1_value + signed_immediate_6bit;
					end
					3'b001: begin // C.JAL
						c_rd_addr = 5'b00001;
						insn_decode_valid = 1;
						rd_addr_valid  = 1;
						rd_wdata = pc_next_no_branch;
						pc_next = insn_addr + immediate_c_J;
						pc_next_valid = insn_complete;
					end
					3'b010: begin // C.LI 
						rd_addr_valid = 1;
						insn_decode_valid = 1;
						rd_wdata = signed_immediate_6bit;
					end
					3'b011: begin // C.LUI or C.ADDI16SP
                        if (c_insn_field_rd == 2) begin // C.ADDI16SP
                            c_rd_addr = 5'b00010;
                            rs1_addr_valid = 1;
                            rd_addr_valid = 1;
                            insn_decode_valid = 1;
                            rd_wdata = c_rs1_value + immediate_ADDI16SP;
                        end else begin // C.LUI
						    rd_addr_valid = 1;
                            insn_decode_valid = 1;
                            rd_wdata = {signed_immediate_6bit[19:0], 12'b0};
                        end
					end
					3'b100: begin
						case (c_insn_field_funct2)
							2'b00: begin // C.SRLI
								rs1_addr_valid = 1;
								rd_addr_valid  = 1;
								insn_decode_valid = 1;  
								rd_wdata = c_rs1_value >> {16'b0, unsigned_immediate_6bit[4:0]};
							end
							2'b01: begin // C.SRAI
								if(unsigned_immediate_6bit[5] == 0) begin
									rs1_addr_valid = 1;
									rd_addr_valid  = 1;
									insn_decode_valid = 1; 
									rd_wdata = ({{32{rs1_value[31]}}, c_rs1_value} >> unsigned_immediate_6bit[4:0]);
								end
							end
							2'b10:  begin // C.ANDI
							    rs1_addr_valid = 1;
								rd_addr_valid  = 1;
								insn_decode_valid = 1;
								rd_wdata = c_rs1_value & signed_immediate_6bit;
							end
							2'b11: begin
								case (c_insn_field_funct1_2)
									3'b000: begin // C.SUB
										rs1_addr_valid = 1;
										rs2_addr_valid = 1;
										rd_addr_valid = 1;
										insn_decode_valid = 1;
										rd_wdata = c_rs1_value - c_rs2_value;
									end
									3'b001: begin // C.XOR
										rs1_addr_valid = 1;
										rs2_addr_valid = 1;
										rd_addr_valid = 1;
										insn_decode_valid = 1;
										rd_wdata = c_rs1_value ^ c_rs2_value;
									end
									3'b010: begin // C.OR
										rs1_addr_valid = 1;
										rs2_addr_valid = 1;
										rd_addr_valid = 1;
										insn_decode_valid = 1;
										rd_wdata = c_rs1_value | c_rs2_value;
									end
									3'b011: begin // C.AND
										rs1_addr_valid = 1;
										rs2_addr_valid = 1;
										rd_addr_valid = 1;
										insn_decode_valid = 1;
										rd_wdata = c_rs1_value & c_rs2_value;
									end
									default: begin
										rs1_addr_valid = 0;
										rs2_addr_valid = 0;
										rd_addr_valid = 0;
										insn_decode_valid = 0;
										rd_wdata = 32'b0;
									end
								endcase
							end
						endcase
					end
					3'b101: begin // C.J
						insn_decode_valid = 1; 
						pc_next = insn_addr + immediate_c_J;
						pc_next_valid = insn_complete;
					end
					3'b110: begin // C.BEZ
						insn_decode_valid = 1;
						rs1_addr_valid = 1 ;
						if(cond_eqz) begin
							pc_next = pc_next_branch_c;
						end
						pc_next_valid = insn_complete;
					end
					3'b111: begin // C.BNEZ 
						insn_decode_valid = 1;
						rs1_addr_valid = 1 ;
						if(cond_neqz) begin
							pc_next = pc_next_branch_c;
						end
						pc_next_valid = insn_complete;
					end
				endcase
			end
			else if (c_insn_field_opcode == 2'b10) begin // C2
				case (c_insn_field_funct3) 
					3'b000: begin // C.SLLI
                        if (unsigned_immediate_6bit[5] == 0) begin
                            rs1_addr_valid = 1;
                            rd_addr_valid  = 1;
                            insn_decode_valid = 1;  
                            rd_wdata = c_rs1_value << unsigned_immediate_6bit[4:0];
                        end
					end
					3'b010: begin // C.LWSP
						c_rs1_addr = 5'b00010;
						insn_decode_valid = 1;
						mem_valid = 1;
						rs1_addr_valid = 1;
						rd_addr_valid  = 1;  
						mem_addr = c_rs1_value + immediate_LWSP;
						mem_rmask = 4'b1111;
						rd_wdata = mem_rdata;
					end
					3'b100: begin
						case (c_insn_field_funct)
							1'b0: begin // C.JR or C.MV
								if (insn[6:2] == 5'b00000) begin  // C.JR
									rs1_addr_valid = 1;
									insn_decode_valid = 1; 
									pc_next = c_rs1_value & 32'hFFFF_FFFE;
									pc_next_valid = insn_complete;
								end else begin	                  // C.MV
									rs2_addr_valid = 1;
									rd_addr_valid = 1;
									insn_decode_valid = 1;
									rd_wdata = c_rs2_value;
								end
							end 
							1'b1: begin // C.JALR or C.ADD
                                if (c_insn_field_rs2 == 0) begin // C.JALR
                                    c_rd_addr = 5'b00001;
                                    rs1_addr_valid = 1;
                                    rd_addr_valid = 1;
                                    rd_wdata = pc_next_no_branch;
                                    insn_decode_valid = 1;
                                    pc_next = c_rs1_value & 32'hFFFF_FFFE;
                                    pc_next_valid = insn_complete;
                                end else begin                   // C.ADD
                                    rs1_addr_valid = 1;
                                    rs2_addr_valid = 1;
                                    rd_addr_valid  = 1;
                                    insn_decode_valid = 1;
                                    rd_wdata = c_rs1_value + c_rs2_value;
                                end
							end
						endcase
					end
					3'b110: begin // C.SWSP
						c_rs1_addr = 5'b00010;
						insn_decode_valid = 1;
						mem_valid = 1;
						rs1_addr_valid = 1;
						rs2_addr_valid = 1;
						mem_addr = c_rs1_value + immediate_SWSP;
						mem_wdata = c_rs2_value;
						mem_wstrb = 4'b1111;
					end
					default: begin
						c_rs1_addr = 0;
						insn_decode_valid = 0;
						mem_valid = 0;
						rs1_addr_valid = 0;
						rd_addr_valid  = 0;  
						mem_addr = 0;
						mem_rmask = 0;
						rd_wdata = 0;
					end
				endcase			
			end
            /**********************************************************************/
		end
		if ( trap ) begin
			pc_next = pc; // this will lock up the simulation, but prevents it doing stuff it shouldn't
		end
	end

endmodule
