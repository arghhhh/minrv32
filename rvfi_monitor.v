



// decode the instruction to check that the rvfi supplied fields are correct

// 

module rvfi_monitor (


    input          clk,
    input          reset,

	input          rvfi_valid,
	input   [63:0] rvfi_order,
	input   [31:0] rvfi_insn,
	input          rvfi_trap,
	input          rvfi_halt,
	input          rvfi_intr,
	input   [ 1:0] rvfi_mode,
	input   [ 1:0] rvfi_ixl,
	input   [ 4:0] rvfi_rs1_addr,
	input   [ 4:0] rvfi_rs2_addr,
	input   [31:0] rvfi_rs1_rdata,
	input   [31:0] rvfi_rs2_rdata,
	input   [ 4:0] rvfi_rd_addr,
	input   [31:0] rvfi_rd_wdata,
	input   [31:0] rvfi_pc_rdata,
	input   [31:0] rvfi_pc_wdata,

	input   [31:0] rvfi_mem_addr,
	input   [ 3:0] rvfi_mem_rmask,
	input   [ 3:0] rvfi_mem_wmask,
	input   [31:0] rvfi_mem_rdata,
	input   [31:0] rvfi_mem_wdata,

	input   [63:0] rvfi_csr_mcycle_rmask,
	input   [63:0] rvfi_csr_mcycle_wmask,
	input   [63:0] rvfi_csr_mcycle_rdata,
	input   [63:0] rvfi_csr_mcycle_wdata,

	input   [63:0] rvfi_csr_minstret_rmask,
	input   [63:0] rvfi_csr_minstret_wmask,
	input   [63:0] rvfi_csr_minstret_rdata,
	input   [63:0] rvfi_csr_minstret_wdata

);



// should measure cycle count per instruction


wire             expect_rvfi_valid        ;
wire      [63:0] expect_rvfi_order        ;
wire      [31:0] expect_rvfi_insn         ;
wire             expect_rvfi_trap         ;
wire             expect_rvfi_halt         ;
wire             expect_rvfi_intr         ;
wire      [ 1:0] expect_rvfi_mode         ;
wire      [ 1:0] expect_rvfi_ixl          ;
wire      [ 4:0] expect_rvfi_rs1_addr     ;
wire      [ 4:0] expect_rvfi_rs2_addr     ;
wire      [31:0] expect_rvfi_rs1_rdata    ;
wire      [31:0] expect_rvfi_rs2_rdata    ;
wire      [ 4:0] expect_rvfi_rd_addr      ;
wire      [31:0] expect_rvfi_rd_wdata     ;
wire      [31:0] expect_rvfi_pc_rdata     ;
wire      [31:0] expect_rvfi_pc_wdata     ;
wire      [31:0] expect_rvfi_mem_addr     ;
wire      [ 3:0] expect_rvfi_mem_rmask    ;
wire      [ 3:0] expect_rvfi_mem_wmask    ; 
wire      [31:0] expect_rvfi_mem_rdata    ;
wire      [31:0] expect_rvfi_mem_wdata    ;
wire      [63:0] expect_rvfi_csr_mcycle_rmask      ;
wire      [63:0] expect_rvfi_csr_mcycle_wmask      ;
wire      [63:0] expect_rvfi_csr_mcycle_rdata      ;
wire      [63:0] expect_rvfi_csr_mcycle_wdata      ;
wire      [63:0] expect_rvfi_csr_minstret_rmask    ;
wire      [63:0] expect_rvfi_csr_minstret_wmask    ;
wire      [63:0] expect_rvfi_csr_minstret_rdata    ;
wire      [63:0] expect_rvfi_csr_minstret_wdata    ;

wire      [31:0] expect_pc_next                  ;
wire      [31:0] expect_insn_addr                ;
wire      [ 4:0] expect_rs1_addr                 ;
wire      [ 4:0] expect_rs2_addr                 ;
wire      [ 4:0] expect_rd_addr                 ;

wire             expect_rs1_request             ;
wire             expect_rs2_request             ;
wire             expect_rd_request              ;



wire      [31:0] shadow_rs1_rdata               ;
wire      [31:0] shadow_rs2_rdata               ;
wire      [31:0] expect_rd_wdata                ;

wire             expect_mem_valid               ;
wire             expect_mem_instr               ;
wire      [31:0] expect_mem_addr                ;
wire      [31:0] expect_mem_wdata               ;
wire      [ 3:0] expect_mem_wstrb               ;
wire      [ 3:0] expect_mem_rmask               ;

wire      [63:0] expect_csr_time                ;
wire      [63:0] expect_csr_cycle               ;
wire      [63:0] expect_csr_instret             ;



integer   fd;
initial begin
    fd = $fopen( "rvfi_mon.txt", "w" );
end

wire  insn_valid = 1 ;  // for now, insn is always valid




reg [63:0]  csr_cycle;
reg [63:0]  csr_time;
reg [63:0]  csr_instret;

always @(posedge clk) begin
	if ( reset ) begin
		csr_cycle   <= 0 ;
		csr_time    <= 0 ; 
		csr_instret <= 0 ;
	end else begin
		csr_cycle   <= csr_cycle   + 1 ;
		csr_time    <= csr_time    + 1 ; 
		if (rvfi_valid) begin
            csr_instret <= csr_instret + 1 ;
        end
	end
end

integer instruction_cycles;
integer instruction_cycles_next;
always @(*) begin
	instruction_cycles_next = reset ? 0 : rvfi_valid ? 1 : (instruction_cycles+1) ; 
end
always @(posedge clk) begin
	instruction_cycles <= instruction_cycles_next;
end

localparam PROGADDR_RESET = 'h10000;
localparam STACKADDR      = 'h10000;

// wire insn_complete;

reg [31:0] pc;
// wire [31:0] pc_next;


/*
wire  [ 4:0] rs1_addr         ;
wire  [ 4:0] rs2_addr         ;
wire  [ 4:0] rd_addr          ;
wire         rs1_addr_valid   ;
wire         rs2_addr_valid   ;
wire         rd_addr_valid    ;
wire  [31:0] rd_wdata         ;
wire  [31:0] rs1_rdata        ;
wire  [31:0] rs2_rdata        ;
*/


// shadow copy of RISC-V state : PC + Regs

initial pc = PROGADDR_RESET;
always @(posedge clk) begin
	pc <= reset ? PROGADDR_RESET : ( expect_insn_complete ? expect_pc_next : pc ) ;
end

integer i;
reg [31:0] registers [ 0:31 ];
always @(posedge clk) begin
	if ( reset ) begin
		registers[ 0] <= 0;
        registers[ 1] <= 32'hx;
		registers[ 2] <= STACKADDR;
        for( i=3; i <32; i=i+1 ) begin
            registers[i] <= 32'hx;
        end
	end
	if ( expect_rd_request ) begin
		registers[ expect_rd_addr ] <= expect_rd_wdata;
	end
end

assign shadow_rs1_rdata = expect_rs1_request ? registers[ expect_rs1_addr ] : 32'bx;
assign shadow_rs2_rdata = expect_rs2_request ? registers[ expect_rs2_addr ] : 32'bx;


wire [3:0] mem_wstrb;







comb_rv32 comb_rv32 (

	  .trap            ( expect_trap           )
	, .pc              ( rvfi_pc_rdata         ) // from incoming RVFI
	, .pc_next         ( expect_pc_next        )
	, .pc_next_valid   ( expect_pc_next_valid  )
	, .insn_addr       ( expect_insn_addr      )
	, .insn            ( rvfi_insn             )  //
	, .insn_valid      ( rvfi_valid            )  // everything predicated on incoming RVFI valid
	, .insn_complete   ( expect_insn_complete  )
	, .rs1_addr        ( expect_rs1_addr       )
	, .rs2_addr        ( expect_rs2_addr       )
	, .rd_addr         ( expect_rd_addr        )
	, .rs1_addr_valid  ( expect_rs1_addr_valid )
	, .rs2_addr_valid  ( expect_rs2_addr_valid )
	, .rd_addr_valid   ( expect_rd_addr_valid  )

	, .rs1_request     ( expect_rs1_request           )   // rs1_addr_valid && rs1_addr!=0
	, .rs2_request     ( expect_rs2_request           )
	, .rd_request      ( expect_rd_request            )

	, .rs1_rdata       ( shadow_rs1_rdata      )  //
	, .rs2_rdata       ( shadow_rs2_rdata      )  //
	, .rd_wdata        ( expect_rd_wdata       )
	, .rs1_ready       (             1'b1      )  //
	, .rs2_ready       (             1'b1      )  //
	, .rd_ready        (             1'b1      )  //
	, .mem_valid       ( expect_mem_valid      )
	, .mem_instr       ( expect_mem_instr      )
	, .mem_ready       (             1'b1      )  //
	, .mem_addr        ( expect_mem_addr       )
	, .mem_wdata       ( expect_mem_wdata      )
	, .mem_wstrb       ( expect_mem_wstrb      )
	, .mem_rmask       ( expect_mem_rmask      )
	, .mem_rdata       (   rvfi_mem_rdata      )  // not shadowing memory - so take trust RVFI
	, .csr_time        (        csr_time       )  //
	, .csr_cycle       (        csr_cycle      )  //
	, .csr_instret     (        csr_instret    )  //


// not sure that this is necessary.....
	, .rvfi_valid              ( expect_rvfi_valid                  )
	, .rvfi_order              ( expect_rvfi_order                  )
	, .rvfi_insn               ( expect_rvfi_insn                   )
	, .rvfi_trap               ( expect_rvfi_trap                   )
	, .rvfi_halt               ( expect_rvfi_halt                   )
	, .rvfi_intr               ( expect_rvfi_intr                   )
	, .rvfi_mode               ( expect_rvfi_mode                   )
	, .rvfi_ixl                ( expect_rvfi_ixl                    )
	, .rvfi_rs1_addr           ( expect_rvfi_rs1_addr               )
	, .rvfi_rs2_addr           ( expect_rvfi_rs2_addr               )
	, .rvfi_rs1_rdata          ( expect_rvfi_rs1_rdata              )
	, .rvfi_rs2_rdata          ( expect_rvfi_rs2_rdata              )
	, .rvfi_rd_addr            ( expect_rvfi_rd_addr                )
	, .rvfi_rd_wdata           ( expect_rvfi_rd_wdata               )
	, .rvfi_pc_rdata           ( expect_rvfi_pc_rdata               )
	, .rvfi_pc_wdata           ( expect_rvfi_pc_wdata               )
	, .rvfi_mem_addr           ( expect_rvfi_mem_addr               )
	, .rvfi_mem_rmask          ( expect_rvfi_mem_rmask              )
	, .rvfi_mem_wmask          ( expect_rvfi_mem_wmask              )
	, .rvfi_mem_rdata          ( expect_rvfi_mem_rdata              )
	, .rvfi_mem_wdata          ( expect_rvfi_mem_wdata              )
	, .rvfi_csr_mcycle_rmask   ( expect_rvfi_csr_mcycle_rmask       )
	, .rvfi_csr_mcycle_wmask   ( expect_rvfi_csr_mcycle_wmask       )
	, .rvfi_csr_mcycle_rdata   ( expect_rvfi_csr_mcycle_rdata       )
	, .rvfi_csr_mcycle_wdata   ( expect_rvfi_csr_mcycle_wdata       )
	, .rvfi_csr_minstret_rmask ( expect_rvfi_csr_minstret_rmask     )
	, .rvfi_csr_minstret_wmask ( expect_rvfi_csr_minstret_wmask     )
	, .rvfi_csr_minstret_rdata ( expect_rvfi_csr_minstret_rdata     )
	, .rvfi_csr_minstret_wdata ( expect_rvfi_csr_minstret_wdata     )

);


reg  [5*8-1:0] mnemonic = "???";









	assign insn_addr = { pc[31:1], 1'b0 };

//	assign rvfi_order = csr_instret;

	reg insn_decode_valid;
	//reg gen_trap;
//	assign trap = !insn_decode_valid || gen_trap;

	wire [31:0] insn = rvfi_insn;

	wire [6:0] insn_field_opcode = insn[6:0];
	wire [2:0] insn_field_funct3 = insn[14:12];
	wire [6:0] insn_field_funct7 = insn[31:25];
	wire [4:0] insn_field_rd     = insn[11:7];
	wire [4:0] insn_field_rs1    = insn[19:15];
	wire [4:0] insn_field_rs2    = insn[24:20];



//	reg [3:0] mem_wmask;
//	assign mem_wstrb = mem_wmask;

// will decode the instruction into expected_rvfi_* fields
// which will then be asserted against the supplied rvfi_* fields

/*
	reg            expected_rvfi_valid          ;
	reg     [63:0] expected_rvfi_order          ;
	reg     [31:0] expected_rvfi_insn           ;
	reg            expected_rvfi_trap           ;
	reg            expected_rvfi_halt           ;
	reg            expected_rvfi_intr           ;
	reg     [ 1:0] expected_rvfi_mode           ;
	reg     [ 1:0] expected_rvfi_ixl            ;
	reg     [ 4:0] expected_rvfi_rs1_addr       ;
	reg     [ 4:0] expected_rvfi_rs2_addr       ;
	reg     [31:0] expected_rvfi_rs1_rdata      ;
	reg     [31:0] expected_rvfi_rs2_rdata      ;
	reg     [ 4:0] expected_rvfi_rd_addr        ;
	reg     [31:0] expected_rvfi_rd_wdata       ;
	reg     [31:0] expected_rvfi_pc_rdata       ;
	reg     [31:0] expected_rvfi_pc_wdata       ;

	reg     [31:0] expected_rvfi_mem_addr       ;
	reg     [ 3:0] expected_rvfi_mem_rmask      ;
	reg     [ 3:0] expected_rvfi_mem_wmask      ;
	reg     [31:0] expected_rvfi_mem_rdata      ;
	reg     [31:0] expected_rvfi_mem_wdata      ;
*/


	reg is_alu_immediate;

	wire [31:0] immediate_12bit = { {20{insn[31]}}, insn[31:20] };
	wire [31:0] immediate_12bit_for_stores = { immediate_12bit[31:5], insn[11:7] };
	wire [31:0] immediate_for_jal = { {12{insn[31]}}, insn[19:12], insn[20], insn[30:21], 1'b0 };
	wire [31:0] immediate_for_branches = { {20{insn[31]}}, insn[7], insn[ 30:25], insn[11:8], 1'b0 };

	wire [31:0] pc_next_no_branch = insn_addr + 4;
	wire [31:0] pc_next_branch = ( insn_addr + immediate_for_branches ) & 32'hFFFF_FFFE;


wire [8*4*8:0] branch_mnemonics = "BGEUBLTUBGE BLT         BNE BEQ ";
//					   ( ( insn_field_funct3 == 3'b000 ) && cond_eq  ) // BEQ branch equal
//					|| ( ( insn_field_funct3 == 3'b001 ) && cond_neq ) // BNE branch not equal
//					|| ( ( insn_field_funct3 == 3'b100 ) && cond_lt  ) // BLT branch less than
//					|| ( ( insn_field_funct3 == 3'b101 ) && cond_ge  ) // BGE branch greater or equal
//					|| ( ( insn_field_funct3 == 3'b110 ) && cond_ltu ) // BLTU branch less than unsigned
//					|| ( ( insn_field_funct3 == 3'b111 ) && cond_geu ) // BGEU branch greater or equal unsigned

integer immediate;
reg     has_immediate;

reg     is_load;
reg     is_store;

	always @* begin
//		insn_decode_valid = 0;
//		gen_trap = 0;
//		rs1_addr_valid = 0 ;
//		rs2_addr_valid = 0 ;
//		rd_addr_valid  = 0 ;
//		rd_wdata = 32'b0;
//		pc_next = pc_next_no_branch;
//		pc_next_valid = 0;  // not valid until insn_valid
//
		is_alu_immediate = 0;
		mnemonic = "???";
		immediate = 0;
		has_immediate = 0;
		is_load = 0;
		is_store = 0;

//
//		rs1_value = ( insn_field_rs1 == 0 ) ? 0 : rs1_rdata;
//		rs2_value = ( insn_field_rs2 == 0 ) ? 0 : rs2_rdata;  // may override this for immediate instructions
//
//		mem_valid = 0;
//		mem_instr = 0;
//		mem_addr  = 0;
//		mem_wdata = 0;
//		mem_rmask = 0;
//		mem_wstrb = 0;
//
		if ( insn_valid ) begin 
//			pc_next_valid = 1;  // by default, valid, except for jumps and branches
			if (insn_field_opcode == 7'b 01_101_11) begin
				insn_decode_valid = 1; // LUI load upper immediate
				mnemonic = "LUI";
//				rd_addr_valid  = 1 ;
//				rd_wdata = { insn[31:12], 12'b0 };
				has_immediate = 1;
				immediate = insn[31:12];
			end
			if (insn_field_opcode == 7'b 00_101_11) begin
				insn_decode_valid = 1; // AUIPC add upper immediate program counter
				mnemonic = "AUIPC";
//				rd_addr_valid  = 1 ;
//				rd_wdata = { insn[31:12], 12'b0 } + insn_addr;
				has_immediate = 1;
				immediate = insn[31:12];

			end
			if (insn_field_opcode == 7'b 11_011_11) begin
				insn_decode_valid = 1; // JAL jump and link
				mnemonic = "JAL";
//				rd_addr_valid  = 1 ;
//				rd_wdata = pc_next_no_branch;
//				pc_next = insn_addr + immediate_for_jal;
//				pc_next_valid = insn_complete;
//				gen_trap = |pc_next[1:0];
				has_immediate = 1;
				immediate = immediate_for_jal;

			end
			if (insn_field_opcode == 7'b 11_001_11) begin // JALR jump and link register
				if (  insn_field_funct3 == 3'b 000 ) begin
					insn_decode_valid = 1;
					mnemonic = "JALR";
//					rs1_addr_valid = 1 ;
//					rd_addr_valid  = 1 ;
//					rd_wdata = pc_next_no_branch;
//					pc_next = ( rs1_value + immediate_12bit ) & 32'hFFFF_FFFE;
//					gen_trap = |pc_next[1:0];
					has_immediate = 1;
					immediate = immediate_12bit;

				end
			end

			if (insn_field_opcode == 7'b 11_000_11) begin // BRANCH
				insn_decode_valid = (insn_field_funct3 != 3'b 010) && (insn_field_funct3 != 3'b 011);
//				rs1_addr_valid = 1 ;
//				rs2_addr_valid = 1 ;

				if ( insn_decode_valid ) begin
					mnemonic = branch_mnemonics[ 4*8*insn_field_funct3 +: 4*8];
					has_immediate = 1;
					immediate = immediate_for_branches;
//				if (
//					   ( ( insn_field_funct3 == 3'b000 ) && cond_eq  ) // BEQ branch equal
//					|| ( ( insn_field_funct3 == 3'b001 ) && cond_neq ) // BNE branch not equal
//					|| ( ( insn_field_funct3 == 3'b100 ) && cond_lt  ) // BLT branch less than
//					|| ( ( insn_field_funct3 == 3'b101 ) && cond_ge  ) // BGE branch greater or equal
//					|| ( ( insn_field_funct3 == 3'b110 ) && cond_ltu ) // BLTU branch less than unsigned
//					|| ( ( insn_field_funct3 == 3'b111 ) && cond_geu ) // BGEU branch greater or equal unsigned
//					) begin
////						pc_next = pc_next_branch;
					end
//				pc_next_valid = insn_complete;
//				gen_trap = |pc_next[1:0];
			end

			if (insn_field_opcode == 7'b 00_000_11) begin // LOAD
				insn_decode_valid = (insn_field_funct3 != 3'b 011) && (insn_field_funct3 != 3'b 110) && (insn_field_funct3 != 3'b 111);
//				rs1_addr_valid = 1 ;
//				rd_addr_valid  = 1 ;
//				mem_valid = 1;
//				mem_addr = rs1_value + immediate_12bit;
					has_immediate = 1;
					immediate = immediate_12bit;
					is_load = 1;

				if (insn_field_funct3 == 3'b000 ) begin // LB  load byte
//					mem_rmask = 4'b0001;
//					rd_wdata = { {24{mem_rdata[7]}}, mem_rdata[7:0] };
					mnemonic = "LB";
				end
				if (insn_field_funct3 == 3'b100 ) begin // LBU  load byte unsigned
//					mem_rmask = 4'b0001;
//					rd_wdata = { 24'b0, mem_rdata[7:0] };
					mnemonic = "LBU";
				end
				if (insn_field_funct3 == 3'b001 ) begin // LH  load half word
//					mem_rmask = 4'b0011;
//					rd_wdata = { {16{mem_rdata[15]}}, mem_rdata[15:0] };
					mnemonic = "LH";
				end
				if (insn_field_funct3 == 3'b101 ) begin // LHU load half word unsigned
//					mem_rmask = 4'b0011;
//					rd_wdata = { 16'b0, mem_rdata[15:0] };
					mnemonic = "LHU";
				end
				if (insn_field_funct3 == 3'b010 ) begin // LW  load word
//					mem_rmask = 4'b1111;
//					rd_wdata = mem_rdata;
					mnemonic = "LW";
				end
			end

			if (insn_field_opcode == 7'b 01_000_11) begin // STORE
				insn_decode_valid = (insn_field_funct3 == 3'b 000) || (insn_field_funct3 == 3'b 001) || (insn_field_funct3 == 3'b 010);

//				rs1_addr_valid = 1 ;
//				rs2_addr_valid = 1 ;
//				mem_valid = 1;
//				mem_wdata = rs2_value;

					has_immediate = 1;
					immediate = immediate_12bit_for_stores;
					is_store = 1;

//				mem_addr = rs1_value + immediate_12bit_for_stores;
				if (insn_field_funct3 == 3'b000 ) begin // SB  store byte
//					mem_wstrb = 4'b0001;
					mnemonic = "SB";
				end
				if (insn_field_funct3 == 3'b001 ) begin // SH  store half word
//					mem_wstrb = 4'b0011;
					mnemonic = "SH";
				end
				if (insn_field_funct3 == 3'b010 ) begin // SW  store word
//					mem_wstrb = 4'b1111;
					mnemonic = "SW";
				end

			end

			if ( (insn_field_opcode == 7'b 01_100_11) || (insn_field_opcode == 7'b 00_100_11) ) begin // OP or OP-IMM
				is_alu_immediate = (insn_field_opcode == 7'b 00_100_11);
				if ( is_alu_immediate ) begin
//					rs2_value = immediate_12bit;
					has_immediate = 1;
					immediate = immediate_12bit;					
				end

				case (insn_field_funct3)
					3'b 000: begin // ADD SUB
//						rs1_addr_valid = 1 ;
//						rs2_addr_valid = !is_alu_immediate ;
//						rd_addr_valid  = 1 ;

						if (is_alu_immediate || insn_field_funct7 == 7'b 0000000) begin
							insn_decode_valid = 1; // ADD
//							rd_wdata = rs1_value + rs2_value;
							mnemonic = has_immediate ? "ADDI" : "ADD";
						end else if ( insn_field_funct7 == 7'b 0100000 ) begin
							insn_decode_valid = 1; // SUB
//							rd_wdata = rs1_value - rs2_value;
							mnemonic = has_immediate ? "SUBI" : "SUB";						end
					end
					3'b 101: begin // SRL SRA SRLI SRAI
//						rs1_addr_valid = 1 ;
//						rs2_addr_valid = !is_alu_immediate ;
//						rd_addr_valid  = 1 ;

//	//					if ( (!is_alu_immediate && insn_field_funct7 == 7'b 0000000 ) || ( is_alu_immediate && insn_field_funct7 == 7'b 0000000 ) ) begin
						if ( insn_field_funct7 == 7'b 0000000 ) begin
							insn_decode_valid = 1; // SRL
//							rd_wdata = rs1_value >> rs2_value[4:0];
							mnemonic = has_immediate ? "SRLI" : "SRL";
						end else if ( (!is_alu_immediate && insn_field_funct7 == 7'b 0100000 ) || ( is_alu_immediate && insn_field_funct7 == 7'b 0100000 ) ) begin
							insn_decode_valid = 1; // SRA
							mnemonic = has_immediate ? "SRAI" : "SRA";
//							rd_wdata = ( { {32{rs1_value[31]}}, rs1_value } >> rs2_value[4:0] ); //[31:0];
						end
					end
					3'b001: begin 
//	//					if ( (!is_alu_immediate && insn_field_funct7 == 7'b 0000000 ) || ( is_alu_immediate && insn_field_funct7 == 7'b 0000000 ) ) begin 
						if ( insn_field_funct7 == 7'b 0000000 ) begin 
							insn_decode_valid = 1; // SLLI
//							rs1_addr_valid = 1 ;
//							rs2_addr_valid = !is_alu_immediate ;
//							rd_addr_valid  = 1 ;
//							rd_wdata = rs1_value << rs2_value[4:0] ;
							mnemonic = is_alu_immediate ? "SLLI" : "SLL";

						end
					end
					default: begin
						if ( is_alu_immediate || insn_field_funct7 == 7'b 0000000 ) begin
							insn_decode_valid = 1;
//							rs1_addr_valid = 1 ;
//							rs2_addr_valid = !is_alu_immediate ;
//							rd_addr_valid  = 1 ;
							case (insn_field_funct3)
					//			3'b001: begin // SLL
					//				rd_wdata = rs1_value << rs2_value[4:0] ;
					//			end
								3'b010: begin // SLT set less than
//									rd_wdata = { 31'b0, cond_lt };
									mnemonic = is_alu_immediate ? "SLTI" : "SLT";
								end
								3'b011: begin // SLTU set less than unsigned (with a signed extended immediate if immediate mode)
//									rd_wdata = { 31'b0, cond_ltu }; 
									mnemonic = is_alu_immediate ? "SLTUI" : "SLTU";
								end
								3'b100: begin // XOR
//									rd_wdata = rs1_value ^ rs2_value ;
									mnemonic = is_alu_immediate ? "XORI" : "XOR";
								end
								3'b110: begin // OR
//									rd_wdata = rs1_value | rs2_value ;
									mnemonic = is_alu_immediate ? "ORI" : "OR";
								end
								3'b111: begin // AND
//									rd_wdata = rs1_value & rs2_value ;
									mnemonic = is_alu_immediate ? "ANDI" : "AND";
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
//							rd_addr_valid = 1;
//							rd_wdata = csr_cycle[31:0] ;
						end
						12'hC01: begin
							insn_decode_valid = 1 ;
//							rd_addr_valid = 1;
//							rd_wdata = csr_time[31:0] ;
						end
						12'hC02: begin
							insn_decode_valid = 1 ;
//							rd_addr_valid = 1;
//							rd_wdata = csr_instret[31:0] ;
						end
						12'hC80: begin
							insn_decode_valid = 1 ;
//							rd_addr_valid = 1;
//							rd_wdata = csr_cycle[63:32] ;
						end
						12'hC81: begin
							insn_decode_valid = 1 ;
//							rd_addr_valid = 1;
//							rd_wdata = csr_time[63:32] ;
						end
						12'hC82: begin
							insn_decode_valid = 1 ;
//							rd_addr_valid = 1;
//							rd_wdata = csr_instret[63:32] ;
						end
					endcase
					end
			end
		end
	end



wire mismatch = 0;


    // also want a version of this using the ABI names..
	/*
    reg [32*3*8:0] register_names  = {
          "x31x30x29x28x27x26x25x24"
        , "x23x22x21x20x19x18x17x16"
        , "x15x14x13x12x11x10x9 x8 "
        , "x7 x6 x5 x4 x3 x2 x1 x0 "
    };
*/

	reg [32*3*8:0] register_names  = {
		  " t6 t5 t4 t3s11s10 s9 s8"
		, " s7 s6 s5 s4 s3 s2 a7 a6"
		, " a5 a4 a3 a2 a1 a0 s1 fp"
		, " t2 t1 t0 tp gp sp ra 00"
	};



    // want to do a cycle count between instuctions retired...



    always @(posedge clk) begin
        #1;  // wait for stuff to settle
        // maybe should $strobe instead?


        if (reset ) begin
            $fdisplay( fd, "reset" );
        end else begin
            if (rvfi_valid /* && insn_decode_valid */ ) begin
				if ( insn_decode_valid ) begin 

	                if ( mismatch ) begin
	                    $fdisplay( fd );
	                end

	                $fwrite( fd, "%08x ", rvfi_pc_rdata );
	                $fwrite( fd, "%08x ", rvfi_insn );
	                $fwrite( fd, "%s ", mnemonic );


					if ( expect_rd_addr_valid ) begin
	                	$fwrite( fd, "%s <= " , register_names[ expect_rd_addr*3*8  +: 3*8 ] );
						$fwrite( fd, "%x ", expect_rd_wdata );
	                end else begin
						$fwrite( fd, "     " );
					end

					if ( expect_rs1_addr_valid ) begin
						$fwrite( fd, "%s " , register_names[ expect_rs1_addr*3*8 +: 3*8 ] );
						if ( expect_rs1_addr != 0 ) $fwrite( fd, "(%x) ", shadow_rs1_rdata );
					end else begin
						$fwrite( fd, "     " );
					end

					if ( expect_rs2_addr_valid ) begin
						$fwrite( fd, "%s " , register_names[ expect_rs2_addr*3*8 +: 3*8 ] );
						if ( expect_rs2_addr != 0 )$fwrite( fd, "(%x) ", shadow_rs2_rdata );
					end else begin
						$fwrite( fd, "     " );
					end

					if ( has_immediate ) $fwrite( fd, "%x ", immediate );

					if ( is_store ) $fwrite( fd, "write mem[%x] = %x ", rvfi_mem_addr, rvfi_mem_wdata );
					if ( is_load  ) $fwrite( fd, "read  mem[%x] = %x ", rvfi_mem_addr, rvfi_mem_rdata );



					$fdisplay( fd );
					// compare points:
					$fdisplay( fd, "pc_next:  %x : %x", rvfi_pc_wdata, expect_pc_next );
					if ( expect_rd_request ) begin
	                	$fdisplay( fd, "rd_addr: %x : %x" , rvfi_rd_addr, expect_rd_addr );
						$fdisplay( fd, "rd_data: %x : %x" , rvfi_rd_wdata, expect_rd_wdata );
					end
					if ( expect_rs1_request ) begin
	                	$fdisplay( fd, "rs1_addr: %x : %x" , rvfi_rs1_addr, expect_rs1_addr );
						$fdisplay( fd, "rs1_data: %x : %x" , rvfi_rs1_rdata, shadow_rs1_rdata );
					end
					if ( expect_rs2_request ) begin
	                	$fdisplay( fd, "rs2_addr: %x : %x" , rvfi_rs2_addr, expect_rs2_addr );
						$fdisplay( fd, "rs2_data: %x : %x" , rvfi_rs2_rdata, shadow_rs2_rdata );
					end
					if ( is_store || is_load ) begin
						// can compare addresses
						// but cannot compare data because that is not being shadowed
						$fdisplay( fd, "mem_addr: %x : %x" , rvfi_mem_addr, expect_mem_addr );
					end




	                if ( mismatch ) begin
	                    $fwrite( fd, "%08x ", rvfi_pc_rdata );
	                    $fwrite( fd, "%08x ", rvfi_insn );
	                    $fwrite( fd, "%s ", mnemonic );
	                    $fwrite( fd, "%s " , register_names[ expect_rd_addr*3*8 +: 3*8 ] );
	                    $fwrite( fd, "%s " , register_names[ expect_rs1_addr*3*8 +: 3*8 ] );
	                    $fwrite( fd, "%s " , register_names[ expect_rs2_addr*3*8 +: 3*8 ] );

	                    $fdisplay( fd );
	                end


	         //       $write( "%s", rs1_str );
	         //       $write( "%s", rs2_str );
	       //         $write( "%s", immediate_str );

            	end else begin
                	$fwrite( fd, "illegal instruction" );
            	end

            	$fdisplay( fd );
			end
        end
    end

endmodule
