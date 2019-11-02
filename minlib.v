


module priority_arb #( parameter integer N = 1 ) (
      input [N-1:0] readyIn
    , output [N-1:0] readyOut
);

reg [N-1:0] s;
integer i;
reg done;
always @(*) begin
    s = 0;
    done = 0;
    for ( i=0; i < N ; i=i+1 ) begin
        if ( readyIn[i] && !done ) begin
            s = 1 << i;
            done = 1;
        end
    end
end

assign readyOut = s;

endmodule

module sequencer #( parameter integer N = 0, parameter [N-1:0] sticky = 0 ) (
      input      [N-1:0] readyIn
    , output reg [N-1:0] readyOut

    , input clk
    , input resetn
);

reg [     N-1 : 0] outs;

parameter Nbits = $clog2( N );
reg [ Nbits-1 : 0] state;      // state
reg [ Nbits-1 : 0] state1;     // combinatorial
reg [ Nbits-1 : 0] state_next; // combinatorial - next state

/*
always @(*) begin
    assert( state      < N );
    assert( state1     < N );
    assert( state_next < N );
end
*/

integer i;
reg done;
always @(*) begin
    state1 = state;

    // state outputs, default to 0
    outs = 0;
    done = 0;
    for( i = 0; i < N; i = i + 1 ) begin
        if ( !done && ( i == state1 ) ) begin
            outs[i] = 1;
            if ( readyIn[i] ) begin
                state1 = ( i+1 < N ) ? i+1 : 0;
                end
            if ( sticky[i] ) done = 1; // don't allow going to next state combinatoritally
        end
    end
    
    // assign this at end to minimize zero delay events on the outputs
    readyOut = outs;
    state_next = state1;
end

always @(posedge clk  ) begin
    if ( !resetn ) state <= 0;
    else           state <= state_next;
end

endmodule

module one_hot_mux #( parameter integer Ninputs = 0, parameter integer Nbits = 1 ) (
      input [ Ninputs*Nbits -1 : 0 ] ins
    , input [ Ninputs       -1 : 0 ] select
    , output reg [ Nbits-1 : 0 ] out
);

reg [ Nbits-1 : 0 ] out1;
reg [ Nbits-1 : 0 ] tmp;

integer i;

always @(*) begin

/*
    out1 = {Nbits{1'b1}};
    for( i =0; i< Ninputs; i=i+1 ) begin
        tmp = ins >> ( Nbits * i );
        if ( select[i] ) begin
            out1 = out1 & tmp;
        end
    end
*/
    out1 = 0;
    for( i =0; i< Ninputs; i=i+1 ) begin
        tmp = ins >> ( Nbits * i );
        out1 = out1 | ( select[i] ? tmp : 0 );
    end




    out = out1;
end

endmodule

module register #( parameter Nbits = 1 ) (
      input  [ Nbits-1:0 ] in
    , input                enable
    , output reg [ Nbits-1:0 ] out

    , input                clk
);

always @(posedge clk) begin
    if ( enable ) out <= in;
end

endmodule

module register_resetable #( parameter Nbits = 1 ) (
      input  [ Nbits-1:0 ] in
    , input                enable
    , output reg [ Nbits-1:0 ] out

    , input                clk
    , input                 resetn
);

always @(posedge clk ) begin
    if (!resetn) out <= 0;  // TODO parameterize this
    else if ( enable ) out <= in;
end

endmodule


module pipeline #( parameter Nbits = 1, parameter Nstages = 1 ) (
      input [ Nbits-1:0] in
    , output [ Nbits-1:0 ] out
    , input clk
);

wire [ Nbits-1:0 ] reg_outs [ Nstages-1:0 ];

genvar i;
// generate
for ( i=0; i< Nstages ; i=i+1 ) begin
    register #(Nbits) r( .in( i==0 ? in : reg_outs[ (i==0) ? 0 : (i-1) ] ), .out( reg_outs[i] ), .enable(1'b1), .clk(clk) );
end
// endgenerate

    assign out = (Nstages ==0) ? in : reg_outs[ Nstages==0 ? 0 : ( Nstages-1 ) ];

endmodule

module pipeline_resetable #( parameter Nbits = 1, parameter Nstages = 1 ) (
      input [ Nbits-1:0] in
    , output [ Nbits-1:0 ] out
    , input clk
    , input resetn
);

wire [ Nbits-1:0 ] reg_outs [ Nstages-1:0 ];

genvar i;
// generate
for ( i=0; i< Nstages ; i=i+1 ) begin
    register_resetable #(Nbits) r( 
          .in( i==0 ? in : reg_outs[ (i==0) ? 0 : (i-1) ] )
        , .out( reg_outs[i] )
        , .enable(1'b1)
        , .clk(clk) 
        , .resetn( resetn )
        );
end
// endgenerate

    assign out = (Nstages ==0) ? in : reg_outs[ Nstages==0 ? 0 : ( Nstages-1 ) ];

endmodule
