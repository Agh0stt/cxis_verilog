`timescale 1ns/1ps
`default_nettype none

module fpu_tb;

reg        clk, rst_n, start;
reg [5:0]  op;
reg [63:0] src_a, src_b;
wire [63:0] result;
wire        done;
wire [31:0] cmp_flags;

cxis_fpu dut (
    .clk(clk),.rst_n(rst_n),.start(start),
    .op(op),.src_a(src_a),.src_b(src_b),
    .result(result),.done(done),.cmp_flags(cmp_flags)
);

always #5 clk = ~clk;

integer pass, fail;
integer wait_cnt;

// Pulse start for 1 cycle then wait for done to go high
task run_fpu;
    input [5:0]  the_op;
    input [63:0] a, b;
    begin
        @(posedge clk); #1;   // setup just after rising edge
        op=the_op; src_a=a; src_b=b; start=1;
        @(posedge clk); #1;
        start=0;
        // wait up to 20 cycles for done
        wait_cnt=0;
        while (!done && wait_cnt<20) begin
            @(posedge clk); #1;
            wait_cnt=wait_cnt+1;
        end
        // done is high -- wait one more clock for result to settle
        @(posedge clk); #1;
        if (wait_cnt==20) $display("TIMEOUT op=%0d",the_op);
    end
endtask

initial begin
    clk=0; rst_n=0; start=0; op=0; src_a=0; src_b=0;
    pass=0; fail=0;
    repeat(4) @(posedge clk); #1;
    rst_n=1;
    repeat(2) @(posedge clk); #1;

    // FADD: 1.5 + 2.5 = 4.0
    run_fpu(6'd0, {32'h0,32'h3FC00000}, {32'h0,32'h40200000});
    if (result[31:0]==32'h40800000) begin $display("PASS  FADD  1.5+2.5=4.0     bits=%h",result[31:0]); pass=pass+1; end
    else begin $display("FAIL  FADD  got=%h expect=40800000",result[31:0]); fail=fail+1; end

    // FSUB: 5.0 - 3.0 = 2.0
    run_fpu(6'd1, {32'h0,32'h40A00000}, {32'h0,32'h40400000});
    if (result[31:0]==32'h40000000) begin $display("PASS  FSUB  5.0-3.0=2.0     bits=%h",result[31:0]); pass=pass+1; end
    else begin $display("FAIL  FSUB  got=%h expect=40000000",result[31:0]); fail=fail+1; end

    // FMUL: 3.0 * 4.0 = 12.0
    run_fpu(6'd2, {32'h0,32'h40400000}, {32'h0,32'h40800000});
    if (result[31:0]==32'h41400000) begin $display("PASS  FMUL  3.0*4.0=12.0    bits=%h",result[31:0]); pass=pass+1; end
    else begin $display("FAIL  FMUL  got=%h expect=41400000",result[31:0]); fail=fail+1; end

    // FDIV: 7.0 / 2.0 = 3.5
    run_fpu(6'd3, {32'h0,32'h40E00000}, {32'h0,32'h40000000});
    if (result[31:0]==32'h40600000) begin $display("PASS  FDIV  7.0/2.0=3.5     bits=%h",result[31:0]); pass=pass+1; end
    else begin $display("FAIL  FDIV  got=%h expect=40600000",result[31:0]); fail=fail+1; end

    // FDIV by zero -> +Inf
    run_fpu(6'd3, {32'h0,32'h3F800000}, {32'h0,32'h00000000});
    if (result[31:0]==32'h7F800000) begin $display("PASS  FDIV/0 -> +Inf        bits=%h",result[31:0]); pass=pass+1; end
    else begin $display("FAIL  FDIV/0 got=%h expect=7F800000",result[31:0]); fail=fail+1; end

    // FSQRT: sqrt(9.0) = 3.0
    run_fpu(6'd6, {32'h0,32'h41100000}, 64'h0);
    if (result[31:0]==32'h40400000) begin $display("PASS  FSQRT sqrt(9)=3.0     bits=%h",result[31:0]); pass=pass+1; end
    else begin $display("FAIL  FSQRT got=%h expect=40400000",result[31:0]); fail=fail+1; end

    // FNEG: -(-7.0) = 7.0
    run_fpu(6'd4, {32'h0,32'hC0E00000}, 64'h0);
    if (result[31:0]==32'h40E00000) begin $display("PASS  FNEG  -(-7)=7.0       bits=%h",result[31:0]); pass=pass+1; end
    else begin $display("FAIL  FNEG  got=%h expect=40E00000",result[31:0]); fail=fail+1; end

    // FABS: |-7.0| = 7.0
    run_fpu(6'd5, {32'h0,32'hC0E00000}, 64'h0);
    if (result[31:0]==32'h40E00000) begin $display("PASS  FABS  |-7|=7.0        bits=%h",result[31:0]); pass=pass+1; end
    else begin $display("FAIL  FABS  got=%h expect=40E00000",result[31:0]); fail=fail+1; end

    // FFLOOR: floor(2.7) = 2.0
    run_fpu(6'd8, {32'h0,32'h402CCCCD}, 64'h0);
    if (result[31:0]==32'h40000000) begin $display("PASS  FFLOOR floor(2.7)=2.0 bits=%h",result[31:0]); pass=pass+1; end
    else begin $display("FAIL  FFLOOR got=%h expect=40000000",result[31:0]); fail=fail+1; end

    // FCEIL: ceil(2.1) = 3.0
    run_fpu(6'd9, {32'h0,32'h40066666}, 64'h0);
    if (result[31:0]==32'h40400000) begin $display("PASS  FCEIL  ceil(2.1)=3.0  bits=%h",result[31:0]); pass=pass+1; end
    else begin $display("FAIL  FCEIL  got=%h expect=40400000",result[31:0]); fail=fail+1; end

    // FROUND: round(2.5) = 3.0
    run_fpu(6'd10, {32'h0,32'h40200000}, 64'h0);
    if (result[31:0]==32'h40400000) begin $display("PASS  FROUND round(2.5)=3.0 bits=%h",result[31:0]); pass=pass+1; end
    else begin $display("FAIL  FROUND got=%h expect=40400000",result[31:0]); fail=fail+1; end

    // FCMP: 1.0 < 2.0 -> flags[1]=1 flags[0]=0
    run_fpu(6'd28, {32'h0,32'h3F800000}, {32'h0,32'h40000000});
    if (cmp_flags==32'h2) begin $display("PASS  FCMP  1.0<2.0  flags=%b",cmp_flags[1:0]); pass=pass+1; end
    else begin $display("FAIL  FCMP  flags=%b expect=10",cmp_flags[1:0]); fail=fail+1; end

    // FCMP: 2.0 == 2.0 -> flags[0]=1
    run_fpu(6'd28, {32'h0,32'h40000000}, {32'h0,32'h40000000});
    if (cmp_flags==32'h1) begin $display("PASS  FCMP  2.0==2.0 flags=%b",cmp_flags[1:0]); pass=pass+1; end
    else begin $display("FAIL  FCMP  flags=%b expect=01",cmp_flags[1:0]); fail=fail+1; end

    // ITOF: 42 -> 42.0 = 0x42280000
    run_fpu(6'd30, {32'h0,32'd42}, 64'h0);
    if (result[31:0]==32'h42280000) begin $display("PASS  ITOF  42->42.0        bits=%h",result[31:0]); pass=pass+1; end
    else begin $display("FAIL  ITOF  got=%h expect=42280000",result[31:0]); fail=fail+1; end

    // FTOI: -7.9 -> -7
    run_fpu(6'd32, {32'h0,32'hC0FCCCCD}, 64'h0);
    if ($signed(result[31:0])==-7) begin $display("PASS  FTOI  -7.9->-7        val=%0d",$signed(result[31:0])); pass=pass+1; end
    else begin $display("FAIL  FTOI  got=%0d expect=-7",$signed(result[31:0])); fail=fail+1; end

    // DADD: 1.1 + 2.2 ~ 3.3
    run_fpu(6'd14, $realtobits(1.1), $realtobits(2.2));
    begin
        real got; got=$bitstoreal(result);
        if (got>3.299 && got<3.301) begin $display("PASS  DADD  1.1+2.2=%f",got); pass=pass+1; end
        else begin $display("FAIL  DADD  got=%f",got); fail=fail+1; end
    end

    // DCMP: 2.71 < 3.14 -> less=1
    run_fpu(6'd29, $realtobits(2.71), $realtobits(3.14));
    if (cmp_flags[1]==1'b1) begin $display("PASS  DCMP  2.71<3.14 flags=%b",cmp_flags[1:0]); pass=pass+1; end
    else begin $display("FAIL  DCMP  flags=%b expect=1x",cmp_flags[1:0]); fail=fail+1; end

    // ITOL: sign-extend -1 (i32) -> all-ones (i64)
    run_fpu(6'd36, {32'h0,32'hFFFFFFFF}, 64'h0);
    if (result==64'hFFFFFFFFFFFFFFFF) begin $display("PASS  ITOL  -1 sign-ext=%h",result); pass=pass+1; end
    else begin $display("FAIL  ITOL  got=%h",result); fail=fail+1; end

    $display("─────────────────────────────────────");
    $display("Results: %0d PASS  %0d FAIL", pass, fail);
    if (fail==0) $display("ALL TESTS PASSED");
    else         $display("SOME TESTS FAILED");
    $finish;
end

initial begin #100000; $display("GLOBAL TIMEOUT"); $finish; end

endmodule
