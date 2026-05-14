// =============================================================================
//  cxis_fpu.v  —  Software-model FPU for CXIS  (Verilog-2001)
//
//  Implements IEEE 754 single (f32) and double (f64) operations using
//  Verilog's built-in real arithmetic (simulation-accurate).
//
//  Portability: uses only $bitstoreal/$realtobits (Verilog-2001 standard).
//  f32<->real via portable bit-manipulation functions (no $bitstoshortreal).
// =============================================================================

`default_nettype none

module cxis_fpu (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [5:0]  op,
    input  wire [63:0] src_a,
    input  wire [63:0] src_b,
    output reg  [63:0] result,
    output reg         done,
    output reg  [31:0] cmp_flags   // [1]=less [0]=equal
);

localparam [5:0]
    FPU_FADD   = 6'd0,  FPU_FSUB   = 6'd1,  FPU_FMUL   = 6'd2,
    FPU_FDIV   = 6'd3,  FPU_FNEG   = 6'd4,  FPU_FABS   = 6'd5,
    FPU_FSQRT  = 6'd6,  FPU_FMOD   = 6'd7,  FPU_FFLOOR = 6'd8,
    FPU_FCEIL  = 6'd9,  FPU_FROUND = 6'd10, FPU_FSIN   = 6'd11,
    FPU_FCOS   = 6'd12, FPU_FTAN   = 6'd13,
    FPU_DADD   = 6'd14, FPU_DSUB   = 6'd15, FPU_DMUL   = 6'd16,
    FPU_DDIV   = 6'd17, FPU_DNEG   = 6'd18, FPU_DABS   = 6'd19,
    FPU_DSQRT  = 6'd20, FPU_DMOD   = 6'd21, FPU_DFLOOR = 6'd22,
    FPU_DCEIL  = 6'd23, FPU_DROUND = 6'd24, FPU_DSIN   = 6'd25,
    FPU_DCOS   = 6'd26, FPU_DTAN   = 6'd27,
    FPU_FCMP   = 6'd28, FPU_DCMP   = 6'd29,
    FPU_ITOF   = 6'd30, FPU_ITOD   = 6'd31,
    FPU_FTOI   = 6'd32, FPU_DTOI   = 6'd33,
    FPU_FTOD   = 6'd34, FPU_DTOF   = 6'd35,
    FPU_ITOL   = 6'd36, FPU_LTOF   = 6'd37,
    FPU_LTOD   = 6'd38, FPU_FTOL   = 6'd39,
    FPU_DTOL   = 6'd40, FPU_LTOI   = 6'd41;

// ── f32 <-> real bit-cast  (IEEE 754 single <-> double expansion) ─────────
function real f32_to_real;
    input [31:0] b;
    reg [63:0] f64;
    reg [7:0]  exp32;
    reg [22:0] mant32;
    reg        sign;
    reg [10:0] exp64;
    begin
        sign   = b[31];
        exp32  = b[30:23];
        mant32 = b[22:0];
        if (exp32 == 8'hFF)
            f64 = {sign, 11'h7FF, mant32, 29'h0};
        else if (exp32 == 8'h00)
            f64 = {sign, 63'h0};
        else begin
            exp64 = ({3'b000, exp32} - 11'd127) + 11'd1023;
            f64   = {sign, exp64, mant32, 29'h0};
        end
        f32_to_real = $bitstoreal(f64);
    end
endfunction

function [31:0] real_to_f32;
    input real r;
    reg [63:0] f64;
    reg        sign;
    reg [10:0] exp64;
    reg [51:0] mant64;
    reg [7:0]  exp32;
    reg [22:0] mant32;
    begin
        f64    = $realtobits(r);
        sign   = f64[63];
        exp64  = f64[62:52];
        mant64 = f64[51:0];
        if (exp64 == 11'h7FF) begin
            exp32  = 8'hFF;
            mant32 = mant64[51:29];
        end else if (exp64 == 11'h000) begin
            exp32  = 8'h00;
            mant32 = 23'h0;
        end else begin
            exp32  = (exp64 - 11'd1023) + 8'd127;
            mant32 = mant64[51:29];
        end
        real_to_f32 = {sign, exp32, mant32};
    end
endfunction

// Pipeline registers (4-stage: compute -> p1 -> p2 -> output)
reg [63:0] pipe_result [1:3];
reg [31:0] pipe_cmpflg [1:3];
reg        pipe_done   [1:4];

real fa, fb, fr;
real da, db, dr;
integer ia;
reg [63:0] raw_result;
reg [31:0] raw_cmpflg;

always @(posedge clk or negedge rst_n) begin : fpu_compute
    if (!rst_n) begin
        raw_result     <= 64'h0;
        raw_cmpflg     <= 32'h0;
        pipe_result[1] <= 64'h0; pipe_result[2] <= 64'h0; pipe_result[3] <= 64'h0;
        pipe_cmpflg[1] <= 32'h0; pipe_cmpflg[2] <= 32'h0; pipe_cmpflg[3] <= 32'h0;
        pipe_done[1]   <= 1'b0;  pipe_done[2]   <= 1'b0;  pipe_done[3]   <= 1'b0;  pipe_done[4] <= 1'b0;
        result         <= 64'h0;
        cmp_flags      <= 32'h0;
        done           <= 1'b0;
    end else begin
        raw_result   <= 64'h0;
        raw_cmpflg   <= 32'h0;
        pipe_done[1] <= start;

        if (start) begin
            case (op)
            FPU_FADD: begin fa=f32_to_real(src_a[31:0]); fb=f32_to_real(src_b[31:0]); fr=fa+fb; raw_result<={32'h0,real_to_f32(fr)}; end
            FPU_FSUB: begin fa=f32_to_real(src_a[31:0]); fb=f32_to_real(src_b[31:0]); fr=fa-fb; raw_result<={32'h0,real_to_f32(fr)}; end
            FPU_FMUL: begin fa=f32_to_real(src_a[31:0]); fb=f32_to_real(src_b[31:0]); fr=fa*fb; raw_result<={32'h0,real_to_f32(fr)}; end
            FPU_FDIV: begin
                fa=f32_to_real(src_a[31:0]); fb=f32_to_real(src_b[31:0]);
                fr=(fb==0.0) ? $bitstoreal(64'h7FF0000000000000) : fa/fb;
                raw_result<={32'h0,real_to_f32(fr)};
            end
            FPU_FNEG:   begin fa=f32_to_real(src_a[31:0]); fr=-fa;                         raw_result<={32'h0,real_to_f32(fr)}; end
            FPU_FABS:   begin fa=f32_to_real(src_a[31:0]); fr=(fa<0.0)?-fa:fa;             raw_result<={32'h0,real_to_f32(fr)}; end
            FPU_FSQRT:  begin fa=f32_to_real(src_a[31:0]); fr=$sqrt(fa);                   raw_result<={32'h0,real_to_f32(fr)}; end
            FPU_FMOD:   begin fa=f32_to_real(src_a[31:0]); fb=f32_to_real(src_b[31:0]); fr=fa-($itor($rtoi(fa/fb))*fb); raw_result<={32'h0,real_to_f32(fr)}; end
            FPU_FFLOOR: begin fa=f32_to_real(src_a[31:0]); fr=$floor(fa);                  raw_result<={32'h0,real_to_f32(fr)}; end
            FPU_FCEIL:  begin fa=f32_to_real(src_a[31:0]); fr=$ceil(fa);                   raw_result<={32'h0,real_to_f32(fr)}; end
            FPU_FROUND: begin fa=f32_to_real(src_a[31:0]); fr=$floor(fa+0.5);              raw_result<={32'h0,real_to_f32(fr)}; end
            FPU_FSIN:   begin fa=f32_to_real(src_a[31:0]); fr=$sin(fa);                    raw_result<={32'h0,real_to_f32(fr)}; end
            FPU_FCOS:   begin fa=f32_to_real(src_a[31:0]); fr=$cos(fa);                    raw_result<={32'h0,real_to_f32(fr)}; end
            FPU_FTAN:   begin fa=f32_to_real(src_a[31:0]); fr=$sin(fa)/$cos(fa);           raw_result<={32'h0,real_to_f32(fr)}; end

            FPU_DADD:   begin da=$bitstoreal(src_a); db=$bitstoreal(src_b); dr=da+db; raw_result<=$realtobits(dr); end
            FPU_DSUB:   begin da=$bitstoreal(src_a); db=$bitstoreal(src_b); dr=da-db; raw_result<=$realtobits(dr); end
            FPU_DMUL:   begin da=$bitstoreal(src_a); db=$bitstoreal(src_b); dr=da*db; raw_result<=$realtobits(dr); end
            FPU_DDIV: begin
                da=$bitstoreal(src_a); db=$bitstoreal(src_b);
                dr=(db==0.0)?$bitstoreal(64'h7FF0000000000000):da/db;
                raw_result<=$realtobits(dr);
            end
            FPU_DNEG:   begin da=$bitstoreal(src_a); dr=-da;                        raw_result<=$realtobits(dr); end
            FPU_DABS:   begin da=$bitstoreal(src_a); dr=(da<0.0)?-da:da;            raw_result<=$realtobits(dr); end
            FPU_DSQRT:  begin da=$bitstoreal(src_a); dr=$sqrt(da);                  raw_result<=$realtobits(dr); end
            FPU_DMOD:   begin da=$bitstoreal(src_a); db=$bitstoreal(src_b); dr=da-($itor($rtoi(da/db))*db); raw_result<=$realtobits(dr); end
            FPU_DFLOOR: begin da=$bitstoreal(src_a); dr=$floor(da);                 raw_result<=$realtobits(dr); end
            FPU_DCEIL:  begin da=$bitstoreal(src_a); dr=$ceil(da);                  raw_result<=$realtobits(dr); end
            FPU_DROUND: begin da=$bitstoreal(src_a); dr=$floor(da+0.5);             raw_result<=$realtobits(dr); end
            FPU_DSIN:   begin da=$bitstoreal(src_a); dr=$sin(da);                   raw_result<=$realtobits(dr); end
            FPU_DCOS:   begin da=$bitstoreal(src_a); dr=$cos(da);                   raw_result<=$realtobits(dr); end
            FPU_DTAN:   begin da=$bitstoreal(src_a); dr=$sin(da)/$cos(da);          raw_result<=$realtobits(dr); end

            FPU_FCMP: begin
                fa=f32_to_real(src_a[31:0]); fb=f32_to_real(src_b[31:0]);
                raw_cmpflg<={30'h0,(fa<fb)?1'b1:1'b0,(fa==fb)?1'b1:1'b0};
                raw_result<=64'h0;
            end
            FPU_DCMP: begin
                da=$bitstoreal(src_a); db=$bitstoreal(src_b);
                raw_cmpflg<={30'h0,(da<db)?1'b1:1'b0,(da==db)?1'b1:1'b0};
                raw_result<=64'h0;
            end

            FPU_ITOF: begin ia=$signed(src_a[31:0]); fa=$itor(ia); raw_result<={32'h0,real_to_f32(fa)}; end
            FPU_ITOD: begin ia=$signed(src_a[31:0]); da=$itor(ia); raw_result<=$realtobits(da); end
            FPU_FTOI: begin fa=f32_to_real(src_a[31:0]); raw_result<={32'h0,$signed($rtoi(fa))}; end
            FPU_DTOI: begin da=$bitstoreal(src_a); raw_result<={32'h0,$signed($rtoi(da))}; end
            FPU_FTOD: begin fa=f32_to_real(src_a[31:0]); raw_result<=$realtobits(fa); end
            FPU_DTOF: begin da=$bitstoreal(src_a); raw_result<={32'h0,real_to_f32(da)}; end
            FPU_ITOL: raw_result <= {{32{src_a[31]}}, src_a[31:0]};
            FPU_LTOI: raw_result <= {32'h0, src_a[31:0]};
            FPU_LTOF: begin da=$itor($signed(src_a)); raw_result<={32'h0,real_to_f32(da)}; end
            FPU_LTOD: begin da=$itor($signed(src_a)); raw_result<=$realtobits(da); end
            FPU_FTOL: begin fa=f32_to_real(src_a[31:0]); raw_result<=$signed($rtoi(fa)); end
            FPU_DTOL: begin da=$bitstoreal(src_a); raw_result<=$signed($rtoi(da)); end
            default:  raw_result <= 64'h0;
            endcase
        end

        pipe_result[1] <= raw_result;   pipe_cmpflg[1] <= raw_cmpflg;
        pipe_result[2] <= pipe_result[1]; pipe_cmpflg[2] <= pipe_cmpflg[1]; pipe_done[2] <= pipe_done[1];
        pipe_result[3] <= pipe_result[2]; pipe_cmpflg[3] <= pipe_cmpflg[2]; pipe_done[3] <= pipe_done[2];
        pipe_done[4]   <= pipe_done[3];

        result    <= pipe_result[3];
        cmp_flags <= pipe_cmpflg[3];
        done      <= pipe_done[4];
    end
end

endmodule
