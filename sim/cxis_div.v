// cxis_div.v — 32/32 pipelined divider (32 stages), div-by-zero safe
// Wraps divfunc (fully registered pipeline, STAGE_LIST=32'hFFFFFFFF)

// ---------------------------------------------------------------------------
// divfunc — generic pipelined divider
// ---------------------------------------------------------------------------
`define N(n)                   [(n)-1:0]
`define FFx(signal,bits)       always @ ( posedge clk or posedge rst ) \
                                   if ( rst ) signal <= bits; else

module divfunc
#(
    parameter                  XLEN       = 32,
    parameter `N(XLEN)         STAGE_LIST = 0
)
(
    input              clk,
    input              rst,
    input  `N(XLEN)    a,
    input  `N(XLEN)    b,
    input              vld,
    output `N(XLEN)    quo,
    output `N(XLEN)    rem,
    output             ack
);
    reg               ready    `N(XLEN+1);
    reg `N(XLEN)      dividend `N(XLEN+1);
    reg `N(XLEN)      divisor  `N(XLEN+1);
    reg `N(XLEN)      quotient `N(XLEN+1);

    always @* begin
        ready[0]    = vld;
        dividend[0] = a;
        divisor[0]  = b;
        quotient[0] = 0;
    end

    generate
        genvar i;
        for (i = 0; i < XLEN; i = i + 1) begin : gen_div
            wire [i:0]      m = dividend[i] >> (XLEN-i-1);
            wire [i:0]      n = divisor[i];
            wire            q = (|(divisor[i] >> (i+1))) ? 1'b0 : (m >= n);
            wire [i:0]      t = q ? (m - n) : m;
            wire [XLEN-1:0] u = dividend[i] << (i+1);
            wire [XLEN+i:0] d = {t, u} >> (i+1);

            if (STAGE_LIST[XLEN-i-1]) begin : gen_ff
                `FFx(ready[i+1],    0) ready[i+1]    <= ready[i];
                `FFx(dividend[i+1], 0) dividend[i+1] <= d;
                `FFx(divisor[i+1],  0) divisor[i+1]  <= divisor[i];
                `FFx(quotient[i+1], 0) quotient[i+1] <= quotient[i] | (q << (XLEN-i-1));
            end else begin : gen_comb
                always @* begin
                    ready[i+1]    = ready[i];
                    dividend[i+1] = d;
                    divisor[i+1]  = divisor[i];
                    quotient[i+1] = quotient[i] | (q << (XLEN-i-1));
                end
            end
        end
    endgenerate

    assign quo = quotient[XLEN];
    assign rem = dividend[XLEN];
    assign ack = ready[XLEN];
endmodule


// ---------------------------------------------------------------------------
// cxis_div — wrapper matching the CPU interface
//   - rst_n (active-low) converted to rst (active-high) for divfunc
//   - div-by-zero detected before entering the pipeline (done in 1 cycle)
//   - normal division: 32-cycle pipeline latency (STAGE_LIST = all-1s)
// ---------------------------------------------------------------------------
module cxis_div (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [31:0] a,
    input  wire [31:0] b,
    output reg  [31:0] quotient,
    output reg  [31:0] remainder,
    output reg         done,
    output reg         div_zero
);

wire rst = ~rst_n;

// Only feed the pipeline when b != 0
wire        vld_inner = start && (b != 32'h0);
wire [31:0] quo_inner;
wire [31:0] rem_inner;
wire        ack_inner;

divfunc #(
    .XLEN      (32),
    .STAGE_LIST(32'hFFFFFFFF)   // all 32 stages registered → 32-cycle latency
) u_divfunc (
    .clk (clk),
    .rst (rst),
    .a   (a),
    .b   (b),
    .vld (vld_inner),
    .quo (quo_inner),
    .rem (rem_inner),
    .ack (ack_inner)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        quotient  <= 32'h0;
        remainder <= 32'h0;
        done      <= 1'b0;
        div_zero  <= 1'b0;
    end else begin
        done     <= 1'b0;
        div_zero <= 1'b0;

        if (start && (b == 32'h0)) begin
            // Divide-by-zero: return FFFFFFFF immediately (next cycle)
            quotient  <= 32'hFFFFFFFF;
            remainder <= a;
            div_zero  <= 1'b1;
            done      <= 1'b1;
        end else if (ack_inner) begin
            // Pipeline result ready after 32 cycles
            quotient  <= quo_inner;
            remainder <= rem_inner;
            done      <= 1'b1;
        end
    end
end

endmodule
