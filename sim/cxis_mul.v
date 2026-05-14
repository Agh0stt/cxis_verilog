// cxis_mul.v — 32x32 shift-and-add multiplier, 32 cycles
module cxis_mul (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [31:0] a,
    input  wire [31:0] b,
    output reg  [31:0] result,
    output reg  [31:0] result_hi,
    output reg         done
);

reg [63:0] acc;
reg [31:0] shift_a;
reg [31:0] shift_b;
reg [5:0]  count;
reg        active;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        acc       <= 64'h0;
        shift_a   <= 32'h0;
        shift_b   <= 32'h0;
        count     <= 6'd0;
        active    <= 1'b0;
        done      <= 1'b0;
        result    <= 32'h0;
        result_hi <= 32'h0;
    end else if (start && !active) begin
        acc     <= 64'h0;
        shift_a <= a;
        shift_b <= b;
        count   <= 6'd0;
        active  <= 1'b1;
        done    <= 1'b0;
    end else if (active) begin
        if (shift_b[0])
            acc <= acc + {32'h0, shift_a};
        shift_a <= shift_a << 1;
        shift_b <= shift_b >> 1;
        count   <= count + 6'd1;
        if (count == 6'd31) begin
            result    <= acc[31:0];
            result_hi <= acc[63:32];
            active    <= 1'b0;
            done      <= 1'b1;
        end
    end else begin
        done <= 1'b0;
    end
end

endmodule
