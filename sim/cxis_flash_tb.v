// =============================================================================
//  cxis_flash_tb.v  —  testbench for cxis_cpu with flash ROM loading
//
//  Loads a .cxe image via $readmemh (pre-converted by cxe2hex.py),
//  boots the CPU, and monitors UART output.
//
//  Run with:
//      python3 cxe2hex.py tests/hello.cxe hello.hex
//      iverilog -o flash_tb cxis_flash_tb.v cxis_cpu.v
//      vvp flash_tb
//
//  Or override at compile time:
//      iverilog -DHEX_FILE=\"my_prog.hex\" -o flash_tb cxis_flash_tb.v cxis_cpu.v
//      vvp flash_tb
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

// Default hex file — override with -DHEX_FILE=\"...\" on iverilog command line
`ifndef HEX_FILE
  `define HEX_FILE "hello.hex"
`endif

// Simulation timeout in clock cycles (default 2M cycles ≈ 20ms @ 100MHz)
`ifndef SIM_TIMEOUT
  `define SIM_TIMEOUT 2000000
`endif

module cxis_flash_tb;

// ---------------------------------------------------------------------------
// Clock & reset
// ---------------------------------------------------------------------------
reg clk   = 0;
reg rst_n = 0;

localparam CLK_PERIOD = 10;   // 100 MHz
always #(CLK_PERIOD/2) clk = ~clk;

// ---------------------------------------------------------------------------
// DUT
// ---------------------------------------------------------------------------
wire        uart_tx_line;
reg         uart_rx_line = 1'b1;
wire        halted;
wire [31:0] dbg_pc;
wire [31:0] dbg_i0;

// Small BAUD_DIV for fast simulation
localparam BAUD_DIV_SIM = 4;

cxis_cpu #(
    .RAM_WORDS  (16384),
    .FLASH_WORDS(16384),
    .FLASH_HEX  (`HEX_FILE),
    .BAUD_DIV   (BAUD_DIV_SIM)
) dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .uart_tx  (uart_tx_line),
    .uart_rx  (uart_rx_line),
    .halted   (halted),
    .dbg_pc   (dbg_pc),
    .dbg_i0   (dbg_i0)
);

// ---------------------------------------------------------------------------
// UART RX monitor — decode bytes from uart_tx_line and $write to console
// ---------------------------------------------------------------------------
reg [7:0]  rx_data;
reg [15:0] rx_cnt;
reg [3:0]  rx_bit;
reg        rx_active;

initial begin
    rx_active = 0;
    rx_cnt    = 0;
    rx_bit    = 0;
    rx_data   = 0;
end

always @(posedge clk) begin
    if (!rx_active) begin
        if (!uart_tx_line) begin
            // Start bit detected — wait ~1.5 bit periods to hit centre of bit 0
            rx_active <= 1;
            rx_cnt    <= 1;
            rx_bit    <= 0;
            rx_data   <= 0;
        end
    end else begin
        rx_cnt <= rx_cnt + 1;
        if (rx_cnt == BAUD_DIV_SIM + BAUD_DIV_SIM/2 - 1 + rx_bit * BAUD_DIV_SIM) begin
            if (rx_bit < 8) begin
                rx_data[rx_bit] <= uart_tx_line;
                rx_bit          <= rx_bit + 1;
            end else begin
                // Stop bit zone — byte complete
                $write("%c", rx_data);
                rx_active <= 0;
            end
        end
    end
end

// ---------------------------------------------------------------------------
// Halt detector
// ---------------------------------------------------------------------------
always @(posedge clk) begin
    if (halted) begin
        repeat (50) @(posedge clk);
        $display("\n[flash_tb] CPU halted.  PC=0x%08X  i0=0x%08X", dbg_pc, dbg_i0);
        $finish;
    end
end

// ---------------------------------------------------------------------------
// Main sequence
// ---------------------------------------------------------------------------
initial begin
    $dumpfile("cxis_flash_tb.vcd");
    $dumpvars(0, cxis_flash_tb);

    $display("[flash_tb] Loading flash image: %s", `HEX_FILE);

    // Hold reset for a few cycles
    rst_n = 0;
    repeat (4) @(posedge clk);
    rst_n = 1;

    // Flash loader FSM runs inside the CPU; just wait for halt or timeout
    repeat (`SIM_TIMEOUT) @(posedge clk);

    $display("\n[flash_tb] TIMEOUT (%0d cycles) — CPU did not halt.", `SIM_TIMEOUT);
    $finish;
end

endmodule
