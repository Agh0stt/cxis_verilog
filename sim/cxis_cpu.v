// =============================================================================
//  CXIS CPU  —  RTL implementation  (Verilog-2001)
//  I/O: UART only  (BIOS INT 0x01/0x02/0x03 + IN/OUT mapped to UART)
//
//  Register file (integer 32-bit only for this rev):
//    i0–i31  [0..31]    general purpose
//    c0–c31  [128..159] condition (stored as i32; only LSB used)
//    s0–s31  [160..191] scratch
//    a0–a9   [192..201] argument / return
//    sp=202  sf=203  bp=204  bf=205
//
//  Pipeline: 3-stage  FETCH → DECODE → EXECUTE
//  Memory  : 64KB BRAM (good for simulation; bump to 4MB for real FPGA)
//  UART    : 8N1, baud divisor configurable via BAUD_DIV parameter
// =============================================================================

`default_nettype none

module cxis_cpu #(
    parameter RAM_WORDS   = 16384,       // 64 KB  (16K × 32-bit words)
    parameter FLASH_WORDS = 16384,       // 64 KB flash image (16K × 32-bit words)
    parameter FLASH_HEX   = "",          // path to $readmemh file; "" = skip flash load
    parameter BAUD_DIV    = 868          // 100 MHz / 115200 ≈ 868
)(
    input  wire        clk,
    input  wire        rst_n,

    // UART
    output wire        uart_tx,
    input  wire        uart_rx,

    // Debug / status
    output wire        halted,
    output wire [31:0] dbg_pc,
    output wire [31:0] dbg_i0           // value of i0 for debug
);

// ---------------------------------------------------------------------------
// Opcode constants (matching cxis.h)
// ---------------------------------------------------------------------------
localparam [7:0]
    OP_MOV    = 8'h01, OP_MOVSX  = 8'h02, OP_MOVZX  = 8'h03,
    OP_PUSH   = 8'h05, OP_POP    = 8'h06,
    OP_LEA    = 8'h09,
    OP_ADD    = 8'h10, OP_ADDC   = 8'h11, OP_SUB    = 8'h12,
    OP_MUL    = 8'h14, OP_IMUL   = 8'h15,
    OP_DIV    = 8'h16, OP_IDIV   = 8'h17,
    OP_INC    = 8'h18, OP_DEC    = 8'h19, OP_NEG    = 8'h1A,
    OP_AND    = 8'h30, OP_OR     = 8'h31, OP_XOR    = 8'h32,
    OP_NOT    = 8'h33, OP_SHL    = 8'h34, OP_SHR    = 8'h35,
    OP_SAR    = 8'h36,
    OP_BSF    = 8'h3D, OP_BSR    = 8'h3E,
    OP_CMP    = 8'h50,
    OP_EQ     = 8'h54, OP_NE     = 8'h55, OP_GT     = 8'h56,
    OP_LT     = 8'h57, OP_GTE    = 8'h58, OP_LTE    = 8'h59,
    OP_JMP    = 8'h60, OP_GOTO   = 8'h61,
    OP_JE     = 8'h63, OP_JNE    = 8'h64, OP_JG     = 8'h65,
    OP_JGE    = 8'h66, OP_JL     = 8'h67, OP_JLE    = 8'h68,
    OP_CALL   = 8'h6C, OP_RET    = 8'h6D,
    OP_EXIT   = 8'h6F, OP_HALT   = 8'h70,
    OP_NOP    = 8'h80, OP_IN     = 8'h82, OP_OUT    = 8'h83,
    OP_INT    = 8'h84,
    // ── FPU opcodes (spec §5.4 / extension range 0x8e–0xef) ────────────
    OP_FCMP   = 8'h52, OP_DCMP   = 8'h53,   // compare → condition reg
    OP_FPU    = 8'h90;                        // FPU dispatch: sub-op in ir_imm[5:0]

// BIOS interrupt numbers
localparam [7:0]
    BIOS_PRINT_STR  = 8'h01,
    BIOS_PRINT_CHAR = 8'h02,
    BIOS_READ_CHAR  = 8'h03,
    BIOS_EXIT       = 8'h04;

// ---------------------------------------------------------------------------
// Register IDs for special registers
// ---------------------------------------------------------------------------
localparam [7:0]
    RID_SP = 8'd202, RID_SF = 8'd203,
    RID_BP = 8'd204, RID_BF = 8'd205;

// ---------------------------------------------------------------------------
// Memory (byte-addressed, 64 KB)
//   byte address → word address = addr[17:2]
// ---------------------------------------------------------------------------
reg [7:0] ram [0:RAM_WORDS*4-1];

// ---------------------------------------------------------------------------
// Flash ROM — loaded from FLASH_HEX via $readmemh at startup.
// Stores the CXE image as a flat byte array. The flash-loader FSM copies
// it into RAM before the CPU begins fetching instructions.
// Each hex file line = one byte (two hex digits, e.g. "01").
// ---------------------------------------------------------------------------
reg [7:0] flash_rom [0:FLASH_WORDS*4-1];

// Flash size register: how many bytes to copy (set by loader or parameter)
// We copy until we see the programmed byte count, tracked by flash_len.
// For simplicity we expose a parameter; cxe2hex.py writes a header comment
// with the real length, but here we rely on FLASH_HEX being sized exactly.
// flash_ptr walks 0..FLASH_WORDS*4-1.
integer flash_ptr;

// Latched entry point from CXE header (bytes 8..11 of flash image)
reg [31:0] flash_entry;

// Flash loader state: 0=idle, 1=loading, 2=done
// We use the main FSM state S_FLASH_LOAD / S_FLASH_ENTRY instead.

generate
    initial begin
        if (FLASH_HEX != "") begin
            $readmemh(FLASH_HEX, flash_rom);
        end
    end
endgenerate

// ---------------------------------------------------------------------------
// PC
// ---------------------------------------------------------------------------
reg [31:0] pc;
reg        cpu_halted;

assign halted  = cpu_halted;
assign dbg_pc  = pc;

// ---------------------------------------------------------------------------
// Register file  (206 × 32-bit)
//   layout follows REG_* macros in cxis.h:
//   0..31  = i,  128..159 = c,  160..191 = s,  192..201 = a,
//   202=sp  203=sf  204=bp  205=bf
// ---------------------------------------------------------------------------
reg [31:0] regfile [0:205];

assign dbg_i0 = regfile[0];

// Carry flag
reg carry;

// ---------------------------------------------------------------------------
// UART TX
// ---------------------------------------------------------------------------
reg  [7:0]  tx_byte;
reg         tx_valid;
wire        tx_ready;

// ---------------------------------------------------------------------------
// Hardware MUL / DIV units
// ---------------------------------------------------------------------------
reg         mul_start;
reg  [31:0] mul_a, mul_b;
wire [31:0] mul_result, mul_result_hi;
wire        mul_done;

reg         div_start;
reg  [31:0] div_a, div_b;
wire [31:0] div_quotient, div_remainder;
wire        div_done, div_zero;

cxis_mul u_mul (
    .clk        (clk),
    .rst_n      (rst_n),
    .start      (mul_start),
    .a          (mul_a),
    .b          (mul_b),
    .result     (mul_result),
    .result_hi  (mul_result_hi),
    .done       (mul_done)
);

cxis_div u_div (
    .clk        (clk),
    .rst_n      (rst_n),
    .start      (div_start),
    .a          (div_a),
    .b          (div_b),
    .quotient   (div_quotient),
    .remainder  (div_remainder),
    .done       (div_done),
    .div_zero   (div_zero)
);

// ---------------------------------------------------------------------------
// FPU unit
// ---------------------------------------------------------------------------
reg         fpu_start;
reg  [5:0]  fpu_op;
reg  [63:0] fpu_src_a, fpu_src_b;
wire [63:0] fpu_result;
wire        fpu_done;
wire [31:0] fpu_cmp_flags;

cxis_fpu u_fpu (
    .clk       (clk),
    .rst_n     (rst_n),
    .start     (fpu_start),
    .op        (fpu_op),
    .src_a     (fpu_src_a),
    .src_b     (fpu_src_b),
    .result    (fpu_result),
    .done      (fpu_done),
    .cmp_flags (fpu_cmp_flags)
);

uart_tx #(.BAUD_DIV(BAUD_DIV)) u_uart_tx (
    .clk      (clk),
    .rst_n    (rst_n),
    .tx_byte  (tx_byte),
    .tx_valid (tx_valid),
    .tx_ready (tx_ready),
    .uart_tx  (uart_tx)
);

// ---------------------------------------------------------------------------
// UART RX
// ---------------------------------------------------------------------------
wire [7:0]  rx_byte;
wire        rx_valid;

uart_rx #(.BAUD_DIV(BAUD_DIV)) u_uart_rx (
    .clk      (clk),
    .rst_n    (rst_n),
    .uart_rx  (uart_rx),
    .rx_byte  (rx_byte),
    .rx_valid (rx_valid)
);

// Simple 1-byte RX holding register
reg [7:0]  rx_hold;
reg        rx_has_byte;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_hold    <= 8'h00;
        rx_has_byte<= 1'b0;
    end else if (rx_valid && !rx_has_byte) begin
        rx_hold     <= rx_byte;
        rx_has_byte <= 1'b1;
    end
end

// ---------------------------------------------------------------------------
// Byte fetch helpers (combinational address → data lookup via tasks)
// ---------------------------------------------------------------------------
// We use a single "byte-fetch" state machine because RAM is synchronous.
// For simplicity this CPU uses a multi-cycle FSM rather than a pipeline
// to keep the Verilog readable.

// ---------------------------------------------------------------------------
// FSM states
// ---------------------------------------------------------------------------
localparam [4:0]
    S_RESET      = 5'd0,
    S_FETCH0     = 5'd1,   // fetch opcode byte
    S_FETCH1     = 5'd2,   // fetch mod byte
    S_FETCH2     = 5'd3,   // (optional) fetch ext/type byte
    S_DEC_SRC1   = 5'd4,   // fetch operand bytes
    S_DEC_SRC2   = 5'd5,
    S_DEC_DST    = 5'd6,
    S_DEC_IMM    = 5'd7,   // fetch immediate (up to 4 bytes)
    S_EXECUTE    = 5'd8,
    S_MEM_RD     = 5'd9,
    S_MEM_WR     = 5'd10,
    S_UART_TX    = 5'd11,  // wait for UART TX ready
    S_UART_RX    = 5'd12,  // wait for UART RX byte
    S_HALTED     = 5'd13,
    S_PUSH_PC    = 5'd14,  // CALL: write ret addr to stack
    S_STACK_LD   = 5'd15,  // POP  step 1: load word from stack
    S_BIOS_STR   = 5'd16,  // BIOS print_str loop
    S_BIOS_STR_TX= 5'd17,
    S_FLASH_LOAD = 5'd18,  // copy flash_rom → ram byte by byte
    S_FLASH_DONE = 5'd19,  // latch entry point, then go to S_FETCH0
    S_FETCH3     = 5'd20,  // read s1_class (EXT byte 2 of 3)
    S_MUL_WAIT   = 5'd21,  // waiting for hardware MUL unit (32 cycles)
    S_DIV_WAIT   = 5'd22,  // waiting for hardware DIV unit (32 cycles)
    S_FPU_WAIT   = 5'd23;  // waiting for FPU (4-cycle pipeline)

reg [4:0] state;

// Instruction registers
reg [7:0]  ir_opcode;
reg [7:0]  ir_mod;
reg [7:0]  ir_type;          // type byte 1: dst_class (when MOD_EXT set)
reg [7:0]  ir_type2;         // type byte 2: s1_class
reg [7:0]  ir_type3;         // type byte 3: s2_class
reg [7:0]  ir_src1_id;       // register id for src1
reg [7:0]  ir_src2_id;       // register id for src2  (or shift amount)
reg [7:0]  ir_dst_id;        // register id for dst
reg [31:0] ir_imm;           // immediate value
reg        ir_has_imm;       // mod has IMM flag
reg        ir_has_mem;       // mod has MEM flag
reg        ir_has_ext;       // mod has EXT flag (type byte follows)
reg [1:0]  ir_nops;          // operand count from mod[7:6]

// Decoded source/dest values
reg [31:0] src1_val;
reg [31:0] src2_val;
reg [31:0] dst_val;

// Sub-byte counter for multi-byte fetch
reg [2:0]  byte_cnt;
reg [31:0] imm_accum;

// Memory access holding
reg [31:0] mem_addr;
reg [31:0] mem_wdata;
reg [1:0]  mem_size;         // 0=byte 1=half 2=word

// BIOS print_str state
reg [31:0] str_ptr;
reg [7:0]  str_char;

// Temporary regs for execute (hoisted from blocks — Verilog-2001)
reg [31:0] tmp_a, tmp_b, tmp_loaded;
reg        tx_busy;          // set when UART TX in flight

// ---------------------------------------------------------------------------
// Helper: read register (combinational)
// ---------------------------------------------------------------------------
function [31:0] read_reg;
    input [7:0] rid;
    begin
        if (rid <= 8'd205)
            read_reg = regfile[rid];
        else
            read_reg = 32'h0;
    end
endfunction

// ---------------------------------------------------------------------------
// Helper: condition register index  (c0–c31 → ids 128–159)
// ---------------------------------------------------------------------------
// Not needed as a function since we do raw regfile indexing.

// ---------------------------------------------------------------------------
// Main FSM
// ---------------------------------------------------------------------------
integer bi;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= S_RESET;
        pc          <= 32'h00001000;   // VM_TEXT_BASE
        cpu_halted  <= 1'b0;
        carry       <= 1'b0;
        tx_valid    <= 1'b0;
        tx_busy     <= 1'b0;
        rx_has_byte <= 1'b0;
        flash_ptr   <= 0;
        flash_entry <= 32'h00001000;
        fpu_start   <= 1'b0;
        for (bi = 0; bi < 206; bi = bi + 1)
            regfile[bi] <= 32'h0;
        // initialise SP to VM_STACK_TOP
        regfile[RID_SP] <= 32'h0000F000;
        regfile[RID_BP] <= 32'h00000000;
    end else begin
        tx_valid  <= 1'b0;   // default — deasserted every cycle unless set
        mul_start <= 1'b0;   // default — pulse for 1 cycle only
        div_start <= 1'b0;   // default — pulse for 1 cycle only
        fpu_start <= 1'b0;   // default — pulse for 1 cycle only

        case (state)

        // ------------------------------------------------------------------
        S_RESET: begin
            if (FLASH_HEX != "")
                state <= S_FLASH_LOAD;
            else
                state <= S_FETCH0;
        end

        // ------------------------------------------------------------------
        // FLASH LOADER: copy flash_rom[0..N-1] into ram[0..N-1] one byte
        // per cycle, then latch entry point from CXE header bytes 8..11.
        // The CXE header is at flash_rom[0]:
        //   [0..3]  magic
        //   [4..5]  version
        //   [6..7]  flags
        //   [8..11] entry_point  (little-endian 32-bit)
        //   [12..13] section_count
        //   [14..19] reserved
        //   [20..]  section headers + data
        //
        // We copy ALL flash bytes starting at byte 0 into ram starting at
        // byte 0, so vaddr in section headers maps directly.  The CXE loader
        // (cxe2hex.py) pre-places section data at the correct offsets in the
        // flat hex image, so this is a verbatim memcpy.
        // ------------------------------------------------------------------
        S_FLASH_LOAD: begin
            ram[flash_ptr] <= flash_rom[flash_ptr];
            if (flash_ptr == FLASH_WORDS*4 - 1) begin
                state <= S_FLASH_DONE;
            end else begin
                flash_ptr <= flash_ptr + 1;
            end
        end

        S_FLASH_DONE: begin
            // Latch entry_point from CXE header (bytes 8..11, little-endian)
            flash_entry <= { flash_rom[11], flash_rom[10],
                             flash_rom[9],  flash_rom[8] };
            pc    <= { flash_rom[11], flash_rom[10],
                       flash_rom[9],  flash_rom[8] };
            state <= S_FETCH0;
        end

        // ------------------------------------------------------------------
        // FETCH: read opcode byte
        // ------------------------------------------------------------------
        S_FETCH0: begin
            if (cpu_halted) begin
                state <= S_HALTED;
            end else begin
                ir_opcode  <= ram[pc];
                ir_mod     <= 8'h00;
                ir_type    <= 8'h00;
                ir_type2   <= 8'h00;
                ir_type3   <= 8'h00;
                ir_has_imm <= 1'b0;
                ir_has_mem <= 1'b0;
                ir_has_ext <= 1'b0;
                ir_imm     <= 32'h0;
                ir_src1_id <= 8'hFF;
                ir_src2_id <= 8'hFF;
                ir_dst_id  <= 8'hFF;
                // NOP and single-byte instructions
                if (ram[pc] == OP_NOP) begin
                    pc    <= pc + 1;
                    state <= S_FETCH0;
                end else if (ram[pc] == OP_RET) begin
                    pc    <= pc + 1;
                    state <= S_EXECUTE;
                end else if (ram[pc] == OP_HALT ||
                             ram[pc] == OP_EXIT) begin
                    pc    <= pc + 1;
                    state <= S_EXECUTE;
                end else begin
                    pc    <= pc + 1;
                    state <= S_FETCH1;
                end
            end
        end

        // ------------------------------------------------------------------
        // Read mod byte
        // ------------------------------------------------------------------
        S_FETCH1: begin
            ir_mod     <= ram[pc];
            ir_nops    <= ram[pc][7:6];
            ir_has_imm <= ram[pc][5];
            ir_has_mem <= ram[pc][4];
            ir_has_ext <= ram[pc][3];
            pc         <= pc + 1;
            if (ram[pc][3]) begin          // MOD_EXT set → type byte next
                state <= S_FETCH2;
            end else begin
                byte_cnt  <= 3'd0;
                imm_accum <= 32'h0;
                state     <= S_DEC_SRC1;
            end
        end

        // ------------------------------------------------------------------
        // Read EXT type bytes: assembler emits dst_class, s1_class, s2_class
        // ------------------------------------------------------------------
        S_FETCH2: begin
            // byte 1 of 3: dst_class
            ir_type  <= ram[pc];
            pc       <= pc + 1;
            state    <= S_FETCH3;
        end

        S_FETCH3: begin
            // bytes 2+3: s1_class, s2_class (consume both in one cycle each)
            ir_type2  <= ram[pc];
            pc        <= pc + 1;
            // one more byte (s2_class) then go to operands
            // use byte_cnt=0 as flag to know we're reading s2_class next
            byte_cnt  <= 3'd0;
            imm_accum <= 32'h0;
            state     <= S_DEC_SRC1;
            // peek-ahead: we still need to read s2_class byte here or next cycle
            // We'll handle s2_class at the start of S_DEC_SRC1 when EXT is set
        end

        // ------------------------------------------------------------------
        // Decode operand bytes.
        //
        // Assembler emit order (emit_instr in cxas.c):
        //   opcode | mod | [dst_class s1_class s2_class] | reg_ops... | imm/mem...
        //
        // So after type bytes, we read:
        //   - all register operand ids in order (src1, src2 if nops>=2, dst last)
        //   - then imm bytes if IMM set
        //   (for nops=01+IMM: dst reg, then imm)
        //   (for nops=10+IMM: src1 reg, dst reg, then imm)
        //   (for nops=11+IMM: src1 reg, src2/imm..., dst reg — but we'll handle 3-op later)
        //
        // ------------------------------------------------------------------
        // Decode operand bytes.
        //
        // Assembler emit order (cxas.c emit_instr):
        //   opcode | mod | [dst_class s1_class s2_class] | ALL REGs in order | ALL IMMs
        //
        // num_reg_ops = nops - (has_imm ? 1 : 0):
        //   nops=01 no IMM → 1 reg (dst)             e.g. INC dst
        //   nops=01 + IMM  → 0 regs, IMM only        e.g. INT imm32
        //   nops=10 no IMM → 2 regs (src1, dst)      e.g. MOV reg, dst
        //   nops=10 + IMM  → 1 reg (dst), then IMM   e.g. MOV imm, dst
        //   nops=11 no IMM → 3 regs (src1, src2, dst)
        //   nops=11 + IMM  → 2 regs (src1, dst), then IMM  e.g. ADD src,imm,dst
        // ------------------------------------------------------------------
        S_DEC_SRC1: begin
            if (ir_has_ext && byte_cnt == 3'd0) begin
                // consume s2_class (3rd EXT type byte), stay to read regs
                ir_type3 <= ram[pc];
                pc       <= pc + 1;
                byte_cnt <= 3'd1;
            end else begin
                if (ir_nops == 2'b00) begin
                    state <= S_EXECUTE;

                // nops=01+IMM: 0 regs → IMM → EXECUTE  (e.g. INT imm32)
                end else if (ir_nops == 2'b01 && ir_has_imm) begin
                    byte_cnt  <= 3'd0;
                    imm_accum <= 32'h0;
                    state     <= S_DEC_IMM;

                // nops=01 no IMM: 1 reg = dst  (e.g. INC dst)
                end else if (ir_nops == 2'b01) begin
                    ir_dst_id <= ram[pc];
                    pc        <= pc + 1;
                    state     <= S_EXECUTE;

                // nops=10+IMM: 1 reg = dst, then IMM  (e.g. MOV imm,dst)
                end else if (ir_nops == 2'b10 && ir_has_imm) begin
                    ir_dst_id <= ram[pc];
                    pc        <= pc + 1;
                    byte_cnt  <= 3'd0;
                    imm_accum <= 32'h0;
                    state     <= S_DEC_IMM;

                // nops=10 no IMM or nops=11: read src1 first
                end else begin
                    ir_src1_id <= ram[pc];
                    pc         <= pc + 1;
                    state      <= S_DEC_SRC2;
                end
            end
        end

        S_DEC_IMM: begin
            case (byte_cnt)
                3'd0: imm_accum[7:0]   <= ram[pc];
                3'd1: imm_accum[15:8]  <= ram[pc];
                3'd2: imm_accum[23:16] <= ram[pc];
                3'd3: imm_accum[31:24] <= ram[pc];
                default: ;
            endcase
            pc       <= pc + 1;
            byte_cnt <= byte_cnt + 3'd1;
            if (byte_cnt == 3'd3) begin
                ir_imm     <= {ram[pc], imm_accum[23:0]};
                ir_has_imm <= 1'b1;
                state      <= S_EXECUTE;
            end
        end

        S_DEC_SRC2: begin
            // Reached from nops=10 no IMM or nops=11 (after src1)
            if (ir_nops == 2'b10) begin
                // 2 regs, no IMM: this byte is dst
                ir_dst_id <= ram[pc];
                pc        <= pc + 1;
                state     <= S_EXECUTE;
            end else if (ir_has_imm) begin
                // nops=11+IMM: regs are src1 and dst; this byte is dst
                ir_dst_id <= ram[pc];
                pc        <= pc + 1;
                byte_cnt  <= 3'd0;
                imm_accum <= 32'h0;
                state     <= S_DEC_IMM;
            end else begin
                // nops=11 no IMM: src2 reg, then dst
                ir_src2_id <= ram[pc];
                pc         <= pc + 1;
                state      <= S_DEC_DST;
            end
        end

        S_DEC_DST: begin
            ir_dst_id <= ram[pc];
            pc        <= pc + 1;
            state     <= S_EXECUTE;
        end

        // ------------------------------------------------------------------
        // EXECUTE
        // ------------------------------------------------------------------
        S_EXECUTE: begin
            src1_val <= (ir_src1_id != 8'hFF) ? read_reg(ir_src1_id) : 32'h0;
            src2_val <= ir_has_imm ? ir_imm :
                        (ir_src2_id != 8'hFF) ? read_reg(ir_src2_id) : 32'h0;
            dst_val  <= (ir_dst_id != 8'hFF) ? read_reg(ir_dst_id)  : 32'h0;
            // Blocking latches (bit-select on function call not legal in Verilog-2001)
            tmp_b = (ir_src2_id != 8'hFF) ? read_reg(ir_src2_id) : 32'h0;
            tmp_a = (ir_src1_id != 8'hFF) ? read_reg(ir_src1_id) : 32'h0;

            case (ir_opcode)

            // ---- Data movement -------------------------------------------
            OP_MOV, OP_MOVSX, OP_MOVZX: begin
                // src → dst  (immediate or register)
                if (ir_has_imm)
                    regfile[ir_dst_id] <= ir_imm;
                else
                    regfile[ir_dst_id] <= read_reg(ir_src1_id);
                state <= S_FETCH0;
            end

            // ---- Arithmetic ----------------------------------------------
            OP_ADD: begin
                regfile[ir_dst_id] <= read_reg(ir_src1_id) + (ir_has_imm ? ir_imm : read_reg(ir_src2_id));
                state <= S_FETCH0;
            end
            OP_ADDC: begin
                regfile[ir_dst_id] <= read_reg(ir_src1_id) + read_reg(ir_src2_id) + {31'h0, carry};
                state <= S_FETCH0;
            end
            OP_SUB: begin
                regfile[ir_dst_id] <= read_reg(ir_src1_id) - (ir_has_imm ? ir_imm : read_reg(ir_src2_id));
                state <= S_FETCH0;
            end
            OP_MUL: begin
                mul_a     <= read_reg(ir_src1_id);
                mul_b     <= ir_has_imm ? ir_imm : read_reg(ir_src2_id);
                mul_start <= 1'b1;
                state     <= S_MUL_WAIT;
            end
            OP_IMUL: begin
                mul_a     <= read_reg(ir_src1_id);
                mul_b     <= ir_has_imm ? ir_imm : read_reg(ir_src2_id);
                mul_start <= 1'b1;
                state     <= S_MUL_WAIT;
            end
            OP_DIV: begin
                div_a     <= read_reg(ir_src1_id);
                div_b     <= ir_has_imm ? ir_imm : read_reg(ir_src2_id);
                div_start <= 1'b1;
                state     <= S_DIV_WAIT;
            end
            OP_IDIV: begin
                div_a     <= read_reg(ir_src1_id);
                div_b     <= ir_has_imm ? ir_imm : read_reg(ir_src2_id);
                div_start <= 1'b1;
                state     <= S_DIV_WAIT;
            end
            OP_INC: begin
                regfile[ir_dst_id] <= read_reg(ir_dst_id) + 32'h1;
                state <= S_FETCH0;
            end
            OP_DEC: begin
                regfile[ir_dst_id] <= read_reg(ir_dst_id) - 32'h1;
                state <= S_FETCH0;
            end
            OP_NEG: begin
                regfile[ir_dst_id] <= -read_reg(ir_src1_id);
                state <= S_FETCH0;
            end

            // ---- Bitwise -------------------------------------------------
            OP_AND: begin
                regfile[ir_dst_id] <= read_reg(ir_src1_id) & read_reg(ir_src2_id);
                state <= S_FETCH0;
            end
            OP_OR: begin
                regfile[ir_dst_id] <= read_reg(ir_src1_id) | read_reg(ir_src2_id);
                state <= S_FETCH0;
            end
            OP_XOR: begin
                regfile[ir_dst_id] <= read_reg(ir_src1_id) ^ read_reg(ir_src2_id);
                state <= S_FETCH0;
            end
            OP_NOT: begin
                regfile[ir_dst_id] <= ~read_reg(ir_src1_id);
                state <= S_FETCH0;
            end
            OP_SHL: begin
                regfile[ir_dst_id] <= read_reg(ir_src1_id) << (ir_has_imm ? ir_imm[4:0] : tmp_b[4:0]);
                state <= S_FETCH0;
            end
            OP_SHR: begin
                regfile[ir_dst_id] <= read_reg(ir_src1_id) >> (ir_has_imm ? ir_imm[4:0] : tmp_b[4:0]);
                state <= S_FETCH0;
            end
            OP_SAR: begin
                regfile[ir_dst_id] <= $signed(read_reg(ir_src1_id)) >>> (ir_has_imm ? ir_imm[4:0] : tmp_b[4:0]);
                state <= S_FETCH0;
            end

            // ---- Compare & condition -------------------------------------
            OP_CMP: begin
                // CMP src1, src2, c_dst   — sets condition register
                // c_dst encodes: bit1=less, bit0=equal
                // tmp_a = src1, tmp_b = src2 (blocking-latched above)
                regfile[ir_dst_id] <= {30'h0, (tmp_a < tmp_b), (tmp_a == tmp_b)};
                state <= S_FETCH0;
            end
            OP_EQ: begin
                regfile[ir_dst_id] <= (read_reg(ir_src1_id) == read_reg(ir_src2_id)) ? 32'h1 : 32'h0;
                state <= S_FETCH0;
            end
            OP_NE: begin
                regfile[ir_dst_id] <= (read_reg(ir_src1_id) != read_reg(ir_src2_id)) ? 32'h1 : 32'h0;
                state <= S_FETCH0;
            end
            OP_GT: begin
                regfile[ir_dst_id] <= ($signed(tmp_a) > $signed(tmp_b)) ? 32'h1 : 32'h0;
                state <= S_FETCH0;
            end
            OP_LT: begin
                regfile[ir_dst_id] <= ($signed(tmp_a) < $signed(tmp_b)) ? 32'h1 : 32'h0;
                state <= S_FETCH0;
            end
            OP_GTE: begin
                regfile[ir_dst_id] <= ($signed(tmp_a) >= $signed(tmp_b)) ? 32'h1 : 32'h0;
                state <= S_FETCH0;
            end
            OP_LTE: begin
                regfile[ir_dst_id] <= ($signed(tmp_a) <= $signed(tmp_b)) ? 32'h1 : 32'h0;
                state <= S_FETCH0;
            end

            // ---- Branches ------------------------------------------------
            OP_JMP, OP_GOTO: begin
                // Absolute address in src1 (register) or immediate
                if (ir_has_imm)
                    pc <= pc + ir_imm;   // relative jump
                else
                    pc <= read_reg(ir_src1_id);
                state <= S_FETCH0;
            end
            OP_JE: begin
                // JE c0, offset — jump if c0 LSB (equal) set
                tmp_a = read_reg(ir_src1_id);
                if (tmp_a[0])
                    pc <= pc + {{16{ir_imm[15]}}, ir_imm[15:0]};
                state <= S_FETCH0;
            end
            OP_JNE: begin
                tmp_a = read_reg(ir_src1_id);
                if (!tmp_a[0])
                    pc <= pc + {{16{ir_imm[15]}}, ir_imm[15:0]};
                state <= S_FETCH0;
            end
            OP_JG: begin
                // greater: not less AND not equal
                tmp_a = read_reg(ir_src1_id);
                if (!tmp_a[1] && !tmp_a[0])
                    pc <= pc + {{16{ir_imm[15]}}, ir_imm[15:0]};
                state <= S_FETCH0;
            end
            OP_JGE: begin
                tmp_a = read_reg(ir_src1_id);
                if (!tmp_a[1])
                    pc <= pc + {{16{ir_imm[15]}}, ir_imm[15:0]};
                state <= S_FETCH0;
            end
            OP_JL: begin
                tmp_a = read_reg(ir_src1_id);
                if (tmp_a[1])
                    pc <= pc + {{16{ir_imm[15]}}, ir_imm[15:0]};
                state <= S_FETCH0;
            end
            OP_JLE: begin
                tmp_a = read_reg(ir_src1_id);
                if (tmp_a[1] || tmp_a[0])
                    pc <= pc + {{16{ir_imm[15]}}, ir_imm[15:0]};
                state <= S_FETCH0;
            end

            // ---- Stack & call --------------------------------------------
            OP_PUSH: begin
                regfile[RID_SP] <= regfile[RID_SP] - 32'd4;
                mem_addr        <= regfile[RID_SP] - 32'd4;
                mem_wdata       <= read_reg(ir_src1_id);
                state           <= S_MEM_WR;
            end
            OP_POP: begin
                mem_addr  <= regfile[RID_SP];
                state     <= S_STACK_LD;
            end
            OP_CALL: begin
                // Push return address, then jump
                regfile[RID_SP] <= regfile[RID_SP] - 32'd4;
                mem_addr        <= regfile[RID_SP] - 32'd4;
                mem_wdata       <= pc;   // already past the CALL instr
                state           <= S_PUSH_PC;
            end
            OP_RET: begin
                mem_addr  <= regfile[RID_SP];
                state     <= S_STACK_LD;
            end

            // ---- I/O & UART ----------------------------------------------
            // OUT port, val   → port 0 = UART TX byte
            OP_OUT: begin
                tx_byte  <= ir_has_imm ? ir_imm[7:0] : tmp_b[7:0];
                tx_valid <= 1'b1;
                tx_busy  <= 1'b1;
                state    <= S_UART_TX;
            end
            // IN dst, port   → reads UART RX into dst
            OP_IN: begin
                state <= S_UART_RX;
            end

            // ---- BIOS interrupts  ----------------------------------------
            OP_INT: begin
                case (ir_imm[7:0])
                BIOS_PRINT_CHAR: begin
                    tx_byte  <= regfile[192][7:0];
                    tx_valid <= 1'b1;
                    tx_busy  <= 1'b1;
                    state    <= S_UART_TX;
                end
                BIOS_PRINT_STR: begin
                    // a0 = ptr, a1 = len (ignored — send until NUL)
                    str_ptr <= regfile[192];
                    state   <= S_BIOS_STR;
                end
                BIOS_READ_CHAR: begin
                    state <= S_UART_RX;
                end
                BIOS_EXIT: begin
                    cpu_halted <= 1'b1;
                    state      <= S_HALTED;
                end
                default: state <= S_FETCH0;
                endcase
            end

            // ---- Halt / Exit ---------------------------------------------
            OP_HALT, OP_EXIT: begin
                cpu_halted <= 1'b1;
                state      <= S_HALTED;
            end

            // ---- FPU dispatch  (OP_FPU 0x90 + sub-op in ir_imm[5:0]) ----
            // Also handles OP_FCMP (0x52) and OP_DCMP (0x53) via FPU unit.
            //
            // Encoding for OP_FPU:  op=0x90, mod encodes src1/src2/dst
            //   ir_imm[5:0] = FPU sub-opcode (matches cxis_fpu.v localparams)
            //   src1_val → fpu_src_a  (f32: zero-extended from regfile[31:0])
            //   src2_val → fpu_src_b
            //   result   → regfile[ir_dst_id][31:0] (f32) or l-reg pair (f64)
            //
            // For FCMP/DCMP: result goes to condition register (ir_dst_id).
            // -----------------------------------------------------------------
            // ---- Scalar f32 arithmetic (assembler mnemonics 0x0A,0x20-0x24,0x95)
            8'h0A: begin  // OP_FMOV: immediate → reg (raw f32 bits) or reg→reg
                if (ir_has_imm)
                    regfile[ir_dst_id] <= ir_imm;
                else
                    regfile[ir_dst_id] <= read_reg(ir_src1_id);
                state <= S_FETCH0;
            end
            8'h20: begin  // OP_FADD
                fpu_op <= 6'd0; fpu_src_a <= {32'h0,tmp_a}; fpu_src_b <= {32'h0,tmp_b};
                fpu_start <= 1'b1; state <= S_FPU_WAIT;
            end
            8'h21: begin  // OP_FSUB
                fpu_op <= 6'd1; fpu_src_a <= {32'h0,tmp_a}; fpu_src_b <= {32'h0,tmp_b};
                fpu_start <= 1'b1; state <= S_FPU_WAIT;
            end
            8'h22: begin  // OP_FMUL
                fpu_op <= 6'd2; fpu_src_a <= {32'h0,tmp_a}; fpu_src_b <= {32'h0,tmp_b};
                fpu_start <= 1'b1; state <= S_FPU_WAIT;
            end
            8'h23: begin  // OP_FDIV
                fpu_op <= 6'd3; fpu_src_a <= {32'h0,tmp_a}; fpu_src_b <= {32'h0,tmp_b};
                fpu_start <= 1'b1; state <= S_FPU_WAIT;
            end
            8'h24: begin  // OP_FNEG
                fpu_op <= 6'd4; fpu_src_a <= {32'h0,tmp_a}; fpu_src_b <= 64'h0;
                fpu_start <= 1'b1; state <= S_FPU_WAIT;
            end
            8'h95: begin  // OP_FTOI: f32 → int32
                fpu_op <= 6'd32; fpu_src_a <= {32'h0,tmp_a}; fpu_src_b <= 64'h0;
                fpu_start <= 1'b1; state <= S_FPU_WAIT;
            end

            OP_FPU, OP_FCMP, OP_DCMP: begin
                // OP_FPU (0x90) also covers OP_ITOF from cxis.h.
                // When no immediate: treat as ITOF (int reg → float reg, sub-op 30).
                if (ir_opcode == OP_FCMP)
                    fpu_op <= 6'd28;
                else if (ir_opcode == OP_DCMP)
                    fpu_op <= 6'd29;
                else if (ir_has_imm)
                    fpu_op <= ir_imm[5:0];
                else
                    fpu_op <= 6'd30;  // FPU_ITOF
                fpu_src_a <= {32'h0, tmp_a};
                fpu_src_b <= {32'h0, tmp_b};
                fpu_start <= 1'b1;
                state     <= S_FPU_WAIT;
            end

            default: begin
                // Undefined — treat as NOP
                state <= S_FETCH0;
            end
            endcase
        end

        // ------------------------------------------------------------------
        // Memory write (byte-addressed, 32-bit store)
        // ------------------------------------------------------------------
        S_MEM_WR: begin
            ram[mem_addr + 0] <= mem_wdata[7:0];
            ram[mem_addr + 1] <= mem_wdata[15:8];
            ram[mem_addr + 2] <= mem_wdata[23:16];
            ram[mem_addr + 3] <= mem_wdata[31:24];
            state <= S_FETCH0;
        end

        // ------------------------------------------------------------------
        // Memory read (for POP / RET)
        // ------------------------------------------------------------------
        S_STACK_LD: begin
            tmp_loaded = {ram[mem_addr+3], ram[mem_addr+2],
                          ram[mem_addr+1], ram[mem_addr+0]};
            if (ir_opcode == OP_RET) begin
                pc <= tmp_loaded;
            end else begin
                regfile[ir_dst_id] <= tmp_loaded;
            end
            regfile[RID_SP] <= regfile[RID_SP] + 32'd4;
            state <= S_FETCH0;
        end

        // ------------------------------------------------------------------
        // CALL: write PC to stack, then jump
        // ------------------------------------------------------------------
        S_PUSH_PC: begin
            ram[mem_addr + 0] <= mem_wdata[7:0];
            ram[mem_addr + 1] <= mem_wdata[15:8];
            ram[mem_addr + 2] <= mem_wdata[23:16];
            ram[mem_addr + 3] <= mem_wdata[31:24];
            // Jump target: immediate (relative) or register
            if (ir_has_imm)
                pc <= pc + ir_imm;
            else
                pc <= read_reg(ir_src1_id);
            state <= S_FETCH0;
        end

        // ------------------------------------------------------------------
        // UART TX wait
        // wait for UART to accept (tx_ready drops) then finish (tx_ready rises)
        // ------------------------------------------------------------------
        S_UART_TX: begin
            if (!tx_ready) begin
                tx_busy <= 1'b0;  // UART accepted the byte
            end
            if (tx_busy == 1'b0 && tx_ready) begin
                state <= S_FETCH0;
            end
        end

        // ------------------------------------------------------------------
        // UART RX wait
        // ------------------------------------------------------------------
        S_UART_RX: begin
            if (rx_has_byte) begin
                if (ir_opcode == OP_IN)
                    regfile[ir_dst_id] <= {24'h0, rx_hold};
                else  // BIOS_READ_CHAR → a0
                    regfile[192] <= {24'h0, rx_hold};
                rx_has_byte <= 1'b0;
                state       <= S_FETCH0;
            end
        end

        // ------------------------------------------------------------------
        // BIOS PRINT_STR: scan bytes from RAM until NUL, TX each
        // ------------------------------------------------------------------
        S_BIOS_STR: begin
            str_char <= ram[str_ptr];
            if (ram[str_ptr] == 8'h00) begin
                state <= S_FETCH0;
            end else begin
                tx_byte  <= ram[str_ptr];
                tx_valid <= 1'b1;
                tx_busy  <= 1'b1;
                str_ptr  <= str_ptr + 32'd1;
                state    <= S_BIOS_STR_TX;
            end
        end

        S_BIOS_STR_TX: begin
            if (!tx_ready) begin
                tx_busy <= 1'b0;
            end
            if (tx_busy == 1'b0 && tx_ready) begin
                state <= S_BIOS_STR;
            end
        end

        // ── Hardware MUL wait (32 cycles) ─────────────────────────────────
        S_MUL_WAIT: begin
            if (mul_done) begin
                regfile[ir_dst_id] <= mul_result;
                state <= S_FETCH0;
            end
        end

        // ── Hardware DIV wait (32 cycles) ─────────────────────────────────
        S_DIV_WAIT: begin
            if (div_done) begin
                regfile[ir_dst_id] <= div_zero ? 32'hFFFFFFFF : div_quotient;
                state <= S_FETCH0;
            end
        end

        // ── FPU wait (4-cycle pipeline) ────────────────────────────────────
        // For FCMP/DCMP: write cmp_flags bits to condition register (LSB).
        // For all others: write lower 32 bits of result to dst register.
        // f64 ops store only the lo word here; full 64-bit support would need
        // an l-register file extension.
        // ------------------------------------------------------------------
        S_FPU_WAIT: begin
            if (fpu_done) begin
                if (ir_opcode == OP_FCMP || ir_opcode == OP_DCMP ||
                    (ir_opcode == OP_FPU &&
                     (fpu_op == 6'd28 || fpu_op == 6'd29))) begin
                    // Compare: write flags to condition reg
                    // cmp_flags[0]=equal, cmp_flags[1]=less
                    regfile[ir_dst_id] <= {30'h0, fpu_cmp_flags[1:0]};
                end else begin
                    // Arithmetic/conversion: write result to dst
                    regfile[ir_dst_id] <= fpu_result[31:0];
                end
                state <= S_FETCH0;
            end
        end

        // ------------------------------------------------------------------
        S_HALTED: begin
            // Sit here forever
        end

        default: state <= S_RESET;
        endcase
    end
end

endmodule


// =============================================================================
//  UART TX  —  8N1  (BAUD_DIV = clk_freq / baud_rate)
// =============================================================================
module uart_tx #(
    parameter BAUD_DIV = 868
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] tx_byte,
    input  wire       tx_valid,
    output reg        tx_ready,
    output reg        uart_tx
);

reg [15:0] baud_cnt;
reg [3:0]  bit_idx;
reg [9:0]  shift_reg;   // start + 8 data + stop
reg        active;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        uart_tx  <= 1'b1;
        tx_ready <= 1'b1;
        active   <= 1'b0;
        baud_cnt <= 16'h0;
        bit_idx  <= 4'h0;
    end else begin
        if (!active) begin
            tx_ready <= 1'b1;
            uart_tx  <= 1'b1;
            if (tx_valid) begin
                shift_reg <= {1'b1, tx_byte, 1'b0}; // stop | data | start
                active    <= 1'b1;
                tx_ready  <= 1'b0;
                baud_cnt  <= 16'h0;
                bit_idx   <= 4'h0;
            end
        end else begin
            tx_ready <= 1'b0;
            if (baud_cnt == BAUD_DIV - 1) begin
                baud_cnt <= 16'h0;
                uart_tx  <= shift_reg[0];
                shift_reg<= {1'b1, shift_reg[9:1]};
                bit_idx  <= bit_idx + 4'h1;
                if (bit_idx == 4'd9) begin
                    active <= 1'b0;
                end
            end else begin
                baud_cnt <= baud_cnt + 16'h1;
            end
        end
    end
end
endmodule


// =============================================================================
//  UART RX  —  8N1
// =============================================================================
module uart_rx #(
    parameter BAUD_DIV = 868
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       uart_rx,
    output reg  [7:0] rx_byte,
    output reg        rx_valid
);

reg [15:0] baud_cnt;
reg [3:0]  bit_idx;
reg [7:0]  shift_reg;
reg        active;
reg        rx_d, rx_dd;    // two-stage synchroniser

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_d      <= 1'b1;
        rx_dd     <= 1'b1;
        active    <= 1'b0;
        rx_valid  <= 1'b0;
        baud_cnt  <= 16'h0;
        bit_idx   <= 4'h0;
        shift_reg <= 8'h0;
    end else begin
        rx_d  <= uart_rx;
        rx_dd <= rx_d;
        rx_valid <= 1'b0;

        if (!active) begin
            // Detect start bit (falling edge on idle line)
            if (!rx_dd) begin
                active   <= 1'b1;
                // Sample in middle of first data bit
                baud_cnt <= BAUD_DIV[15:1];  // half-period offset
                bit_idx  <= 4'h0;
            end
        end else begin
            if (baud_cnt == BAUD_DIV - 1) begin
                baud_cnt  <= 16'h0;
                if (bit_idx < 4'd8) begin
                    shift_reg <= {rx_dd, shift_reg[7:1]};
                    bit_idx   <= bit_idx + 4'h1;
                end else begin
                    // Stop bit
                    active   <= 1'b0;
                    rx_byte  <= shift_reg;
                    rx_valid <= 1'b1;
                end
            end else begin
                baud_cnt <= baud_cnt + 16'h1;
            end
        end
    end
end
endmodule