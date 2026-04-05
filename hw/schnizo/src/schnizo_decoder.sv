// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// The decoder for the Schnizo Core. Based on CVA6.
module schnizo_decoder import schnizo_pkg::*; import riscv_instr::*; #(
  parameter int unsigned XLEN        = 32,
  parameter bit          Xdma        = 0,
  parameter bit          Xfrep       = 1,
  /// Enable F Extension (single).
  parameter bit          RVF         = 1,
  /// Enable D Extension (double).
  parameter bit          RVD         = 0,
  /// Spatz: Enable RVV Extension (vector).
  parameter bit          RVV          = 1,
  parameter bit          XF16        = 0,
  parameter bit          XF16ALT     = 0,
  parameter bit          XF8         = 0,
  parameter bit          XF8ALT      = 0,
  parameter type         instr_dec_t = logic
) (
  // For assertions only.
  input logic                   clk_i,
  input logic                   rst_i,

  input  logic [31:0]           instr_fetch_data_i,
  input  logic                  instr_fetch_data_valid_i,
  input  fpnew_pkg::roundmode_e fpu_round_mode_i,
  input  fpnew_pkg::fmt_mode_t  fpu_fmt_mode_i,
  output logic                  instr_valid_o,
  output logic                  instr_illegal_o,
  output instr_dec_t            instr_dec_o
);

  // --------------------
  // Instruction Types
  // --------------------
  typedef struct packed {
    logic [31:25] funct7;
    logic [24:20] rs2;
    logic [19:15] rs1;
    logic [14:12] funct3;
    logic [11:7]  rd;
    logic [6:0]   opcode;
  } rtype_t;

  typedef struct packed {
    logic [31:27] rs3;
    logic [26:25] funct2;
    logic [24:20] rs2;
    logic [19:15] rs1;
    logic [14:12] funct3;
    logic [11:7]  rd;
    logic [6:0]   opcode;
  } r4type_t;

  typedef struct packed {
    logic [31:20] imm;
    logic [19:15] rs1;
    logic [14:12] funct3;
    logic [11:7]  rd;
    logic [6:0]   opcode;
  } itype_t;

  typedef struct packed {
    logic [31:25] imm;
    logic [24:20] rs2;
    logic [19:15] rs1;
    logic [14:12] funct3;
    logic [11:7]  imm0;
    logic [6:0]   opcode;
  } stype_t;

  typedef struct packed {
    logic [31:12] imm;
    logic [11:7]  rd;
    logic [6:0]   opcode;
  } utype_t;

  typedef struct packed {
    logic [31:27] funct5;
    logic [26:25] fmt;
    logic [24:20] rs2;
    logic [19:15] rs1;
    logic [14:12] rm;
    logic [11:7]  rd;
    logic [6:0]   opcode;
  } rftype_t;  // floating-point

  typedef struct packed {
    logic [31:27] funct5;
    logic         aq;
    logic         rl;
    logic [24:20] rs2;
    logic [19:15] rs1;
    logic [14:12] funct3;
    logic [11:7]  rd;
    logic [6:0]   opcode;
  } atype_t;  // atomic

  typedef struct packed {
    logic [31:20] max_instr;
    logic [19:15] max_iters_reg;
    logic [14:12] stagger_max; // only for snitch
    logic [11:8]  stagger_mask; // only for snitch
    logic         frep_mode;
    logic [6:0]   opcode;
  } freptype_t;  // FREP

  typedef union packed {
    logic [31:0] instr;
    rtype_t      rtype;
    r4type_t     r4type;
    rftype_t     rftype;
    itype_t      itype;
    stype_t      stype;
    utype_t      utype;
    atype_t      atype;
    freptype_t   freptype;
  } instruction_t;

  // --------------------
  // Opcodes
  // --------------------
  // RV32/64G listings:
  // Quadrant 0
  localparam logic[6:0] OpcodeLoad =    7'b00_000_11;
  localparam logic[6:0] OpcodeLoadFp =  7'b00_001_11;
  localparam logic[6:0] OpcodeCustom0 = 7'b00_010_11;
  localparam logic[6:0] OpcodeMiscMem = 7'b00_011_11;
  localparam logic[6:0] OpcodeOpImm =   7'b00_100_11;
  localparam logic[6:0] OpcodeAuipc =   7'b00_101_11;
  localparam logic[6:0] OpcodeOpImm32 = 7'b00_110_11;
  // Quadrant 1
  localparam logic[6:0] OpcodeStore =   7'b01_000_11;
  localparam logic[6:0] OpcodeStoreFp = 7'b01_001_11;
  localparam logic[6:0] OpcodeCustom1 = 7'b01_010_11;
  localparam logic[6:0] OpcodeAmo =     7'b01_011_11;
  localparam logic[6:0] OpcodeOpReg =   7'b01_100_11;
  localparam logic[6:0] OpcodeLui =     7'b01_101_11;
  localparam logic[6:0] OpcodeOp32 =    7'b01_110_11;
  // Quadrant 2
  localparam logic[6:0] OpcodeMadd =    7'b10_000_11;
  localparam logic[6:0] OpcodeMsub =    7'b10_001_11;
  localparam logic[6:0] OpcodeNmsub =   7'b10_010_11;
  localparam logic[6:0] OpcodeNmadd =   7'b10_011_11;
  localparam logic[6:0] OpcodeOpFp =    7'b10_100_11;
  localparam logic[6:0] OpcodeVec =     7'b10_101_11;
  localparam logic[6:0] OpcodeCustom2 = 7'b10_110_11;
  // Quadrant 3
  localparam logic[6:0] OpcodeBranch =  7'b11_000_11;
  localparam logic[6:0] OpcodeJalr =    7'b11_001_11;
  localparam logic[6:0] OpcodeRsrvd2 =  7'b11_010_11;
  localparam logic[6:0] OpcodeJal =     7'b11_011_11;
  localparam logic[6:0] OpcodeSystem =  7'b11_100_11;
  localparam logic[6:0] OpcodeRsrvd3 =  7'b11_101_11;
  localparam logic[6:0] OpcodeCustom3 = 7'b11_110_11;

  // --------------------
  // Immediate select
  // --------------------
  typedef enum logic [3:0] {
    NOIMM,
    IIMM,
    SIMM,
    SBIMM,
    UIMM,
    JIMM,
    RS3,
    MUX_RD_RS3
  } imm_select_e;
  imm_select_e imm_select;

  logic [XLEN-1:0] imm_i_type;
  logic [XLEN-1:0] imm_s_type;
  logic [XLEN-1:0] imm_sb_type;
  logic [XLEN-1:0] imm_u_type;
  logic [XLEN-1:0] imm_uj_type;

  // --------------------
  // Decoder
  // --------------------
  // Cast instruction encoding to union struct for simplified decoder
  instruction_t instr;
  assign instr = instruction_t'(instr_fetch_data_i);

  // this instruction needs floating-point rounding-mode verification
  logic check_fpround_mode;

  logic illegal_instr;

  assign instr_valid_o = instr_fetch_data_valid_i & ~illegal_instr;
  assign instr_illegal_o = instr_fetch_data_valid_i & illegal_instr;

  always_comb begin
    illegal_instr = 1'b0;
    imm_select = NOIMM;

    instr_dec_o.fu = schnizo_pkg::NONE;
    instr_dec_o.alu_op = schnizo_pkg::AluOpAdd;
    instr_dec_o.lsu_op = schnizo_pkg::LsuOpLoad;
    instr_dec_o.csr_op = schnizo_pkg::CsrOpNone;
    instr_dec_o.fpu_op = schnizo_pkg::FpuOpFadd;
    // Set the default rd and rs_is_fp to zero such that if there is no write back required
    // we target register x0. x0 is read only and thus we have encoded that we have no write.
    instr_dec_o.rd        = '0;
    instr_dec_o.rd_is_fp  = 0;
    instr_dec_o.rs1       = '0;
    instr_dec_o.rs1_is_fp = 0;
    instr_dec_o.rs2       = '0;
    instr_dec_o.rs2_is_fp = 0;
    instr_dec_o.use_imm_as_rs3 = 1'b0;
    instr_dec_o.lsu_size       = Word;
    instr_dec_o.fpu_fmt_src    = fpnew_pkg::FP32;
    instr_dec_o.fpu_fmt_dst    = fpnew_pkg::FP32;
    instr_dec_o.fpu_rnd_mode   = fpnew_pkg::RNE;
    instr_dec_o.use_pc_as_op_a      = 1'b0;
    instr_dec_o.use_rs1addr_as_op_a = 1'b0;
    instr_dec_o.is_branch  = 1'b0;
    instr_dec_o.is_jal     = 1'b0;
    instr_dec_o.is_jalr    = 1'b0;
    instr_dec_o.is_fence   = 1'b0;
    instr_dec_o.is_fence_i = 1'b0;
    instr_dec_o.is_ecall   = 1'b0;
    instr_dec_o.is_ebreak  = 1'b0;
    instr_dec_o.is_mret    = 1'b0;
    instr_dec_o.is_sret    = 1'b0;
    instr_dec_o.is_wfi     = 1'b0;
    instr_dec_o.is_frep       = 1'b0;
    instr_dec_o.frep_bodysize = '0;
    instr_dec_o.frep_mode     = schnizo_pkg::frep_mode_e'(1'b0);

    check_fpround_mode = 1'b0;

    unique case (instr.rtype.opcode)
      // --------------------------------
      // Reg-Immediate Operations
      // --------------------------------
      OpcodeOpImm: begin
        instr_dec_o.fu = schnizo_pkg::ALU;
        imm_select = IIMM;
        instr_dec_o.rs1 = instr.itype.rs1;
        instr_dec_o.rd = instr.itype.rd;
        unique case (instr.itype.funct3)
          3'b000: instr_dec_o.alu_op = schnizo_pkg::AluOpAdd;  // ADDI
          3'b010: instr_dec_o.alu_op = schnizo_pkg::AluOpSlt;  // SLTI
          3'b011: instr_dec_o.alu_op = schnizo_pkg::AluOpSltu; // SLTIU
          3'b100: instr_dec_o.alu_op = schnizo_pkg::AluOpXor;  // XORI
          3'b110: instr_dec_o.alu_op = schnizo_pkg::AluOpOr;   // ORI
          3'b111: instr_dec_o.alu_op = schnizo_pkg::AluOpAnd;  // ANDI

          3'b001: begin
            instr_dec_o.alu_op = schnizo_pkg::AluOpSll; // SLLI
            if (instr.instr[31:25] != 7'b0) illegal_instr = 1'b1;
          end
          3'b101: begin
            if (instr.instr[31:25] == 7'b0) begin
              instr_dec_o.alu_op = schnizo_pkg::AluOpSrl; // SRLI
            end else if (instr.instr[31:25] == 7'b010_0000) begin
              instr_dec_o.alu_op = schnizo_pkg::AluOpSra; // SRAI
            end else begin
              illegal_instr = 1'b1;
            end
          end
        endcase
      end
      // --------------------------------
      // Integer Reg-Reg Operations
      // --------------------------------
      OpcodeOpReg: begin
        instr_dec_o.fu = schnizo_pkg::ALU;
        instr_dec_o.rs1 = instr.rtype.rs1;
        instr_dec_o.rs2 = instr.rtype.rs2;
        instr_dec_o.rd  = instr.rtype.rd;

        unique case ({instr.rtype.funct7, instr.rtype.funct3})
          {7'b000_0000, 3'b000} : instr_dec_o.alu_op = schnizo_pkg::AluOpAdd; //Add
          {7'b010_0000, 3'b000} : instr_dec_o.alu_op = schnizo_pkg::AluOpSub; //Sub
          {7'b000_0000, 3'b010} : instr_dec_o.alu_op = schnizo_pkg::AluOpSlt; //Set Lower Than
          {7'b000_0000, 3'b011} : instr_dec_o.alu_op = schnizo_pkg::AluOpSltu;//Set Lower Than Uns.
          {7'b000_0000, 3'b100} : instr_dec_o.alu_op = schnizo_pkg::AluOpXor; //Xor
          {7'b000_0000, 3'b110} : instr_dec_o.alu_op = schnizo_pkg::AluOpOr;  //Or
          {7'b000_0000, 3'b111} : instr_dec_o.alu_op = schnizo_pkg::AluOpAnd; //And
          {7'b000_0000, 3'b001} : instr_dec_o.alu_op = schnizo_pkg::AluOpSll; //Shift Left Logical
          {7'b000_0000, 3'b101} : instr_dec_o.alu_op = schnizo_pkg::AluOpSrl; //Shift Right Logical
          {7'b010_0000, 3'b101} : instr_dec_o.alu_op = schnizo_pkg::AluOpSra; //Shift Right Arithm
          {7'b000_0001, 3'b000} : begin
            instr_dec_o.fu = schnizo_pkg::MUL;
            instr_dec_o.alu_op = schnizo_pkg::AluOpMul;
          end
          {7'b000_0001, 3'b001} : begin
            instr_dec_o.fu = schnizo_pkg::MUL;
            instr_dec_o.alu_op = schnizo_pkg::AluOpMulh;
          end
          {7'b000_0001, 3'b010} : begin
            instr_dec_o.fu = schnizo_pkg::MUL;
            instr_dec_o.alu_op = schnizo_pkg::AluOpMulhsu;
          end
          {7'b000_0001, 3'b011} : begin
            instr_dec_o.fu = schnizo_pkg::MUL;
            instr_dec_o.alu_op = schnizo_pkg::AluOpMulhu;
          end
          // Division: Offloaded to accelerator
          {7'b000_0001, 3'b100},       // DIV
          {7'b000_0001, 3'b101},       // DIVU
          {7'b000_0001, 3'b110},       // REM
          {7'b000_0001, 3'b111}: begin // REMU
            instr_dec_o.fu = schnizo_pkg::MULDIV;
          end
          default: begin
            illegal_instr = 1'b1;
          end
        endcase
      end
      // --------------------------------
      // LSU
      // --------------------------------
      OpcodeStore: begin
        instr_dec_o.fu = schnizo_pkg::STORE;
        imm_select = SIMM;
        instr_dec_o.rs1 = instr.stype.rs1;
        instr_dec_o.rs2 = instr.stype.rs2;
        instr_dec_o.lsu_op = schnizo_pkg::LsuOpStore;
        // determine store size
        instr_dec_o.lsu_size = lsu_size_e'(instr.stype.funct3[13:12]);
        unique case (instr.stype.funct3)
          3'b000,   // SB
          3'b001,   // SH
          3'b010: ; // SW
          3'b011: illegal_instr = 1'b1;
          default: illegal_instr = 1'b1;
        endcase
      end
      OpcodeLoad: begin
        instr_dec_o.fu = schnizo_pkg::LOAD;
        imm_select = IIMM;
        instr_dec_o.rs1 = instr.itype.rs1;
        instr_dec_o.rd = instr.itype.rd;
        instr_dec_o.lsu_op = schnizo_pkg::LsuOpLoad;
        // determine load size and signed type
        instr_dec_o.lsu_size = lsu_size_e'(instr.itype.funct3[13:12]);
        unique case (instr.itype.funct3)
          3'b000,   // LB
          3'b001,   // LH
          3'b010: ; // LW
          3'b100,   // LBU
          3'b101: instr_dec_o.lsu_op = schnizo_pkg::LsuOpLoadUnsigned; // LHU
          3'b110,
          3'b011: illegal_instr = 1'b1; // both for RV64I
          default: illegal_instr = 1'b1;
        endcase
      end
      // --------------------------------
      // Floating-Point Load/store
      // --------------------------------
      OpcodeStoreFp: begin // STORE-FP
        // Added vector store (RVV) dependency decoding before scalar FP store decode
        if (RVV) begin
          logic vector_store_handled;
          vector_store_handled = 1'b0;
          // Vector stores use Store-FP major opcode (0100111)
          // Only track dependencies (base rs1 integer, data rs2 vector)
          casez (instr.instr)
            // Basic element stores
            VSE8_V, VSE16_V, VSE32_V, VSE64_V,
            // Indexed unordered stores
            VSUXEI8_V, VSUXEI16_V, VSUXEI32_V, VSUXEI64_V,
            // Indexed ordered (segment) stores
            VSOXEI8_V, VSOXEI16_V, VSOXEI32_V, VSOXEI64_V: begin
              instr_dec_o.fu        = schnizo_pkg::SPATZ;
              instr_dec_o.rs1       = instr.stype.rs1; // base integer register
              // rd left at x0 (no writeback)
              vector_store_handled  = 1'b1;
            end
            default: ;
          endcase
          if (vector_store_handled) begin
            // Skip scalar FP decoding
          end else begin
            instr_dec_o.fu = schnizo_pkg::STORE;
            imm_select = SIMM;
            instr_dec_o.rs1 = instr.stype.rs1;
            instr_dec_o.rs2 = instr.stype.rs2;
            instr_dec_o.rs2_is_fp = 1'b1;
            instr_dec_o.lsu_op = schnizo_pkg::LsuOpFpStore;
            instr_dec_o.lsu_size = lsu_size_e'(instr.stype.funct3[13:12]);
            unique case (instr.stype.funct3)
              3'b000: if (!(XF8 | XF8ALT))   illegal_instr = 1'b1; // FSB
              3'b001: if (!(XF16 | XF16ALT)) illegal_instr = 1'b1; // FSH
              3'b010: if (!RVF)              illegal_instr = 1'b1; // FSW
              3'b011: if (!RVD)              illegal_instr = 1'b1; // FSD
              default: illegal_instr = 1'b1;
            endcase
          end
        end else begin
          // ...existing code (original scalar-only path)...
          instr_dec_o.fu = schnizo_pkg::STORE;
          imm_select = SIMM;
          instr_dec_o.rs1 = instr.stype.rs1;
          instr_dec_o.rs2 = instr.stype.rs2;
          instr_dec_o.rs2_is_fp = 1'b1;
          instr_dec_o.lsu_op = schnizo_pkg::LsuOpFpStore;
          instr_dec_o.lsu_size = lsu_size_e'(instr.stype.funct3[13:12]);
          unique case (instr.stype.funct3)
            3'b000: if (!(XF8 | XF8ALT))   illegal_instr = 1'b1;
            3'b001: if (!(XF16 | XF16ALT)) illegal_instr = 1'b1;
            3'b010: if (!RVF)              illegal_instr = 1'b1;
            3'b011: if (!RVD)              illegal_instr = 1'b1;
            default: illegal_instr = 1'b1;
          endcase
        end
      end
      OpcodeLoadFp: begin // LOAD-FP
                // Added vector load (RVV) dependency decoding before scalar FP load decode
        if (RVV) begin
          logic vector_load_handled;
          vector_load_handled = 1'b0;
          // Vector loads use Load-FP major opcode (0000111)
            casez (instr.instr)
              // Basic element loads
              VLE8_V, VLE16_V, VLE32_V, VLE64_V,
              // Indexed unordered loads
              VLUXEI8_V, VLUXEI16_V, VLUXEI32_V, VLUXEI64_V,
              // Indexed ordered (segment) loads
              VLOXEI8_V, VLOXEI16_V, VLOXEI32_V, VLOXEI64_V: begin
                instr_dec_o.fu        = schnizo_pkg::SPATZ;
                instr_dec_o.rs1       = instr.itype.rs1; // base integer register
                vector_load_handled   = 1'b1;
              end
              default: ;
            endcase
          if (vector_load_handled) begin
            // Skip scalar FP decoding
          end else begin
            instr_dec_o.fu = schnizo_pkg::LOAD;
            imm_select = IIMM;
            instr_dec_o.rs1 = instr.itype.rs1;
            instr_dec_o.rd = instr.itype.rd;
            instr_dec_o.rd_is_fp = 1'b1;
            instr_dec_o.lsu_op = schnizo_pkg::LsuOpFpLoad;
            instr_dec_o.lsu_size = lsu_size_e'(instr.itype.funct3[13:12]);
            unique case (instr.itype.funct3)
              3'b000: if (!(XF8 | XF8ALT))   illegal_instr = 1'b1; // FLB
              3'b001: if (!(XF16 | XF16ALT)) illegal_instr = 1'b1; // FLH
              3'b010: if (!RVF)              illegal_instr = 1'b1; // FLW
              3'b011: if (!RVD)              illegal_instr = 1'b1; // FLD
              default: illegal_instr = 1'b1;
            endcase
          end
        end else begin
          // ...existing code (original scalar-only path)...
          instr_dec_o.fu = schnizo_pkg::LOAD;
          imm_select = IIMM;
          instr_dec_o.rs1 = instr.itype.rs1;
          instr_dec_o.rd = instr.itype.rd;
          instr_dec_o.rd_is_fp = 1'b1;
          instr_dec_o.lsu_op = schnizo_pkg::LsuOpFpLoad;
          instr_dec_o.lsu_size = lsu_size_e'(instr.itype.funct3[13:12]);
          unique case (instr.itype.funct3)
            3'b000: if (!(XF8 | XF8ALT))   illegal_instr = 1'b1;
            3'b001: if (!(XF16 | XF16ALT)) illegal_instr = 1'b1;
            3'b010: if (!RVF)              illegal_instr = 1'b1;
            3'b011: if (!RVD)              illegal_instr = 1'b1;
            default: illegal_instr = 1'b1;
          endcase
        end
      end
      // --------------------------------
      // Floating-Point Fused Operations
      // --------------------------------
      OpcodeMadd, OpcodeMsub, OpcodeNmsub, OpcodeNmadd: begin
        if (RVF || RVD) begin
          instr_dec_o.fu             = schnizo_pkg::FPU;
          instr_dec_o.rs1            = instr.r4type.rs1;
          instr_dec_o.rs1_is_fp      = 1'b1;
          instr_dec_o.rs2            = instr.r4type.rs2;
          instr_dec_o.rs2_is_fp      = 1'b1;
          instr_dec_o.rd             = instr.r4type.rd;
          instr_dec_o.rd_is_fp       = 1'b1;
          imm_select                = RS3;
          instr_dec_o.use_imm_as_rs3 = 1'b1;

          // Per default use the encoded format for src and dst (dst is not used for fused
          // operations). Whether this format is suppored (i.e. RVF/RVD) is checked below.
          // Any alternative format is set also below depending on the value in the CSR.
          instr_dec_o.fpu_fmt_src = fpnew_pkg::fp_format_e'(instr.rftype.fmt);
          instr_dec_o.fpu_fmt_dst = fpnew_pkg::fp_format_e'(instr.rftype.fmt);
          // The integer format is not decoded here. Instead a separate fpu_op is defined.
          // This is more space efficient as the int fmt differs only for a few instructions.

          // The rounding mode contains additional decoding details for certain instructions.
          instr_dec_o.fpu_rnd_mode = fpnew_pkg::roundmode_e'(instr.rftype.rm) == fpnew_pkg::DYN ?
                                     fpu_round_mode_i : fpnew_pkg::roundmode_e'(instr.rftype.rm);
          // Whether we check that the encoding contains a valid rounding mode.
          // Certain instructions set the round mode for additional sub operations (e.g. FCMP)
          // For such instructions the decoder sets the round mode to the appropriate value and no
          // check can be performed anymore.
          check_fpround_mode = 1'b1;

          unique case (instr.r4type.opcode)
            7'b1000011: instr_dec_o.fpu_op = schnizo_pkg::FpuOpFmadd; // FMADD
            7'b1000111: instr_dec_o.fpu_op = schnizo_pkg::FpuOpFmsub; // FMSUB
            7'b1001011: instr_dec_o.fpu_op = schnizo_pkg::FpuOpFnmsub; // FNMSUB
            7'b1001111: instr_dec_o.fpu_op = schnizo_pkg::FpuOpFnmadd; // FNMADD
            default: illegal_instr = 1'b1;
          endcase

          // --------------------------------
          // Checks and alternative format assignment
          // --------------------------------
          // Assign alternative formats if enabled
          if (fpu_fmt_mode_i.src == 1'b1) begin
            unique case (instr_dec_o.fpu_fmt_src)
              fpnew_pkg::FP16: instr_dec_o.fpu_fmt_src = fpnew_pkg::FP16ALT;
              fpnew_pkg::FP8:  instr_dec_o.fpu_fmt_src = fpnew_pkg::FP8ALT;
              default: ;
            endcase
          end
          if (fpu_fmt_mode_i.dst == 1'b1) begin
            unique case (instr_dec_o.fpu_fmt_dst)
              fpnew_pkg::FP16: instr_dec_o.fpu_fmt_dst = fpnew_pkg::FP16ALT;
              fpnew_pkg::FP8:  instr_dec_o.fpu_fmt_dst = fpnew_pkg::FP8ALT;
              default: ;
            endcase
          end

          // Check the FP format
          unique case (fpnew_pkg::fp_format_e'(instr.rftype.fmt))
            fpnew_pkg::FP32: if (~RVF) illegal_instr = 1'b1;
            fpnew_pkg::FP64: if (~RVD) illegal_instr = 1'b1;
            fpnew_pkg::FP16: if (~XF16) illegal_instr = 1'b1;
            fpnew_pkg::FP16ALT: if (~XF16ALT) illegal_instr = 1'b1;
            fpnew_pkg::FP8:  if (~XF8) illegal_instr = 1'b1;
            fpnew_pkg::FP8ALT: if (~XF8ALT) illegal_instr = 1'b1;
            default: illegal_instr = 1'b1;
          endcase

          if (check_fpround_mode) begin
            unique case (instr.rftype.rm) inside
              [ 3'b000 : 3'b100]: ; // legal round modes
              3'b101: illegal_instr = 1'b1; // alternative Half-Precision mode. Not supported.
              3'b111: begin
                // round mode from CSR
                unique case (fpu_round_mode_i) inside
                  [3'b000 : 3'b100]: ;
                  default: illegal_instr = 1'b1;
                endcase
              end
              default: illegal_instr = 1'b1;
            endcase
          end
        end else begin
          illegal_instr = 1'b1;
        end
      end
      // --------------------------------
      // Floating-Point Reg-Reg Operations
      // --------------------------------
      OpcodeOpFp: begin
        if (RVF || RVD) begin
          instr_dec_o.fu        = schnizo_pkg::FPU;
          instr_dec_o.rs1       = instr.rftype.rs1;
          instr_dec_o.rs1_is_fp = 1'b1;
          instr_dec_o.rs2       = instr.rftype.rs2;
          instr_dec_o.rs2_is_fp = 1'b1;
          instr_dec_o.rd        = instr.rftype.rd;
          instr_dec_o.rd_is_fp  = 1'b1;

          // Per default use the encoded format for src and dst. Whether this format is suppored
          // (i.e. RVF/RVD) is checked below. Any alternative format is set also below depending
          // on the value in the CSR. For conversion instructions, the decoder overwrite these
          // values.
          instr_dec_o.fpu_fmt_src = fpnew_pkg::fp_format_e'(instr.rftype.fmt);
          instr_dec_o.fpu_fmt_dst = fpnew_pkg::fp_format_e'(instr.rftype.fmt);
          // The integer format is not decoded here. Instead a separate fpu_op is defined.
          // This is more space efficient as the int fmt differs only for a few instructions.

          // The rounding mode contains additional decoding details for certain instructions.
          instr_dec_o.fpu_rnd_mode = fpnew_pkg::roundmode_e'(instr.rftype.rm) == fpnew_pkg::DYN ?
                                     fpu_round_mode_i : fpnew_pkg::roundmode_e'(instr.rftype.rm);
          // Whether we check that the encoding contains a valid rounding mode.
          // Certain instructions set the round mode for additional sub operations (e.g. FCMP)
          // For such instructions the decoder sets the round mode to the appropriate value and no
          // check can be performed anymore.
          check_fpround_mode = 1'b1;

          unique case (instr.rftype.funct5)
            5'b00000: begin
              instr_dec_o.fpu_op = schnizo_pkg::FpuOpFadd; // FADD_S, FADD_D
            end
            5'b00001: begin
              instr_dec_o.fpu_op = schnizo_pkg::FpuOpFsub; // FSUB_S, FSUB_D
            end
            5'b00010: begin
              instr_dec_o.fpu_op = schnizo_pkg::FpuOpFmul; // FMUL_S, FMUL_D
            end
            5'b00011: begin
              instr_dec_o.fpu_op = schnizo_pkg::FpuOpFdiv; // FDIV_S, FDIV_D
            end
            5'b01011: begin
              instr_dec_o.fpu_op = schnizo_pkg::FpuOpFsqrt; // FSQRT_S, FSQRT_D
              if (instr.rftype.rs2 != '0) illegal_instr = 1'b1;
            end
            5'b00100: begin
              // FSGNJ, FSGNJN, FSGNJX - round mode represents the sub operation.
              // Disable round mode check
              check_fpround_mode = 1'b0;
              instr_dec_o.fpu_op = schnizo_pkg::FpuOpFsgnj; // we only use op_mode zero (NaN boxed)
              unique case (instr.rftype.rm)
                3'b000: instr_dec_o.fpu_rnd_mode = fpnew_pkg::RNE; // FSGNJ_S,  FSGNJ_D
                3'b001: instr_dec_o.fpu_rnd_mode = fpnew_pkg::RTZ; // FSGNJN_S, FSGNJN_D
                3'b010: instr_dec_o.fpu_rnd_mode = fpnew_pkg::RDN; // FSGNJX_S, FSGNJX_D
                default: illegal_instr = 1'b1;
              endcase
            end
            5'b00101: begin
              // FMIN, FMAX - round mode represents the sub operation.
              // Disable round mode check
              check_fpround_mode = 1'b0;
              instr_dec_o.fpu_op = schnizo_pkg::FpuOpFminmax;
              unique case (instr.rftype.rm)
                3'b000: instr_dec_o.fpu_rnd_mode = fpnew_pkg::RNE; // FMIN_S, FMIN_D
                3'b001: instr_dec_o.fpu_rnd_mode = fpnew_pkg::RTZ; // FMAX_S, FMAX_D
                default: illegal_instr = 1'b1;
              endcase
            end
            5'b01000: begin
              // overwrite the destination format. Check the source format.
              unique case ({instr.rftype.fmt, instr.rftype.rs2})
                7'b01_00000: begin
                  instr_dec_o.fpu_op = schnizo_pkg::FpuOpF2F; // FCVT_D_S
                  instr_dec_o.fpu_fmt_src = fpnew_pkg::FP32;
                end
                7'b00_00001: begin
                  instr_dec_o.fpu_op = schnizo_pkg::FpuOpF2F; // FCVT_S_D
                  instr_dec_o.fpu_fmt_src = fpnew_pkg::FP64;
                end
                default: illegal_instr = 1'b1;
              endcase
            end
            5'b11000: begin
              instr_dec_o.rd_is_fp = 1'b0;
              unique case (instr.rftype.rs2)
                5'b00000: instr_dec_o.fpu_op = schnizo_pkg::FpuOpF2I;        //FCVT_W_S, FCVT_W_D
                5'b00001: instr_dec_o.fpu_op = schnizo_pkg::FpuOpF2Iunsigned;//FCVT_WU_S, FCVT_WU_D
                default: illegal_instr = 1'b1;
              endcase
            end
            5'b10100: begin
              check_fpround_mode = 1'b0;
              instr_dec_o.rd_is_fp = 1'b0;
              instr_dec_o.fpu_op = schnizo_pkg::FpuOpFcmp;
              unique case (instr.rftype.rm)
                3'b000: instr_dec_o.fpu_rnd_mode = fpnew_pkg::RNE; // FLE_S, FLE_D
                3'b001: instr_dec_o.fpu_rnd_mode = fpnew_pkg::RTZ; // FLT_S, FLT_D
                3'b010: instr_dec_o.fpu_rnd_mode = fpnew_pkg::RDN; // FEQ_S, FEQ_D
                default: illegal_instr = 1'b1;
              endcase
            end
            5'b11100: begin
              check_fpround_mode = 1'b0;
              unique case (instr.rftype.rm)
                3'b000: begin
                  instr_dec_o.rd_is_fp = 1'b0;
                  // use sign injection with passthrough and sign-extend result
                  instr_dec_o.fpu_op = schnizo_pkg::FpuOpFsgnjSignExt; // FMV_X_W
                  instr_dec_o.fpu_rnd_mode = fpnew_pkg::RUP; // passthrough
                end
                3'b001: begin
                  instr_dec_o.rd_is_fp = 1'b0;
                  instr_dec_o.fpu_op = schnizo_pkg::FpuOpFclassify; // FCLASS_S, FCLASS_D
                  instr_dec_o.fpu_rnd_mode = fpnew_pkg::RNE;
                end
                default: illegal_instr = 1'b1;
              endcase
              if (instr.rftype.rs2 != '0) illegal_instr = 1'b1;
            end
            5'b11010: begin
              instr_dec_o.rs1_is_fp  = 1'b0;
              unique case (instr.rftype.rs2)
                5'b00000: instr_dec_o.fpu_op = schnizo_pkg::FpuOpI2F;        //FCVT_S_W, FCVT_D_W
                5'b00001: instr_dec_o.fpu_op = schnizo_pkg::FpuOpI2Funsigned;//FCVT_S_WU, FCVT_D_WU
                default: illegal_instr = 1'b1;
              endcase
            end
            5'b11110: begin
              if (instr.rftype.rs2 == '0 && instr.rftype.rm == '0) begin
              instr_dec_o.rs1_is_fp  = 1'b0;
                instr_dec_o.fpu_op = schnizo_pkg::FpuOpFsgnj; // FMV_W_X
                instr_dec_o.fpu_rnd_mode = fpnew_pkg::RUP; // passthrough
              end else begin
                illegal_instr = 1'b1;
              end
            end
            default: illegal_instr = 1'b1;
          endcase
          // --------------------------------
          // Checks and alternative format assignment
          // --------------------------------
          // Assign alternative formats if enabled
          if (fpu_fmt_mode_i.src == 1'b1) begin
            unique case (instr_dec_o.fpu_fmt_src)
              fpnew_pkg::FP16: instr_dec_o.fpu_fmt_src = fpnew_pkg::FP16ALT;
              fpnew_pkg::FP8:  instr_dec_o.fpu_fmt_src = fpnew_pkg::FP8ALT;
              default: ;
            endcase
          end

          if (fpu_fmt_mode_i.dst == 1'b1) begin
            unique case (instr_dec_o.fpu_fmt_dst)
              fpnew_pkg::FP16: instr_dec_o.fpu_fmt_dst = fpnew_pkg::FP16ALT;
              fpnew_pkg::FP8:  instr_dec_o.fpu_fmt_dst = fpnew_pkg::FP8ALT;
              default: ;
            endcase
          end

          // Check the FP format
          unique case (fpnew_pkg::fp_format_e'(instr.rftype.fmt))
            fpnew_pkg::FP32: if (~RVF) illegal_instr = 1'b1;
            fpnew_pkg::FP64: if (~RVD) illegal_instr = 1'b1;
            fpnew_pkg::FP16: if (~XF16) illegal_instr = 1'b1;
            fpnew_pkg::FP16ALT: if (~XF16ALT) illegal_instr = 1'b1;
            fpnew_pkg::FP8:  if (~XF8) illegal_instr = 1'b1;
            fpnew_pkg::FP8ALT: if (~XF8ALT) illegal_instr = 1'b1;
            default: illegal_instr = 1'b1;
          endcase

          if (check_fpround_mode) begin
            unique case (instr.rftype.rm) inside
              [ 3'b000 : 3'b100]: ; // legal round modes
              3'b101: illegal_instr = 1'b1; // alternative Half-Precision mode. Not supported.
              3'b111: begin
                // round mode from CSR
                unique case (fpu_round_mode_i) inside
                  [3'b000 : 3'b100]: ;
                  default: illegal_instr = 1'b1;
                endcase
              end
              default: illegal_instr = 1'b1;
            endcase
          end

        end else begin
          illegal_instr = 1'b1;
        end
      end
      // --------------------------------
      // Control Flow Instructions
      // --------------------------------
      OpcodeBranch: begin
        instr_dec_o.fu        = schnizo_pkg::CTRL_FLOW;
        imm_select            = SBIMM;
        instr_dec_o.rs1       = instr.stype.rs1;
        instr_dec_o.rs2       = instr.stype.rs2;

        instr_dec_o.is_branch = 1'b1;

        case (instr.stype.funct3)
          3'b000: instr_dec_o.alu_op = schnizo_pkg::AluOpEq;  // BEQ
          3'b001: instr_dec_o.alu_op = schnizo_pkg::AluOpNeq; // BNE
          3'b100: instr_dec_o.alu_op = schnizo_pkg::AluOpLt;  // BLT
          3'b101: instr_dec_o.alu_op = schnizo_pkg::AluOpGe;  // BGE
          3'b110: instr_dec_o.alu_op = schnizo_pkg::AluOpLtu; // BLTU
          3'b111: instr_dec_o.alu_op = schnizo_pkg::AluOpGeu; // BGEU
          default: begin
            illegal_instr           = 1'b1;
            instr_dec_o.is_branch    = 1'b0;
          end
        endcase
      end
      // Jump and link - JAL
      OpcodeJal: begin
        instr_dec_o.fu             = schnizo_pkg::CTRL_FLOW;
        instr_dec_o.alu_op         = schnizo_pkg::AluOpAdd;
        imm_select                 = JIMM;
        instr_dec_o.rd             = instr.utype.rd;
        instr_dec_o.is_jal         = 1'b1;
        instr_dec_o.use_pc_as_op_a = 1'b1;
      end
      // Jump and link register - JALR
      OpcodeJalr: begin
        instr_dec_o.fu      = schnizo_pkg::CTRL_FLOW;
        instr_dec_o.alu_op  = schnizo_pkg::AluOpAdd;
        instr_dec_o.rs1     = instr.itype.rs1;
        imm_select          = IIMM;
        instr_dec_o.rd      = instr.itype.rd;
        instr_dec_o.is_jalr = 1'b1;
        // invalid jump and link register -> reserved for vector encoding
        if (instr.itype.funct3 != 3'b0) illegal_instr = 1'b1;
      end
      OpcodeAuipc: begin // AUIPC
        instr_dec_o.fu = schnizo_pkg::ALU;
        imm_select = UIMM;
        instr_dec_o.rd = instr.utype.rd;
        instr_dec_o.use_pc_as_op_a = 1'b1;
      end
      OpcodeLui: begin // LUI
        instr_dec_o.fu = schnizo_pkg::ALU;
        imm_select = UIMM;
        instr_dec_o.rd = instr.utype.rd;
      end
      // --------------------------------
      // MISC-MEM
      // --------------------------------
      OpcodeMiscMem: begin
        instr_dec_o.fu = schnizo_pkg::NONE;
        // Don't check the unused bits for forward compatibility.
        // See RISC-V spec vol 1, v20240411, page 45,
        // Chapter 6. "Zifencei" Extension for Instruction-Fetch Fence, Ver. 2.0
        case (instr.stype.funct3)
          3'b000: begin
            instr_dec_o.is_fence = 1'b1; // FENCE - implemented as NOP
          end
          3'b001: begin
            instr_dec_o.is_fence_i = 1'b1; // FENCE.I
          end
          default: illegal_instr = 1'b1;
        endcase
      end
      // --------------------------------
      // SYSTEM
      // --------------------------------
      OpcodeSystem: begin
        instr_dec_o.fu = schnizo_pkg::CSR;
        instr_dec_o.rd = instr.itype.rd;
        instr_dec_o.rs1 = instr.itype.rs1;
        case (instr.itype.funct3)
          3'b000: begin // non CSR related SYSTEM instructions
            unique case (instr.itype.imm)
              12'h000: instr_dec_o.is_ecall  = 1'b1; // ECALL
              12'h001: instr_dec_o.is_ebreak = 1'b1; // EBREAK
              12'h302: instr_dec_o.is_mret   = 1'b1; // MRET
              12'h102: instr_dec_o.is_sret   = 1'b1; // SRET
              12'h105: instr_dec_o.is_wfi    = 1'b1; // WFI
              default: illegal_instr = 1'b1;
            endcase
          end
          // CSR instructions
          3'b001: begin
            // CSRRW: atomically swaps values in the CSR and integer register.
            //        If rd = x0 do not read the CSR / do not cause any read side effects.
            imm_select = IIMM;
            if (instr.itype.rd == '0) instr_dec_o.csr_op = schnizo_pkg::CsrOpWrite;
            else instr_dec_o.csr_op = schnizo_pkg::CsrOpSwap;
          end
          3'b010: begin
            // CSRRS: atomically Read and set Bits in the CSR based on rs1. Write to rd.
            //        If rs1 = x0, then do not write to CSR, just read.
            imm_select = IIMM;
            if (instr.itype.rs1 == '0) instr_dec_o.csr_op = schnizo_pkg::CsrOpRead;
            else instr_dec_o.csr_op = schnizo_pkg::CsrOpSet;
          end
          3'b011: begin
            // CSRRC: atomically Read and clear Bits in the CSR based on rs1. Write to rd.
            //        If rs1 = x0, then do not write to CSR, just read.
            imm_select = IIMM;
            if (instr.itype.rs1 == '0) instr_dec_o.csr_op = schnizo_pkg::CsrOpRead;
            else instr_dec_o.csr_op = schnizo_pkg::CsrOpClear;
          end
          3'b101: begin
            // CSRRWI: atomically read the CSR, write the immediate to the CSR,
            //         and write the old value to rd.
            //         If rd = x0 do not read the CSR / do not cause any read side effects.
            imm_select = IIMM;
            instr_dec_o.use_rs1addr_as_op_a = 1'b1;
            if (instr.itype.rd == '0) instr_dec_o.csr_op = schnizo_pkg::CsrOpWrite;
            else instr_dec_o.csr_op = schnizo_pkg::CsrOpSwap;
          end
          3'b110: begin
            // CSRRSI: atomically read the CSR, set bits based on immediate (rs1 address),
            //         and write the old value to rd. If rs1 = x0, then do not write to CSR,
            //         just read.
            imm_select = IIMM;
            instr_dec_o.use_rs1addr_as_op_a = 1'b1;
            if (instr.itype.rs1 == '0) instr_dec_o.csr_op = schnizo_pkg::CsrOpRead;
            else instr_dec_o.csr_op = schnizo_pkg::CsrOpSet;
          end
          3'b111: begin
            // CSRRCI: autmically read the CSR, clear bits based on immediate (rs1 address),
            //         and write the old value to rd. If rs1 = x0, then do not write to CSR,
            //         just read.
            imm_select = IIMM;
            instr_dec_o.use_rs1addr_as_op_a = 1'b1;
            if (instr.itype.rs1 == '0) instr_dec_o.csr_op = schnizo_pkg::CsrOpRead;
            else instr_dec_o.csr_op = schnizo_pkg::CsrOpClear;
          end
          default: illegal_instr = 1'b1;
        endcase
      end
      // --------------------------------
      // DMA & SSR instructions
      // --------------------------------
      OpcodeCustom1: begin
        if (Xdma) begin
          unique case (instr.rtype.funct3)
            3'b000: begin // DMA instructions
              instr_dec_o.fu  = schnizo_pkg::DMA;
              instr_dec_o.rd  = instr.rtype.rd;
              instr_dec_o.rs1 = instr.rtype.rs1;
              instr_dec_o.rs2 = instr.rtype.rs2;
              // Check fixed bits
              unique case (instr.rtype.funct7)
                7'b0000000,       // DMSRC
                7'b0000001,       // DMDST
                7'b0000110: begin // DMSTR
                  if (instr.rtype.rd != '0) illegal_instr = 1'b1;
                end
                7'b0000111: begin // DMREP
                  if (instr.rtype.rd != '0) illegal_instr = 1'b1;
                  if (instr.rtype.rs2 != '0) illegal_instr = 1'b1;
                end
                7'b0000010,   // DMCPYI
                7'b0000011: ; // DMCPY - no additional check required
                7'b0000100,       // DMSTATI
                7'b0000101: begin // DMSTAT
                  if (instr.rtype.rs1 != '0) illegal_instr = 1'b1;
                end
                default: illegal_instr = 1'b1;
              endcase
            end
            3'b001,
            3'b010: begin // SSR instructions
              // The Schnizo does not feature SSRs
              illegal_instr = 1'b1;
            end
            default: illegal_instr = 1'b1;
          endcase
        end else begin
          illegal_instr = 1'b1;
        end
      end
      // --------------------------------
      // Frep extension instructions
      // --------------------------------
      OpcodeCustom0: begin
        if (Xfrep) begin
          instr_dec_o.is_frep = 1'b1;
          // TODO(colluca): why does this comment not violate the 100 character line-length limit?
          // The parsed max_instr is actually -1 of the instructions we loop. This is to match the Snitch behaviour.
          // When executing the loop we actually execute max_instr+1 instructions.
          instr_dec_o.frep_bodysize = instr.freptype.max_instr;
          instr_dec_o.frep_mode     = schnizo_pkg::frep_mode_e'(instr.freptype.frep_mode);
          // The iterations are from a register specified by the max_iters field
          instr_dec_o.rs1_is_fp = 1'b0;
          instr_dec_o.rs1       = instr.freptype.max_iters_reg;
        end else begin
          illegal_instr = 1'b1;
        end
      end
      // --------------------------------
      // Atomic instructions
      // --------------------------------
      // We ignore the aq and lr flags!
      OpcodeAmo: begin
        instr_dec_o.fu  = schnizo_pkg::LOAD;
        instr_dec_o.rd  = instr.atype.rd;
        instr_dec_o.rs1 = instr.atype.rs1;
        instr_dec_o.rs2 = instr.atype.rs2;

        instr_dec_o.lsu_size = lsu_size_e'(instr.stype.funct3[13:12]);
        // We only support the W size
        if (instr.stype.funct3 != 3'b010) illegal_instr = 1'b1;

        // This implementation ignores the aq and rl bits!
        // Extending the lsu_op enum saves a bit in the Reservation Station FFs compared to
        // decoding the AMU type here and passing it to the RS.
        unique case (instr.atype.funct5)
          5'b00010: begin // LR_W
            instr_dec_o.lsu_op = schnizo_pkg::LsuOpAmoLr;
            if (instr.atype.rs2 != '0) illegal_instr = 1'b1;
          end
          5'b00011: begin // SC_W
            instr_dec_o.lsu_op = schnizo_pkg::LsuOpAmoSc;
          end
          5'b00001: begin // AMOSWAP_w
            instr_dec_o.lsu_op = schnizo_pkg::LsuOpAmoSwap;
          end
          5'b00000: begin // AMOADD_W
            instr_dec_o.lsu_op = schnizo_pkg::LsuOpAmoAdd;
          end
          5'b00100: begin // AMOXOR_W
            instr_dec_o.lsu_op = schnizo_pkg::LsuOpAmoXor;
          end
          5'b01100: begin // AMOAND_W
            instr_dec_o.lsu_op = schnizo_pkg::LsuOpAmoAnd;
          end
          5'b01000: begin // AMOOR_W
            instr_dec_o.lsu_op = schnizo_pkg::LsuOpAmoOr;
          end
          5'b10000: begin // AMOMIN_W
            instr_dec_o.lsu_op = schnizo_pkg::LsuOpAmoMin;
          end
          5'b10100: begin // AMOMAX_W
            instr_dec_o.lsu_op = schnizo_pkg::LsuOpAmoMax;
          end
          5'b11000: begin // AMOMINU_W
            instr_dec_o.lsu_op = schnizo_pkg::LsuOpAmoMinU;
          end
          5'b11100: begin // AMOMAXU_W
            instr_dec_o.lsu_op = schnizo_pkg::LsuOpAmoMaxU;
          end
          default: illegal_instr = 1'b1;
        endcase
      end
      OpcodeVec: begin
        if (RVV) begin
          // Tag as SPATZ functional unit.
          instr_dec_o.fu = schnizo_pkg::SPATZ;
          // Dependency-only partial decode of a subset of RVV instructions.
          // Vector registers are flagged with *_is_fp = 1'b1 (single scoreboard domain with FP).
          unique casez (instr.instr)
            // --- Configuration / setup (integer destination) ---
            VSETIVLI: begin
              instr_dec_o.rd        = instr.rtype.rd;
            end
            VSETVLI: begin
              instr_dec_o.rd        = instr.rtype.rd;
              instr_dec_o.rs1       = instr.rtype.rs1; // AVL in integer reg
            end
            VSETVL: begin
              instr_dec_o.rd        = instr.rtype.rd;
              instr_dec_o.rs1       = instr.rtype.rs1;
              instr_dec_o.rs2       = instr.rtype.rs2;
            end
            // Move scalar from vector element to integer
            VMV_X_S: begin
              instr_dec_o.rd        = instr.rtype.rd;          // integer rd
            end

            //TODO: All the float vector instructions are missing

            VMACC_VX: begin
              instr_dec_o.rs1       = instr.rtype.rs1;
            end

            // --- Vector-Vector arithmetic (VV): vd, vs1, vs2 are vector ---
            VFADD_VV, VADD_VV, VSUB_VV, VMIN_VV, VMINU_VV, VMAX_VV, VMAXU_VV,
            VAND_VV, VOR_VV, VXOR_VV,
            VADC_VVM, VMADC_VV, VSLL_VV, VSRL_VV, VSRA_VV, //Attenzione a VADC_VVM e VMADC_VV TODO
            VMSEQ_VV, VMSNE_VV, VMSLTU_VV, VMSLT_VV, VMSLEU_VV, VMSLE_VV,
            VDIV_VV, VDIVU_VV, VREM_VV, VREMU_VV,
            VWMUL_VV, VWMULU_VV, VWMULSU_VV,
            VWMACC_VV, VWMACCU_VV, VWMACCSU_VV: begin
            end

            // --- Vector-Scalar integer (VX): rs1 integer, rs2 vector ---
            VADD_VX, VSUB_VX, VRSUB_VX,
            VAND_VX, VOR_VX, VXOR_VX,
            VSLL_VX, VSRL_VX, VSRA_VX,
            VMIN_VX, VMINU_VX, VMAX_VX, VMAXU_VX,
            VMSEQ_VX, VMSNE_VX, VMSLTU_VX, VMSLT_VX, VMSLEU_VX, VMSLE_VX,
            VDIV_VX, VDIVU_VX, VREM_VX, VREMU_VX,
            VMUL_VX, VMULH_VX, VMULHU_VX, VMULHSU_VX,
            VWMUL_VX, VWMULU_VX, VWMULSU_VX,
            VWMACC_VX, VWMACCU_VX, VWMACCSU_VX, VWMACCUS_VX: begin
              instr_dec_o.rs1       = instr.rtype.rs1;  // integer scalar
            end

            // --- Vector-Immediate (VI): rs1 field is immediate => no rs1 reg ---
            VADD_VI, VRSUB_VI,
            VAND_VI, VOR_VI, VXOR_VI,
            VSLL_VI, VSRL_VI, VSRA_VI,
            VMSEQ_VI, VMSNE_VI, VMSLE_VI, VMSLEU_VI, VMV_V_I,
            VMSGT_VI, VMSGTU_VI: begin
            end

            // --- Vector loads: base rs1 (integer), rd vector ---
            VLE8_V, VLE16_V, VLE32_V, VLE64_V,
            VLUXEI8_V, VLUXEI16_V, VLUXEI32_V, VLUXEI64_V,
            VLOXEI8_V, VLOXEI16_V, VLOXEI32_V, VLOXEI64_V: begin
              instr_dec_o.rs1       = instr.rtype.rs1; // base integer
            end

            // --- Vector stores: base rs1 (integer), rs2 vector (data) ---
            VSE8_V, VSE16_V, VSE32_V, VSE64_V,
            VSUXEI8_V, VSUXEI16_V, VSUXEI32_V, VSUXEI64_V,
            VSOXEI8_V, VSOXEI16_V, VSOXEI32_V, VSOXEI64_V: begin
              instr_dec_o.rs1       = instr.rtype.rs1; // base integer
            end


          /// VECTOR FLOATING POINT INSTRUCTIONS: TODO: Double check, they were made with GPT
            // --- Vector-Float vector-scalar (VF): rs1 is FP scalar ---
            VFADD_VF, VFSUB_VF, VFMIN_VF, VFMAX_VF,
            VFSGNJ_VF, VFSGNJN_VF, VFSGNJX_VF,
            VFSLIDE1UP_VF, VFSLIDE1DOWN_VF,
            VFMERGE_VFM, VFMV_V_F, VFMV_S_F,
            VMFEQ_VF, VMFLE_VF, VMFLT_VF, VMFNE_VF, VMFGT_VF, VMFGE_VF,
            VFDIV_VF, VFRDIV_VF, VFMUL_VF, VFRSUB_VF,
            VFMADD_VF, VFNMADD_VF, VFMSUB_VF, VFNMSUB_VF,
            VFMACC_VF, VFNMACC_VF, VFMSAC_VF, VFNMSAC_VF,
            VFWADD_VF, VFWSUB_VF, VFWADD_WF, VFWSUB_WF,
            VFWMUL_VF, VFWDOTP_VF, VFWMACC_VF, VFWNMACC_VF, VFWMSAC_VF, VFWNMSAC_VF: begin
              instr_dec_o.rs1       = instr.rtype.rs1;  // FP scalar
              instr_dec_o.rs1_is_fp = 1'b1;
            end

            // --- Vector-Float move from vector element to FP scalar ---
            VFMV_F_S: begin
              instr_dec_o.rd        = instr.rtype.rd;   // FP scalar destination
              instr_dec_o.rd_is_fp  = 1'b1;
            end

            // --- Vector-Float vector-vector (VV) ops and reductions (no scalar deps) ---
            VFADD_VV, VFSUB_VV, VFMIN_VV, VFMAX_VV,
            VFSGNJ_VV, VFSGNJN_VV, VFSGNJX_VV,
            VFDIV_VV, VFMUL_VV,
            VFMADD_VV, VFNMADD_VV, VFMSUB_VV, VFNMSUB_VV,
            VFMACC_VV, VFNMACC_VV, VFMSAC_VV, VFNMSAC_VV,
            VFWADD_VV, VFWSUB_VV, VFWADD_WV, VFWSUB_WV,
            VFWMUL_VV, VFWDOTP_VV,
            VFWMACC_VV, VFWNMACC_VV, VFWMSAC_VV, VFWNMSAC_VV,
            VFREDUSUM_VS, VFREDOSUM_VS, VFREDMIN_VS, VFREDMAX_VS,
            VFWREDUSUM_VS, VFWREDOSUM_VS: begin
            end

            // --- Vector-Float conversions and misc (no scalar deps) ---
            VFSQRT_V, VFRSQRT7_V, VFREC7_V, VFCLASS_V,
            VFCVT_XU_F_V, VFCVT_X_F_V, VFCVT_F_XU_V, VFCVT_F_X_V,
            VFCVT_RTZ_XU_F_V, VFCVT_RTZ_X_F_V,
            VFWCVT_XU_F_V, VFWCVT_X_F_V, VFWCVT_F_XU_V, VFWCVT_F_X_V,
            VFWCVT_F_F_V, VFWCVT_RTZ_XU_F_V, VFWCVT_RTZ_X_F_V,
            VFNCVT_XU_F_W, VFNCVT_X_F_W, VFNCVT_F_XU_W, VFNCVT_F_X_W,
            VFNCVT_F_F_W, VFNCVT_ROD_F_F_W, VFNCVT_RTZ_XU_F_W, VFNCVT_RTZ_X_F_W: begin
            end

            default: illegal_instr = 1'b1;
          endcase
        end else begin
          illegal_instr = 1'b1;
        end
      end
      default: begin
        illegal_instr = 1'b1;
      end
    endcase
  end

  // --------------------------------
  // Sign extend immediate
  // --------------------------------
  always_comb begin : sign_extend
    imm_i_type = {{XLEN - 12{instr_fetch_data_i[31]}}, instr_fetch_data_i[31:20]};
    imm_s_type = {
      {XLEN - 12{instr_fetch_data_i[31]}}, instr_fetch_data_i[31:25], instr_fetch_data_i[11:7]
    };
    imm_sb_type = {
      {XLEN - 13{instr_fetch_data_i[31]}},
      instr_fetch_data_i[31],
      instr_fetch_data_i[7],
      instr_fetch_data_i[30:25],
      instr_fetch_data_i[11:8],
      1'b0
    };
    imm_u_type = {{XLEN - 32{instr_fetch_data_i[31]}}, instr_fetch_data_i[31:12], 12'b0};
    imm_uj_type = {
      {XLEN - 20{instr_fetch_data_i[31]}},
      instr_fetch_data_i[19:12],
      instr_fetch_data_i[20],
      instr_fetch_data_i[30:21],
      1'b0
    };

    // NOIMM, IIMM, SIMM, SBIMM, UIMM, JIMM, RS3
    // select immediate
    case (imm_select)
      IIMM: begin
        instr_dec_o.imm = imm_i_type;
        instr_dec_o.use_imm_as_op_b = 1'b1;
      end
      SIMM: begin
        instr_dec_o.imm = imm_s_type;
        instr_dec_o.use_imm_as_op_b = 1'b1;
      end
      SBIMM: begin
        instr_dec_o.imm = imm_sb_type;
        instr_dec_o.use_imm_as_op_b = 1'b1;
      end
      UIMM: begin
        instr_dec_o.imm = imm_u_type;
        instr_dec_o.use_imm_as_op_b = 1'b1;
      end
      JIMM: begin
        instr_dec_o.imm = imm_uj_type;
        instr_dec_o.use_imm_as_op_b = 1'b1;
      end
      RS3: begin
        // imm holds address of fp operand rs3
        instr_dec_o.imm = {{XLEN - 5{1'b0}}, instr.r4type.rs3};
        instr_dec_o.use_imm_as_op_b = 1'b0;
      end
      MUX_RD_RS3: begin
        // imm holds address of operand rs3 which is in rd field
        instr_dec_o.imm = {{XLEN - 5{1'b0}}, instr.rtype.rd};
        instr_dec_o.use_imm_as_op_b = 1'b0;
      end
      default: begin
        instr_dec_o.imm = {XLEN{1'b0}};
        instr_dec_o.use_imm_as_op_b = 1'b0;
      end
    endcase
  end

endmodule
