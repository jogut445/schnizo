// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

`include "reqrsp_interface/typedef.svh"
`include "tcdm_interface/typedef.svh"

package schnizo_synth_pkg;

	localparam int unsigned NumIntOutstandingLoads = 4;
	localparam int unsigned NumIntOutstandingMem = 4;
	localparam logic [31:0] BootAddr = 32'h80000000;
	localparam int unsigned MaxIterationsW = 16;

  localparam int unsigned AddrWidth = 48;
  localparam int unsigned DataWidth = 64;
  localparam int unsigned CoreUserWidth = 64;

  typedef logic [DataWidth-1:0] data_t;
  typedef logic [AddrWidth-1:0] addr_t;
  typedef logic [DataWidth/8-1:0] strb_t;
  typedef logic [CoreUserWidth-1:0] user_t;

  `REQRSP_TYPEDEF_ALL(data, addr_t, data_t, strb_t, user_t)

  // TCDM types for Spatz. Address width matches 128 kB TCDM (default config).
  localparam int unsigned TCDMAddrWidth = 17;
  typedef logic [TCDMAddrWidth-1:0] tcdm_addr_t;
  // 2-bit user: [0] = is_dma, [1] = core_id (single compute core cluster)
  typedef logic [1:0] tcdm_user_t;
  `TCDM_TYPEDEF_ALL(tcdm, tcdm_addr_t, data_t, strb_t, tcdm_user_t)

  typedef struct packed {
    snitch_pkg::acc_addr_e addr;
    logic [4:0]            id;
    logic [31:0]           data_op;
    data_t                 data_arga;
    data_t                 data_argb;
    addr_t                 data_argc;
  } acc_req_t;

    typedef struct packed {
    logic [4:0] id;
    logic       error;
    data_t      data;
  } acc_resp_t;

  localparam integer unsigned MaxNofRss = 128;
  localparam integer unsigned MaxNofRs = 8;
  localparam integer unsigned SlotIdWidth = cf_math_pkg::idx_width(MaxNofRss);
  localparam integer unsigned RsIdWidth = cf_math_pkg::idx_width(MaxNofRs);

  typedef logic [SlotIdWidth-1:0] slot_id_t;
  typedef logic [RsIdWidth-1:0]   rs_id_t;

  typedef struct packed {
    slot_id_t slot_id;
    rs_id_t   rs_id;
  } producer_id_t;

  localparam integer unsigned FLEN = 64;
  localparam integer unsigned XLEN = 32;
  localparam integer unsigned OpLen = FLEN;

  typedef struct packed {
    schnizo_pkg::fu_t       fu;
    schnizo_pkg::alu_op_e   alu_op;
    schnizo_pkg::lsu_op_e   lsu_op;
    schnizo_pkg::csr_op_e   csr_op;
    schnizo_pkg::fpu_op_e   fpu_op;
    logic [OpLen-1:0]       operand_a;
    logic                   use_operand_a;
    logic [OpLen-1:0]       operand_b;
    logic                   use_operand_b;
    logic [OpLen-1:0]       imm;
    logic                   use_imm;
    schnizo_pkg::lsu_size_e lsu_size;
    fpnew_pkg::fp_format_e  fpu_fmt_src;
    fpnew_pkg::fp_format_e  fpu_fmt_dst;
    fpnew_pkg::roundmode_e  fpu_rnd_mode;
    logic [XLEN-1:0]        raw_instr;
  } fu_data_t;

  typedef struct packed {
    producer_id_t producer;
    logic         valid;
  } rmt_entry_t;

  typedef struct packed {
    fu_data_t                fu_data;
    rmt_entry_t              producer_op_a;
    rmt_entry_t              producer_op_b;
    rmt_entry_t              producer_op_c;
    rmt_entry_t              current_producer_dest;
    schnizo_pkg::instr_tag_t tag;
  } disp_req_t;

  typedef struct packed {
    producer_id_t producer;
  } disp_rsp_t;

  typedef struct packed {
    fu_data_t                fu_data;
    schnizo_pkg::instr_tag_t tag;
  } issue_req_t;

  typedef logic [XLEN-1:0] alu_res_val_t;

  typedef struct packed {
    alu_res_val_t result;
    logic         compare_res;
  } alu_result_t;

  typedef struct packed {
    logic valid;
    logic iteration;
  } available_result_t;

  typedef struct packed {
    logic        requested_iter;
    slot_id_t    slot_id;
  } res_req_t;

  typedef struct packed {
    rs_id_t   producer;
    res_req_t request;
  } operand_req_t;

  typedef logic [OpLen-1:0] operand_t;

  localparam integer unsigned NofOperandIfs = 18;

  typedef logic [NofOperandIfs-1:0] dest_mask_t;

  typedef struct packed {
    dest_mask_t dest_mask;
    operand_t   operand;
  } res_rsp_t;

  typedef struct packed {
    dest_mask_t  dest_mask;
    slot_id_t    slot_id;
  } ext_res_req_t;

  localparam integer unsigned NofOperandIfsW = cf_math_pkg::idx_width(NofOperandIfs);

  typedef logic [NofOperandIfsW-1:0] operand_id_t;

  localparam fpnew_pkg::fpu_implementation_t FpuImplementation = {
    PipeRegs: // FMA Block
              '{
                '{  2, // FP32
                    3, // FP64
                    1, // FP16
                    1, // FP8
                    1, // FP16alt
                    1  // FP8alt
                  },
                '{1, 1, 1, 1, 1, 1},   // DIVSQRT
                '{1,
                  1,
                  1,
                  1,
                  1,
                  1},   // NONCOMP
                '{2,
                  2,
                  2,
                  2,
                  2,
                  2},   // CONV
                '{3,
                  3,
                  3,
                  3,
                  3,
                  3}    // DOTP
                },
    UnitTypes: '{'{fpnew_pkg::MERGED,
                    fpnew_pkg::MERGED,
                    fpnew_pkg::MERGED,
                    fpnew_pkg::MERGED,
                    fpnew_pkg::MERGED,
                    fpnew_pkg::MERGED},  // FMA
                '{fpnew_pkg::DISABLED,
                    fpnew_pkg::DISABLED,
                    fpnew_pkg::DISABLED,
                    fpnew_pkg::DISABLED,
                    fpnew_pkg::DISABLED,
                    fpnew_pkg::DISABLED}, // DIVSQRT
                '{fpnew_pkg::PARALLEL,
                    fpnew_pkg::PARALLEL,
                    fpnew_pkg::PARALLEL,
                    fpnew_pkg::PARALLEL,
                    fpnew_pkg::PARALLEL,
                    fpnew_pkg::PARALLEL}, // NONCOMP
                '{fpnew_pkg::MERGED,
                    fpnew_pkg::MERGED,
                    fpnew_pkg::MERGED,
                    fpnew_pkg::MERGED,
                    fpnew_pkg::MERGED,
                    fpnew_pkg::MERGED},   // CONV
                '{fpnew_pkg::DISABLED,
                    fpnew_pkg::DISABLED,
                    fpnew_pkg::DISABLED,
                    fpnew_pkg::DISABLED,
                    fpnew_pkg::DISABLED,
                    fpnew_pkg::DISABLED}}, // DOTP
    PipeConfig: fpnew_pkg::BEFORE
  };

endpackage
