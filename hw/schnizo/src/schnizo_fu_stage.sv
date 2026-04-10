// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// TODO(colluca): there's a lot of common boilerplate in the connection between an
// RS and its FU. Can we reduce it and improve code reuse?
// TODO(colluca): move ODN to its own module.

`include "common_cells/registers.svh"

// The module hosting all RSs and FUs.
//
// Instantiates all the FUs and connects each FU to an FU block (containing the RS).
// Further instantiates the operand distribution network (ODN) and connects FU blocks to it.
module schnizo_fu_stage import schnizo_pkg::*, schnizo_tracer_pkg::*; #(
  // Globally enable the superscalar feature
  parameter bit          Xfrep             = 1,
  parameter bit          MulInAlu0         = 1'b1,
  parameter int unsigned NofAlus           = 1,
  parameter int unsigned AluNofRss         = 3,
  parameter int unsigned AluNofOperands    = 2,
  parameter int unsigned AluNofOpPorts     = 1,
  parameter int unsigned AluNofResReqIfs   = 3,
  parameter int unsigned AluNofResRspPorts = 1,
  parameter int unsigned NofLsus           = 1,
  parameter int unsigned LsuNofRss         = 3,
  parameter int unsigned LsuNofOperands    = 4,
  parameter int unsigned LsuNofOpPorts     = 1,
  parameter int unsigned LsuNofResReqIfs   = 3,
  parameter int unsigned LsuNofResRspPorts = 1,
  parameter int unsigned NofFpus           = 1,
  parameter int unsigned FpuNofRss         = 2,
  parameter int unsigned FpuNofOperands    = 3,
  parameter int unsigned FpuNofOpPorts     = 1,
  parameter int unsigned FpuNofResReqIfs   = 3,
  parameter int unsigned FpuNofResRspPorts = 1,
  // Spatz Parameter
  parameter int unsigned SpatzNofRss       = 1,
  parameter int unsigned SpatzNofOperands  = 2,
  parameter int unsigned SpatzNofOpPorts   = 1,
  parameter int unsigned SpatzNofResReqIfs = 1,
  parameter int unsigned SpatzNofResRspIfs = 1,
  // The following 3 NofIfs parameters depend directly on the previous FU specific Nof parameters
  // but they must be defined on the outer scope as they are needed there as well.
  // Make sure to match them!
  // TODO(colluca): use a function or something to ensure the consistency of these parameters.
  parameter int unsigned NofOperandIfs   = 1,
  parameter int unsigned NofResReqIfs    = 1,
  parameter int unsigned XLEN            = 32,
  parameter int unsigned FLEN            = 64,
  parameter int unsigned OpLen           = 64,
  parameter int unsigned AddrWidth       = 32,
  parameter int unsigned DataWidth       = 32,
  parameter int unsigned RegAddrWidth    = 5,
  parameter int unsigned MaxIterationsW  = 5,
  // Consistency Address Queue (CAQ) parameters
  parameter int unsigned CaqDepth    = 0,
  parameter int unsigned CaqTagWidth = 0,
  /// How many issued loads the LSU and thus the CAQ (consistency address queue) can hold.
  // This applies to all LSUs (each LSU can handle NumOutstandingLoads loads).
  parameter int unsigned NumOutstandingLoads = 0,
  /// How many total transactions (load and store) the LSU can handle at once
  // This applies to all LSUs (each LSU can handle NumOutstandingMem transactions).
  parameter int unsigned NumOutstandingMem = 0,
  /// FPU parameters
  parameter fpnew_pkg::fpu_implementation_t FPUImplementation = '0,
  parameter bit          RVF            = 1,
  parameter bit          RVD            = 1,
  parameter bit          XF16           = 0,
  parameter bit          XF16ALT        = 0,
  parameter bit          XF8            = 0,
  parameter bit          XF8ALT         = 0,
  // Vectors are not implemented! Here for compatibility with Snitch.
  parameter bit          XFVEC          = 0,
  // Register the signals directly before the FPnew instance
  parameter bit          RegisterFPUIn  = 0,
  // Register the signals directly after the FPnew instance
  parameter bit          RegisterFPUOut = 0,
  /// Others
  parameter type         producer_id_t  = logic,
  parameter type         slot_id_t      = logic,
  parameter type         rs_id_t        = logic,
  parameter type         operand_id_t   = logic, // used for tracer function
  parameter type         disp_req_t     = logic,
  parameter type         disp_rsp_t     = logic,
  parameter type         fu_data_t      = logic,
  parameter type         instr_tag_t    = logic,
  parameter type         alu_result_t   = logic,
  parameter type         alu_res_val_t  = logic,
  // Spatz Result can be floating point or integer
  parameter type         spatz_result_t = logic [FLEN-1:0],
  parameter type         dreq_t         = logic,
  parameter type         drsp_t         = logic,
  // SPATZ Parameters
  parameter bit          RVV            = 1,
  parameter int unsigned NumSpatzFPUs   = 4,
  parameter int unsigned NumSpatzIPUs   = 1,
  // TCDM Types for Spatz
  parameter type         tcdm_req_chan_t  = logic,
  parameter type         tcdm_rsp_chan_t  = logic,
  parameter type         tcdm_req_t       = logic,
  parameter type         tcdm_rsp_t       = logic,

  /// Derived parameter *Do not override*
  parameter int unsigned NumSpatzFUs         = (NumSpatzFPUs > NumSpatzIPUs) ? NumSpatzFPUs : NumSpatzIPUs,
  parameter int unsigned NumMemPortsPerSpatz = NumSpatzFUs,
  parameter int unsigned TCDMPorts           = RVV ? NumMemPortsPerSpatz + NofLsus : NofLsus,

  localparam type addr_t = logic [AddrWidth-1:0],
  localparam type data_t = logic [DataWidth-1:0]
) (
  input  logic        clk_i,
  input  logic        rst_i,
  input  logic [31:0] hard_id_i,

  // Trace outputs
  // pragma translate_off
  output issue_alu_trace_t  alu_trace_o        [NofAlus-1:0],
  output issue_lsu_trace_t  lsu_trace_o        [NofLsus-1:0],
  output issue_fpu_trace_t  fpu_trace_o        [NofFpus-1:0],
  output retire_fu_trace_t  alu_retire_trace_o [NofAlus-1:0],
  output retire_fu_trace_t  lsu_retire_trace_o [NofLsus-1:0],
  output retire_fu_trace_t  fpu_retire_trace_o [NofFpus-1:0],
  // pragma translate_on

  /// RS control signals
  input  logic                      restart_i,
  input  loop_state_e               loop_state_i,
  input  logic [MaxIterationsW-1:0] lep_iterations_i,
  input  logic                      goto_lcp2_i,
  // Asserted if all RS are either finishing in this cycle or have already finished
  output logic                      all_rs_finish_o,

  /// Instruction streams & FU status signals
  input  disp_req_t               disp_req_i,
  // Commit the dispatch request and allow the downstream passing.
  // The FU blocks do not require a commit signal as when we won't commit we have an exception
  // and thus anyway abort the LxP and reset the RS and RSSs.
  input  logic                    instr_exec_commit_i,
  input  logic      [NofAlus-1:0] alu_disp_reqs_valid_i,
  output logic      [NofAlus-1:0] alu_disp_reqs_ready_o,
  output disp_rsp_t [NofAlus-1:0] alu_disp_rsp_o,
  output logic      [NofAlus-1:0] alu_rs_full_o,

  input  logic      [NofLsus-1:0] lsu_disp_reqs_valid_i,
  output logic      [NofLsus-1:0] lsu_disp_reqs_ready_o,
  output disp_rsp_t [NofLsus-1:0] lsu_disp_rsp_o,
  output logic                    lsu_empty_o,
  output logic                    lsu_addr_misaligned_o,
  output dreq_t     [NofLsus-1:0] lsu_dreq_o,
  input  drsp_t     [NofLsus-1:0] lsu_drsp_i,
  output logic      [NofLsus-1:0] lsu_rs_full_o,
  input  addr_t     [NofLsus-1:0] caq_addr_i,
  input  logic      [NofLsus-1:0] caq_track_write_i,
  input  logic      [NofLsus-1:0] caq_req_valid_i,
  output logic      [NofLsus-1:0] caq_req_ready_o,
  input  logic      [NofLsus-1:0] caq_rsp_valid_i,
  output logic      [NofLsus-1:0] caq_rsp_valid_o,

  input  logic               [NofFpus-1:0] fpu_disp_reqs_valid_i,
  output logic               [NofFpus-1:0] fpu_disp_reqs_ready_o,
  output disp_rsp_t          [NofFpus-1:0] fpu_disp_rsp_o,
  output logic               [NofFpus-1:0] fpu_rs_full_o,
  // Combined status of all FPUs
  output fpnew_pkg::status_t fpu_status_o,
  output logic               fpu_status_valid_o,

  // Minimum amount of signals for spatz. May need to add more if required.
  input  logic      spatz_disp_reqs_valid_i,
  output logic      spatz_disp_reqs_ready_o,
  output disp_rsp_t spatz_disp_rsp_o,
  output logic      spatz_loop_finish_o,
  output logic      spatz_rs_full_o,

  // Writeback ports. We only have one per FU type.
  output alu_result_t alu_wb_result_o,
  output instr_tag_t  alu_wb_result_tag_o,
  output logic        alu_wb_result_valid_o,
  input  logic        alu_wb_result_ready_i,
  output alu_result_t branch_result_o,

  output data_t      lsu_wb_result_o,
  output instr_tag_t lsu_wb_result_tag_o,
  output logic       lsu_wb_result_valid_o,
  input  logic       lsu_wb_result_ready_i,

  output logic [FLEN-1:0] fpu_wb_result_o,
  output instr_tag_t      fpu_wb_result_tag_o,
  output logic            fpu_wb_result_valid_o,
  input  logic            fpu_wb_result_ready_i,

  // Spatz writeback signals

  output spatz_result_t   spatz_wb_result_o, // TODO: Understand what width is needed here
  output instr_tag_t      spatz_wb_result_tag_o,
  output logic            spatz_wb_result_valid_o,
  input  logic            spatz_wb_result_ready_i,

  output logic            spatz_running_instrs_o,

  // SPATZ TCDM Ports
  output tcdm_req_t    [NumMemPortsPerSpatz-1:0] spatz_tcdm_req_o,
  input  tcdm_rsp_t    [NumMemPortsPerSpatz-1:0] spatz_tcdm_rsp_i

);

  /////////////////////////////////////
  // Parameters and type definitions //
  /////////////////////////////////////

  // ---------
  // Issue
  // ---------

  typedef struct packed {
    fu_data_t fu_data;
    instr_tag_t tag;
  } issue_req_t;

  // ---------------------------
  // RSS
  // ---------------------------

  // We need to know the total operands such that we can allocate the correct width for the
  // consumer count inside the FU stage.
  // In theory each other RSS in the system and the RSS itself could be a consumer. Thus use full width.
  // TODO: Add consumer count restriction to achieve a feasible bit width.
  localparam integer unsigned ConsumerCount = AluNofOperands * AluNofRss * NofAlus +
                                              LsuNofOperands * LsuNofRss * NofLsus +
                                              FpuNofOperands * FpuNofRss * NofFpus +
                                              (RVV ? (SpatzNofOperands * SpatzNofRss * 1) : 0); // only 1 Spatz

  // ---------------------------
  // Operand distribution network
  // ---------------------------

  localparam int unsigned NofRs = NofAlus + NofLsus + NofFpus;
  localparam int unsigned TotalNofRss = NofAlus * AluNofRss +
                                        NofLsus * LsuNofRss +
                                        NofFpus * FpuNofRss;
  localparam int unsigned TotalNofResRspPorts = NofAlus * AluNofResRspPorts +
                                                NofLsus * LsuNofResRspPorts +
                                                NofFpus * FpuNofResRspPorts;

  typedef int unsigned rs_param_array_t [NofRs-1:0];

  function automatic rs_param_array_t gen_rs_param_array(int AluParam, int LsuParam, int FpuParam);
    rs_param_array_t tmp;
    int unsigned k;

    k = 0;
    for (int unsigned i = 0; i < NofAlus; i++) begin
      tmp[k] = AluParam;
      k++;
    end
    for (int unsigned i = 0; i < NofLsus; i++) begin
      tmp[k] = LsuParam;
      k++;
    end
    for (int unsigned i = 0; i < NofFpus; i++) begin
      tmp[k] = FpuParam;
      k++;
    end

    return tmp;
  endfunction

  localparam rs_param_array_t NofRss = gen_rs_param_array(AluNofRss, LsuNofRss, FpuNofRss);
  localparam rs_param_array_t NofRspPorts = gen_rs_param_array(AluNofResRspPorts,
    LsuNofResRspPorts, FpuNofResRspPorts);

  // The request arriving at the crossbar output connections. This is converted to a destination
  // mask inside the crossbar output logic. This mask is used to send the result to multiple
  // operands at once.
  typedef struct packed {
    logic        requested_iter;
    slot_id_t    slot_id;
  } res_req_t;

  // The request going into the request crossbar
  // TODO(colluca): replace with a flat struct, called result_tag_t
  typedef struct packed {
    rs_id_t   producer; // where to place the request
    res_req_t request;
  } operand_req_t;

  // TODO(colluca): what does W stand for? In schnizo.sv there is actually a difference between the
  // two parameters
  localparam integer unsigned NofOperandIfsW = NofOperandIfs;

  // To which operand we should send the current result. This is a bitvector selecting each operand
  // where we want to send the result. The actual request uses the operand_id_t because this is
  // smaller and thus the request crossbar is also smaller. The conversion happens at the output of
  // the request crossbar. This signal then controls the response crossbar.
  typedef logic [NofOperandIfsW-1:0] dest_mask_t;

  // The data coming out of the response crossbar.
  typedef logic [OpLen-1:0] operand_t;

  // The request going into the response crossbar.
  typedef struct packed {
    dest_mask_t dest_mask; // where to send the response to
    operand_t   operand;
  } res_rsp_t;

  // ---------------------------
  // RS ID generation
  // ---------------------------

  // Each RS needs a globally unique ID. We simply count all RS.
  localparam integer unsigned AluRsIdOffset = 0;
  localparam integer unsigned LsuRsIdOffset = AluRsIdOffset + NofAlus;
  localparam integer unsigned FpuRsIdOffset = LsuRsIdOffset + NofLsus;
  localparam integer unsigned SpatzRsIdOffset = FpuRsIdOffset + NofFpus;

  // Each RS needs a globally unique ID for its operand request ports.
  // These IDs are shared between the RSSs of a reservation station.
  localparam integer unsigned AluOpIdOffset = 0;
  localparam integer unsigned LsuOpIdOffset = AluOpIdOffset +
                                              NofAlus * (AluNofOperands * AluNofOpPorts);
  localparam integer unsigned FpuOpIdOffset = LsuOpIdOffset +
                                              NofLsus * (LsuNofOperands * LsuNofOpPorts);
  localparam integer unsigned SpatzOpIdOffset = FpuOpIdOffset +
                                              NofFpus * (FpuNofOperands * FpuNofOpPorts);

  ////////////////////////////////////////
  // Operand distribution network (ODN) //
  ////////////////////////////////////////

  operand_req_t [NofAlus-1:0][AluNofOpPorts-1:0][AluNofOperands-1:0]  alu_op_reqs;
  logic         [NofAlus-1:0][AluNofOpPorts-1:0][AluNofOperands-1:0]  alu_op_reqs_valid;
  logic         [NofAlus-1:0][AluNofOpPorts-1:0][AluNofOperands-1:0]  alu_op_reqs_ready;
  operand_req_t [NofAlus-1:0][AluNofRss-1:0]                          alu_available_results;
  dest_mask_t   [NofAlus-1:0][AluNofRss-1:0]                          alu_res_reqs;
  logic         [NofAlus-1:0][AluNofRss-1:0]                          alu_res_reqs_valid;
  logic         [NofAlus-1:0][AluNofRss-1:0]                          alu_res_reqs_ready;
  res_rsp_t     [NofAlus-1:0][AluNofRss-1:0]                          alu_res_rsps;
  logic         [NofAlus-1:0][AluNofRss-1:0]                          alu_res_rsps_valid;
  logic         [NofAlus-1:0][AluNofRss-1:0]                          alu_res_rsps_ready;
  operand_t     [NofAlus-1:0][AluNofOpPorts-1:0][AluNofOperands-1:0]  alu_op_rsps;
  logic         [NofAlus-1:0][AluNofOpPorts-1:0][AluNofOperands-1:0]  alu_op_rsps_valid;
  logic         [NofAlus-1:0][AluNofOpPorts-1:0][AluNofOperands-1:0]  alu_op_rsps_ready;

  operand_req_t [NofLsus-1:0][LsuNofOpPorts-1:0][LsuNofOperands-1:0]  lsu_op_reqs;
  logic         [NofLsus-1:0][LsuNofOpPorts-1:0][LsuNofOperands-1:0]  lsu_op_reqs_valid;
  logic         [NofLsus-1:0][LsuNofOpPorts-1:0][LsuNofOperands-1:0]  lsu_op_reqs_ready;
  operand_req_t [NofLsus-1:0][LsuNofRss-1:0]                          lsu_available_results;
  dest_mask_t   [NofLsus-1:0][LsuNofRss-1:0]                          lsu_res_reqs;
  logic         [NofLsus-1:0][LsuNofRss-1:0]                          lsu_res_reqs_valid;
  logic         [NofLsus-1:0][LsuNofRss-1:0]                          lsu_res_reqs_ready;
  res_rsp_t     [NofLsus-1:0][LsuNofRss-1:0]                          lsu_res_rsps;
  logic         [NofLsus-1:0][LsuNofRss-1:0]                          lsu_res_rsps_valid;
  logic         [NofLsus-1:0][LsuNofRss-1:0]                          lsu_res_rsps_ready;
  operand_t     [NofLsus-1:0][LsuNofOpPorts-1:0][LsuNofOperands-1:0]  lsu_op_rsps;
  logic         [NofLsus-1:0][LsuNofOpPorts-1:0][LsuNofOperands-1:0]  lsu_op_rsps_valid;
  logic         [NofLsus-1:0][LsuNofOpPorts-1:0][LsuNofOperands-1:0]  lsu_op_rsps_ready;

  operand_req_t [NofFpus-1:0][FpuNofOpPorts-1:0][FpuNofOperands-1:0]  fpu_op_reqs;
  logic         [NofFpus-1:0][FpuNofOpPorts-1:0][FpuNofOperands-1:0]  fpu_op_reqs_valid;
  logic         [NofFpus-1:0][FpuNofOpPorts-1:0][FpuNofOperands-1:0]  fpu_op_reqs_ready;
  operand_req_t [NofFpus-1:0][FpuNofRss-1:0]                          fpu_available_results;
  dest_mask_t   [NofFpus-1:0][FpuNofRss-1:0]                          fpu_res_reqs;
  logic         [NofFpus-1:0][FpuNofRss-1:0]                          fpu_res_reqs_valid;
  logic         [NofFpus-1:0][FpuNofRss-1:0]                          fpu_res_reqs_ready;
  res_rsp_t     [NofFpus-1:0][FpuNofRss-1:0]                          fpu_res_rsps;
  logic         [NofFpus-1:0][FpuNofRss-1:0]                          fpu_res_rsps_valid;
  logic         [NofFpus-1:0][FpuNofRss-1:0]                          fpu_res_rsps_ready;
  operand_t     [NofFpus-1:0][FpuNofOpPorts-1:0][FpuNofOperands-1:0]  fpu_op_rsps;
  logic         [NofFpus-1:0][FpuNofOpPorts-1:0][FpuNofOperands-1:0]  fpu_op_rsps_valid;
  logic         [NofFpus-1:0][FpuNofOpPorts-1:0][FpuNofOperands-1:0]  fpu_op_rsps_ready;

  operand_req_t [SpatzNofOpPorts-1:0][SpatzNofOperands-1:0]  spatz_op_reqs;
  logic         [SpatzNofOpPorts-1:0][SpatzNofOperands-1:0]  spatz_op_reqs_valid;
  logic         [SpatzNofOpPorts-1:0][SpatzNofOperands-1:0]  spatz_op_reqs_ready;
  operand_req_t [SpatzNofRss-1:0]                            spatz_available_results;
  dest_mask_t   [SpatzNofRss-1:0]                            spatz_res_reqs;
  logic         [SpatzNofRss-1:0]                            spatz_res_reqs_valid;
  logic         [SpatzNofRss-1:0]                            spatz_res_reqs_ready;
  res_rsp_t     [SpatzNofRss-1:0]                            spatz_res_rsps;
  logic         [SpatzNofRss-1:0]                            spatz_res_rsps_valid;
  logic         [SpatzNofRss-1:0]                            spatz_res_rsps_ready;
  operand_t     [SpatzNofOpPorts-1:0][SpatzNofOperands-1:0]  spatz_op_rsps;
  logic         [SpatzNofOpPorts-1:0][SpatzNofOperands-1:0]  spatz_op_rsps_valid;
  logic         [SpatzNofOpPorts-1:0][SpatzNofOperands-1:0]  spatz_op_rsps_ready;

  operand_req_t [NofOperandIfs-1:0] op_reqs;
  logic         [NofOperandIfs-1:0] op_reqs_valid;
  logic         [NofOperandIfs-1:0] op_reqs_ready;

  dest_mask_t   [TotalNofRss-1:0] res_reqs;
  logic         [TotalNofRss-1:0] res_reqs_valid;
  logic         [TotalNofRss-1:0] res_reqs_ready;
  operand_req_t [TotalNofRss-1:0] available_results;

  res_rsp_t     [TotalNofRss-1:0] res_rsps;
  logic         [TotalNofRss-1:0] res_rsps_valid;
  logic         [TotalNofRss-1:0] res_rsps_ready;

  operand_t     [NofOperandIfs-1:0] op_rsps;
  logic         [NofOperandIfs-1:0] op_rsps_valid;
  logic         [NofOperandIfs-1:0] op_rsps_ready;

  if (Xfrep) begin : gen_odn

    // ---------------------------
    // Pack operand interfaces
    // ---------------------------

    // Pack the FUs' operand requests and responses into a linear array to connect to the XBAR.
    // The array index must match the operand / consumer id.
    // TODO(colluca): think if this code can be streamlined
    always_comb begin : fu_op_reqs_rsps
      automatic integer ope_if = 0;

      op_reqs           = '0;
      op_reqs_valid     = '0;
      alu_op_reqs_ready = '0;
      lsu_op_reqs_ready = '0;
      fpu_op_reqs_ready = '0;
      spatz_op_reqs_ready = '0;

      op_rsps_ready     = '0;
      alu_op_rsps       = '0;
      alu_op_rsps_valid = '0;
      lsu_op_rsps       = '0;
      lsu_op_rsps_valid = '0;
      fpu_op_rsps       = '0;
      fpu_op_rsps_valid = '0;
      spatz_op_rsps     = '0;
      spatz_op_rsps_valid = '0;

      for (int alu = 0; alu < NofAlus; alu++) begin
        for (int port = 0; port < AluNofOpPorts; port++) begin
          for (int op = 0; op < AluNofOperands; op++) begin
            // operand requests
            op_reqs[ope_if]                  = alu_op_reqs[alu][port][op];
            op_reqs_valid[ope_if]            = alu_op_reqs_valid[alu][port][op];
            alu_op_reqs_ready[alu][port][op] = op_reqs_ready[ope_if];
            // operand responses
            alu_op_rsps[alu][port][op]       = op_rsps[ope_if];
            alu_op_rsps_valid[alu][port][op] = op_rsps_valid[ope_if];
            op_rsps_ready[ope_if]            = alu_op_rsps_ready[alu][port][op];
            ope_if = ope_if + 1;
          end
        end
      end
      for (int lsu = 0; lsu < NofLsus; lsu++) begin
        for (int port = 0; port < LsuNofOpPorts; port++) begin
          for (int op = 0; op < LsuNofOperands; op++) begin
            // operand requests
            op_reqs[ope_if]                  = lsu_op_reqs[lsu][port][op];
            op_reqs_valid[ope_if]            = lsu_op_reqs_valid[lsu][port][op];
            lsu_op_reqs_ready[lsu][port][op] = op_reqs_ready[ope_if];
            // operand responses
            lsu_op_rsps[lsu][port][op]       = op_rsps[ope_if];
            lsu_op_rsps_valid[lsu][port][op] = op_rsps_valid[ope_if];
            op_rsps_ready[ope_if]            = lsu_op_rsps_ready[lsu][port][op];
            ope_if = ope_if + 1;
          end
        end
      end
      for (int fpu = 0; fpu < NofFpus; fpu++) begin
        for (int port = 0; port < FpuNofOpPorts; port++) begin
          for (int op = 0; op < FpuNofOperands; op++) begin
            // operand requests
            op_reqs[ope_if]                  = fpu_op_reqs[fpu][port][op];
            op_reqs_valid[ope_if]            = fpu_op_reqs_valid[fpu][port][op];
            fpu_op_reqs_ready[fpu][port][op] = op_reqs_ready[ope_if];
            // operand responses
            fpu_op_rsps[fpu][port][op]       = op_rsps[ope_if];
            fpu_op_rsps_valid[fpu][port][op] = op_rsps_valid[ope_if];
            op_rsps_ready[ope_if]            = fpu_op_rsps_ready[fpu][port][op];
            ope_if = ope_if + 1;
          end
        end
      end
      // Spatz integration
      if(RVV) begin
        for (int port = 0; port < SpatzNofOpPorts; port++) begin
          for (int op = 0; op < SpatzNofOperands; op++) begin
            // operand requests
            op_reqs[ope_if]                    = spatz_op_reqs[port][op];
            op_reqs_valid[ope_if]              = spatz_op_reqs_valid[port][op];
            spatz_op_reqs_ready[port][op]      = op_reqs_ready[ope_if];
            // operand responses
            spatz_op_rsps[port][op]            = op_rsps[ope_if];
            spatz_op_rsps_valid[port][op]      = op_rsps_valid[ope_if];
            op_rsps_ready[ope_if]              = spatz_op_rsps_ready[port][op];
            ope_if = ope_if + 1;
          end
        end
      end
    end

    // Unpack the linear array of result requests onto the FUs' result request interfaces.
    // Pack the FUs' result responses (one per slot) into a linear array to connect to the XBAR.
    // The array index must match the result / producer id.
    // TODO(colluca): think if this code can be streamlined
    always_comb begin : fu_res_reqs_rsps
      automatic integer req_if = 0;
      automatic integer rsp_if = 0;

      res_reqs_ready     = '0;
      alu_res_reqs       = '0;
      alu_res_reqs_valid = '0;
      lsu_res_reqs       = '0;
      fpu_res_reqs       = '0;
      spatz_res_reqs     = '0;

      res_rsps           = '0;
      res_rsps_valid     = '0;
      alu_res_rsps_ready = '0;
      lsu_res_rsps_ready = '0;
      fpu_res_rsps_ready = '0;
      spatz_res_rsps_ready = '0;

      for (int alu = 0; alu < NofAlus; alu++) begin
        for (int alu_req_if = 0; alu_req_if < AluNofRss; alu_req_if++) begin
          // requests
          alu_res_reqs[alu][alu_req_if]       = res_reqs[req_if];
          alu_res_reqs_valid[alu][alu_req_if] = res_reqs_valid[req_if];
          res_reqs_ready[req_if]              = alu_res_reqs_ready[alu][alu_req_if];
          available_results[req_if]           = alu_available_results[alu][alu_req_if];
          req_if = req_if + 1;
        end
        for (int rsp = 0; rsp < AluNofRss; rsp++) begin
          // responses
          res_rsps[rsp_if]             = alu_res_rsps[alu][rsp];
          res_rsps_valid[rsp_if]       = alu_res_rsps_valid[alu][rsp];
          alu_res_rsps_ready[alu][rsp] = res_rsps_ready[rsp_if];
          rsp_if = rsp_if + 1;
        end
      end
      for (int lsu = 0; lsu < NofLsus; lsu++) begin
        for (int lsu_req_if = 0; lsu_req_if < LsuNofRss; lsu_req_if++) begin
          // requests
          lsu_res_reqs[lsu][lsu_req_if]       = res_reqs[req_if];
          lsu_res_reqs_valid[lsu][lsu_req_if] = res_reqs_valid[req_if];
          res_reqs_ready[req_if]              = lsu_res_reqs_ready[lsu][lsu_req_if];
          available_results[req_if]           = lsu_available_results[lsu][lsu_req_if];
          req_if = req_if + 1;
        end
        for (int rsp = 0; rsp < LsuNofRss; rsp++) begin
          // responses
          res_rsps[rsp_if]             = lsu_res_rsps[lsu][rsp];
          res_rsps_valid[rsp_if]       = lsu_res_rsps_valid[lsu][rsp];
          lsu_res_rsps_ready[lsu][rsp] = res_rsps_ready[rsp_if];
          rsp_if = rsp_if + 1;
        end
      end
      for (int fpu = 0; fpu < NofFpus; fpu++) begin
        for (int fpu_req_if = 0; fpu_req_if < FpuNofRss; fpu_req_if++) begin
          // requests
          fpu_res_reqs[fpu][fpu_req_if]       = res_reqs[req_if];
          fpu_res_reqs_valid[fpu][fpu_req_if] = res_reqs_valid[req_if];
          res_reqs_ready[req_if]              = fpu_res_reqs_ready[fpu][fpu_req_if];
          available_results[req_if]           = fpu_available_results[fpu][fpu_req_if];
          req_if = req_if + 1;
        end
        for (int rsp = 0; rsp < FpuNofRss; rsp++) begin
          // responses
          res_rsps[rsp_if]             = fpu_res_rsps[fpu][rsp];
          res_rsps_valid[rsp_if]       = fpu_res_rsps_valid[fpu][rsp];
          fpu_res_rsps_ready[fpu][rsp] = res_rsps_ready[rsp_if];
          rsp_if = rsp_if + 1;
        end
      end
      if (RVV) begin
        //SPATZ
        for (int spatz_req_if = 0; spatz_req_if < SpatzNofResReqIfs; spatz_req_if++) begin
          // requests
          spatz_res_reqs[spatz_req_if]       = res_reqs[req_if];
          spatz_res_reqs_valid[spatz_req_if] = res_reqs_valid[req_if];
          res_reqs_ready[req_if]             = spatz_res_reqs_ready[spatz_req_if];
          available_results[req_if]           = spatz_available_results[spatz_req_if];
          req_if = req_if + 1;
        end
        for (int rsp = 0; rsp < SpatzNofRss; rsp++) begin
          // responses
          res_rsps[rsp_if]             = spatz_res_rsps[rsp];
          res_rsps_valid[rsp_if]       = spatz_res_rsps_valid[rsp];
          spatz_res_rsps_ready[rsp]    = res_rsps_ready[rsp_if];
          rsp_if = rsp_if + 1;
        end
      end
    end

    // ---------------------------
    // Operand request XBAR
    // ---------------------------

    schnizo_req_xbar #(
      .NofOperandReqs(NofOperandIfs),
      .NofResRspIfs  (TotalNofRss),
      .operand_req_t (operand_req_t),
      .dest_mask_t   (dest_mask_t)
    ) i_request_xbar (
      .op_reqs_i          (op_reqs),
      .op_reqs_valid_i    (op_reqs_valid),
      .op_reqs_ready_o    (op_reqs_ready),
      .available_results_i(available_results),
      .res_reqs_o         (res_reqs),
      .res_reqs_valid_o   (res_reqs_valid),
      .res_reqs_ready_i   (res_reqs_ready)
    );

    // ---------------------------
    // Operand distribution network - response xbar
    // ---------------------------
    operand_t   [TotalNofRss-1:0] res_rsps_operands;
    dest_mask_t [TotalNofRss-1:0] res_rsps_dest_masks;

    for (genvar i = 0; i < TotalNofRss; i++) begin : gen_flatten_res_rsps
      assign res_rsps_operands[i]  = res_rsps[i].operand;
      assign res_rsps_dest_masks[i] = res_rsps[i].dest_mask;
    end

    schnizo_rsp_xbar #(
      .NofRs           (NofRs),
      .NofRss          (NofRss),
      .NofRspPorts     (NofRspPorts),
      .TotalNofRspPorts(TotalNofResRspPorts),
      .NumInp          (TotalNofRss),
      .NumOut          (NofOperandIfs),
      .payload_t       (operand_t)
    ) i_response_xbar (
      .clk_i,
      .rst_ni (!rst_i),
      .data_i (res_rsps_operands),
      .sel_i  (res_rsps_dest_masks),
      .valid_i(res_rsps_valid),
      .ready_o(res_rsps_ready),
      .data_o (op_rsps),
      .valid_o(op_rsps_valid),
      .ready_i(op_rsps_ready)
    );

  end else begin : gen_no_odn
    // Tie down all signals of the operand distribution network which are set either by a crossbar
    // or when distributing to the reservation stations.
    assign op_reqs           = '0;
    assign op_reqs_valid     = '0;
    assign alu_op_reqs_ready = '0;
    assign lsu_op_reqs_ready = '0;
    assign fpu_op_reqs_ready = '0;
    assign spatz_op_reqs_ready = '0;

    assign op_rsps_ready     = '0;
    assign alu_op_rsps       = '0;
    assign alu_op_rsps_valid = '0;
    assign lsu_op_rsps       = '0;
    assign lsu_op_rsps_valid = '0;
    assign fpu_op_rsps       = '0;
    assign fpu_op_rsps_valid = '0;
    assign spatz_op_rsps     = '0;
    assign spatz_op_rsps_valid = '0;

    assign res_reqs_ready     = '0;
    assign alu_res_reqs       = '0;
    assign alu_res_reqs_valid = '0;
    assign lsu_res_reqs       = '0;
    assign fpu_res_reqs       = '0;
    assign spatz_res_reqs     = '0;

    assign res_rsps           = '0;
    assign res_rsps_valid     = '0;
    assign alu_res_rsps_ready = '0;
    assign lsu_res_rsps_ready = '0;
    assign fpu_res_rsps_ready = '0;
    assign spatz_res_rsps_ready = '0;

    assign op_reqs_ready  = '0;
    assign res_reqs       = '0;
    assign res_reqs_valid = '0;

    assign res_rsps_ready = '0;
    assign op_rsps        = '0;
    assign op_rsps_valid  = '0;
  end

  // ---------------------------
  // LxP data path selection
  // ---------------------------
  // We generate one global signal which then controls all request/issue/result/wb MUXs.
  // This should enable the timing separation for the branch result.
  logic in_lxp;
  assign in_lxp = Xfrep ? loop_state_i inside {LoopLcp1, LoopLcp2, LoopLep} : 1'b0;

  logic in_lcp;
  assign in_lcp = Xfrep ? loop_state_i inside {LoopLcp1, LoopLcp2} : 1'b0;

  //////////
  // ALUs //
  //////////

  typedef logic [cf_math_pkg::idx_width(AluNofRss)-1:0] alu_rs_tag_t;

  typedef logic [cf_math_pkg::max($bits(alu_rs_tag_t),$bits(instr_tag_t))-1:0] alu_instr_tag_t;

  typedef struct packed {
    fu_data_t       fu_data;
    alu_instr_tag_t tag;
  } alu_issue_req_t;

  typedef struct packed {
    alu_result_t result;
    instr_tag_t  tag;
  } alu_result_and_tag_t;

  alu_result_and_tag_t [NofAlus-1:0] alu_wbs_result_and_tag;
  logic                [NofAlus-1:0] alu_wbs_result_valid;
  logic                [NofAlus-1:0] alu_wbs_result_ready;

  logic [NofAlus-1:0] alu_loop_finish;

  for (genvar alu = 0; alu < NofAlus; alu++) begin : gen_alus
    // Helper signals to merge the result and tag
    alu_res_val_t   alu_wb_result_value;
    alu_instr_tag_t alu_wb_result_tag;

    // Signals connecting the FU block and the actual FU
    alu_issue_req_t alu_issue_req;
    logic           alu_issue_req_valid;
    logic           alu_issue_req_ready;
    logic           alu_exec_commit;
    alu_result_t    alu_result;
    alu_res_val_t   alu_result_value;
    alu_instr_tag_t alu_result_tag;
    logic           alu_result_valid_raw;
    logic           alu_result_valid;
    logic           alu_result_ready;
    logic           alu_busy;

    producer_id_t producer_start_id;
    assign producer_start_id = producer_id_t'{
      slot_id: '0, // does not matter
      rs_id:   rs_id_t'(AluRsIdOffset + alu)
    };

    // pragma translate_off
    issue_alu_trace_t alu_trace_int;
    // pragma translate_on

    schnizo_fu_block #(
      .Xfrep         (Xfrep),
      .disp_req_t    (disp_req_t),
      .disp_rsp_t    (disp_rsp_t),
      .issue_req_t   (alu_issue_req_t),
      .result_t      (alu_res_val_t),
      .instr_tag_t   (alu_instr_tag_t),
      .NofRss        (AluNofRss),
      .NofOperands   (AluNofOperands),
      .NofOpPorts    (AluNofOpPorts),
      .NofResRspIfs  (AluNofRss),
      .ConsumerCount (ConsumerCount),
      .RegAddrWidth  (RegAddrWidth),
      .MaxIterationsW(MaxIterationsW),
      .producer_id_t (producer_id_t),
      .slot_id_t     (slot_id_t),
      .operand_req_t (operand_req_t),
      .operand_t     (operand_t),
      .res_req_t     (res_req_t),
      .dest_mask_t   (dest_mask_t),
      .res_rsp_t     (res_rsp_t)
    ) i_fu_block (
      .clk_i,
      .rst_i,
      /// RS control signals
      .producer_id_i      (producer_start_id),
      .restart_i          (restart_i),
      .loop_state_i       (loop_state_i),
      .in_lxp_i           (in_lxp),
      .lep_iterations_i   (lep_iterations_i),
      .goto_lcp2_i        (goto_lcp2_i),
      .fu_busy_i          (alu_busy),
      .loop_finish_o      (alu_loop_finish[alu]),
      .rs_full_o          (alu_rs_full_o[alu]),
      /// Instruction stream
      // From dispatcher
      .disp_req_i         (disp_req_i),
      .disp_req_valid_i   (alu_disp_reqs_valid_i[alu]),
      .disp_req_ready_o   (alu_disp_reqs_ready_o[alu]),
      .instr_exec_commit_i(instr_exec_commit_i),
      .disp_rsp_o         (alu_disp_rsp_o[alu]),
      // To FU
      .issue_req_o        (alu_issue_req),
      .issue_req_valid_o  (alu_issue_req_valid),
      .issue_req_ready_i  (alu_issue_req_ready),
      .instr_exec_commit_o(alu_exec_commit),
      // From FU
      .result_i           (alu_result.result),
      .result_tag_i       (alu_result_tag),
      .result_valid_i     (alu_result_valid),
      .result_ready_o     (alu_result_ready),
      // To writeback
      .wb_result_o        (alu_wb_result_value),
      .wb_result_tag_o    (alu_wb_result_tag),
      .wb_result_valid_o  (alu_wbs_result_valid[alu]),
      .wb_result_ready_i  (alu_wbs_result_ready[alu]),
      /// Operand distribution network
      .available_results_o(alu_available_results[alu]),
      .op_reqs_o          (alu_op_reqs[alu]),
      .op_reqs_valid_o    (alu_op_reqs_valid[alu]),
      .op_reqs_ready_i    (alu_op_reqs_ready[alu]),
      .res_reqs_i         (alu_res_reqs[alu]),
      .res_reqs_valid_i   (alu_res_reqs_valid[alu]),
      .res_reqs_ready_o   (alu_res_reqs_ready[alu]),
      .res_rsps_o         (alu_res_rsps[alu]),
      .res_rsps_valid_o   (alu_res_rsps_valid[alu]),
      .res_rsps_ready_i   (alu_res_rsps_ready[alu]),
      .op_rsps_i          (alu_op_rsps[alu]),
      .op_rsps_valid_i    (alu_op_rsps_valid[alu]),
      .op_rsps_ready_o    (alu_op_rsps_ready[alu])
    );
    // DANGER!
    // HACK: We do not pass the branch result into the RS so keep the same RS implementation for
    // all FUs. For any write back we directly take the branch result from the FU. This is
    // possible because if we want to do a writeback the RS accepts the result only if the
    // writeback also accepts the writeback. Thus we can bypass the RS.
    // TODO: find a clean solution how to handle the branch result.
    assign alu_wbs_result_and_tag[alu].result = '{
      result:      alu_wb_result_value,
      compare_res: alu_result.compare_res
    };
    assign alu_wbs_result_and_tag[alu].tag = alu_wb_result_tag;

    schnizo_alu #(
      .XLEN         (XLEN),
      .HasBranch    (alu == '0), // only the first ALU has the branch logic
      .HasMultiplier((alu == '0) && MulInAlu0), // only the first ALU has the multiplier
      .issue_req_t  (alu_issue_req_t),
      .instr_tag_t  (alu_instr_tag_t)
    ) i_alu (
      .clk_i,
      .rst_i,
      // pragma translate_off
      .trace_o          (alu_trace_int),
      // pragma translate_on
      .issue_req_i      (alu_issue_req),
      .issue_req_valid_i(alu_issue_req_valid),
      .issue_req_ready_o(alu_issue_req_ready),
      .result_o         (alu_result.result),
      .compare_res_o    (alu_result.compare_res),
      .tag_o            (alu_result_tag),
      .result_valid_o   (alu_result_valid_raw),
      .result_ready_i   (alu_result_ready),
      .busy_o           (alu_busy)
    );

    // Populate the producer field of the trace
    // pragma translate_off
    string producer;
    always_comb producer = $sformatf("ALU%0d", alu);
    always_comb begin
      alu_trace_o[alu] = alu_trace_int;
      alu_trace_o[alu].producer = producer;
    end
    assign alu_retire_trace_o[alu] = '{
      valid:    alu_result_valid && alu_result_ready,
      producer: producer
    };
    // pragma translate_on

    // Guard the result with the commit signal
    // If we want to dispatch an ALU instruction, in particular a branching instruction, we may only
    // commit the instruction if there is no instruction address misaligned exception.
    // Therefore, we must "kill" the writeback if we don't commit. The kill must be after the result
    // as otherwise we create a loop. For the other FUs the "kill" is before we pass the instruction
    // downstream.
    // TODO(colluca): this can no longer be applied after addition of the multiplier which, taking
    // multiple cycles, does not receive alu_exec_commit in the cycle of its writeback. As a result,
    // the writeback never occurs.
    // In any case, this does not sound needed to me. Branch instructions never writeback to the RF.
    // The only side effect they have is on the PC, but this does not seem to depend on
    // alu_result_valid at all.
    // assign alu_result_valid = alu_result_valid_raw & alu_exec_commit;
    assign alu_result_valid = alu_result_valid_raw;
  end

  // ALU branch result forwarding
  // We bypass the arbiter for branch results as only ALU 0 has branch logic.
  // We need the branch result at least in LCP1 to support jump & branch instructions where we
  // fallback into the HW loop mode. If we decide to crash if an unsupported instruction is
  // encountered, it is possible to optimize the timing by only forwarding the ALU result in
  // regular mode. This will gain around 10ps for a 1ns target clock cycle.
  assign branch_result_o = alu_wbs_result_and_tag[0].result;

  // ALU writeback arbiter
  // The stream_arbiter has a feed through for 1 input so no special handling for disabled FREP
  // is required.
  alu_result_and_tag_t alu_wb_result_and_tag_out;
  stream_arbiter #(
    .DATA_T (alu_result_and_tag_t),
    .N_INP  (NofAlus),
    .ARBITER("prio")
  ) i_alu_wb_arbiter (
    .clk_i,
    .rst_ni     (~rst_i),
    .inp_data_i (alu_wbs_result_and_tag),
    .inp_valid_i(alu_wbs_result_valid),
    .inp_ready_o(alu_wbs_result_ready),
    .oup_data_o (alu_wb_result_and_tag_out),
    .oup_valid_o(alu_wb_result_valid_o),
    .oup_ready_i(alu_wb_result_ready_i)
  );

  assign alu_wb_result_o     = alu_wb_result_and_tag_out.result;
  assign alu_wb_result_tag_o = alu_wb_result_and_tag_out.tag;

  //////////
  // LSUs //
  //////////

  // The LSU always returns a data_t value. Define a type for clarity.
  typedef data_t lsu_result_t;
  typedef struct packed {
    lsu_result_t result;
    instr_tag_t  tag;
  } lsu_result_and_tag_t;

  logic                [NofLsus-1:0] lsu_empty;
  logic                [NofLsus-1:0] lsu_addr_misaligned;
  lsu_result_and_tag_t [NofLsus-1:0] lsu_wbs_result_and_tag;
  logic                [NofLsus-1:0] lsu_wbs_result_valid;
  logic                [NofLsus-1:0] lsu_wbs_result_ready;

  logic [NofLsus-1:0] lsu_loop_finish;

  for (genvar lsu = 0; lsu < NofLsus; lsu++) begin : gen_lsus
    // Helper signals to merge the result and tag
    lsu_result_t lsu_wb_result;
    instr_tag_t  lsu_wb_result_tag;

    // Signals connecting the FU block and the actual FU
    issue_req_t  lsu_issue_req;
    logic        lsu_issue_req_valid;
    logic        lsu_issue_req_ready;
    logic        lsu_exec_commit;
    logic        lsu_addr_misaligned_raw;
    lsu_result_t lsu_result;
    instr_tag_t  lsu_result_tag;
    logic        lsu_result_valid;
    logic        lsu_result_ready;
    logic        lsu_busy;

    producer_id_t producer_start_id;
    assign producer_start_id = producer_id_t'{
      slot_id: '0, // does not matter
      rs_id:   rs_id_t'(LsuRsIdOffset + lsu)
    };

    // pragma translate_off
    issue_lsu_trace_t lsu_trace_int;
    // pragma translate_on

    schnizo_fu_block #(
      .Xfrep         (Xfrep),
      .disp_req_t    (disp_req_t),
      .disp_rsp_t    (disp_rsp_t),
      .issue_req_t   (issue_req_t),
      .result_t      (lsu_result_t),
      .instr_tag_t   (instr_tag_t),
      .NofRss        (LsuNofRss),
      .NofOperands   (LsuNofOperands),
      .NofOpPorts    (LsuNofOpPorts),
      .NofResRspIfs  (LsuNofRss),
      .ConsumerCount (ConsumerCount),
      .RegAddrWidth  (RegAddrWidth),
      .MaxIterationsW(MaxIterationsW),
      .producer_id_t (producer_id_t),
      .slot_id_t     (slot_id_t),
      .operand_req_t (operand_req_t),
      .operand_t     (operand_t),
      .res_req_t     (res_req_t),
      .dest_mask_t   (dest_mask_t),
      .res_rsp_t     (res_rsp_t)
    ) i_fu_block (
      .clk_i,
      .rst_i,
      /// RS control signals
      .producer_id_i      (producer_start_id),
      .restart_i          (restart_i),
      .loop_state_i       (loop_state_i),
      .in_lxp_i           (in_lxp),
      .lep_iterations_i   (lep_iterations_i),
      .goto_lcp2_i        (goto_lcp2_i),
      .fu_busy_i          (lsu_busy),
      .loop_finish_o      (lsu_loop_finish[lsu]),
      .rs_full_o          (lsu_rs_full_o[lsu]),
      /// Instruction stream
      // From dispatcher
      .disp_req_i         (disp_req_i),
      .disp_req_valid_i   (lsu_disp_reqs_valid_i[lsu]),
      .disp_req_ready_o   (lsu_disp_reqs_ready_o[lsu]),
      .instr_exec_commit_i(instr_exec_commit_i),
      .disp_rsp_o         (lsu_disp_rsp_o[lsu]),
      // To FU
      .issue_req_o        (lsu_issue_req),
      .issue_req_valid_o  (lsu_issue_req_valid),
      .issue_req_ready_i  (lsu_issue_req_ready),
      .instr_exec_commit_o(lsu_exec_commit),
      // From FU
      .result_i           (lsu_result),
      .result_tag_i       (lsu_result_tag),
      .result_valid_i     (lsu_result_valid),
      .result_ready_o     (lsu_result_ready),
      // To writeback
      .wb_result_o        (lsu_wb_result),
      .wb_result_tag_o    (lsu_wb_result_tag),
      .wb_result_valid_o  (lsu_wbs_result_valid[lsu]),
      .wb_result_ready_i  (lsu_wbs_result_ready[lsu]),
      /// Operand distribution network
      .available_results_o(lsu_available_results[lsu]),
      .op_reqs_o          (lsu_op_reqs[lsu]),
      .op_reqs_valid_o    (lsu_op_reqs_valid[lsu]),
      .op_reqs_ready_i    (lsu_op_reqs_ready[lsu]),
      .res_reqs_i         (lsu_res_reqs[lsu]),
      .res_reqs_valid_i   (lsu_res_reqs_valid[lsu]),
      .res_reqs_ready_o   (lsu_res_reqs_ready[lsu]),
      .res_rsps_o         (lsu_res_rsps[lsu]),
      .res_rsps_valid_o   (lsu_res_rsps_valid[lsu]),
      .res_rsps_ready_i   (lsu_res_rsps_ready[lsu]),
      .op_rsps_i          (lsu_op_rsps[lsu]),
      .op_rsps_valid_i    (lsu_op_rsps_valid[lsu]),
      .op_rsps_ready_o    (lsu_op_rsps_ready[lsu])
    );
    assign lsu_wbs_result_and_tag[lsu].result = lsu_wb_result;
    assign lsu_wbs_result_and_tag[lsu].tag    = lsu_wb_result_tag;

    schnizo_lsu #(
      .XLEN               (XLEN),
      .issue_req_t        (issue_req_t),
      .AddrWidth          (AddrWidth),
      .DataWidth          (DataWidth),
      .dreq_t             (dreq_t),
      .drsp_t             (drsp_t),
      .tag_t              (instr_tag_t),
      .NumOutstandingMem  (NumOutstandingMem),
      .NumOutstandingLoads(NumOutstandingLoads),
      .Caq                (0), // TODO: Enable
      .CaqDepth           (CaqDepth),
      .CaqTagWidth        (CaqTagWidth),
      .CaqRespSrc         (0),
      .CaqRespTrackSeq    (0)
    ) i_lsu (
      .clk_i,
      .rst_i,
      // pragma translate_off
      .trace_o          (lsu_trace_int),
      // pragma translate_on
      .issue_req_i      (lsu_issue_req),
      .issue_req_valid_i(lsu_issue_req_valid),
      .issue_commit_i   (lsu_exec_commit),
      .issue_req_ready_o(lsu_issue_req_ready),
      .result_o         (lsu_result),
      .tag_o            (lsu_result_tag),
      .result_error_o   (), // ignored
      .result_valid_o   (lsu_result_valid),
      .result_ready_i   (lsu_result_ready),
      .busy_o           (lsu_busy),
      .empty_o          (lsu_empty[lsu]),
      .addr_misaligned_o(lsu_addr_misaligned_raw),
      .data_req_o       (lsu_dreq_o[lsu]),
      .data_rsp_i       (lsu_drsp_i[lsu]),
      .caq_addr_i       (caq_addr_i[lsu]),
      .caq_track_write_i(caq_track_write_i[lsu]),
      .caq_req_valid_i  (caq_req_valid_i[lsu]),
      .caq_req_ready_o  (caq_req_ready_o[lsu]),
      .caq_rsp_valid_i  (caq_rsp_valid_i[lsu]),
      .caq_rsp_valid_o  (caq_rsp_valid_o[lsu])
    );

    // Suppress exceptions in LCP and LEP because we anyway don't handle them one cycle later.
    assign lsu_addr_misaligned[lsu] = lsu_addr_misaligned_raw & !in_lxp;

    // Populate the producer and instr_iter fields of the trace
    // pragma translate_off
    string producer;
    always_comb producer = $sformatf("LSU%0d", lsu);
    always_comb begin
      lsu_trace_o[lsu] = lsu_trace_int;
      lsu_trace_o[lsu].producer = producer;
    end
    assign lsu_retire_trace_o[lsu] = '{
      valid:    lsu_result_valid && lsu_result_ready,
      producer: producer
    };
    // pragma translate_on
  end

  // LSU empty & misalign signal combination
  assign lsu_empty_o = (&lsu_empty);
  assign lsu_addr_misaligned_o =(|lsu_addr_misaligned);

  // LSU writeback arbiter
  // The stream_arbiter has a feed through for 1 input so no special handling for disabled FREP
  // is required.
  lsu_result_and_tag_t lsu_wb_result_and_tag_out;
  stream_arbiter #(
    .DATA_T (lsu_result_and_tag_t),
    .N_INP  (NofLsus),
    .ARBITER("prio")
  ) i_lsu_wb_arbiter (
    .clk_i,
    .rst_ni     (~rst_i),
    .inp_data_i (lsu_wbs_result_and_tag),
    .inp_valid_i(lsu_wbs_result_valid),
    .inp_ready_o(lsu_wbs_result_ready),
    .oup_data_o (lsu_wb_result_and_tag_out),
    .oup_valid_o(lsu_wb_result_valid_o),
    .oup_ready_i(lsu_wb_result_ready_i)
  );

  assign lsu_wb_result_o     = lsu_wb_result_and_tag_out.result;
  assign lsu_wb_result_tag_o = lsu_wb_result_and_tag_out.tag;

  //////////
  // FPUs //
  //////////

  typedef logic [FLEN-1:0] fpu_result_t;

  typedef logic [cf_math_pkg::idx_width(FpuNofRss)-1:0] fpu_rs_tag_t;

  typedef logic [cf_math_pkg::max($bits(fpu_rs_tag_t),$bits(instr_tag_t))-1:0] fpu_instr_tag_t;

  typedef struct packed {
    fu_data_t       fu_data;
    fpu_instr_tag_t tag;
  } fpu_issue_req_t;

  typedef struct packed {
    fpu_result_t result;
    instr_tag_t  tag;
  } fpu_result_and_tag_t;

  // Keep the handshake signals to combine the fpu status
  logic                [NofFpus-1:0] fpu_result_valid;
  logic                [NofFpus-1:0] fpu_result_ready;
  fpnew_pkg::status_t  [NofFpus-1:0] fpu_status;
  fpu_result_and_tag_t [NofFpus-1:0] fpu_wbs_result_and_tag;
  logic                [NofFpus-1:0] fpu_wbs_result_valid;
  logic                [NofFpus-1:0] fpu_wbs_result_ready;

  logic [NofFpus-1:0] fpu_loop_finish;

  for (genvar fpu = 0; fpu < NofFpus; fpu++) begin : gen_fpus
    // Helper signals to merge the result and tag
    fpu_result_t    fpu_wb_result;
    fpu_instr_tag_t fpu_wb_result_tag;

    // Signals connecting the FU block and the actual FU
    fpu_issue_req_t fpu_issue_req;
    logic           fpu_issue_req_valid;
    logic           fpu_issue_req_ready;
    logic           fpu_exec_commit;
    fpu_result_t    fpu_result;
    fpu_instr_tag_t fpu_result_tag;
    logic           fpu_busy;

    producer_id_t producer_start_id;
    assign producer_start_id = producer_id_t'{
      slot_id: '0, // does not matter
      rs_id:   rs_id_t'(FpuRsIdOffset + fpu)
    };

    // pragma translate_off
    issue_fpu_trace_t fpu_trace_int;
    // pragma translate_on

    schnizo_fu_block #(
      .Xfrep         (Xfrep),
      .disp_req_t    (disp_req_t),
      .disp_rsp_t    (disp_rsp_t),
      .issue_req_t   (fpu_issue_req_t),
      .result_t      (fpu_result_t),
      .instr_tag_t   (fpu_instr_tag_t),
      .NofRss        (FpuNofRss),
      .NofOperands   (FpuNofOperands),
      .NofOpPorts    (FpuNofOpPorts),
      .NofResRspIfs  (FpuNofRss),
      .ConsumerCount (ConsumerCount),
      .RegAddrWidth  (RegAddrWidth),
      .MaxIterationsW(MaxIterationsW),
      .producer_id_t (producer_id_t),
      .slot_id_t     (slot_id_t),
      .operand_req_t (operand_req_t),
      .operand_t     (operand_t),
      .res_req_t     (res_req_t),
      .dest_mask_t   (dest_mask_t),
      .res_rsp_t     (res_rsp_t)
    ) i_fu_block (
      .clk_i,
      .rst_i,
      /// RS control signals
      .producer_id_i      (producer_start_id),
      .restart_i          (restart_i),
      .loop_state_i       (loop_state_i),
      .in_lxp_i           (in_lxp),
      .lep_iterations_i   (lep_iterations_i),
      .goto_lcp2_i        (goto_lcp2_i),
      .fu_busy_i          (fpu_busy),
      .loop_finish_o      (fpu_loop_finish[fpu]),
      .rs_full_o          (fpu_rs_full_o[fpu]),
      /// Instruction stream
      // From dispatcher
      .disp_req_i         (disp_req_i),
      .disp_req_valid_i   (fpu_disp_reqs_valid_i[fpu]),
      .disp_req_ready_o   (fpu_disp_reqs_ready_o[fpu]),
      .instr_exec_commit_i(instr_exec_commit_i),
      .disp_rsp_o         (fpu_disp_rsp_o[fpu]),
      // To FU
      .issue_req_o        (fpu_issue_req),
      .issue_req_valid_o  (fpu_issue_req_valid),
      .issue_req_ready_i  (fpu_issue_req_ready),
      .instr_exec_commit_o(fpu_exec_commit),
      // From FU
      .result_i           (fpu_result),
      .result_tag_i       (fpu_result_tag),
      .result_valid_i     (fpu_result_valid[fpu]),
      .result_ready_o     (fpu_result_ready[fpu]),
      // To writeback
      .wb_result_o        (fpu_wb_result),
      .wb_result_tag_o    (fpu_wb_result_tag),
      .wb_result_valid_o  (fpu_wbs_result_valid[fpu]),
      .wb_result_ready_i  (fpu_wbs_result_ready[fpu]),
      /// Operand distribution network
      .available_results_o(fpu_available_results[fpu]),
      .op_reqs_o          (fpu_op_reqs[fpu]),
      .op_reqs_valid_o    (fpu_op_reqs_valid[fpu]),
      .op_reqs_ready_i    (fpu_op_reqs_ready[fpu]),
      .res_reqs_i         (fpu_res_reqs[fpu]),
      .res_reqs_valid_i   (fpu_res_reqs_valid[fpu]),
      .res_reqs_ready_o   (fpu_res_reqs_ready[fpu]),
      .res_rsps_o         (fpu_res_rsps[fpu]),
      .res_rsps_valid_o   (fpu_res_rsps_valid[fpu]),
      .res_rsps_ready_i   (fpu_res_rsps_ready[fpu]),
      .op_rsps_i          (fpu_op_rsps[fpu]),
      .op_rsps_valid_i    (fpu_op_rsps_valid[fpu]),
      .op_rsps_ready_o    (fpu_op_rsps_ready[fpu])
    );
    assign fpu_wbs_result_and_tag[fpu].result = fpu_wb_result;
    assign fpu_wbs_result_and_tag[fpu].tag    = fpu_wb_result_tag;

    schnizo_fpu #(
      .FPUImplementation(FPUImplementation),
      .RVF              (RVF),
      .RVD              (RVD),
      .XF16             (XF16),
      .XF16ALT          (XF16ALT),
      .XF8              (XF8),
      .XF8ALT           (XF8ALT),
      .XFVEC            (XFVEC),
      .FLEN             (FLEN),
      .RegisterFPUIn    (RegisterFPUIn),
      .RegisterFPUOut   (RegisterFPUOut),
      .issue_req_t      (fpu_issue_req_t),
      .instr_tag_t      (fpu_instr_tag_t)
    ) i_fpu (
      .clk_i,
      .rst_ni           (~rst_i),
      // pragma translate_off
      .trace_o          (fpu_trace_int),
      // pragma translate_on
      .hart_id_i        (hard_id_i),
      .issue_req_i      (fpu_issue_req),
      .issue_req_valid_i(fpu_issue_req_valid),
      .issue_commit_i   (fpu_exec_commit),
      .issue_req_ready_o(fpu_issue_req_ready),
      .result_o         (fpu_result),
      .tag_o            (fpu_result_tag),
      .result_valid_o   (fpu_result_valid[fpu]),
      .result_ready_i   (fpu_result_ready[fpu]),
      .status_o         (fpu_status[fpu]),
      .busy_o           (fpu_busy)
    );

    // Populate the producer and instr_iter fields of the trace
    // pragma translate_off
    string producer;
    always_comb producer = $sformatf("FPU%0d", fpu);
    always_comb begin
      fpu_trace_o[fpu] = fpu_trace_int;
      fpu_trace_o[fpu].producer = producer;
    end
    assign fpu_retire_trace_o[fpu] = '{
      valid:    fpu_result_valid[fpu] && fpu_result_ready[fpu],
      producer: producer
    };
    // pragma translate_on
  end

  // FU status combintation
  // During LEP we still want to capture all FCSR status updates. So we must combine all valid
  // status values and generate a valid bit. The valid bit ensures that we update the FCSR only
  // on a handshake.
  logic               fpu_status_valid;
  fpnew_pkg::status_t combined_fpu_status;

  always_comb begin
    fpu_status_valid    = 1'b0;
    combined_fpu_status = '0;
    for (int fpu = 0; fpu < NofFpus; fpu++) begin
      if (fpu_result_valid[fpu] && fpu_result_ready[fpu]) begin
        fpu_status_valid = 1'b1;
        combined_fpu_status = combined_fpu_status | fpu_status[fpu];
      end
    end
  end

  assign fpu_status_o       = combined_fpu_status;
  assign fpu_status_valid_o = fpu_status_valid;

  // FPU writeback arbiter
  // The stream_arbiter has a feed through for 1 input so no special handling for disabled FREP
  // is required.
  fpu_result_and_tag_t fpu_wb_result_and_tag_out;
  stream_arbiter #(
    .DATA_T (fpu_result_and_tag_t),
    .N_INP  (NofFpus),
    .ARBITER("prio")
  ) i_fpu_wb_arbiter (
    .clk_i,
    .rst_ni     (~rst_i),
    .inp_data_i (fpu_wbs_result_and_tag),
    .inp_valid_i(fpu_wbs_result_valid),
    .inp_ready_o(fpu_wbs_result_ready),
    .oup_data_o (fpu_wb_result_and_tag_out),
    .oup_valid_o(fpu_wb_result_valid_o),
    .oup_ready_i(fpu_wb_result_ready_i)
  );

  assign fpu_wb_result_o     = fpu_wb_result_and_tag_out.result;
  assign fpu_wb_result_tag_o = fpu_wb_result_and_tag_out.tag;

  ///////////
  // Spatz //
  ///////////

  if (RVV) begin: gen_rvv_block
  // Signals connecting the FU block and the actual FU
  issue_req_t   spatz_issue_req;
  logic         spatz_issue_req_valid;
  logic         spatz_issue_req_ready;
  logic         spatz_exec_commit;
  spatz_result_t spatz_result;
  instr_tag_t   spatz_result_tag;
  logic         spatz_result_valid;
  logic         spatz_result_ready;
  logic         spatz_busy;

  // Producer ID for the SPATZ RS
  producer_id_t spatz_producer_start_id;
  assign spatz_producer_start_id = producer_id_t'{
    slot_id: '0, // single-slot //TODO: Is not single slot, fix this
    rs_id:   rs_id_t'(SpatzRsIdOffset + 0)
  };

  // Reservation station and ODN wrapper
  schnizo_fu_block #(
    .Xfrep         (Xfrep),
    .disp_req_t    (disp_req_t),
    .disp_rsp_t    (disp_rsp_t),
    .issue_req_t   (issue_req_t),
    .result_t      (spatz_result_t),
    .instr_tag_t   (instr_tag_t),
    .NofRss        (SpatzNofRss),
    .NofOperands   (SpatzNofOperands),
    .NofOpPorts    (SpatzNofOpPorts),
    .NofResRspIfs  (SpatzNofRss),
    .ConsumerCount (ConsumerCount),
    .RegAddrWidth  (RegAddrWidth),
    .MaxIterationsW(MaxIterationsW),
    .producer_id_t (producer_id_t),
    .slot_id_t     (slot_id_t),
    .operand_req_t (operand_req_t),
    .operand_t     (operand_t),
    .res_req_t     (res_req_t),
    .dest_mask_t   (dest_mask_t),
    .res_rsp_t     (res_rsp_t)
  ) i_spatz_block (
    .clk_i,
    .rst_i,
    /// RS control signals
    .producer_id_i      (spatz_producer_start_id),
    .restart_i          (restart_i),
    .loop_state_i       (loop_state_i),
    .in_lxp_i           (in_lxp),
    .lep_iterations_i   (lep_iterations_i),
    .goto_lcp2_i        (goto_lcp2_i),
    .fu_busy_i          (spatz_busy),
    .loop_finish_o      (spatz_loop_finish_o),
    .rs_full_o          (spatz_rs_full_o),
    /// Instruction stream
    // From dispatcher
    .disp_req_i         (disp_req_i),
    .disp_req_valid_i   (spatz_disp_reqs_valid_i),
    .disp_req_ready_o   (spatz_disp_reqs_ready_o),
    .instr_exec_commit_i(instr_exec_commit_i),
    .disp_rsp_o         (spatz_disp_rsp_o),
    // To FU
    .issue_req_o        (spatz_issue_req),
    .issue_req_valid_o  (spatz_issue_req_valid),
    .issue_req_ready_i  (spatz_issue_req_ready),
    .instr_exec_commit_o(spatz_exec_commit),
    // From FU
    .result_i           (spatz_result),
    .result_tag_i       (spatz_result_tag),
    .result_valid_i     (spatz_result_valid),
    .result_ready_o     (spatz_result_ready),
    // To writeback
    .wb_result_o        (spatz_wb_result_o),
    .wb_result_tag_o    (spatz_wb_result_tag_o),
    .wb_result_valid_o  (spatz_wb_result_valid_o),
    .wb_result_ready_i  (spatz_wb_result_ready_i),
    /// Operand distribution network
    .available_results_o(spatz_available_results),
    .op_reqs_o          (spatz_op_reqs),
    .op_reqs_valid_o    (spatz_op_reqs_valid),
    .op_reqs_ready_i    (spatz_op_reqs_ready),
    .res_reqs_i         (spatz_res_reqs),
    .res_reqs_valid_i   (spatz_res_reqs_valid),
    .res_reqs_ready_o   (spatz_res_reqs_ready),
    .res_rsps_o         (spatz_res_rsps),
    .res_rsps_valid_o   (spatz_res_rsps_valid),
    .res_rsps_ready_i   (spatz_res_rsps_ready),
    .op_rsps_i          (spatz_op_rsps),
    .op_rsps_valid_i    (spatz_op_rsps_valid),
    .op_rsps_ready_o    (spatz_op_rsps_ready)
  );


  //TODO: Build the accelerator request based on the decoded instruction

  typedef struct packed {
    logic [5:0] id;
    logic [31:0] data_op;
    data_t data_arga;
    data_t data_argb;
    data_t data_argc;
  } spatz_issue_req_t;

  typedef struct packed {
    logic accept;
    logic writeback;
    logic loadstore;
    logic exception;
    logic isfloat;
  } spatz_issue_rsp_t;

  typedef struct packed {
    logic [5:0] id;
    logic error;
    data_t data;
    logic is_fp;
  } spatz_rsp_t;


  spatz_issue_req_t acc_spatz_req;
  spatz_issue_rsp_t acc_spatz_resp;
  spatz_rsp_t       acc_resp;


  assign acc_spatz_req = '{
    id:        spatz_issue_req.tag.dest_reg,
    data_op:   spatz_issue_req.fu_data.raw_instr,
    data_arga: spatz_issue_req.fu_data.operand_a,
    data_argb: spatz_issue_req.fu_data.operand_b,
    data_argc: '0 // ignored, Spatz can receive at most two scalar values
  };

  assign spatz_result = acc_resp.data;
  assign spatz_result_tag = '{
    dest_reg: acc_resp.id,
    dest_reg_is_fp: acc_resp.is_fp,
    is_branch: '0,
    is_jump: '0
  };



  assign spatz_busy = !spatz_issue_req_ready; // if we cannot dispatch, we are busy

  tcdm_req_chan_t [NumMemPortsPerSpatz-1:0] spatz_mem_req;
  logic           [NumMemPortsPerSpatz-1:0] spatz_mem_req_valid;
  logic           [NumMemPortsPerSpatz-1:0] spatz_mem_req_ready;
  tcdm_rsp_chan_t [NumMemPortsPerSpatz-1:0] spatz_mem_rsp;
  logic           [NumMemPortsPerSpatz-1:0] spatz_mem_rsp_valid;

  fpnew_pkg::status_t spatz_fpu_status;

  fpnew_pkg::fmt_mode_t spatz_fmt_mode;
  assign spatz_fmt_mode = '{
    src: spatz_issue_req.fu_data.fpu_fmt_src,
    dst: spatz_issue_req.fu_data.fpu_fmt_dst
  };

  spatz #(
    .NrMemPorts         (4     ),
    .NumOutstandingLoads(8),
    .FPUImplementation  (FPUImplementation),
    .RegisterRsp        ('0     ),
    .dreq_t             (dreq_t                  ),
    .drsp_t             (drsp_t                  ),
    .spatz_mem_req_t    (tcdm_req_chan_t       ),
    .spatz_mem_rsp_t    (tcdm_rsp_chan_t       ),
    .spatz_issue_req_t  (spatz_issue_req_t         ),
    .spatz_issue_rsp_t  (spatz_issue_rsp_t         ),
    .spatz_rsp_t        (spatz_rsp_t               )
  ) i_spatz (
    .clk_i                   (clk_i                 ),
    .rst_ni                  (~rst_i                ),
    .testmode_i              ('0                    ),
    .hart_id_i               (hard_id_i             ),
    .issue_valid_i           (spatz_issue_req_valid  ),
    .issue_ready_o           (spatz_issue_req_ready  ),
    .issue_req_i             (acc_spatz_req        ), //Special care
    .issue_rsp_o             (acc_spatz_resp       ), //Special care
    .rsp_valid_o             (spatz_result_valid   ),
    .rsp_ready_i             (spatz_result_ready   ),
    .rsp_o                   (acc_resp              ), //Special care
    .running_instrs_o        (spatz_running_instrs_o),
    .spatz_mem_req_o         (spatz_mem_req         ),
    .spatz_mem_req_valid_o   (spatz_mem_req_valid   ),
    .spatz_mem_req_ready_i   (spatz_mem_req_ready   ),
    .spatz_mem_rsp_i         (spatz_mem_rsp         ),
    .spatz_mem_rsp_valid_i   (spatz_mem_rsp_valid   ),
    .spatz_mem_finished_o    (    ),
    .spatz_mem_str_finished_o(),
    .fp_lsu_mem_req_o        (        ),
    .fp_lsu_mem_rsp_i        ('0        ),
    .fpu_rnd_mode_i          (spatz_issue_req.fu_data.fpu_rnd_mode),
    .fpu_fmt_mode_i          (spatz_fmt_mode),
    .fpu_status_o            (spatz_fpu_status)
  );

  // The first indices are assigned to the LSUs, the next to the SPATZ.
   for (genvar p = 0; p < NumMemPortsPerSpatz; p++) begin: gen_tcdm_assignment
    assign spatz_tcdm_req_o[p] = '{
         q      : spatz_mem_req[p],
         q_valid: spatz_mem_req_valid[p]
       };
    assign spatz_mem_req_ready[p] = spatz_tcdm_rsp_i[p].q_ready;

    assign spatz_mem_rsp[p]       = spatz_tcdm_rsp_i[p].p;
    assign spatz_mem_rsp_valid[p] = spatz_tcdm_rsp_i[p].p_valid;
  end

  end else begin

    assign spatz_loop_finish_o = '1;

  end

  ////////////
  // Status //
  ////////////

  // The complete core finishes if all RS finish.
  assign all_rs_finish_o = &{&alu_loop_finish, &lsu_loop_finish, &fpu_loop_finish, &spatz_loop_finish_o};

  ////////////////////
  // Tracer helpers //
  ////////////////////

  // pragma translate_off

  function automatic string rs_to_string(rs_id_t rs_id);
    string fu_name;
    int fu_id;

    if (rs_id < NofAlus) begin
      fu_name = "ALU";
      fu_id = rs_id - 0;
    end else if (rs_id < NofAlus + NofLsus) begin
      fu_name = "LSU";
      fu_id = rs_id - NofAlus;
    end else if (rs_id < NofAlus + NofLsus + NofFpus) begin
      fu_name = "FPU";
      fu_id = rs_id - NofAlus - NofLsus;
    end else begin
      fu_name = "SPATZ";
      fu_id = rs_id - NofAlus - NofLsus - NofFpus;
    end

    return $sformatf("%s%0d", fu_name, fu_id);
  endfunction

  // This function converts a producer id to a string depending on the number of FUs and RSSs.
  // This string converts e.g. producer id 2 to "0.1" (ALU 0, RSS 1)
  // This function must be inside this module to have access to the producer_id_t type and
  // the id computations.
  function automatic string producer_to_string(producer_id_t producer_id);
    string fu_name;

    fu_name = rs_to_string(producer_id.rs_id);

    return $sformatf("%s.%0d", fu_name, producer_id.slot_id);
  endfunction

  // This function converts a consumer id to a string depending on the number of FUs.
  // As the crossbar is independent of the number of slots there is no information about
  // which slot is the actual consumer. But we add the information about the request port.
  // Returns "FU.x.y" where FU is the actual FU type, x is the port id and y the operand id.
  function automatic string consumer_to_string(operand_id_t consumer_id);
    int unsigned consumer = unsigned'(consumer_id);
    int unsigned num_ops;
    int unsigned num_ports;
    int unsigned rs_id;
    int unsigned port_id;
    int unsigned op_id;
    string fu_name;

    if (consumer < LsuOpIdOffset) begin
      num_ports = AluNofOpPorts;
      num_ops = AluNofOperands;
      consumer = consumer - AluOpIdOffset;
      fu_name = "ALU";
    end else if (consumer < FpuOpIdOffset) begin
      num_ports = LsuNofOpPorts;
      num_ops = LsuNofOperands;
      consumer = consumer - LsuOpIdOffset;
      fu_name = "LSU";
    end else if (consumer < SpatzOpIdOffset) begin
      num_ports = FpuNofOpPorts;
      num_ops = FpuNofOperands;
      consumer = consumer - FpuOpIdOffset;
      fu_name = "FPU";
    end else begin
      num_ports = SpatzNofOpPorts;
      num_ops = SpatzNofOperands;
      consumer = consumer - SpatzOpIdOffset;
      fu_name = "SPATZ";
    end

    rs_id = consumer / (num_ports * num_ops);
    // Reduce into operand range of current RS.
    // --> range of 0..((num_ports * num_ops) - 1)
    consumer = consumer - (rs_id * (num_ports * num_ops));
    port_id = consumer / num_ops;
    op_id = consumer % num_ops;

    return $sformatf("%s%0d.%0d.%0d", fu_name, rs_id, port_id, op_id);
  endfunction

  // pragma translate_on

endmodule
