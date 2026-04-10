// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

`include "common_cells/registers.svh"

// The Reservation station slot. It contains the actual instruction buffer logic.
//
// Abbreviations:
// FU:  Functional Unit. This is for example an ALU, FPU or LSU.
// RS:  Reservation Station. Holds multiple RSS for a single FU and controls the execution.
// RSS: Reservation Station Slot. A slot can hold one instruction with all the required
//      information for the superscalar execution.
module schnizo_res_stat_slot import schnizo_pkg::*; #(
  parameter int unsigned NofOperands    = 2,
  parameter int unsigned ConsumerCount  = 16,
  // The bits to address all registers
  parameter int unsigned RegAddrWidth   = 5,
  parameter type         rss_idx_t      = logic,
  parameter type         disp_req_t     = logic,
  parameter type         producer_id_t  = logic,
  parameter type         operand_req_t  = logic,
  parameter type         operand_t      = logic,
  parameter type         res_req_t      = logic,
  parameter type         dest_mask_t    = logic,
  parameter type         res_rsp_t      = logic,
  parameter type         issue_req_t    = logic,
  parameter type         result_t       = logic,
  parameter type         result_tag_t   = logic
) (
  input  logic clk_i,
  input  logic rst_i,

  // Index of this slot in the reservation station.
  input  rss_idx_t     slot_id_i,
  // If restart is asserted, we initialize the slot. THERE MAY NOT BE ANY instruction in flight!
  input  logic         restart_i,
  // Asserted for last LEP dispatch iteration to end the operand fetching.
  input  logic         is_last_disp_iter_i,
  // Asserted for last LEP result iteration to perform the possible writeback (based on result iteration).
  input  logic         is_last_result_iter_i,
  // ID of RSS to place operand requests. This ID must be static.
  input  producer_id_t own_producer_id_i,
  // The current result iteration state
  output logic         res_iter_o,
  // Asserted in the cycle the instruction retires.
  output logic         retired_o,

  // Dispatch interface
  input  disp_req_t disp_req_i,
  input  logic      disp_req_valid_i,
  output logic      disp_req_ready_o,

  // Operand request interface - outgoing - request a result as operand
  output operand_req_t [NofOperands-1:0] op_reqs_o,
  output logic         [NofOperands-1:0] op_reqs_valid_o,
  input  logic         [NofOperands-1:0] op_reqs_ready_i,

  // Result request interface - incoming - translated operand request
  // Result requests are converted to destination masks (where to send the result to) at RS level.
  input  dest_mask_t dest_mask_i,
  input  logic       dest_mask_valid_i,
  output logic       dest_mask_ready_o,

  // Result response interface - outgoing - result as operand response
  output res_rsp_t res_rsp_o,
  output logic     res_rsp_valid_o,
  input  logic     res_rsp_ready_i,

  // Operand response interface - incoming - returning result as operand
  input  operand_t [NofOperands-1:0] op_rsps_i,
  input  logic     [NofOperands-1:0] op_rsps_valid_i,
  output logic     [NofOperands-1:0] op_rsps_ready_o,

  // Issue interface
  output issue_req_t issue_req_o,
  output logic       issue_req_valid_o,
  input  logic       issue_req_ready_i,

  // FU result interface
  input  result_t result_i,
  input  logic    result_valid_i,
  output logic    result_ready_o,

  // RF writeback interface
  output result_tag_t rf_wb_tag_o,
  output logic        rf_do_writeback_o
);

  /////////////////////////////////////
  // Parameters and type definitions //
  /////////////////////////////////////

  localparam integer unsigned ConsumerCountWidth = cf_math_pkg::idx_width(ConsumerCount);

  typedef struct packed {
    // The ID of the producer. Only valid if the isProduced flag is set. Otherwise this operand is
    // constant and fetched during LCP1 and LCP2.
    producer_id_t producer;
    // Signaling whether this operand is produced or not. If set, the value has to be fetched from
    // the producer defined by producer_id. If reset, this operand is constant and is fetched in
    // LCP1 and again in LCP2 and kept for the rest of the loop execution. A constant value can
    // either be a value read once from a register or an immediate of the instruction.
    logic         is_produced;
    // Specifying in which iteration the producer generated the value. If set, the producer is in
    // the same iteration. If reset, this is a loop-carried dependency.
    logic         is_from_current_iter;
    operand_t     value;
    logic         is_valid;
    // Set if we placed a request to the producer
    logic         requested;
  } rss_operand_t;

  typedef struct packed {
    result_t value;
    // If set, the result is valid.
    logic    is_valid;
    // This flag signals to which iteration (“current” or “next”) the currently stored value in
    // the Result buffer belongs to. It is toggled each time a new value is written into the
    // buffer.
    logic    iteration;
  } rss_result_t;

  // TODO(colluca): put all FU-specific fields into a separate struct that is passed
  // as a parameter, and instantiated as a "user" field. Otherwise, only mandatory fields used
  // for control logic should be hardcoded here. `is_store` is one of these, so it should be
  // renamed to reflect its FU-independent function.
  typedef struct packed  {
    // Raw instruction for Spatz
    logic [31:0]                    spatz_raw_instr;
    // Whether the RSS contains an active instruction.
    logic                           is_occupied;
    // How many consumer use the result of this instruction.
    logic [ConsumerCountWidth-1:0]  consumer_count;
    // A counter to keep track how many times the current result has been captured.
    logic [ConsumerCountWidth-1:0]  consumed_by;
    // If the instruction has been issued in LCP1, the flag is set and all notifications targeting
    // this RSS are captured. When the instruction is issued for the 2nd time in LCP2, the flag is
    // reset.
    logic                           do_capture_consumers;
    // The instruction itself. Partially decoded. Depends on FU type.
    // TODO: Can we rely on the synthesis optimization to remove unused signals even if they are
    //       registered here?
    alu_op_e                        alu_op;
    lsu_op_e                        lsu_op;
    fpu_op_e                        fpu_op;
    lsu_size_e                      lsu_size;
    // A store instruction never generates a result. Thus we immediately accept a result when
    // issuing the instruction.
    logic                           is_store;
    fpnew_pkg::fp_format_e          fpu_fmt_src;
    fpnew_pkg::fp_format_e          fpu_fmt_dst;
    fpnew_pkg::roundmode_e          fpu_rnd_mode;
    // The most recent result
    rss_result_t                    result;
    // This flag signals to which iteration (“current” or “next”) the currently
    // “waiting instruction” (not all operands are ready) in the RSS belongs to. It is toggled
    // each time the instruction is issued.
    logic                           instruction_iter;
    // The register ID where this instruction does commit into during regular execution.
    logic [RegAddrWidth-1:0]        dest_id;
    // Whether the destination register is a floating point or integer register.
    logic                           dest_is_fp;
    // Specifying whether the last result of the loop is written into the register defined by
    // destination id. This flag is defined during LCP and ensures that at the end of the loop
    // only the last writing instruction does perform a writeback to the RF.
    logic                           do_writeback;
    // All operands
    // TODO(colluca): optimize by pulling out of RS. Only one RSS per RS will anyways fetch
    // operands at any time. One exception is for immediate values, those need to be always
    // stored, but don't need any of the fields in rss_operand_t beyond `value`.
    // The other exception is actually for operands that are not produced by other FUs, e.g.
    // operands that just keep the value from the RF at the time of dispatch.
    // If we don't buffer these, then we have to be able to fetch them from the RF. Probably
    // a good compromise would be to have a few registers (less than #operands x #slots) in the
    // RS to buffer these "non-produced" operands, and fallback to HW loop mode if we run out.
    rss_operand_t [NofOperands-1:0] operands;
  } rs_slot_t;

  typedef enum logic[2:0] {
    Lcp1Init,
    Lcp1Fetch,
    WaitForLcp1Result,
    Lcp2Init,
    Lcp2Fetch,
    WaitForLcp2Result,
    Lep,
    LepFinished
  } rss_state_e;

  /////////////////
  // Connections //
  /////////////////

  logic [NofOperands-1:0] op_valid;
  logic                   enable_op_request;
  logic                   enforce_valid_reset;
  logic                   enforce_rf_writeback;
  logic                   enable_capture_consumers;
  logic                   all_ops_valid;
  logic                   result_consumed;
  logic                   issued;
  logic                   retired;
  logic                   do_rf_writeback;
  logic                   rss_wb_valid;
  logic                   rss_wb_ready;

  //////////
  // Slot //
  //////////

  rs_slot_t slot_reset_state;
  rs_slot_t slot_q, slot_d;

  `FFAR(slot_q, slot_d, slot_reset_state, clk_i, rst_i);

  assign slot_reset_state = '{
    spatz_raw_instr:     '0,
    is_occupied:          1'b0, // suppresses operand requests
    consumer_count:       '0,
    consumed_by:          '0,
    do_capture_consumers: 1'b0,
    alu_op:               AluOpAdd,
    lsu_op:               LsuOpLoad, // avoid store because the store flag has to be 0
    fpu_op:               FpuOpFadd,
    is_store:             1'b0,
    lsu_size:             Byte,
    fpu_fmt_src:          fpnew_pkg::FP32,
    fpu_fmt_dst:          fpnew_pkg::FP32,
    fpu_rnd_mode:         fpnew_pkg::RNE,
    // We ignore the result part - the iteration flag could be X.
    result:               '0,
    instruction_iter:     1'b0,
    dest_id:              '0,
    dest_is_fp:           '0,
    do_writeback:         1'b0,
    operands:             '0 // invalid operands lead to no issue requests
  };

  //////////////////
  // State update //
  //////////////////

  // TODO(colluca): If I understand correctly, LcpxInit is basically an Idle state.
  // LcpxFetch is an optional state in which we wait for the conditions to be able to
  // issue an instruction.
  // WaitForLcpxResult is an optional state in which we wait for the result of the instruction.
  // Operand requests are enabled from when the slot is selected (disp_req_valid_i)
  // to when the instruction is issued (does it really need to be asserted for multiple
  // cycles?).
  // RF writeback is always enforced during LCP phase.

  rss_state_e state_q, state_d;
  `FFAR(state_q, state_d, Lcp1Init, clk_i, rst_i);

  always_comb begin : rss_state
    state_d = state_q;

    enable_op_request        = 1'b0;
    // TODO(colluca): couldn't this be set to 1'b1 by default and only disabled in LEP?
    enforce_rf_writeback     = 1'b0;
    // Set the capture consumer flag when we captured the result of LCP1.
    // Reset it when we captured the result of LCP2.
    // TODO(colluca): to me it would make more sense to capture looking at issue rather
    // than result capture time. Though it's probably the same, since in LCP phase dependent
    // instructions are in order.
    enable_capture_consumers = 1'b0;

    unique case (state_q)
      Lcp1Init: begin
        if (disp_req_valid_i) begin
          enable_op_request    = 1'b1;
          enforce_rf_writeback = 1'b1;
          state_d              = Lcp1Fetch;
          if (issued) begin
            state_d = WaitForLcp1Result;
          end
          if (retired) begin
            state_d = Lcp2Init;
            enable_capture_consumers = 1'b1;
          end
        end
      end
      Lcp1Fetch: begin
        enable_op_request    = 1'b1;
        enforce_rf_writeback = 1'b1;
        if (issued) begin
          state_d = WaitForLcp1Result;
        end
        if (retired) begin
          state_d = Lcp2Init;
          enable_capture_consumers = 1'b1;
        end
      end
      WaitForLcp1Result: begin
        enforce_rf_writeback = 1'b1;
        if (retired) begin
          state_d = Lcp2Init;
          enable_capture_consumers = 1'b1;
        end
      end
      Lcp2Init: begin
        // Enforce the writeback for the multicycle LCP1 result. Otherwise the result is lost.
        // TODO(colluca): do we actually need this? I would expect the WaitForLcp1Result state
        // to exactly serve this purpose.
        enforce_rf_writeback = 1'b1;
        // Disable fetching because there could be a new producer for LCP2
        // Still capture consumers until we receive the LCP2 result
        enable_capture_consumers = 1'b1;
        if (disp_req_valid_i) begin
          state_d              = Lcp2Fetch;
          enable_op_request    = 1'b1;
          if (issued) begin
            state_d = WaitForLcp2Result;
          end
          if (retired) begin
            enable_capture_consumers = 1'b0;
            state_d = Lep;
          end
        end
      end
      Lcp2Fetch: begin
        enable_op_request    = 1'b1;
        enforce_rf_writeback = 1'b1; // enabled for LCP1 and LCP2 result
        // Still capture consumers until we receive the LCP2 result
        enable_capture_consumers = 1'b1;
        if (issued) begin
          state_d = WaitForLcp2Result;
        end
        if (retired) begin
          enable_capture_consumers = 1'b0;
          state_d = Lep;
        end
      end
      WaitForLcp2Result: begin
        // we must keep the enforce rf writeback enabled until the result of LCP2 returns.
        enable_op_request    = 1'b1;
        enforce_rf_writeback = 1'b1; // enabled for LCP1 and LCP2 result
        // Still capture consumers until we receive the LCP2 result
        enable_capture_consumers = 1'b1;
        if (retired) begin
          state_d = Lep;
          enable_capture_consumers = 1'b0;
        end
      end
      Lep: begin
        // Wait here. We can fetch everything.
        enable_op_request = 1'b1;
        if (is_last_disp_iter_i && issued) begin
          state_d = LepFinished;
        end
      end
      LepFinished: begin
        // Do not request operands which will never be produced. This leads to deadlocks.
        enable_op_request = 1'b0;
        // We will only exit LEP when restarting.
      end
      default: ; // use default of above
    endcase

    // Initialization of the slot has highest prio
    if (restart_i) begin
      state_d              = Lcp1Init;
      enable_op_request    = 1'b0;
      enforce_rf_writeback = 1'b0;
    end
  end

  // TODO(colluca): add assertion that `disp_req_valid_i` is only asserted during LCPxInit.

  /////////////////
  // Slot Update //
  /////////////////

  // The slot FF is updated in multiple steps depending on the current slot state.
  // The update logic dependencies are shown below.
  // The operand requests pass via the Operand Distribution Network (ODN).
  //
  //                                                              Result req / rsp can be in same or other RSS
  //             +-----------------+    +-------------------+                      +-----------------------+
  // Disp req -->| Slot selection  |--->| OP req generation |--------------------->| Result req handling   |
  //             +-----------------+    +-------------------+       via ODN        +-----------------------+
  //                  ^       ^                   |                                            | here we could add a queue / timing cut
  //                  |       |                   v                                            v
  // LxP state -------+       |         +-------------------+                      +-----------------------+
  //                          |         | Op rsp handling   |<---------------------| Result rsp generation |
  //         +----------------+         +-------------------+       via ODN        +-----------------------+
  //         |                                    |                                       |        ^
  //         |                                    v                                       |        |
  //         |                          +-------------------+                             |        |
  //         |            +-------------| Issue             |                             |        |
  //         |            |             +-------------------+                             |        |
  //         |            |                       |                                       |        |
  //         |   +-----------------+              v                                       |        |
  //         |   | Functional Unit |              o<--------------------------------------+        |
  //         |   +-----------------+              |  Merge the updates                             |
  //         |            |                       v                                                |
  //         |            |             +-------------------+                                      |
  //         |            +------------>| Result capture    |                                      |
  //         |                          +-------------------+                                      |
  //         |                                    |                                                |
  //         |                                    v                                                |
  //         |                          +-------------------+                                      |
  //         |                          |> Slot FF          |                                      |
  //         |                          +-------------------+                                      |
  //         |                                    |                                                |
  //         +------------------------------------+------------------------------------------------+

  ////////////////////
  // Slot selection //
  ////////////////////

  // This stage performs an initial state-dependent slot update.

  // The initial operand values when accepting a new instruction
  rss_operand_t op_a_lcp1;
  rss_operand_t op_b_lcp1;
  rss_operand_t op_c_lcp1;

  assign op_a_lcp1 = '{
    producer:             disp_req_i.producer_op_a.producer,
    is_produced:          disp_req_i.producer_op_a.valid,
    is_from_current_iter: disp_req_i.producer_op_a.valid,
    value:                disp_req_i.fu_data.operand_a,
    is_valid:             !disp_req_i.producer_op_a.valid,
    requested:            1'b0
  };

  assign op_b_lcp1 = '{
    producer:             disp_req_i.producer_op_b.producer,
    is_produced:          disp_req_i.producer_op_b.valid,
    is_from_current_iter: disp_req_i.producer_op_b.valid,
    value:                disp_req_i.fu_data.operand_b,
    is_valid:             !disp_req_i.producer_op_b.valid,
    requested:            1'b0
  };

  assign op_c_lcp1 = '{
    producer:             disp_req_i.producer_op_c.producer,
    is_produced:          disp_req_i.producer_op_c.valid,
    is_from_current_iter: disp_req_i.producer_op_c.valid,
    value:                disp_req_i.fu_data.imm,
    is_valid:             !disp_req_i.producer_op_c.valid,
    requested:            1'b0
  };

  // Array to simplify initial operand assignment
  rss_operand_t [2:0] ops_lcp1;
  assign ops_lcp1[0] = op_a_lcp1;
  assign ops_lcp1[1] = op_b_lcp1;
  assign ops_lcp1[2] = op_c_lcp1;

  // Initial value of the slot upon accepting a new instruction
  rs_slot_t slot_lcp1;
  always_comb begin
    slot_lcp1 = '{
      spatz_raw_instr:     disp_req_i.fu_data.raw_instr, //Needed by Spatz
      is_occupied:          1'b1,
      consumer_count:       '0,
      consumed_by:          '0,
      do_capture_consumers: 1'b0,
      alu_op:               disp_req_i.fu_data.alu_op,
      lsu_op:               disp_req_i.fu_data.lsu_op,
      fpu_op:               disp_req_i.fu_data.fpu_op,
      // Duplicate logic for is_store. Once in LSU and once here.
      // TODO: Optimize by passing this to the LSU instead of regenerating it inside the LSU?
      is_store:             (disp_req_i.fu_data.fu == STORE) &&
                            (disp_req_i.fu_data.fpu_op inside {LsuOpStore, LsuOpFpStore}) || (disp_req_i.tag.dest_reg == '0 && !disp_req_i.tag.dest_reg_is_fp), // Handle the casse of other kind of instructions (SPATZ) that do not write back.
      lsu_size:             disp_req_i.fu_data.lsu_size,
      fpu_fmt_src:          disp_req_i.fu_data.fpu_fmt_src,
      fpu_fmt_dst:          disp_req_i.fu_data.fpu_fmt_dst,
      fpu_rnd_mode:         disp_req_i.fu_data.fpu_rnd_mode,
      // We must set the result iteration flag to 1. It gets toggled when writing the first result.
      result:               rss_result_t '{
        value:     '0,
        is_valid:  1'b0,
        iteration: 1'b1
      },
      instruction_iter:     1'b0,
      dest_id:              disp_req_i.tag.dest_reg,
      dest_is_fp:           disp_req_i.tag.dest_reg_is_fp,
      do_writeback:         1'b0,
      operands:             '0
    };

    // Operands must be assigned depending on the number we have
    for (int op = 0; op < NofOperands; op++) begin
      slot_lcp1.operands[op] = ops_lcp1[op];
    end
  end

  // Initial value of the slot upon accepting an instruction in LCP2.
  // Now all operand producers should be known, so we can update the missing producer information.
  // We also now know which instruction is the last in the loop to write to a certain register.
  // We can therefore also update the `do_writeback` flag.
  rs_slot_t slot_lcp2;
  always_comb begin
    slot_lcp2 = slot_q;

    // Update producers if there is not yet one set.
    // TODO(colluca): could we also update the producer for operands which already
    // have a producer? Probably yes, then this would be an energy-saving optimization.
    // Test this so we can better document it.
    if (!slot_lcp2.operands[0].is_produced) begin
      slot_lcp2.operands[0].producer    = disp_req_i.producer_op_a.producer;
      slot_lcp2.operands[0].is_produced = disp_req_i.producer_op_a.valid;
      slot_lcp2.operands[0].is_valid    = !disp_req_i.producer_op_a.valid;
      slot_lcp2.operands[0].value       = disp_req_i.fu_data.operand_a;
    end
    if (!slot_lcp2.operands[1].is_produced) begin
      slot_lcp2.operands[1].producer    = disp_req_i.producer_op_b.producer;
      slot_lcp2.operands[1].is_produced = disp_req_i.producer_op_b.valid;
      slot_lcp2.operands[1].is_valid    = !disp_req_i.producer_op_b.valid;
      slot_lcp2.operands[1].value       = disp_req_i.fu_data.operand_b;
    end
    if (NofOperands >= 3) begin
      if (!slot_lcp2.operands[2].is_produced) begin
        slot_lcp2.operands[2].producer    = disp_req_i.producer_op_c.producer;
        slot_lcp2.operands[2].is_produced = disp_req_i.producer_op_c.valid;
        slot_lcp2.operands[2].is_valid    = !disp_req_i.producer_op_c.valid;
        slot_lcp2.operands[2].value       = disp_req_i.fu_data.imm;
      end
    end
    // Set the writeback flag if we are the last RSS writing to this destination
    // TODO(colluca): couldn't we have the dispatcher calculate `do_writeback` instead?
    // This way we don't need to store it in the cut. It may increase the critical path
    // if the critical path is before the cut.
    if ((own_producer_id_i == disp_req_i.current_producer_dest.producer) &&
        disp_req_i.current_producer_dest.valid) begin
      slot_lcp2.do_writeback = 1'b1;
    end
  end

  // Initial state-dependent slot update
  rs_slot_t selected_slot;
  always_comb begin : slot_selection
    // Update the slot depending on the state.
    selected_slot = slot_q;
    unique case (state_q)
      Lcp1Init: begin
        if (disp_req_valid_i) begin
          selected_slot = slot_lcp1; // Load the new instruction
        end
      end
      Lcp2Init: begin
        if (disp_req_valid_i) begin
          // Update producers if there is not yet one set.
          selected_slot = slot_lcp2;
        end
      end
      Lcp1Fetch,
      Lcp2Fetch,
      WaitForLcp2Result,
      Lep,
      LepFinished: ; // Regular update
      default: ; // TODO: Crash?
    endcase

    // Slot initialization has highest priority
    if (restart_i) begin
      selected_slot = slot_reset_state;
    end
  end

  //////////////////////
  // Operand handling //
  //////////////////////

  // Sends operand requests and accepts responses based on the state of the slot after the
  // slot selection stage.
  // Also updates the slot after requesting operands (operands[i].requested field) and after
  // receiving responses (operands[i].{value,is_valid,requested} fields).
  // TODO(colluca): break this into two. 1) operand request generation. 2) operand response
  // capture. 1) contains both blocks 1) and 2) below, and 2) contains block 3) below.

  // TODO(colluca): break this into separate blocks. 1) Operand request generation
  // driving op_reqs_o and op_reqs_valid_o based on selected_slot. 2) slot update
  // depending on selected_slot, op_reqs_valid_o and op_rsps_valid_i. 3) Operand
  // response handling driving op_rsps_ready_o based on selected_slot.
  rs_slot_t slot_op;
  always_comb begin : slot_operand_handling
    slot_op = selected_slot;
    for (int op = 0; op < NofOperands; op++) begin : gen_operand_req_op
      // Operand request generation
      op_reqs_o[op] = '{
        producer: slot_op.operands[op].producer.rs_id,
        request: res_req_t'{
          // Invert the iteration flag if we desire the result from the previous loop iteration
          requested_iter: slot_op.operands[op].is_from_current_iter ?  slot_op.instruction_iter :
                                                                      ~slot_op.instruction_iter,
          slot_id:        slot_op.operands[op].producer.slot_id
        }
      };

      op_reqs_valid_o[op] = slot_op.operands[op].is_produced && !slot_op.operands[op].is_valid &&
                            !slot_op.operands[op].requested &&
                            slot_op.is_occupied && // Request the operand only if slot is active
                            enable_op_request;

      // Capture request placement at handshake
      if (op_reqs_valid_o[op] && op_reqs_ready_i[op]) begin
        slot_op.operands[op].requested = 1'b1;
      end

      // Operand response handling
      // Handle the response after the request generation to allow same cycle responses.
      // We won't place two requests due to the requested flag.
      op_rsps_ready_o[op] = 1'b0;
      if (slot_op.is_occupied && slot_op.operands[op].is_produced && op_rsps_valid_i[op]) begin
        slot_op.operands[op].value     = op_rsps_i[op];
        slot_op.operands[op].is_valid  = 1'b1;
        slot_op.operands[op].requested = 1'b0;
        // Acknowledge the response
        op_rsps_ready_o[op] = 1'b1;
      end
    end
  end

  ///////////////////////
  // Issue instruction //
  ///////////////////////

  // Issues an instruction if all operands have been received, based on the state of the slot
  // after the operand request stage.
  // Also updates the slot after issuing the instruction (instruction_iter and
  // operands[i].is_valid fields).

  // TODO(colluca): break this into separate blocks. 1) Issue request generation
  // driving issue_req_o and issue_req_valid_o based on slot_op. 2) slot update
  // depending on slot_op, issued, etc.
  rs_slot_t slot_issue;
  always_comb begin : slot_issue_update
    slot_issue = slot_op;

    disp_req_ready_o = 1'b0;
    // Issue the operation if all operands are valid. The FU exerts backpressure if its pipeline
    // is full or the result cannot be written because the current result has not been consumed
    // by all consumers yet.
    // Tag used for the operation is the slot_id, to identify the result destination in case
    // results can come back OoO from the FU (as is the case for the FPU).
    issue_req_o                      = '0;
    issue_req_o.fu_data.fu           = NONE; // Not required by FU
    issue_req_o.fu_data.raw_instr    = slot_issue.spatz_raw_instr; // Spatz needs the raw instruction
    issue_req_o.fu_data.alu_op       = slot_issue.alu_op;
    issue_req_o.fu_data.lsu_op       = slot_issue.lsu_op;
    issue_req_o.fu_data.csr_op       = CsrOpNone; // Not supported in FREP
    issue_req_o.fu_data.fpu_op       = slot_issue.fpu_op;
    issue_req_o.fu_data.operand_a    = slot_issue.operands[0].value;
    issue_req_o.fu_data.operand_b    = slot_issue.operands[1].value;
    issue_req_o.fu_data.imm          = (NofOperands >= 3) ? slot_issue.operands[2].value : '0;
    issue_req_o.fu_data.lsu_size     = slot_issue.lsu_size;
    issue_req_o.fu_data.fpu_fmt_src  = slot_issue.fpu_fmt_src;
    issue_req_o.fu_data.fpu_fmt_dst  = slot_issue.fpu_fmt_dst;
    issue_req_o.fu_data.fpu_rnd_mode = slot_issue.fpu_rnd_mode;
    issue_req_o.tag                  = slot_id_i;

    for (int i = 0; i < NofOperands; i++) begin
      op_valid[i] = slot_issue.operands[i].is_valid;
    end
    all_ops_valid = &op_valid;

    issue_req_valid_o = 1'b0;
    if (all_ops_valid && slot_issue.is_occupied) begin
      // TODO(colluca): can we not just have 1'b1 here? Then we could probably remove
      // the weird dispatch mux in the res_stat
      issue_req_valid_o = disp_req_valid_i;
    end
    // Capture handshake
    issued = issue_req_valid_o && issue_req_ready_i;
    if (issued) begin
      // Toggle instruction state
      slot_issue.instruction_iter = ~slot_issue.instruction_iter;
      // Reset the operands' valid flags depending on loop phase and production state
      // In LCP1 we reset all valid flags
      // In LCP2 and LEP we only reset produced operands' valid flags
      // TODO(colluca): HACK! This signal has no default!!! Results in an inferred latch!!!
      enforce_valid_reset = 1'b0;
      unique case (state_q)
        Lcp1Init:  if (disp_req_valid_i) enforce_valid_reset = 1'b1;
        Lcp1Fetch: enforce_valid_reset = 1'b1;
        default: ;
      endcase

      for (int op = 0; op < NofOperands; op++) begin
        slot_issue.operands[op].is_valid =
          enforce_valid_reset                 ? 1'b0 :
          slot_issue.operands[op].is_produced ? 1'b0 :
                                                slot_issue.operands[op].is_valid;
      end

      // When we issue the instruction, the dispatch request is complete
      disp_req_ready_o = 1'b1;
    end
  end

  ////////////////////////////////
  // Result response generation //
  ////////////////////////////////

  // Always answer requests using the "old" result (before result capture) as otherwise we
  // would create a loop. The loop comes from the connection back to the operand response
  // handling. The generated result response is sent back to the operand interface.
  assign res_rsp_o.dest_mask = dest_mask_i;
  assign res_rsp_o.operand   = slot_q.result.value;
  // We don't need to check the iteration here as it is already checked in the request crossbar.
  assign res_rsp_valid_o     = dest_mask_valid_i &&
                        slot_q.result.is_valid;
  assign dest_mask_ready_o   = res_rsp_ready_i;

  logic [cf_math_pkg::idx_width($bits(dest_mask_i)+1)-1:0] num_current_consumers;
  popcount #(
    .INPUT_WIDTH($bits(dest_mask_i))
  ) i_consumer_popcount (
      .data_i(dest_mask_i),
      .popcount_o(num_current_consumers)
  );

  instr_tag_t rf_wb_tag;
  rs_slot_t slot_wb;
  logic     enable_rf_writeback;
  always_comb begin : result_wb_request_response
    // TODO(colluca): break this block into separate blocks. 1) slot update after result response.
    // 2) capture result. 3) slot update after result capture.
    // 1) should be part of "Generate result responses" and 2) and 3) should be part of
    // a separate top-level block called "Result capture"

    //////////////////////////////////////////////////
    // Update slot after result response generation //
    //////////////////////////////////////////////////

    // TODO(colluca): replace $countones with popcount module.
    slot_wb = slot_issue;
    // When we served a result request, update consumer counter
    if (res_rsp_valid_o && res_rsp_ready_i) begin
      // count the bits in the destination mask
      slot_wb.consumed_by = slot_wb.consumed_by + num_current_consumers;
      // During LCP there may be only one request at at time.. We still count all requests.
      // TODO(colluca): can't we use enable_capture_consumers here, and get rid of
      // do_capture_consumers in the slot altogether?
      if (slot_q.do_capture_consumers) begin
        slot_wb.consumer_count = slot_wb.consumer_count + num_current_consumers;
      end
    end

    ////////////////////
    // Result capture //
    ////////////////////

    // Capture the result:
    // - Always if the current result is invalid
    // - If the current result is valid, result must have been consumed by all consumers
    //
    // We must check also the RF writeback for LCP and last LEP.
    // This is handled with a special stream fork.

    // Compose tag
    rf_wb_tag = '0;
    rf_wb_tag.dest_reg       = slot_wb.dest_id;
    rf_wb_tag.dest_reg_is_fp = slot_wb.dest_is_fp;
    // TODO(colluca): find proper solution for this
    // HACK:
    // We directly pass through the branch and jump details to support jumps in LCP1
    // (where we will fall back into regular HW loop mode). This is possible as the ALU is
    // single cycle and the dispatch request is valid until we write back.
    rf_wb_tag.is_branch = disp_req_i.tag.is_branch;
    rf_wb_tag.is_jump   = disp_req_i.tag.is_jump;

    // Check if we want to write back to the RF. If so, enable the RF path for the dynamic stream
    // fork. A store has no writeback and the last result iteration is "immediately" reached.
    enable_rf_writeback = is_last_result_iter_i && !slot_wb.is_store;
    // TODO(colluca): I think the enforce_rf_writeback signal is superfluous, and this should just
    // be directly written as:
    // do_rf_writeback = (state_q == Lep) ? (slot_wb.do_writeback && enable_rf_writeback) : 1'b1`
    do_rf_writeback = (slot_wb.do_writeback && enable_rf_writeback) || enforce_rf_writeback;

    // The result is consumed when all consumers read the result once
    result_consumed = (slot_wb.consumed_by == slot_wb.consumer_count) &&
                      (slot_wb.consumer_count != '0);

    rss_wb_ready = 1'b0;
    // The ready may not be dependent on the valid. See below for more details of the handshake.
    if ((result_consumed || !slot_wb.result.is_valid || slot_wb.consumer_count == '0) &&
        slot_wb.is_occupied) begin
      rss_wb_ready = 1'b1;
    end

    // We captured a new result when the stream fork signals the handshake to the FU.
    // This includes both cases (only RSS as well as RSS and RF).
    // A store instruction has no result. Thus we capture a dummy result at the same time
    // we issue the store instruction.
    retired = slot_wb.is_store ? issued : (result_valid_i && result_ready_o);

    //////////////////////////////////////
    // Slot update after result capture //
    //////////////////////////////////////

    if (retired) begin
      slot_wb.result.is_valid  = 1'b1;
      slot_wb.result.iteration = !slot_wb.result.iteration;
      // Don't update the result FFs for stores.
      // TODO: Does this MUX really save power (by not updating the FFs) or will it add too much
      //       logic?
      // TODO(colluca): to save power we would want to avoid that it switches at all, i.e.
      // keep the value in slot_q, not '0.
      slot_wb.result.value     = slot_wb.is_store ? '0 : result_i;
      slot_wb.consumed_by      = '0;
    end
    // This signal is generated in the main FSM.
    slot_wb.do_capture_consumers = enable_capture_consumers;
  end

  // Update the slot after all manipulations
  assign slot_d = slot_wb;

  // The current result iteration state is directly passed to the output
  assign res_iter_o = slot_q.result.iteration;

  // Retired signal back to RS to step the result pointer.
  // For all instructions except stores, this retired signal is the same as used inside this RSS.
  // I.e., it is asserted in the cycle we retire the result / handshake it.
  // For stores this is different as stores have no result and thus retire immediately.
  // However, we must signal the retirement "in order" to the result pointer.
  // As loads and stores can be mixed, we could miss the retired signal for a store as it is only
  // asserted once in the cycle we issue it. But in this cycle the RS result pointer could be set
  // to an ongoing load. Thus we signal the retired signal always and as soon as the RS result
  // pointer steps to the store, it immediately "retires" the instruction.
  // TODO(colluca): does the RS really need a pointer? Or is a counter sufficient? In the latter
  // case we wouldn't need this differentiation and the RS would just set `retiring` to
  // |rss_retiring instead of rss_retiring[result_idx]. This would also be easier to extend if
  // we would at some point want to support multiple instructions retiring in the same cycle,
  // within the same RS, e.g. due to the presence of pipelines with different latencies.
  assign retired_o = slot_wb.is_store ? 1'b1 : retired;

  ///////////////
  // Writeback //
  ///////////////

  assign rss_wb_valid = result_valid_i;
  assign result_ready_o = rss_wb_ready;
  assign rf_do_writeback_o = do_rf_writeback;
  assign rf_wb_tag_o = rf_wb_tag;

endmodule
