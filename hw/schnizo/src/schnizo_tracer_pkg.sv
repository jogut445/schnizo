// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Schnizo core tracer package.
//
// Contains type definitions and formatting functions for the Schnizo core tracer.
package schnizo_tracer_pkg;
  import schnizo_pkg::*;
  import spatz_pkg::*;
  // pragma translate_off

  //////////////////////
  // Type definitions //
  //////////////////////

  typedef struct packed {
    priv_lvl_t                    priv_level;
    loop_state_e                  state;
    logic [FrepMaxItersWidth-1:0] iteration;
    logic                         stall;
    logic                         exception;
  } schnizo_core_trace_t;

  typedef struct {
    logic   valid; // high if handshake happens
    longint pc_q;
    longint pc_d;
    longint instr_data;
    longint rs1;
    longint rs2;
    longint rs3; // for fused FPU instructions
    longint rd;
    longint rs1_is_fp;
    longint rs2_is_fp;
    longint rd_is_fp;
    longint is_branch; // jal & jalr are handled via pc_d (add goto if pc_d != pc_q + 4)
    longint branch_taken;
    // FU selection - with known number of FUs and RSS we can reconstruct which FU it was.
    // However, not all FUs (CSR, ACC) have a producer id. We provide both informations and the tracer
    // can select depending on the CPU state.
    string fu_type;
    string disp_resp;
  } schnizo_dispatch_trace_t;

  typedef struct {
    logic   valid; // high if handshake happens
    longint instr_iter;
    string  producer;
    longint alu_opa;
    longint alu_opb;
  } issue_alu_trace_t;

  typedef struct {
    logic   valid; // high if handshake happens
    longint instr_iter;
    string  producer;
    longint lsu_store_data;
    longint lsu_is_float;
    longint lsu_is_load;
    longint lsu_is_store;
    longint lsu_addr; // the computed memory address
    longint lsu_size;
    longint lsu_amo;
    // we don't track the stored data
  } issue_lsu_trace_t;

  typedef struct {
    logic   valid; // high if handshake happens
    longint instr_iter;
    string  producer;
    longint fpu_opa;
    longint fpu_opb;
    longint fpu_opc;
    longint fpu_src_fmt;
    longint fpu_dst_fmt;
    longint fpu_int_fmt;
  } issue_fpu_trace_t;

  typedef struct {
    logic   valid; // high if handshake happens
    longint instr_iter;
    string  producer;
    longint spatz_opa;
    longint spatz_opb;
    spatz_id_t internal_spatz_id; // if the instruction will write back a result
  } issue_spatz_trace_t;

  typedef struct {
    logic   valid; // high if handshake happens
    string  producer;
    longint csr_addr;
    longint csr_read_data;
    longint csr_write_data;
  } issue_csr_trace_t;

  typedef struct {
    logic   valid; // high if handshake happens
    string  producer;
    longint acc_addr;
    longint acc_arga;
    longint acc_argb;
    longint acc_argc;
  } issue_acc_trace_t;

  // retirements
  typedef struct {
    logic   valid; // high if handshake happens
    string  producer;
  } retire_fu_trace_t;

  // writebacks
  typedef struct {
    logic   valid; // high if handshake happens
    longint fu_result;
    longint fu_rd;
    longint fu_rd_is_fp;
  } wb_fu_trace_t;

  // Result requests
  typedef struct {
    logic   valid; // high if handshake happens
    string  producer; // to here this request comes
    string  consumer; // from here this requests originated
    longint requested_iter;
  } resreq_trace_t;

  // Result captures
  typedef struct {
    logic   valid; // high if handshake happens
    string  producer;
    longint result_iter;
    longint enable_rf_wb;
    longint rd;
    longint rd_is_fp;
    longint result;
  } rescap_trace_t;

  // Spatz internal Traces
  typedef struct {
    logic valid; // high if handshake happens
    string name;
  } internal_issue_spatz_trace_t;//It is actually just a one bit flag. Kept it as a struct for consistency with the other tracing structures

  typedef struct {
    logic valid;
    int id;
  } internal_retire_spatz_trace_t;

  ///////////////
  // Functions //
  ///////////////

  // Returns the header of a trace event. This includes common trace data.
  function automatic string format_trace_header(time t, logic[63:0] cycle, schnizo_core_trace_t core);
    return $sformatf("{'time': %0t, 'cycle': %0d, 'priv': \"%s\", 'state': \"%s\", 'iteration': %0d, 'stall': 0x%0x, 'exception': 0x%0x, ",
                     t, cycle, schnizo_pkg::priv_lvl_tostring(core.priv_level),
                     schnizo_pkg::loop_state_tostring(core.state), core.iteration, core.stall, core.exception);
  endfunction

  // Format all dispatch extras as a key value pair list.
  function automatic string format_dispatch_extras(schnizo_dispatch_trace_t trace);
    string extras = "";
    if (!trace.valid) begin
      return "";
    end
    extras = $sformatf("%s'%s':\"%s\", ", extras, "fu_type", trace.fu_type);
    extras = $sformatf("%s'%s':\"%s\", ", extras, "disp_resp", trace.disp_resp);
    extras = $sformatf("%s'%s':0x%08x, ", extras, "pc_q", trace.pc_q);
    extras = $sformatf("%s'%s':0x%08x, ", extras, "pc_d", trace.pc_d);
    extras = $sformatf("%s'%s':0x%08x, ", extras, "instr_data", trace.instr_data);
    extras = $sformatf("%s'%s':0x%02x, ", extras, "rs1", trace.rs1);
    extras = $sformatf("%s'%s':0x%02x, ", extras, "rs2", trace.rs2);
    extras = $sformatf("%s'%s':0x%02x, ", extras, "rs3", trace.rs3);
    extras = $sformatf("%s'%s':0x%02x, ", extras, "rd", trace.rd);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "rs1_is_fp", trace.rs1_is_fp);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "rs2_is_fp", trace.rs2_is_fp);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "rd_is_fp", trace.rd_is_fp);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "is_branch", trace.is_branch);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "branch_taken", trace.branch_taken);
    return extras;
  endfunction

  function automatic string format_alu_trace(issue_alu_trace_t trace);
    string extras = "";
    if (!trace.valid) begin
      return "";
    end
    extras = $sformatf("%s'%s':0x%0x, ", extras, "instr_iter", trace.instr_iter);
    extras = $sformatf("%s'%s':\"%s\", ", extras, "producer", trace.producer);
    extras = $sformatf("%s'%s':0x%08x, ", extras, "alu_opa", trace.alu_opa);
    extras = $sformatf("%s'%s':0x%08x, ", extras, "alu_opb", trace.alu_opb);
    return extras;
  endfunction

  function automatic string format_lsu_trace(issue_lsu_trace_t trace);
    string extras = "";
    if (!trace.valid) begin
      return "";
    end
    extras = $sformatf("%s'%s':0x%0x, ", extras, "instr_iter", trace.instr_iter);
    extras = $sformatf("%s'%s':\"%s\", ", extras, "producer", trace.producer);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "lsu_store_data", trace.lsu_store_data);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "lsu_is_float", trace.lsu_is_float);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "lsu_is_load", trace.lsu_is_load);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "lsu_is_store", trace.lsu_is_store);
    extras = $sformatf("%s'%s':0x%08x, ", extras, "lsu_addr", trace.lsu_addr);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "lsu_size", trace.lsu_size);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "lsu_amo", trace.lsu_amo);
    return extras;
  endfunction

  function automatic string format_fpu_trace(issue_fpu_trace_t trace);
    string extras = "";
    if (!trace.valid) begin
      return "";
    end
    extras = $sformatf("%s'%s':0x%0x, ", extras, "instr_iter", trace.instr_iter);
    extras = $sformatf("%s'%s':\"%s\", ", extras, "producer", trace.producer);
    extras = $sformatf("%s'%s':0x%016x, ", extras, "fpu_opa", trace.fpu_opa);
    extras = $sformatf("%s'%s':0x%016x, ", extras, "fpu_opb", trace.fpu_opb);
    extras = $sformatf("%s'%s':0x%016x, ", extras, "fpu_opc", trace.fpu_opc);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "fpu_src_fmt", trace.fpu_src_fmt);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "fpu_dst_fmt", trace.fpu_dst_fmt);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "fpu_int_fmt", trace.fpu_int_fmt);
    return extras;
  endfunction

  function automatic string format_csr_trace(issue_csr_trace_t trace);
    string extras = "";
    if (!trace.valid) begin
      return "";
    end
    extras = $sformatf("%s'%s':\"%s\", ", extras, "producer", trace.producer);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "csr_addr", trace.csr_addr);
    extras = $sformatf("%s'%s':0x%08x, ", extras, "csr_read_data", trace.csr_read_data);
    extras = $sformatf("%s'%s':0x%08x, ", extras, "csr_write_data", trace.csr_write_data);
    return extras;
  endfunction

  function automatic string format_acc_trace(issue_acc_trace_t trace);
    string extras = "";
    if (!trace.valid) begin
      return "";
    end
    extras = $sformatf("%s'%s':\"%s\", ", extras, "producer", trace.producer);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "acc_addr", trace.acc_addr);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "acc_arga", trace.acc_arga);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "acc_argb", trace.acc_argb);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "acc_argc", trace.acc_argc);
    return extras;
  endfunction

  function automatic string format_fu_retire_trace(retire_fu_trace_t trace);
    string extras = "";
    if (!trace.valid) begin
      return "";
    end
    extras = $sformatf("%s'%s':\"%s\", ", extras, "producer", trace.producer);
    return extras;
  endfunction

  function automatic string format_wb_fu_trace(wb_fu_trace_t trace, string fu);
    string extras = "";
    if (!trace.valid) begin
      return "";
    end
    extras = $sformatf("%s'%s':\"%s\", ", extras, "origin", fu);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "result", trace.fu_result);
    extras = $sformatf("%s'%s':0x%02x, ", extras, "rd", trace.fu_rd);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "rd_is_fp", trace.fu_rd_is_fp);
    return extras;
  endfunction

  function automatic string format_resreq_trace(resreq_trace_t trace);
    string extras = "";
    if (!trace.valid) begin
      return "";
    end
    extras = $sformatf("%s'%s':\"%s\", ", extras, "producer", trace.producer);
    extras = $sformatf("%s'%s':\"%s\", ", extras, "consumer", trace.consumer);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "requested_iter", trace.requested_iter);
    return extras;
  endfunction

  function automatic string format_rescap_trace(rescap_trace_t trace);
    string extras = "";
    if (!trace.valid) begin
      return "";
    end
    extras = $sformatf("%s'%s':\"%s\", ", extras, "producer", trace.producer);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "result_iter", trace.result_iter);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "enable_rf_wb", trace.enable_rf_wb);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "rd", trace.rd);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "rd_is_fp", trace.rd_is_fp);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "result", trace.result);
    return extras;
  endfunction

  function automatic void write_trace_event(int file_id, string trace_header, string event_type,
                                            string trace_extras, logic trace_valid);
    string trace_event;
    trace_event = $sformatf("%s'event':\"%s\", %s", trace_header, event_type, trace_extras);
    if (trace_valid) begin
      // close the extra key value list
      $fwrite(file_id,  $sformatf("%s}\n", trace_event));
    end
  endfunction

  // Spatz Traces

  function automatic string format_spatz_trace(issue_spatz_trace_t trace);
    string extras = "";
    if (!trace.valid) begin
      return "";
    end
    extras = $sformatf("%s'%s':0x%0x, ", extras, "instr_iter", trace.instr_iter);
    extras = $sformatf("%s'%s':\"%s\", ", extras, "producer", trace.producer);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "spatz_opa", trace.spatz_opa);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "spatz_opb", trace.spatz_opb);
    extras = $sformatf("%s'%s':0x%0x, ", extras, "internal_spatz_id", trace.internal_spatz_id);
    return extras;
  endfunction

  function automatic string format_internal_spatz_trace(internal_issue_spatz_trace_t trace, int id, string name);
    string extras = "";
    if (!trace.valid) begin
      return "prova";
    end
    extras = $sformatf("%s'%s':0x%0x, ", extras, "id", id);
    extras = $sformatf("%s'%s':\"%s\", ", extras, "op", name);
    return extras;
  endfunction

  function automatic string format_int_spatz_retire_trace(internal_retire_spatz_trace_t trace);
    string extras = "";
    if (!trace.valid) begin
      return "";
    end
    extras = $sformatf("%s'%s':\"%0d\", ", extras, "id", trace.id);
    return extras;
  endfunction

  // pragma translate_on

endpackage
