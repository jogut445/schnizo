// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// pragma translate_off

// Schnizo core tracer.
module schnizo_tracer import schnizo_pkg::*, schnizo_tracer_pkg::*; import spatz_pkg::*; #(
  parameter int unsigned NofAlus    = 3,
  parameter int unsigned NofLsus    = 1,
  parameter int unsigned NofFpus    = 1,
  parameter int unsigned AluNofRss  = 3,
  parameter int unsigned LsuNofRss  = 2,
  parameter int unsigned FpuNofRss  = 4,
  parameter int unsigned SpatzNofRss = 4,
  parameter int unsigned NrParallelInstructions = 8,
  parameter int unsigned NofOperandIfs = 1,
  parameter bit          Xfrep      = 1
) (
  input  logic clk_i,
  input  logic rst_i,
  input  logic [31:0] hart_id_i,
  input  int unsigned dispatch_rs_id,
  input  schnizo_core_trace_t     core_trace,
  input  schnizo_dispatch_trace_t dispatch_trace,
  input  issue_alu_trace_t        alu_trace [NofAlus],
  input  issue_lsu_trace_t        lsu_trace [NofLsus],
  input  issue_fpu_trace_t        fpu_trace [NofFpus],
  input  issue_spatz_trace_t      spatz_trace,
  input  issue_alu_trace_t        rss_alu_traces [NofAlus][AluNofRss],
  input  issue_lsu_trace_t        rss_lsu_traces [NofLsus][LsuNofRss],
  input  issue_fpu_trace_t        rss_fpu_traces [NofFpus][FpuNofRss],
  input  issue_spatz_trace_t      rss_spatz_traces [SpatzNofRss],  
  input  issue_csr_trace_t        csr_trace,
  input  issue_acc_trace_t        acc_trace,
  input  retire_fu_trace_t        alu_retirements [NofAlus],
  input  retire_fu_trace_t        lsu_retirements [NofLsus],
  input  retire_fu_trace_t        fpu_retirements [NofFpus],
  input  retire_fu_trace_t        csr_retirement,
  input  retire_fu_trace_t        acc_retirement,
  input  internal_retire_spatz_trace_t spatz_retirement [NrParallelInstructions],
  input  wb_fu_trace_t            alu_wb_trace,
  input  wb_fu_trace_t            lsu_wb_trace,
  input  wb_fu_trace_t            fpu_wb_trace,
  input  wb_fu_trace_t            csr_wb_trace,
  input  wb_fu_trace_t            acc_wb_trace,
  input  wb_fu_trace_t            spatz_wb_trace,
  input  resreq_trace_t           alu_resreq_traces   [NofAlus][AluNofRss][NofOperandIfs],
  input  resreq_trace_t           lsu_resreq_traces   [NofLsus][LsuNofRss][NofOperandIfs],
  input  resreq_trace_t           fpu_resreq_traces   [NofFpus][FpuNofRss][NofOperandIfs],
  input  resreq_trace_t           spatz_resreq_traces [SpatzNofRss][NofOperandIfs],
  input  rescap_trace_t           alu_rescap_traces   [NofAlus][AluNofRss],
  input  rescap_trace_t           lsu_rescap_traces   [NofLsus][LsuNofRss],
  input  rescap_trace_t           fpu_rescap_traces   [NofFpus][FpuNofRss],
  input  rescap_trace_t           spatz_rescap_traces [SpatzNofRss],
  input  op_e                     spatz_instrs_names  [NrParallelInstructions],
  input  internal_issue_spatz_trace_t internal_spatz_traces [NrParallelInstructions]
);

  // The tracer first extracts all signals of interest and groups them by functional unit.
  // It also distinguishs between signal groups for regular and FREP exection.
  // The second part then emits a trace entry if the current signal group is valid (active).
  // The signal group validity depends on the handshake as well as the core state.
  // We start with result requests then issuing events and end with the writeback events.
  // This helps to order the events for postprocessing.

  int file_id;
  string file_name;
  logic [63:0] cycle;
  initial begin
    // We need to schedule the assignment into a safe region, otherwise
    // `hart_id_i` won't have a value assigned at the beginning of the first
    // delta cycle.
`ifndef VERILATOR
    #0;
`endif // VERILATOR
    $system("mkdir logs -p");
    $sformat(file_name, "logs/sz_trace_hart_%05x.dasm", hart_id_i);
    file_id = $fopen(file_name, "w");
    $display("[Tracer] Logging Hart %d to %s", hart_id_i, file_name);
  end

  // During LCP the dispatch & trace generation is at least one cycle apart. For the LSU it can
  // even take longer (until the memory accepted the request).
  // If there is a dispatch during LCP, push the details into a queue for each FU.
  // When the FU issues, pop from the queue and generate the trace.
  typedef struct {
    string header;
    schnizo_dispatch_trace_t dispatch_trace;
  } lcp_dispatch_detail_t;

  localparam integer unsigned NofFus = NofAlus + NofLsus + NofFpus + (RVV ? 1 : 0);;
  
  lcp_dispatch_detail_t lcp_dispatch_queue[NofFus][$];

  // verilog_lint: waive-start always-ff-non-blocking
  always_ff @(posedge clk_i) begin
    string trace_header;
    string dispatch_event;
    lcp_dispatch_detail_t lcp_details;
    if (~rst_i) begin
      cycle++;

      // Always generate the core trace. This trace serves as the basis of the trace event.
      // This trace is extended with the details of the active FU.
      trace_header = format_trace_header($time, cycle, core_trace);

      if (RVV) begin: gen_internal_issues
        for (int i = 0; i < NrParallelInstructions; i++) begin
          write_trace_event(file_id, trace_header, "internal_spatz_issue",
                    format_internal_spatz_trace(internal_spatz_traces[i], i, spatz_instrs_names[i].name()),
                    internal_spatz_traces[i].valid);
        end
      end

      lcp_details = '{
        header: trace_header,
        dispatch_trace: dispatch_trace
      };

      if (dispatch_trace.valid && (core_trace.state inside {LoopLcp1, LoopLcp2})) begin
        lcp_dispatch_queue[dispatch_rs_id].push_back(lcp_details);
      end

      // TODO: We should create a class to capture all the common functions...

      // Result request events - these should only be active during LCP and LEP.
      // We can capture them "always".
      for (int alu = 0; alu < NofAlus; alu++) begin
        for (int rss = 0; rss < AluNofRss; rss++) begin
          for (int con = 0; con < NofOperandIfs; con++) begin
            write_trace_event(file_id, trace_header, "resreq",
                              format_resreq_trace(alu_resreq_traces[alu][rss][con]),
                              alu_resreq_traces[alu][rss][con].valid);
          end
        end
      end
      for (int lsu = 0; lsu < NofLsus; lsu++) begin
        for (int rss = 0; rss < LsuNofRss; rss++) begin
          for (int con = 0; con < NofOperandIfs; con++) begin
            write_trace_event(file_id, trace_header, "resreq",
                              format_resreq_trace(lsu_resreq_traces[lsu][rss][con]),
                              lsu_resreq_traces[lsu][rss][con].valid);
          end
        end
      end
      for (int fpu = 0; fpu < NofFpus; fpu++) begin
        for (int rss = 0; rss < FpuNofRss; rss++) begin
          for (int con = 0; con < NofOperandIfs; con++) begin
            write_trace_event(file_id, trace_header, "resreq",
                              format_resreq_trace(fpu_resreq_traces[fpu][rss][con]),
                              fpu_resreq_traces[fpu][rss][con].valid);
          end
        end
      end
      if (RVV) begin
        for (int rss = 0; rss < SpatzNofRss; rss++) begin
          for (int con = 0; con < NofOperandIfs; con++) begin
            write_trace_event(file_id, trace_header, "resreq",
                              format_resreq_trace(spatz_resreq_traces[rss][con]),
                              spatz_resreq_traces[rss][con].valid);
          end
        end
      end

      // Trace events are active depending on CPU states.
      if (core_trace.state inside {LoopRegular, LoopHwLoop}) begin
        // Format the single dispatch event and append all single issue requests. There should
        // only be one active single issue request. The format functions return "" if the trace is
        // not valid. Therefore, we can combine the formatting functions into one chain.
        // The naked dispatch_event contains the None FU dispatches (currently only for FREP).
        dispatch_event = format_dispatch_extras(dispatch_trace);

        for (int alu = 0; alu < NofAlus; alu++) begin
          dispatch_event = $sformatf("%s%s", dispatch_event, format_alu_trace(alu_trace[alu]));
        end
        for (int lsu = 0; lsu < NofLsus; lsu++) begin
          dispatch_event = $sformatf("%s%s", dispatch_event, format_lsu_trace(lsu_trace[lsu]));
        end
        for (int fpu = 0; fpu < NofFpus; fpu++) begin
          dispatch_event = $sformatf("%s%s", dispatch_event, format_fpu_trace(fpu_trace[fpu]));
        end
        dispatch_event = $sformatf("%s%s", dispatch_event, format_csr_trace(csr_trace));
        dispatch_event = $sformatf("%s%s", dispatch_event, format_acc_trace(acc_trace));

        if (RVV) dispatch_event = $sformatf("%s%s", dispatch_event, format_spatz_trace(spatz_trace));

        write_trace_event(file_id, trace_header, "dispatch", dispatch_event, dispatch_trace.valid);
      end else if (core_trace.state inside {LoopLcp1, LoopLcp2}) begin
        // Format the single dispatch event but capture the producer by taking RSS issue trace.
        // There should also be one FU issue request active. Invalid traces are formated as "".

        // If a FU issues, pop from the dispatch details queue and generate the trace.
        lcp_dispatch_detail_t details;
        for (int alu = 0; alu < NofAlus; alu++) begin
          for (int rss = 0; rss < AluNofRss; rss++) begin
            if (rss_alu_traces[alu][rss].valid) begin
              details = lcp_dispatch_queue[alu].pop_front();
              dispatch_event = format_dispatch_extras(details.dispatch_trace);

              dispatch_event = $sformatf("%s%s", dispatch_event,
                                        format_alu_trace(rss_alu_traces[alu][rss]));
              write_trace_event(file_id, details.header, "dispatch",
                                dispatch_event, details.dispatch_trace.valid);
            end
          end
        end

        for (int lsu = 0; lsu < NofLsus; lsu++) begin
          for (int rss = 0; rss < LsuNofRss; rss++) begin
            if (rss_lsu_traces[lsu][rss].valid) begin
              details = lcp_dispatch_queue[NofAlus + lsu].pop_front();
              dispatch_event = format_dispatch_extras(details.dispatch_trace);

              dispatch_event = $sformatf("%s%s", dispatch_event,
                                        format_lsu_trace(rss_lsu_traces[lsu][rss]));
              write_trace_event(file_id, details.header, "dispatch",
                                dispatch_event, details.dispatch_trace.valid);
            end
          end
        end

        for (int fpu = 0; fpu < NofFpus; fpu++) begin
          for (int rss = 0; rss < FpuNofRss; rss++) begin
            if (rss_fpu_traces[fpu][rss].valid) begin
              details = lcp_dispatch_queue[NofAlus + NofLsus + fpu].pop_front();
              dispatch_event = format_dispatch_extras(details.dispatch_trace);

              dispatch_event = $sformatf("%s%s", dispatch_event,
                                        format_fpu_trace(rss_fpu_traces[fpu][rss]));
              write_trace_event(file_id, details.header, "dispatch",
                                dispatch_event, details.dispatch_trace.valid);
            end
          end
        end

        if (RVV) begin
          for (int rss = 0; rss < SpatzNofRss; rss++) begin
            if (rss_spatz_traces[rss].valid) begin
              details = lcp_dispatch_queue[NofAlus + NofLsus + NofFpus].pop_front();
              dispatch_event = format_dispatch_extras(details.dispatch_trace);

              dispatch_event = $sformatf("%s%s", dispatch_event,
                                        format_spatz_trace(rss_spatz_traces[rss]));
              write_trace_event(file_id, details.header, "dispatch",
                                dispatch_event, details.dispatch_trace.valid);
            end
          end
        end

        // CSR and ACC instructions are not supported in FREP but can still execute (fallback in
        // hw loop mode). These are not cut and thus dispatch immediately.
        dispatch_event = format_dispatch_extras(dispatch_trace);
        dispatch_event = $sformatf("%s%s", dispatch_event, format_csr_trace(csr_trace));
        dispatch_event = $sformatf("%s%s", dispatch_event, format_acc_trace(acc_trace));

        write_trace_event(file_id, trace_header, "dispatch", dispatch_event,
                          dispatch_trace.valid && (csr_trace.valid || acc_trace.valid));
      end else if (core_trace.state inside {LoopLep}) begin
        // There is no dispatch request and we can have multiple events per cycle.
        // We must check each RSS issue request on its own.
        for (int alu = 0; alu < NofAlus; alu++) begin
          for (int rss = 0; rss < AluNofRss; rss++) begin
            write_trace_event(file_id, trace_header, "dispatch",
                             format_alu_trace(rss_alu_traces[alu][rss]),
                             rss_alu_traces[alu][rss].valid);
          end
        end
        for (int lsu = 0; lsu < NofLsus; lsu++) begin
          for (int rss = 0; rss < LsuNofRss; rss++) begin
            write_trace_event(file_id, trace_header, "dispatch",
                             format_lsu_trace(rss_lsu_traces[lsu][rss]),
                             rss_lsu_traces[lsu][rss].valid);
          end
        end
        for (int fpu = 0; fpu < NofFpus; fpu++) begin
          for (int rss = 0; rss < FpuNofRss; rss++) begin
            write_trace_event(file_id, trace_header, "dispatch",
                             format_fpu_trace(rss_fpu_traces[fpu][rss]),
                             rss_fpu_traces[fpu][rss].valid);
          end
        end

        if (RVV) begin
          for (int rss = 0; rss < SpatzNofRss; rss++) begin
              write_trace_event(file_id, trace_header, "dispatch",
                              format_spatz_trace(rss_spatz_traces[rss]),
                              rss_spatz_traces[rss].valid);
            end
        end

        // No CSR and ACC events possible
      end else begin
        $warning("Current CPU state (%s) not supported by tracer!",
           schnizo_pkg::loop_state_tostring(core_trace.state));
      end

      // Writeback events - We must consider all writebacks at all times.
      write_trace_event(file_id, trace_header, "writeback",
                        format_wb_fu_trace(alu_wb_trace, "ALU"),
                        alu_wb_trace.valid);
      write_trace_event(file_id, trace_header, "writeback",
                        format_wb_fu_trace(lsu_wb_trace, "LSU"),
                        lsu_wb_trace.valid);
      write_trace_event(file_id, trace_header, "writeback",
                        format_wb_fu_trace(fpu_wb_trace, "FPU"),
                        fpu_wb_trace.valid);
      if (RVV) write_trace_event(file_id, trace_header, "writeback",
                        format_wb_fu_trace(spatz_wb_trace, "SPATZ"),
                        spatz_wb_trace.valid);
      write_trace_event(file_id, trace_header, "writeback",
                        format_wb_fu_trace(csr_wb_trace, "CSR"),
                        csr_wb_trace.valid);
      write_trace_event(file_id, trace_header, "writeback",
                        format_wb_fu_trace(acc_wb_trace, "ACC"),
                        acc_wb_trace.valid);

      // Result capture events - We can always capture them
      // Retirement events - Always active to complete any issue.
      for (int alu = 0; alu < NofAlus; alu++) begin
        write_trace_event(file_id, trace_header, "retirement",
                          format_fu_retire_trace(alu_retirements[alu]),
                          alu_retirements[alu].valid);
        for (int rss = 0; rss < AluNofRss; rss++) begin
          write_trace_event(file_id, trace_header, "rescap",
                            format_rescap_trace(alu_rescap_traces[alu][rss]),
                            alu_rescap_traces[alu][rss].valid);
        end
      end
      for (int lsu = 0; lsu < NofLsus; lsu++) begin
        write_trace_event(file_id, trace_header, "retirement",
                          format_fu_retire_trace(lsu_retirements[lsu]),
                          lsu_retirements[lsu].valid);
        for (int rss = 0; rss < LsuNofRss; rss++) begin
          write_trace_event(file_id, trace_header, "rescap",
                            format_rescap_trace(lsu_rescap_traces[lsu][rss]),
                            lsu_rescap_traces[lsu][rss].valid);
        end
      end
      for (int fpu = 0; fpu < NofFpus; fpu++) begin
        write_trace_event(file_id, trace_header, "retirement",
                          format_fu_retire_trace(fpu_retirements[fpu]),
                          fpu_retirements[fpu].valid);
        for (int rss = 0; rss < FpuNofRss; rss++) begin
          write_trace_event(file_id, trace_header, "rescap",
                            format_rescap_trace(fpu_rescap_traces[fpu][rss]),
                            fpu_rescap_traces[fpu][rss].valid);
        end
      end

      if (RVV) begin
        for (int i = 0; i < NrParallelInstructions; i++) begin
                write_trace_event(file_id, trace_header, "internal_spatz_retirement",
                          format_int_spatz_retire_trace(spatz_retirement[i]),
                          spatz_retirement[i].valid);
        end



        for (int rss = 0; rss < SpatzNofRss; rss++) begin
          write_trace_event(file_id, trace_header, "rescap",
                            format_rescap_trace(spatz_rescap_traces[rss]),
                            spatz_rescap_traces[rss].valid);
        end
      end

      write_trace_event(file_id, trace_header, "retirement",
                        format_fu_retire_trace(csr_retirement),
                        csr_retirement.valid);
      write_trace_event(file_id, trace_header, "retirement",
                        format_fu_retire_trace(acc_retirement),
                        acc_retirement.valid);
    end else begin
      cycle = '0;
    end
  end

  final begin
    $fclose(file_id);
  end

endmodule

// pragma translate_on
