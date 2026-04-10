// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// TODO(colluca): the following would be best moved to the cluster level

`include "common_cells/assertions.svh"
`include "common_cells/registers.svh"
`include "snitch_vm/typedef.svh"

// The Schnizo core complex.
//
// A container for Schnizo and core-local accelerators such as the DMA, connected through the
// accelerator offload interface.
// It also forks data memory requests between SoC and TCDM.
// Finally, it collects all core and accelerator events.
module schnizo_cc #(
  /// Address width of the buses
  parameter int unsigned AddrWidth          = 0,
  /// Data width of the buses.
  parameter int unsigned DataWidth          = 0,
  /// Data width of the AXI DMA buses.
  parameter int unsigned DMADataWidth       = 0,
  /// Id width of the AXI DMA bus.
  parameter int unsigned DMAIdWidth         = 0,
  /// User width of the AXI DMA bus.
  parameter int unsigned DMAUserWidth       = 0,
  parameter int unsigned DMANumAxInFlight   = 0,
  parameter int unsigned DMAReqFifoDepth    = 0,
  parameter int unsigned DMANumChannels     = 0,
  /// Data port request type.
  parameter type         dreq_t             = logic,
  /// Data port response type.
  parameter type         drsp_t             = logic,
  // TCDM Types for Spatz
  parameter type          tcdm_req_chan_t  =  logic,
  parameter type          tcdm_rsp_chan_t  =  logic,
  /// TCDM Address Width
  parameter int unsigned TCDMAddrWidth      = 0,
  /// Data port request type.
  parameter type         tcdm_req_t         = logic,
  /// Data port response type.
  parameter type         tcdm_rsp_t         = logic,
  /// TCDM User Payload
  parameter type         tcdm_user_t        = logic,
  parameter type         axi_ar_chan_t      = logic,
  parameter type         axi_aw_chan_t      = logic,
  parameter type         axi_req_t          = logic,
  parameter type         axi_rsp_t          = logic,
  parameter type         hive_req_t         = logic,
  parameter type         hive_rsp_t         = logic,
  parameter type         acc_req_t          = logic,
  parameter type         acc_resp_t         = logic,
  parameter type         dma_events_t       = logic,
  parameter fpnew_pkg::fpu_implementation_t FPUImplementation = '0,
  /// Boot address of core.
  parameter logic [31:0] BootAddr           = 32'h0000_1000,
  // Enable V extension for Spatz
  parameter bit          RVV                = 1,
  /// Enable F and D Extension
  parameter bit          RVF                = 1,
  parameter bit          RVD                = 1,
  parameter bit          XF8                = 0,
  parameter bit          XF8ALT             = 0,
  parameter bit          XF16               = 0,
  parameter bit          XF16ALT            = 0,
  parameter bit          XFVEC              = 0,
  parameter bit          XFDOTP             = 0,
  /// Enable Snitch DMA
  parameter bit          Xdma               = 0,
  /// Has `frep` support. For Schnizo this is the superscalar feature.
  parameter bit          Xfrep              = 0,
  /// Xfrep config
  parameter int unsigned NumAlus            = 3,
  parameter int unsigned NumLsus            = 3,
  parameter int unsigned NumFpus            = 1,
  parameter int unsigned NumAluRss          = 3,
  parameter int unsigned NumLsuRss          = 2,
  parameter int unsigned NumFpuRss          = 4,
  parameter int unsigned NumAluRspPorts     = 1,
  parameter int unsigned NumLsuRspPorts     = 1,
  parameter int unsigned NumFpuRspPorts     = 1,
  // Spatz RSS config
  parameter int unsigned NumSpatzRss        = 6,
  // LSU parameters
  parameter int unsigned NumIntOutstandingLoads = 0,
  parameter int unsigned NumIntOutstandingMem   = 0,
  // SPATZ specific parameters
  parameter int unsigned NumSpatzFPUs           = 4,
  parameter int unsigned NumSpatzIPUs           = 1,
  /// Derived parameter for Spatz *Do not override*
  parameter int unsigned NumSpatzFUs            = (NumSpatzFPUs > NumSpatzIPUs) ? NumSpatzFPUs : NumSpatzIPUs,
  parameter int unsigned NumMemPortsPerSpatz    = NumSpatzFUs,
  /// Add isochronous clock-domain crossings e.g., make it possible to operate
  /// the core in a slower clock domain.
  parameter bit          IsoCrossing        = 0,
  /// Timing Parameters
  /// Insert Pipeline registers into off-loading path (request)
  parameter bit          RegisterOffloadReq = 0,
  /// Insert Pipeline registers into off-loading path (response)
  parameter bit          RegisterOffloadRsp = 0,
  /// Insert Pipeline registers into data memory path (request)
  parameter bit          RegisterCoreReq    = 0,
  /// Insert Pipeline registers into data memory path (response)
  parameter bit          RegisterCoreRsp    = 0,
  /// Insert Pipeline registers immediately before FPU datapath
  parameter bit          RegisterFPUIn      = 0,
  /// Insert Pipeline registers immediately after FPU datapath
  parameter bit          RegisterFPUOut     = 0,
  parameter snitch_pma_pkg::snitch_pma_t SnitchPMACfg = '{default: 0},
  /// Consistency Address Queue (CAQ) parameters.
  parameter int unsigned CaqDepth     = 0,
  parameter int unsigned CaqTagWidth  = 0,
  /// Enable debug support.
  parameter bit          DebugSupport = 1,
  /// Optional fixed TCDM alias.
  parameter bit          TCDMAliasEnable = 1'b0,
  parameter logic [AddrWidth-1:0] TCDMAliasStart  = '0,
  /// Derived parameter Spatz *Do not override*
  parameter int unsigned TCDMPorts = RVV ? NumMemPortsPerSpatz + NumLsus : NumLsus,
  localparam type addr_t = logic [AddrWidth-1:0],
  localparam type data_t = logic [DataWidth-1:0]
) (
  input  logic                             clk_i,
  input  logic                             clk_d2_i,
  input  logic                             rst_ni,
  input  logic                             rst_int_ss_ni,
  input  logic                             rst_fp_ss_ni,
  input  logic [31:0]                      hart_id_i,
  input  snitch_pkg::interrupts_t          irq_i,
  output hive_req_t                        hive_req_o,
  input  hive_rsp_t                        hive_rsp_i,
  // Core data ports
  output dreq_t                            data_req_o,
  input  drsp_t                            data_rsp_i,
  // TCDM Streamer Ports
  output tcdm_req_t [TCDMPorts-1:0]        tcdm_req_o,
  input  tcdm_rsp_t [TCDMPorts-1:0]        tcdm_rsp_i,
  // DMA ports
  output axi_req_t    [DMANumChannels-1:0] axi_dma_req_o,
  input  axi_rsp_t    [DMANumChannels-1:0] axi_dma_res_i,
  output logic        [DMANumChannels-1:0] axi_dma_busy_o,
  output dma_events_t [DMANumChannels-1:0] axi_dma_events_o,
  // Core event strobes
  output snitch_pkg::core_events_t         core_events_o,
  input  addr_t                            tcdm_addr_base_i,
  // Cluster HW barrier
  output logic                             barrier_o,
  input  logic                             barrier_i
);

  //////////////////////////
  // Parameters and types //
  //////////////////////////

  // FMA architecture is "merged" -> mulexp and macexp instructions are supported
  localparam bit XFauxMerged  = (FPUImplementation.UnitTypes[3] == fpnew_pkg::MERGED);
  localparam bit FPEn = RVF | RVD | XF16 | XF16ALT | XF8 | XF8ALT | XFVEC | XFauxMerged | XFDOTP;
  localparam int unsigned FLEN = RVD     ? 64 : // D ext.
                                 RVF     ? 32 : // F ext.
                                 XF16    ? 16 : // Xf16 ext.
                                 XF16ALT ? 16 : // Xf16alt ext.
                                 XF8     ?  8 :  // Xf8 ext.
                                 XF8ALT  ?  8 :  // Xf8alt ext.
                                            0;             // Unused in case of no FP

                                              // Schnizo Memory requests
  // For now we misuse the NumSsrs parameter to define the number of LSUs. This allows to keep
  // the CC interface the same as the Snitch CC. The NumSsrs defines also the number of TCDM
  // interfaces.
  // Note that NumSsrs=0 and =1 result in only one LSU whereas in Snitch the core and FP SS have
  // their own LSU. However, these LSUs share a TCDM interface. As of this we achieve almost the
  // same performance with only one LSU. The difference is that Snitch can "buffer" more memory
  // requests in the LSU queue (more outstanding transactions).
  // TODO: create parameter for number of FUs.
  localparam int unsigned NofLsus               = NumLsus;
  // Request buffering & cuts
  // For a first implementation we use a buffer depth of 4.
  // This is as similar to the Snitch as possible.
  // The depth of the demux deciding whether to go to the TCDM or SoC.
  localparam int unsigned RespDepthTcdmOrSoc    = 4;
  // Whether to cut the memory requests going to the SoC.
  localparam bit          RegisterToSocReq      = 0;
  // The depth of the MUX merging all requests going to the SoC.
  localparam int unsigned RespDepthToSoc        = 4;
  // The depth of the reqrsp to TCDM conversion
  localparam int unsigned RespDepthReqrspToTcdm = 4;

  localparam int unsigned SelectWidth = cf_math_pkg::idx_width(2);
  typedef logic [SelectWidth-1:0] select_t;

  typedef struct packed {
    int unsigned idx;
    logic [AddrWidth-1:0] base;
    logic [AddrWidth-1:0] mask;
  } reqrsp_rule_t;

  `SNITCH_VM_TYPEDEF(AddrWidth)

  /////////////////
  // Connections //
  /////////////////

  acc_req_t acc_schnizo_req;
  acc_req_t acc_schnizo_demux;
  acc_req_t acc_schnizo_demux_q;
  acc_resp_t acc_demux_schnizo;
  acc_resp_t acc_demux_schnizo_q;
  acc_resp_t dma_resp;

  logic acc_schnizo_demux_qvalid,   acc_schnizo_demux_qready;
  logic acc_schnizo_demux_qvalid_q, acc_schnizo_demux_qready_q;
  logic dma_qvalid, dma_qready;

  logic dma_pvalid, dma_pready;
  logic acc_demux_snitch_valid,   acc_demux_snitch_ready;
  logic acc_demux_snitch_valid_q, acc_demux_snitch_ready_q;

  snitch_pkg::core_events_t schnizo_events;

  dreq_t [NofLsus-1:0] schnizo_dreq;
  drsp_t [NofLsus-1:0] schnizo_drsp;

  // Spatz TDCM Interface
  tcdm_req_t [NumMemPortsPerSpatz - 1 : 0] spatz_tcdm_req;
  tcdm_rsp_t [NumMemPortsPerSpatz - 1 : 0] spatz_tcdm_rsp;

  if (RVV) begin
    assign tcdm_req_o[TCDMPorts-1 : NumLsus] = spatz_tcdm_req;
    assign spatz_tcdm_rsp = tcdm_rsp_i[TCDMPorts-1 : NumLsus];
  end

  /////////////
  // Schnizo //
  /////////////

  schnizo #(
    .BootAddr              (BootAddr),
    .AddrWidth             (AddrWidth),
    .DataWidth             (DataWidth),
    .Xdma                  (Xdma),
    .Xfrep                 (Xfrep),
    .RVF                   (RVF),
    .RVD                   (RVD),
    .XF16                  (XF16),
    .XF16ALT               (XF16ALT),
    .XF8                   (XF8),
    .XF8ALT                (XF8ALT),
    .XFVEC                 (XFVEC),
    .FLEN                  (FLEN),
    .dreq_t                (dreq_t),
    .drsp_t                (drsp_t),
    // Spatz TCDM Interface
    .tcdm_req_chan_t      (tcdm_req_chan_t),
    .tcdm_rsp_chan_t      (tcdm_rsp_chan_t),
    .tcdm_req_t           (tcdm_req_t),
    .tcdm_rsp_t           (tcdm_rsp_t),

    .acc_req_t             (acc_req_t),
    .acc_resp_t            (acc_resp_t),
    // FU configuration
    .NofAlus               (NumAlus),
    .NofLsus               (NumLsus),
    .NofFpus               (NumFpus),
    .AluNofRss             (NumAluRss),
    .LsuNofRss             (NumLsuRss),
    .FpuNofRss             (NumFpuRss),
    .AluNofResRspPorts     (NumAluRspPorts),
    .LsuNofResRspPorts     (NumLsuRspPorts),
    .FpuNofResRspPorts     (NumFpuRspPorts),
    .NumOutstandingLoads   (NumIntOutstandingLoads), // Use the int value for all LSUs
    .NumOutstandingMem     (NumIntOutstandingMem),
    .SnitchPMACfg          (SnitchPMACfg),
    .CaqDepth              (CaqDepth),
    .CaqTagWidth           (CaqTagWidth),
    .DebugSupport          (DebugSupport),
    .FPUImplementation     (FPUImplementation),
    .RegisterFPUIn         (RegisterFPUIn),
    .RegisterFPUOut        (RegisterFPUOut),
    .RVV                   (RVV),
    .NumSpatzFPUs          (NumSpatzFPUs),
    .NumSpatzIPUs          (NumSpatzIPUs)
  ) i_schnizo (
    .clk_i           (clk_d2_i), // if necessary operate on half the frequency
    .rst_i           (~rst_ni),
    .hart_id_i,
    .irq_i,
    .flush_i_valid_o (hive_req_o.flush_i_valid),
    .flush_i_ready_i (hive_rsp_i.flush_i_ready),
    .inst_addr_o     (hive_req_o.inst_addr),
    .inst_cacheable_o(hive_req_o.inst_cacheable),
    .inst_data_i     (hive_rsp_i.inst_data),
    .inst_valid_o    (hive_req_o.inst_valid),
    .inst_ready_i    (hive_rsp_i.inst_ready),
    .acc_qreq_o      (acc_schnizo_demux),
    .acc_qvalid_o    (acc_schnizo_demux_qvalid),
    .acc_qready_i    (acc_schnizo_demux_qready),
    .acc_prsp_i      (acc_demux_schnizo),
    .acc_pvalid_i    (acc_demux_snitch_valid),
    .acc_pready_o    (acc_demux_snitch_ready),
    .data_req_o      (schnizo_dreq),
    .data_rsp_i      (schnizo_drsp),
    .core_events_o   (schnizo_events),
    .barrier_o       (barrier_o),
    .barrier_i       (barrier_i),
    // TCDM Ports for SPATZ
    .tcdm_req_o      (spatz_tcdm_req),
    .tcdm_rsp_i      (spatz_tcdm_rsp)
  );

  /////////////////////////////////
  // Accelerator offloading path //
  /////////////////////////////////

  // Cut off-loading request path
  isochronous_spill_register #(
    .T      (acc_req_t),
    .Bypass (!IsoCrossing && !RegisterOffloadReq)
  ) i_spill_register_acc_demux_req (
    .src_clk_i  (clk_d2_i),
    .src_rst_ni (rst_ni),
    .src_valid_i(acc_schnizo_demux_qvalid),
    .src_ready_o(acc_schnizo_demux_qready),
    .src_data_i (acc_schnizo_demux),
    .dst_clk_i  (clk_i),
    .dst_rst_ni (rst_ni),
    .dst_valid_o(acc_schnizo_demux_qvalid_q),
    .dst_ready_i(acc_schnizo_demux_qready_q),
    .dst_data_o (acc_schnizo_demux_q)
  );

  // Cut off-loading response path
  isochronous_spill_register #(
    .T     (acc_resp_t),
    .Bypass(!IsoCrossing && !RegisterOffloadRsp)
  ) i_spill_register_acc_demux_resp (
    .src_clk_i  (clk_i),
    .src_rst_ni (rst_ni),
    .src_valid_i(acc_demux_snitch_valid_q),
    .src_ready_o(acc_demux_snitch_ready_q),
    .src_data_i (acc_demux_schnizo_q),
    .dst_clk_i  (clk_d2_i),
    .dst_rst_ni (rst_ni),
    .dst_valid_o(acc_demux_snitch_valid),
    .dst_ready_i(acc_demux_snitch_ready),
    .dst_data_o (acc_demux_schnizo)
  );

  // Accelerator Demux Port
  // The new demux with the schnizo enum. We can use it yet.
  // stream_demux #(
  //   .N_OUP(schnizo_pkg::NumAccelerators)
  // ) i_stream_demux_offload (
  //   .inp_valid_i(acc_schnizo_demux_qvalid_q),
  //   .inp_ready_o(acc_schnizo_demux_qready_q),
  //   .oup_sel_i  (acc_schnizo_demux_q.addr[$clog2(schnizo_pkg::NumAccelerators)-1:0]),
  //   .oup_valid_o({dma_qvalid, hive_req_o.acc_qvalid}),
  //   .oup_ready_i({dma_qready, hive_rsp_i.acc_qready})
  // );
  // TODO: We must use the snitch_pkg::acc_addr_e enum. Otherwise we must adapt the whole cluster.
  logic ssr_qvalid, ipu_qvalid, acc_qvalid; // All signals are unused
  logic ssr_qready, ipu_qready, acc_qready; // All signals are unused
  stream_demux #(
    .N_OUP(5)
  ) i_stream_demux_offload (
    .inp_valid_i(acc_schnizo_demux_qvalid_q),
    .inp_ready_o(acc_schnizo_demux_qready_q),
    .oup_sel_i  (acc_schnizo_demux_q.addr[$clog2(5)-1:0]),
    .oup_valid_o({ssr_qvalid, ipu_qvalid, dma_qvalid, hive_req_o.acc_qvalid, acc_qvalid}),
    .oup_ready_i({ssr_qready, ipu_qready, dma_qready, hive_rsp_i.acc_qready, acc_qready})
  );

  // To shared muldiv
  assign hive_req_o.acc_req = acc_schnizo_demux_q;
  // Internal accelerators (only DMA in this case)
  assign acc_schnizo_req = acc_schnizo_demux_q;

  stream_arbiter #(
    .DATA_T(acc_resp_t),
    .N_INP (schnizo_pkg::NumAccelerators)
  ) i_stream_arbiter_offload (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .inp_data_i ({dma_resp,   hive_rsp_i.acc_resp  }),
    .inp_valid_i({dma_pvalid, hive_rsp_i.acc_pvalid}),
    .inp_ready_o({dma_pready, hive_req_o.acc_pready}),
    .oup_data_o (acc_demux_schnizo_q),
    .oup_valid_o(acc_demux_snitch_valid_q),
    .oup_ready_i(acc_demux_snitch_ready_q)
  );

  /////////
  // DMA //
  /////////

  if (Xdma) begin : gen_dma
    idma_inst64_top #(
      .AxiAddrWidth   (AddrWidth),
      .AxiDataWidth   (DMADataWidth),
      .AxiIdWidth     (DMAIdWidth),
      .AxiUserWidth   (DMAUserWidth),
      .NumAxInFlight  (DMANumAxInFlight),
      .DMAReqFifoDepth(DMAReqFifoDepth),
      .NumChannels    (DMANumChannels),
      .DMATracing     (1),
      .axi_ar_chan_t  (axi_ar_chan_t),
      .axi_aw_chan_t  (axi_aw_chan_t),
      .axi_req_t      (axi_req_t),
      .axi_res_t      (axi_rsp_t),
      .acc_req_t      (acc_req_t),
      .acc_res_t      (acc_resp_t),
      .dma_events_t   (dma_events_t)
    ) i_idma_inst64_top (
      .clk_i,
      .rst_ni,
      .testmode_i     (1'b0),
      .axi_req_o      (axi_dma_req_o),
      .axi_res_i      (axi_dma_res_i),
      .busy_o         (axi_dma_busy_o),
      .acc_req_i      (acc_schnizo_req),
      .acc_req_valid_i(dma_qvalid),
      .acc_req_ready_o(dma_qready),
      .acc_res_o      (dma_resp),
      .acc_res_valid_o(dma_pvalid),
      .acc_res_ready_i(dma_pready),
      .hart_id_i      (hart_id_i),
      .events_o       (axi_dma_events_o)
    );

  // no DMA instanciated
  end else begin : gen_no_dma
    // tie-off unused signals
    assign axi_dma_req_o    = '0;
    assign axi_dma_busy_o   = '0;
    assign dma_qready       = '0;
    assign dma_resp         = '0;
    assign dma_pvalid       = '0;
    assign axi_dma_events_o = '0;
  end

  /////////////////////////
  // Memory request path //
  /////////////////////////

  reqrsp_rule_t [TCDMAliasEnable:0] addr_map;
  assign addr_map[0] = '{
    idx: 1,
    base: tcdm_addr_base_i,
    mask: ({AddrWidth{1'b1}} << TCDMAddrWidth)
  };
  if (TCDMAliasEnable) begin : gen_tcdm_alias_rule
    assign addr_map[1] = '{
      idx: 1,
      base: TCDMAliasStart,
      mask: ({AddrWidth{1'b1}} << TCDMAddrWidth)
    };
  end

  dreq_t [NofLsus-1:0] to_soc_req;
  drsp_t [NofLsus-1:0] to_soc_rsp;

  // For each LSU we cut the path and decide whether to go to the SoC or TCDM.
  // Finally we convert the request to a TCDM request and connect it to the interface.
  for (genvar lsu = 0; lsu < NofLsus; lsu++) begin : gen_cut_dreq_drsp
    dreq_t     schnizo_dreq_cut;
    drsp_t     schnizo_drsp_cut;
    dreq_t     to_tcdm_req;
    drsp_t     to_tcdm_rsp;
    tcdm_req_t tcdm_req;
    tcdm_rsp_t tcdm_rsp;

    // Cut req & rsp from core to address decision
    reqrsp_iso #(
      .AddrWidth(AddrWidth),
      .DataWidth(DataWidth),
      .UserWidth(64),
      .req_t    (dreq_t),
      .rsp_t    (drsp_t),
      .BypassReq(!RegisterCoreReq),
      .BypassRsp(!IsoCrossing && !RegisterCoreRsp)
    ) i_cut_schnizo_dreq (
      .src_clk_i (clk_d2_i),
      .src_rst_ni(rst_ni),
      .src_req_i (schnizo_dreq[lsu]),
      .src_rsp_o (schnizo_drsp[lsu]),
      .dst_clk_i (clk_i),
      .dst_rst_ni(rst_ni),
      .dst_req_o (schnizo_dreq_cut),
      .dst_rsp_i (schnizo_drsp_cut)
    );

    // Decide whether to go to SoC or TCDM
    select_t slave_select;
    addr_decode_napot #(
      .NoIndices(2),
      .NoRules  (1 + TCDMAliasEnable),
      .addr_t   (logic [AddrWidth-1:0]),
      .rule_t   (reqrsp_rule_t)
    ) i_addr_decode_napot_tcdm_soc (
      .addr_i          (schnizo_dreq_cut.q.addr),
      .addr_map_i      (addr_map),
      .idx_o           (slave_select),
      .dec_valid_o     (),
      .dec_error_o     (),
      .en_default_idx_i(1'b1),
      .default_idx_i   ('0)
    );

    reqrsp_demux #(
      .NrPorts  (2),
      .req_t    (dreq_t),
      .rsp_t    (drsp_t),
      .RespDepth(RespDepthTcdmOrSoc)
    ) i_reqrsp_demux_tcdm_soc (
      .clk_i,
      .rst_ni,
      .slv_select_i(slave_select),
      .slv_req_i   (schnizo_dreq_cut),
      .slv_rsp_o   (schnizo_drsp_cut),
      .mst_req_o   ({to_tcdm_req, to_soc_req[lsu]}),
      .mst_rsp_i   ({to_tcdm_rsp, to_soc_rsp[lsu]})
    );

    // Convert the request to a TCDM request
    reqrsp_to_tcdm #(
      .AddrWidth   (AddrWidth),
      .DataWidth   (DataWidth),
      .UserWidth   (64),
      .BufDepth    (RespDepthReqrspToTcdm),
      .reqrsp_req_t(dreq_t),
      .reqrsp_rsp_t(drsp_t),
      .tcdm_req_t  (tcdm_req_t),
      .tcdm_rsp_t  (tcdm_rsp_t)
    ) i_reqrsp_to_tcdm (
      .clk_i,
      .rst_ni,
      .reqrsp_req_i(to_tcdm_req),
      .reqrsp_rsp_o(to_tcdm_rsp),
      .tcdm_req_o  (tcdm_req),
      .tcdm_rsp_i  (tcdm_rsp)
    );

    // Connect to TCDM outputs. This works as long as NofLsus == NofTcdmInterfaces.
    assign tcdm_req_o[lsu] = tcdm_req;
    assign tcdm_rsp        = tcdm_rsp_i[lsu];
  end

  // Merge all requests going to the SoC into one
  reqrsp_mux #(
    .NrPorts    (NofLsus),
    .AddrWidth  (AddrWidth),
    .DataWidth  (DataWidth),
    .UserWidth  (64),
    .req_t      (dreq_t),
    .rsp_t      (drsp_t),
    .RespDepth  (RespDepthToSoc),
    .RegisterReq({NofLsus{RegisterToSocReq}})
  ) i_reqrsp_mux_to_soc (
    .clk_i,
    .rst_ni,
    .slv_req_i(to_soc_req),
    .slv_rsp_o(to_soc_rsp),
    .mst_req_o(data_req_o),
    .mst_rsp_i(data_rsp_i),
    .idx_o    ()
  );

  /////////////////
  // Core events //
  /////////////////

  // Core events for performance counters
  assign core_events_o.retired_instr = schnizo_events.retired_instr;
  assign core_events_o.retired_load  = schnizo_events.retired_load;
  assign core_events_o.retired_i     = schnizo_events.retired_i;
  assign core_events_o.retired_acc   = schnizo_events.retired_acc;
  // TODO: Rework the FPU core events. It does not exactly match the Snitch values.
  // See schnizo for more details.
  assign core_events_o.issue_fpu         = schnizo_events.issue_fpu;
  assign core_events_o.issue_fpu_seq     = schnizo_events.issue_fpu_seq;
  assign core_events_o.issue_core_to_fpu = schnizo_events.issue_core_to_fpu;

  ////////////////
  // Assertions //
  ////////////////
  // This Asser does not work with Spatz
  // `ASSERT_INIT(
  //   TcdmAndLsuInterfacesMatch,
  //   NofLsus==TCDMPorts,
  //   "The number of LSU does not match the number of TCDM ports."
  // );

  `ASSERT_INIT(BootAddrAligned, BootAddr[1:0] == 2'b00)

endmodule
