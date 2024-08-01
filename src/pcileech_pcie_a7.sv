//
// PCILeech FPGA.
//
// PCIe module for Artix-7.
//
// (c) Ulf Frisk, 2018-2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_pcie_a7(
    input                   clk_sys,
    input                   rst,

    // PCIe fabric
    output  [0:0]           pcie_tx_p,
    output  [0:0]           pcie_tx_n,
    input   [0:0]           pcie_rx_p,
    input   [0:0]           pcie_rx_n,
    input                   pcie_clk_p,
    input                   pcie_clk_n,
    input                   pcie_perst_n,
    
    // State and Activity LEDs
    output                  led_state,
    
    // PCIe <--> FIFOs
    IfPCIeFifoCfg.mp_pcie   dfifo_cfg,
    IfPCIeFifoTlp.mp_pcie   dfifo_tlp,
    IfPCIeFifoCore.mp_pcie  dfifo_pcie,
    IfShadow2Fifo.shadow    dshadow2fifo
    );
       
    // ----------------------------------------------------------------------------
    // PCIe DEFINES AND WIRES
    // ----------------------------------------------------------------------------
    
    IfPCIeSignals           ctx();
    IfPCIeTlpRxTx           tlp_tx();
    IfPCIeTlpRxTx           tlp_rx();
    IfAXIS128               tlps_tx();
    IfAXIS128               tlps_rx();
    
    IfAXIS128               tlps_static();       // static tlp transmit from cfg->tlp
    wire [15:0]             pcie_id;
    wire                    user_lnk_up;
    
    // system interface
    wire pcie_clk_c;
    wire clk_pcie;
    wire rst_pcie_user;
    wire rst_subsys = rst || rst_pcie_user || dfifo_pcie.pcie_rst_subsys;
    wire rst_pcie = rst || ~pcie_perst_n || dfifo_pcie.pcie_rst_core;
       
    // Buffer for differential system clock
    IBUFDS_GTE2 refclk_ibuf (.O(pcie_clk_c), .ODIV2(), .I(pcie_clk_p), .CEB(1'b0), .IB(pcie_clk_n));
    
    // ----------------------------------------------------
    // TickCount64 PCIe REFCLK and LED OUTPUT
    // ----------------------------------------------------

    time tickcount64_pcie_refclk = 0;
    always @ ( posedge pcie_clk_c )
        tickcount64_pcie_refclk <= tickcount64_pcie_refclk + 1;
    assign led_state = user_lnk_up || tickcount64_pcie_refclk[25];
    
    // ----------------------------------------------------------------------------
    // PCIe CFG RX/TX <--> FIFO below
    // ----------------------------------------------------------------------------
    wire [31:0] base_address_register;
    
    pcileech_pcie_cfg_a7 i_pcileech_pcie_cfg_a7(
        .rst                        ( rst_subsys                ),
        .clk_sys                    ( clk_sys                   ),
        .clk_pcie                   ( clk_pcie                  ),
        .dfifo                      ( dfifo_cfg                 ),        
        .ctx                        ( ctx                       ),
        .tlps_static                ( tlps_static.source        ),
        .pcie_id                    ( pcie_id                   ),  // -> [15:0]
        .base_address_register      ( base_address_register     )
    );
    
    // ----------------------------------------------------------------------------
    // PCIe TLP RX/TX <--> FIFO below
    // ----------------------------------------------------------------------------
    
    pcileech_tlps128_src64 i_pcileech_tlps128_src64(
        .rst                        ( rst_subsys                ),
        .clk_pcie                   ( clk_pcie                  ),
        .tlp_rx                     ( tlp_rx.sink               ),
        .tlps_out                   ( tlps_rx.source_lite       )
    );
    
    pcileech_pcie_tlp_a7 i_pcileech_pcie_tlp_a7(
        .rst                        ( rst_subsys                ),
        .clk_pcie                   ( clk_pcie                  ),
        .clk_sys                    ( clk_sys                   ),
        .dfifo                      ( dfifo_tlp                 ),
        .tlps_tx                    ( tlps_tx.source            ),       
        .tlps_rx                    ( tlps_rx.sink_lite         ),
        .tlps_static                ( tlps_static.sink          ),
        .dshadow2fifo               ( dshadow2fifo              ),
        .pcie_id                    ( pcie_id                   ),  // <- [15:0]
        .base_address_register      ( base_address_register     )
    );
    
    pcileech_tlps128_dst64 i_pcileech_tlps128_dst64(
        .rst                        ( rst                       ),
        .clk_pcie                   ( clk_pcie                  ),
        .tlp_tx                     ( tlp_tx.source             ),
        .tlps_in                    ( tlps_tx.sink              )
    );
    
    nic i_nic (
        .pcie_clk		   	(pcie_clk),
        .pcie_rst_n			(pcie_rst_n),
        .s_axis_tx_tdata		(s_axis_tx_tdata),
        .s_axis_tx_tkeep		(s_axis_tx_tkeep),
        .s_axis_tx_tlast		(s_axis_tx_tlast),
        .s_axis_tx_tready		(s_axis_tx_tready),
        .s_axis_tx_tvalid		(s_axis_tx_tvalid),
        .m_axis_rx_tdata		(m_axis_rx_tdata),
        .m_axis_rx_tkeep		(m_axis_rx_tkeep),
        .m_axis_rx_tlast		(m_axis_rx_tlast),
        .m_axis_rx_tready		(m_axis_rx_tready),
        .m_axis_rx_tvalid		(m_axis_rx_tvalid),
        .cfg_mgmt_dwaddr		(cfg_mgmt_dwaddr),
        .cfg_mgmt_byte_en		(cfg_mgmt_byte_en),
        .cfg_mgmt_do			(cfg_mgmt_do),
        .cfg_mgmt_rd_en			(cfg_mgmt_rd_en),
        .cfg_mgmt_rd_wr_done		(cfg_mgmt_rd_wr_done),
        .cfg_mgmt_wr_en			(cfg_mgmt_wr_en),
        .cfg_mgmt_di			(cfg_mgmt_di),
        .cfg_interrupt			(cfg_interrupt),
        .cfg_interrupt_rdy		(cfg_interrupt_rdy),
        .phy_tx_clk			(phy_tx_clk),
        .phy_txd			(phy_txd),
        .phy_tx_en			(phy_tx_en),
        .phy_tx_er			(phy_tx_er),
        .phy_rx_clk			(phy_rx_clk),
        .phy_rxd			(phy_rxd),
        .phy_rx_dv			(phy_rx_dv),
        .phy_rx_er			(phy_rx_er),
        .phy_reset_n			(phy_reset_n),
        .phy_pause_req			(phy_pause_req),
        .phy_pause_ack			(phy_pause_ack),
        .tx_packet_count		(tx_packet_count),
        .rx_packet_count		(rx_packet_count),
        .rx_error_count			(rx_error_count),
        .tx_error_count			(tx_error_count),
        .rx_dropped_count		(rx_dropped_count),
        .vlan_tagged_count		(vlan_tagged_count),
        .ipv6_packet_count		(ipv6_packet_count),
        .multicast_packet_count		(multicast_packet_count)
    );
    // might need a few adjustments or things deleted czs of things that are already defined.

    // ----------------------------------------------------------------------------
    // PCIe CORE BELOW
    // ---------------------------------------------------------------------------- 
      
    pcie_7x_0 i_pcie_7x_0 (
        // pcie_7x_mgt
        .pci_exp_txp                ( pcie_tx_p                 ),  // ->
        .pci_exp_txn                ( pcie_tx_n                 ),  // ->
        .pci_exp_rxp                ( pcie_rx_p                 ),  // <-
        .pci_exp_rxn                ( pcie_rx_n                 ),  // <-
        .sys_clk                    ( pcie_clk_c                ),  // <-
        .sys_rst_n                  ( ~rst_pcie                 ),  // <-
    
        // s_axis_tx (transmit data)
        .s_axis_tx_tdata            ( tlp_tx.data               ),  // <- [63:0]
        .s_axis_tx_tkeep            ( tlp_tx.keep               ),  // <- [7:0]
        .s_axis_tx_tlast            ( tlp_tx.last               ),  // <-
        .s_axis_tx_tready           ( tlp_tx.ready              ),  // ->
        .s_axis_tx_tuser            ( 4'b0                      ),  // <- [3:0]
        .s_axis_tx_tvalid           ( tlp_tx.valid              ),  // <-
    
        // s_axis_rx (receive data)
        .m_axis_rx_tdata            ( tlp_rx.data               ),  // -> [63:0]
        .m_axis_rx_tkeep            ( tlp_rx.keep               ),  // -> [7:0]
        .m_axis_rx_tlast            ( tlp_rx.last               ),  // -> 
        .m_axis_rx_tready           ( tlp_rx.ready              ),  // <-
        .m_axis_rx_tuser            ( tlp_rx.user               ),  // -> [21:0]
        .m_axis_rx_tvalid           ( tlp_rx.valid              ),  // ->
    
        // pcie_cfg_mgmt
        .cfg_mgmt_dwaddr            ( ctx.cfg_mgmt_dwaddr       ),  // <- [9:0]
        .cfg_mgmt_byte_en           ( ctx.cfg_mgmt_byte_en      ),  // <- [3:0]
        .cfg_mgmt_do                ( ctx.cfg_mgmt_do           ),  // -> [31:0]
        .cfg_mgmt_rd_en             ( ctx.cfg_mgmt_rd_en        ),  // <-
        .cfg_mgmt_rd_wr_done        ( ctx.cfg_mgmt_rd_wr_done   ),  // ->
        .cfg_mgmt_wr_readonly       ( ctx.cfg_mgmt_wr_readonly  ),  // <-
        .cfg_mgmt_wr_rw1c_as_rw     ( ctx.cfg_mgmt_wr_rw1c_as_rw ), // <-
        .cfg_mgmt_di                ( ctx.cfg_mgmt_di           ),  // <- [31:0]
        .cfg_mgmt_wr_en             ( ctx.cfg_mgmt_wr_en        ),  // <-
    
        // special core config
        //.pcie_cfg_vend_id           ( dfifo_pcie.pcie_cfg_vend_id       ),  // <- [15:0]
        //.pcie_cfg_dev_id            ( dfifo_pcie.pcie_cfg_dev_id        ),  // <- [15:0]
        //.pcie_cfg_rev_id            ( dfifo_pcie.pcie_cfg_rev_id        ),  // <- [7:0]
        //.pcie_cfg_subsys_vend_id    ( dfifo_pcie.pcie_cfg_subsys_vend_id ), // <- [15:0]
        //.pcie_cfg_subsys_id         ( dfifo_pcie.pcie_cfg_subsys_id     ),  // <- [15:0]
    
        // pcie2_cfg_interrupt
        .cfg_interrupt_assert       ( ctx.cfg_interrupt_assert          ),  // <-
        .cfg_interrupt              ( ctx.cfg_interrupt                 ),  // <-
        .cfg_interrupt_mmenable     ( ctx.cfg_interrupt_mmenable        ),  // -> [2:0]
        .cfg_interrupt_msienable    ( ctx.cfg_interrupt_msienable       ),  // ->
        .cfg_interrupt_msixenable   ( ctx.cfg_interrupt_msixenable      ),  // ->
        .cfg_interrupt_msixfm       ( ctx.cfg_interrupt_msixfm          ),  // ->
        .cfg_pciecap_interrupt_msgnum ( ctx.cfg_pciecap_interrupt_msgnum ), // <- [4:0]
        .cfg_interrupt_rdy          ( ctx.cfg_interrupt_rdy             ),  // ->
        .cfg_interrupt_do           ( ctx.cfg_interrupt_do              ),  // -> [7:0]
        .cfg_interrupt_stat         ( ctx.cfg_interrupt_stat            ),  // <-
        .cfg_interrupt_di           ( ctx.cfg_interrupt_di              ),  // <- [7:0]
        
        // pcie2_cfg_control
        .cfg_ds_bus_number          ( ctx.cfg_bus_number                ),  // <- [7:0]
        .cfg_ds_device_number       ( ctx.cfg_device_number             ),  // <- [4:0]
        .cfg_ds_function_number     ( ctx.cfg_function_number           ),  // <- [2:0]
        .cfg_dsn                    ( ctx.cfg_dsn                       ),  // <- [63:0]
        .cfg_pm_force_state         ( ctx.cfg_pm_force_state            ),  // <- [1:0]
        .cfg_pm_force_state_en      ( ctx.cfg_pm_force_state_en         ),  // <-
        .cfg_pm_halt_aspm_l0s       ( ctx.cfg_pm_halt_aspm_l0s          ),  // <-
        .cfg_pm_halt_aspm_l1        ( ctx.cfg_pm_halt_aspm_l1           ),  // <-
        .cfg_pm_send_pme_to         ( ctx.cfg_pm_send_pme_to            ),  // <-
        .cfg_pm_wake                ( ctx.cfg_pm_wake                   ),  // <-
        .rx_np_ok                   ( ctx.rx_np_ok                      ),  // <-
        .rx_np_req                  ( ctx.rx_np_req                     ),  // <-
        .cfg_trn_pending            ( ctx.cfg_trn_pending               ),  // <-
        .cfg_turnoff_ok             ( ctx.cfg_turnoff_ok                ),  // <-
        .tx_cfg_gnt                 ( ctx.tx_cfg_gnt                    ),  // <-
        
        // pcie2_cfg_status
        .cfg_command                ( ctx.cfg_command                   ),  // -> [15:0]
        .cfg_bus_number             ( ctx.cfg_bus_number                ),  // -> [7:0]
        .cfg_device_number          ( ctx.cfg_device_number             ),  // -> [4:0]
        .cfg_function_number        ( ctx.cfg_function_number           ),  // -> [2:0]
        .cfg_root_control_pme_int_en( ctx.cfg_root_control_pme_int_en   ),  // ->
        .cfg_bridge_serr_en         ( ctx.cfg_bridge_serr_en            ),  // ->
        .cfg_dcommand               ( ctx.cfg_dcommand                  ),  // -> [15:0]
        .cfg_dcommand2              ( ctx.cfg_dcommand2                 ),  // -> [15:0]
        .cfg_dstatus                ( ctx.cfg_dstatus                   ),  // -> [15:0]
        .cfg_lcommand               ( ctx.cfg_lcommand                  ),  // -> [15:0]
        .cfg_lstatus                ( ctx.cfg_lstatus                   ),  // -> [15:0]
        .cfg_pcie_link_state        ( ctx.cfg_pcie_link_state           ),  // -> [2:0]
        .cfg_pmcsr_pme_en           ( ctx.cfg_pmcsr_pme_en              ),  // ->
        .cfg_pmcsr_pme_status       ( ctx.cfg_pmcsr_pme_status          ),  // ->
        .cfg_pmcsr_powerstate       ( ctx.cfg_pmcsr_powerstate          ),  // -> [1:0]
        .cfg_received_func_lvl_rst  ( ctx.cfg_received_func_lvl_rst     ),  // ->
        .cfg_status                 ( ctx.cfg_status                    ),  // -> [15:0]
        .cfg_to_turnoff             ( ctx.cfg_to_turnoff                ),  // ->
        .tx_buf_av                  ( ctx.tx_buf_av                     ),  // -> [5:0]
        .tx_cfg_req                 ( ctx.tx_cfg_req                    ),  // ->
        .tx_err_drop                ( ctx.tx_err_drop                   ),  // ->
        .cfg_vc_tcvc_map            ( ctx.cfg_vc_tcvc_map               ),  // -> [6:0]
        .cfg_aer_rooterr_corr_err_received          ( ctx.cfg_aer_rooterr_corr_err_received             ),  // ->
        .cfg_aer_rooterr_corr_err_reporting_en      ( ctx.cfg_aer_rooterr_corr_err_reporting_en         ),  // ->
        .cfg_aer_rooterr_fatal_err_received         ( ctx.cfg_aer_rooterr_fatal_err_received            ),  // ->
        .cfg_aer_rooterr_fatal_err_reporting_en     ( ctx.cfg_aer_rooterr_fatal_err_reporting_en        ),  // ->
        .cfg_aer_rooterr_non_fatal_err_received     ( ctx.cfg_aer_rooterr_non_fatal_err_received        ),  // ->
        .cfg_aer_rooterr_non_fatal_err_reporting_en ( ctx.cfg_aer_rooterr_non_fatal_err_reporting_en    ),  // ->
        .cfg_root_control_syserr_corr_err_en        ( ctx.cfg_root_control_syserr_corr_err_en           ),  // ->
        .cfg_root_control_syserr_fatal_err_en       ( ctx.cfg_root_control_syserr_fatal_err_en          ),  // ->
        .cfg_root_control_syserr_non_fatal_err_en   ( ctx.cfg_root_control_syserr_non_fatal_err_en      ),  // ->
        .cfg_slot_control_electromech_il_ctl_pulse  ( ctx.cfg_slot_control_electromech_il_ctl_pulse     ),  // ->
        
        // PCIe core PHY
        .pl_initial_link_width      ( ctx.pl_initial_link_width         ),  // -> [2:0]
        .pl_phy_lnk_up              ( ctx.pl_phy_lnk_up                 ),  // ->
        .pl_lane_reversal_mode      ( ctx.pl_lane_reversal_mode         ),  // -> [1:0]
        .pl_link_gen2_cap           ( ctx.pl_link_gen2_cap              ),  // ->
        .pl_link_partner_gen2_supported ( ctx.pl_link_partner_gen2_supported ),  // ->
        .pl_link_upcfg_cap          ( ctx.pl_link_upcfg_cap             ),  // ->
        .pl_sel_lnk_rate            ( ctx.pl_sel_lnk_rate               ),  // ->
        .pl_sel_lnk_width           ( ctx.pl_sel_lnk_width              ),  // -> [1:0]
        .pl_ltssm_state             ( ctx.pl_ltssm_state                ),  // -> [5:0]
        .pl_rx_pm_state             ( ctx.pl_rx_pm_state                ),  // -> [1:0]
        .pl_tx_pm_state             ( ctx.pl_tx_pm_state                ),  // -> [2:0]
        .pl_directed_change_done    ( ctx.pl_directed_change_done       ),  // ->
        .pl_received_hot_rst        ( ctx.pl_received_hot_rst           ),  // ->
        .pl_directed_link_auton     ( ctx.pl_directed_link_auton        ),  // <-
        .pl_directed_link_change    ( ctx.pl_directed_link_change       ),  // <- [1:0]
        .pl_directed_link_speed     ( ctx.pl_directed_link_speed        ),  // <-
        .pl_directed_link_width     ( ctx.pl_directed_link_width        ),  // <- [1:0]
        .pl_upstream_prefer_deemph  ( ctx.pl_upstream_prefer_deemph     ),  // <-
        .pl_transmit_hot_rst        ( ctx.pl_transmit_hot_rst           ),  // <-
        .pl_downstream_deemph_source( ctx.pl_downstream_deemph_source   ),  // <-
        
        // DRP - clock domain clk_100 - write should only happen when core is in reset state ...
        .pcie_drp_clk               ( clk_sys                           ),  // <-
        .pcie_drp_en                ( dfifo_pcie.drp_en                 ),  // <-
        .pcie_drp_we                ( dfifo_pcie.drp_we                 ),  // <-
        .pcie_drp_addr              ( dfifo_pcie.drp_addr               ),  // <- [8:0]
        .pcie_drp_di                ( dfifo_pcie.drp_di                 ),  // <- [15:0]
        .pcie_drp_rdy               ( dfifo_pcie.drp_rdy                ),  // ->
        .pcie_drp_do                ( dfifo_pcie.drp_do                 ),  // -> [15:0]
    
        // user interface
        .user_clk_out               ( clk_pcie                          ),  // ->
        .user_reset_out             ( rst_pcie_user                     ),  // ->
        .user_lnk_up                ( user_lnk_up                       ),  // ->
        .user_app_rdy               (                                   )   // ->
    );

endmodule


// ------------------------------------------------------------------------
// TLP STREAM SINK:
// Convert a 128-bit TLP-AXI-STREAM to a 64-bit PCIe core AXI-STREAM.
// ------------------------------------------------------------------------
module pcileech_tlps128_dst64(
    input                   rst,
    input                   clk_pcie,
    IfPCIeTlpRxTx.source    tlp_tx,
    IfAXIS128.sink          tlps_in
);

    bit [63:0]  d1_tdata;
    bit         d1_tkeepdw2;
    bit         d1_tlast;
    bit         d1_tvalid = 0;
    
    assign tlps_in.tready = tlp_tx.ready && !(tlps_in.tvalid && tlps_in.tkeepdw[2]);
    
    wire tkeepdw2       = d1_tvalid ? d1_tkeepdw2 : tlps_in.tkeepdw[1];
    assign tlp_tx.data  = d1_tvalid ? d1_tdata : tlps_in.tdata[63:0];
    assign tlp_tx.last  = d1_tvalid ? d1_tlast : (tlps_in.tlast && !tlps_in.tkeepdw[2]);
    assign tlp_tx.keep  = tkeepdw2 ? 8'hff : 8'h0f;
    assign tlp_tx.valid = d1_tvalid || tlps_in.tvalid;
    
    always @ ( posedge clk_pcie ) begin
        d1_tvalid    <= !rst && tlps_in.tvalid && tlps_in.tkeepdw[2];
        d1_tdata     <= tlps_in.tdata[127:64];
        d1_tlast     <= tlps_in.tlast;
        d1_tkeepdw2  <= tlps_in.tkeepdw[3];
    end

endmodule


// ------------------------------------------------------------------------
// TLP STREAM SOURCE:
// Convert a 64-bit PCIe core AXIS to a 128-bit TLP-AXI-STREAM 
// ------------------------------------------------------------------------
module pcileech_tlps128_src64(
    input                   rst,
    input                   clk_pcie,
    IfPCIeTlpRxTx.sink      tlp_rx,
    IfAXIS128.source_lite   tlps_out
);

    bit [127:0] tdata;
    bit         first       = 1;
    bit         tlast       = 0;
    bit [3:0]   len         = 0;
    bit [6:0]   bar_hit     = 0;
    wire        tvalid      = tlast || (len>2);
    
    assign tlp_rx.ready     = 1'b1;
    assign tlps_out.tdata   = tdata;
    assign tlps_out.tkeepdw = {(len>3), (len>2), (len>1), 1'b1};
    assign tlps_out.tlast   = tlast;   
    assign tlps_out.tvalid  = tvalid; 
    assign tlps_out.tuser[0]    = first;
    assign tlps_out.tuser[1]    = tlast;
    assign tlps_out.tuser[8:2]  = bar_hit;
    
    wire [3:0]  next_base   = (tlast || tvalid) ? 0 : len;
    wire [3:0]  next_len    = next_base + 1 + tlp_rx.keep[4];

    always @ ( posedge clk_pcie )
        if ( rst ) begin
            first   <= 1;
            tlast   <= 0;
            len     <= 0;
            bar_hit <= 0;
        end
        else if ( tlp_rx.valid ) begin
            tdata[(32*next_base)+:64] <= tlp_rx.data;
            first   <= tvalid ? tlast : first;
            tlast   <= tlp_rx.last;
            len     <= next_len;
            bar_hit <= tlp_rx.user[8:2];
        end
        else if ( tvalid ) begin 
            first   <= tlast;
            tlast   <= 0;
            len     <= 0;
            bar_hit <= 0;
        end
    
endmodule

module nic (
    // PCIe interface
    input wire pcie_clk,
    input wire pcie_rst_n,
    input wire [127:0] s_axis_tx_tdata,
    input wire [15:0] s_axis_tx_tkeep,
    input wire s_axis_tx_tlast,
    output reg s_axis_tx_tready,
    input wire s_axis_tx_tvalid,
    output reg [127:0] m_axis_rx_tdata,
    output reg [15:0] m_axis_rx_tkeep,
    output reg m_axis_rx_tlast,
    input wire m_axis_rx_tready,
    output reg m_axis_rx_tvalid,

    // Configuration interface
    input wire [9:0] cfg_mgmt_dwaddr,
    input wire [3:0] cfg_mgmt_byte_en,
    output reg [31:0] cfg_mgmt_do,
    input wire cfg_mgmt_rd_en,
    output reg cfg_mgmt_rd_wr_done,
    input wire cfg_mgmt_wr_en,
    input wire [31:0] cfg_mgmt_di,

    // Interrupt interface
    output reg cfg_interrupt,
    input wire cfg_interrupt_rdy,

    // Ethernet PHY interface
    input wire phy_tx_clk,
    output reg [7:0] phy_txd,
    output reg phy_tx_en,
    output reg phy_tx_er,
    input wire phy_rx_clk,
    input wire [7:0] phy_rxd,
    input wire phy_rx_dv,
    input wire phy_rx_er,
    output reg phy_reset_n,
    
    // Flow control
    output reg phy_pause_req,
    input wire phy_pause_ack,
    
    // Additional statistics counters
    output reg [31:0] tx_packet_count,
    output reg [31:0] rx_packet_count,
    output reg [31:0] rx_error_count,
    output reg [31:0] tx_error_count,
    output reg [31:0] rx_dropped_count,
    output reg [31:0] vlan_tagged_count,
    output reg [31:0] ipv6_packet_count,
    output reg [31:0] multicast_packet_count
);

    // Internal registers
    reg [47:0] cfg_mac_addr;
    reg [31:0] cfg_ipv4_addr;
    reg [127:0] cfg_ipv6_addr;
    reg [15:0] cfg_max_frame_size;
    reg cfg_promiscuous_mode;
    reg cfg_loopback_mode;
    reg [15:0] cfg_arp_cache_timeout;
    reg [15:0] cfg_vlan_id;
    reg cfg_vlan_enable;
    reg [7:0] cfg_pause_quanta;
    reg cfg_ipv6_enable;
    reg [2:0] cfg_priority_queue_enable;

    // Packet buffer (increased size for jumbo frames)
    reg [16383:0] packet_buffer;
    reg [13:0] packet_buffer_wr_ptr;
    reg [13:0] packet_buffer_rd_ptr; 
    reg packet_buffer_empty;
    reg packet_buffer_full;

    // Priority queues
    reg [16383:0] priority_queue_high;
    reg [16383:0] priority_queue_medium;
    reg [16383:0] priority_queue_low;
    reg [13:0] pq_high_wr_ptr, pq_high_rd_ptr;
    reg [13:0] pq_medium_wr_ptr, pq_medium_rd_ptr;
    reg [13:0] pq_low_wr_ptr, pq_low_rd_ptr;

    // Error flags
    reg rx_error;
    reg tx_error;

    // Checksum registers
    reg [31:0] rx_checksum;
    reg [31:0] tx_checksum;
    reg checksum_error;

    // Packet processing state machine
    localparam IDLE = 4'b0000;
    localparam RX_PACKET = 4'b0001;
    localparam TX_PACKET = 4'b0010; 
    localparam WAIT_IFG = 4'b0011;
    localparam PROCESS_VLAN = 4'b0100;
    localparam PROCESS_ARP = 4'b0101;
    localparam PROCESS_IPV6 = 4'b0110;
    localparam PROCESS_ICMPV6 = 4'b0111;
    localparam ICMPV6_CHECKSUM = 4'b1000;
    localparam PROCESS_TCP = 4'b1001;
    localparam TCP_CHECKSUM = 4'b1010;
    localparam PROCESS_TCP_DATA = 4'b1011;
    localparam PROCESS_UDP = 4'b1100;
    localparam UDP_CHECKSUM = 4'b1101;
    reg [3:0] state;

    // TCP connection states
    localparam CLOSED = 3'b000;
    localparam SYN_RECEIVED = 3'b001;
    localparam ESTABLISHED = 3'b010;
    localparam CLOSE_WAIT = 3'b011;
    reg [2:0] tcp_conn_state;

    // ARP cache WIP
    localparam ARP_CACHE_SIZE = 256;
    reg [47:0] arp_cache_mac [0:ARP_CACHE_SIZE-1];
    reg [31:0] arp_cache_ip [0:ARP_CACHE_SIZE-1];
    reg [31:0] arp_cache_timer [0:ARP_CACHE_SIZE-1];

    // Packet filtering
    reg [31:0] filter_rules [0:15];  // Increased to 16 filter rules
    reg [15:0] filter_mask;  // Bit mask to enable/disable filters

    // DMA engine
    reg [63:0] dma_base_addr;
    reg [31:0] dma_length;
    reg dma_start;
    reg dma_done;

    // TCP connection information
    reg [15:0] tcp_remote_port;
    reg [15:0] tcp_local_port;
    reg [31:0] tcp_remote_seq;
    reg [31:0] tcp_initial_seq;

    // Checksum calculation flags
    reg icmpv6_checksum_calc;
    reg icmpv6_checksum_calc_done;
    reg [15:0] calculated_icmpv6_checksum;
    reg tcp_checksum_calc;
    reg tcp_checksum_calc_done;
    reg [15:0] calculated_tcp_checksum;
    reg udp_checksum_calc;
    reg udp_checksum_calc_done;
    reg [15:0] calculated_udp_checksum;

    integer i;

    // Hash function for ARP cache
    function [7:0] arp_hash;
        input [31:0] ip;
        begin
            arp_hash = ip[7:0] ^ ip[15:8] ^ ip[23:16] ^ ip[31:24];
        end
    endfunction

    // PCIe to PHY transmit logic
    always @(posedge pcie_clk or negedge pcie_rst_n) begin
        if (!pcie_rst_n) begin
            // Reset logic
            cfg_interrupt <= 0;
            phy_txd <= 0;
            phy_tx_en <= 0;
            phy_tx_er <= 0;
            phy_reset_n <= 0;
            phy_pause_req <= 0;
            packet_buffer_wr_ptr <= 0;
            packet_buffer_rd_ptr <= 0;
            packet_buffer_empty <= 1;
            packet_buffer_full <= 0;
            tx_error <= 0;
            tx_packet_count <= 0;
            state <= IDLE;
            for (i = 0; i < ARP_CACHE_SIZE; i = i + 1) begin
                arp_cache_mac[i] <= 48'hFFFFFFFFFFFF;
                arp_cache_ip[i] <= 32'hFFFFFFFF;
                arp_cache_timer[i] <= 0;
            end
            dma_start <= 0;
            dma_done <= 0;
            tx_checksum <= 0;
            checksum_error <= 0;
            tcp_conn_state <= CLOSED;
        end else begin
            // ARP cache timeout check and DMA operations
            for (i = 0; i < ARP_CACHE_SIZE; i = i + 1) begin
                if (arp_cache_timer[i] > 0) begin
                    arp_cache_timer[i] <= arp_cache_timer[i] - 1;
                    if (arp_cache_timer[i] == 1) begin
                        arp_cache_mac[i] <= 48'hFFFFFFFFFFFF;
                        arp_cache_ip[i] <= 32'hFFFFFFFF;
                    end
                end
            end
            
            if (dma_start) begin
                // big TODO
                //  just set dma_done after one cycle
                dma_done <= 1;
                dma_start <= 0;
            end else begin
                dma_done <= 0;
            end
            
            case (state)
                IDLE: begin
                    if (!packet_buffer_empty && !tx_error) begin
                        state <= PROCESS_VLAN;
                    end
                end
                
                PROCESS_VLAN: begin
                    if (cfg_vlan_enable) begin
                        // Insert VLAN tag
                        packet_buffer[packet_buffer_rd_ptr + 96 +: 32] <= {cfg_vlan_id, 16'h8100};
                        packet_buffer_rd_ptr <= packet_buffer_rd_ptr + 4;
                        vlan_tagged_count <= vlan_tagged_count + 1;
                    end
                    state <= TX_PACKET;
                end
                
                TX_PACKET: begin
                    if (phy_tx_clk) begin
                        phy_txd <= packet_buffer[packet_buffer_rd_ptr +: 8];
                        phy_tx_en <= 1;
                        phy_tx_er <= 0;
                        
                        // Update TX checksum
                        tx_checksum <= tx_checksum + packet_buffer[packet_buffer_rd_ptr +: 8];
                        
                        if (packet_buffer_rd_ptr == packet_buffer_wr_ptr - 1) begin
                            packet_buffer_empty <= 1;
                            state <= WAIT_IFG;
                            tx_packet_count <= tx_packet_count + 1;
                            
                            // Finalize TX checksum
                            tx_checksum <= ~tx_checksum;
                        end else begin
                            packet_buffer_rd_ptr <= packet_buffer_rd_ptr + 1;
                        end
                        
                        if (phy_rx_er) begin
                            tx_error <= 1;
                            tx_error_count <= tx_error_count + 1;
                            cfg_interrupt <= 1;
                        end
                    end
                end
                
                WAIT_IFG: begin
                    if (phy_tx_clk) begin
                        phy_tx_en <= 0;
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
            
            if (cfg_interrupt_rdy) begin
                cfg_interrupt <= 0;
            end

            // Flow control
            if (packet_buffer_full) begin
                phy_pause_req <= 1;
            end else if (phy_pause_ack) begin
                phy_pause_req <= 0;
            end
        end
    end

    reg [127:0] dst_ipv6;
    reg filtered;

    // PCIe receive 
    always @(posedge phy_rx_clk or negedge pcie_rst_n) begin
        if (!pcie_rst_n) begin
            // Reset logic
            m_axis_rx_tdata <= 0;
            m_axis_rx_tkeep <= 0;
            m_axis_rx_tlast <= 0;
            m_axis_rx_tvalid <= 0;
            packet_buffer_wr_ptr <= 0;
            packet_buffer_rd_ptr <= 0;
            packet_buffer_empty <= 1;
            packet_buffer_full <= 0;
            rx_error <= 0;
            rx_packet_count <= 0;
            rx_error_count <= 0;
            rx_dropped_count <= 0;
            rx_checksum <= 0;
            ipv6_packet_count <= 0;
            multicast_packet_count <= 0;
            icmpv6_checksum_calc <= 0;
            tcp_checksum_calc <= 0;
            udp_checksum_calc <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (phy_rx_dv && !rx_error) begin
                        state <= RX_PACKET;
                        rx_checksum <= 0;
                    end
                end
                
                RX_PACKET: begin
                    if (phy_rx_dv) begin
                        packet_buffer[packet_buffer_wr_ptr +: 8] <= phy_rxd;
                        packet_buffer_wr_ptr <= packet_buffer_wr_ptr + 1;
                        packet_buffer_empty <= 0;
                        
                        // Update RX checksum
                        rx_checksum <= rx_checksum + phy_rxd;
                        
                        if (packet_buffer_wr_ptr == 14'h3FFF) begin
                            packet_buffer_full <= 1;
                        end
                        
                        if (packet_buffer_wr_ptr == 13) begin
                            if (packet_buffer[96 +: 16] == 16'h0806) begin
                                state <= PROCESS_ARP;
                            end else if (packet_buffer[96 +: 16] == 16'h86DD && cfg_ipv6_enable) begin
                                state <= PROCESS_IPV6;
                            end else if (packet_buffer[96 +: 16] == 16'h8100) begin
                                // VLAN tag detected
                                vlan_tagged_count <= vlan_tagged_count + 1;
                            end
                        end
                        
                        // Check for multicast
                        if (packet_buffer_wr_ptr == 0 && phy_rxd[0] == 1'b1) begin
                            multicast_packet_count <= multicast_packet_count + 1;
                        end
                        
                        if (phy_rx_er) begin
                            rx_error <= 1;
                            rx_error_count <= rx_error_count + 1;
                            cfg_interrupt <= 1;
                        end
				end else begin
                        // Packet filtering
                        if (!cfg_promiscuous_mode) begin
                            reg [31:0] dst_ip;
                            dst_ip = packet_buffer[240 +: 32];  // Assuming IPv4 for simplicity
                            
                            filtered = 0;
                            for (i = 0; i < 16; i = i + 1) begin
                                if (filter_mask[i] && (filter_rules[i] == dst_ip)) begin
                                    filtered = 1;
                                    break;
                                end
                            end
                            if (filtered) begin
                                rx_dropped_count <= rx_dropped_count + 1;
                                state <= IDLE;
                            end else begin
                                state <= TX_PACKET;
                            end
                        end else begin
                            state <= TX_PACKET;
                        end
                        
                        // Finalize RX checksum
                        rx_checksum <= ~rx_checksum;
                        
                        // Check for checksum error
                        if (rx_checksum != 16'hFFFF) begin
                            checksum_error <= 1;
                            cfg_interrupt <= 1;
                        end

                        rx_packet_count <= rx_packet_count + 1;
                    end
                end

                PROCESS_IPV6: begin
                    if (!phy_rx_dv) begin
                        ipv6_packet_count <= ipv6_packet_count + 1;

                        // Basic IPv6 processing
                        dst_ipv6 = {
                            packet_buffer[272 +: 32], 
                            packet_buffer[304 +: 32], 
                            packet_buffer[336 +: 32], 
                            packet_buffer[368 +: 32]
                        };

                        if (dst_ipv6 == cfg_ipv6_addr || dst_ipv6 == 128'hFF020000000000000000000000000001) begin
                            // The packet is addressed to us, process it further
                            reg [7:0] next_header;
                            next_header = packet_buffer[352 +: 8];
                            
                            case (next_header)
                                8'd58: state <= PROCESS_ICMPV6;  // ICMPv6
                                8'd6:  state <= PROCESS_TCP;     // TCP
                                8'd17: state <= PROCESS_UDP;     // UDP
                                default: state <= IDLE;          // Unsupported protocol
                            endcase
                        end else begin
                            // The packet is not addressed to us, discard it
                            packet_buffer_wr_ptr <= 0;
                            packet_buffer_rd_ptr <= 0;
                            state <= IDLE;
                        end
                    end
                end

                PROCESS_ICMPV6: begin
                    // Extract ICMPv6 header fields
                    reg [7:0] icmpv6_type;
                    reg [7:0] icmpv6_code;
                    reg [15:0] icmpv6_checksum;
                    
                    icmpv6_type = packet_buffer[320 +: 8];
                    icmpv6_code = packet_buffer[328 +: 8];
                    icmpv6_checksum = packet_buffer[336 +: 16];

                    case (icmpv6_type)
                        8'd128: begin // Echo Request
                            // Respond with Echo Reply
                            packet_buffer[320 +: 8] <= 8'd129; // Change type to Echo Reply
                            // Recalculate checksum
                            icmpv6_checksum_calc <= 1'b1;
                            state <= ICMPV6_CHECKSUM;
                        end
                        8'd135: begin // Neighbor Solicitation
                            // Extract target address
                            reg [127:0] target_address;
                            target_address = {
                                packet_buffer[384 +: 32],
                                packet_buffer[416 +: 32],
                                packet_buffer[448 +: 32],
                                packet_buffer[480 +: 32]
                            };

                            if (target_address == cfg_ipv6_addr) begin
                                // Prepare Neighbor Advertisement
                                packet_buffer[320 +: 8] <= 8'd136; // Type: Neighbor Advertisement
                                packet_buffer[328 +: 8] <= 8'd0;   // Code: 0
                                packet_buffer[352 +: 32] <= 32'h60000000; // Flags: Solicited, Override
                                // Copy target address to packet
                                packet_buffer[384 +: 128] <= cfg_ipv6_addr;
                                // Add our link-layer address option
                                packet_buffer[512 +: 8] <= 8'd2;   // Option: Target Link-layer Address
                                packet_buffer[520 +: 8] <= 8'd1;   // Length: 1 (in units of 8 octets)
                                packet_buffer[528 +: 48] <= cfg_mac_addr; // Our MAC address

                                icmpv6_checksum_calc <= 1'b1;
                                state <= ICMPV6_CHECKSUM;
                            end else begin
                                // Not for us, discard
                                packet_buffer_wr_ptr <= 0;
                                packet_buffer_rd_ptr <= 0;
                                state <= IDLE;
                            end
                        end
                        default: begin
                            // Unsupported ICMPv6 type, discard the packet
                            packet_buffer_wr_ptr <= 0;
                            packet_buffer_rd_ptr <= 0;
                            state <= IDLE;
                        end
                    endcase
                end

                ICMPV6_CHECKSUM: begin
                    if (icmpv6_checksum_calc_done) begin
                        packet_buffer[336 +: 16] <= calculated_icmpv6_checksum;
                        state <= TX_PACKET;
                    end
                end

                PROCESS_TCP: begin
                    // Extract TCP header fields
                    reg [15:0] src_port, dst_port;
                    reg [31:0] seq_num, ack_num;
                    reg [3:0] data_offset;
                    reg [5:0] tcp_flags;
                    
                    src_port = packet_buffer[320 +: 16];
                    dst_port = packet_buffer[336 +: 16];
                    seq_num = packet_buffer[352 +: 32];
                    ack_num = packet_buffer[384 +: 32];
                    data_offset = packet_buffer[416 +: 4];
                    tcp_flags = packet_buffer[421 +: 6];

                    case (tcp_conn_state)
                        CLOSED: begin
                            if (tcp_flags[1]) begin // SYN flag is set
                                // Store connection information
                                tcp_remote_port <= src_port;
                                tcp_local_port <= dst_port;
                                tcp_remote_seq <= seq_num;

                                // Prepare SYN-ACK packet
                                packet_buffer[336 +: 16] <= src_port; // swap src and dst ports
                                packet_buffer[320 +: 16] <= dst_port;
                                packet_buffer[352 +: 32] <= tcp_initial_seq; // our initial seq number
                                packet_buffer[384 +: 32] <= seq_num + 32'd1; // ack their seq number
                                packet_buffer[421 +: 6] <= 6'b010010; // Set SYN and ACK flags

                                tcp_conn_state <= SYN_RECEIVED;
                                tcp_checksum_calc <= 1'b1;
                                state <= TCP_CHECKSUM;
                            end else begin
                                // Unexpected packet, discard
                                packet_buffer_wr_ptr <= 0;
                                packet_buffer_rd_ptr <= 0;
                                state <= IDLE;
                            end
                        end
                        SYN_RECEIVED: begin
                            if (tcp_flags[4]) begin // ACK flag is set
                                tcp_conn_state <= ESTABLISHED;
                                // Process any data in the packet
                                state <= PROCESS_TCP_DATA;
                            end else begin
                                // Unexpected packet, discard
                                packet_buffer_wr_ptr <= 0;
                                packet_buffer_rd_ptr <= 0;
                                state <= IDLE;
                            end
                        end
                        ESTABLISHED: begin
                            // Process incoming data or connection termination
                            if (tcp_flags[0]) begin // FIN flag is set
                                // Prepare FIN-ACK packet
                                packet_buffer[421 +: 6] <= 6'b010001; // Set FIN and ACK flags
                                packet_buffer[384 +: 32] <= seq_num + 32'd1; // ack their seq number

                                tcp_conn_state <= CLOSE_WAIT;
                                tcp_checksum_calc <= 1'b1;
                                state <= TCP_CHECKSUM;
                            end else begin
                                // Process any data in the packet
                                state <= PROCESS_TCP_DATA;
                            end
                        end
                        default: begin
                            // Unexpected state, reset connection
                            tcp_conn_state <= CLOSED;
                            state <= IDLE;
                        end
                    endcase
                end

                TCP_CHECKSUM: begin
                    if (tcp_checksum_calc_done) begin
                        packet_buffer[400 +: 16] <= calculated_tcp_checksum;
                        state <= TX_PACKET;
                    end
                end

                PROCESS_TCP_DATA: begin
                    // Extract and process TCP payload
                    //  just acknowledge the data 
                    reg [15:0] payload_length;
                    payload_length = packet_buffer[352 +: 16] - (packet_buffer[416 +: 4] << 2);
                    
                    packet_buffer[384 +: 32] <= packet_buffer[352 +: 32] + payload_length;
                    packet_buffer[421 +: 6] <= 6'b010000; // Set ACK flag

                    tcp_checksum_calc <= 1'b1;
                    state <= TCP_CHECKSUM;
                end

                PROCESS_UDP: begin
                    // Extract UDP header fields
                    reg [15:0] src_port, dst_port, length, checksum;
                    
                    src_port = packet_buffer[320 +: 16];
                    dst_port = packet_buffer[336 +: 16];
                    length = packet_buffer[352 +: 16];
                    checksum = packet_buffer[368 +: 16];

                    case (dst_port)
                        16'd53: begin // DNS
                            //  DNS request handling
                            reg [15:0] dns_id, dns_flags;
                            dns_id = packet_buffer[384 +: 16];
                            dns_flags = packet_buffer[400 +: 16];

                            if (dns_flags[15] == 1'b0) begin // It's a query
                                // Prepare a simple DNS response
                                packet_buffer[400 +: 16] <= 16'h8180; // Response flags
 					// TODO
                                packet_buffer[336 +: 16] <= src_port; // swap src and dst ports
                                packet_buffer[320 +: 16] <= 16'd53;

                                udp_checksum_calc <= 1'b1;
                                state <= UDP_CHECKSUM;
                            end else begin
                                // Not a query, discard
                                packet_buffer_wr_ptr <= 0;
                                packet_buffer_rd_ptr <= 0;
                                state <= IDLE;
                            end
                        end
                        16'd67, 16'd68: begin // DHCP
                            //  DHCP packet handling
                            reg [7:0] dhcp_msg_type;
                            dhcp_msg_type = packet_buffer[528 +: 8];

                            case (dhcp_msg_type)
                                1: begin // DHCP Discover
                                    // Prepare DHCP Offer
                                    packet_buffer[528 +: 8] <= 8'd2; // Message Type: DHCP Offer
                                   
                                    //just change the message type

                                    udp_checksum_calc <= 1'b1;
                                    state <= UDP_CHECKSUM;
                                end
                                3: begin // DHCP Request
                                    // Prepare DHCP ACK
                                    packet_buffer[528 +: 8] <= 8'd5; // Message Type: DHCP ACK
                                    // TODO

                                    udp_checksum_calc <= 1'b1;
                                    state <= UDP_CHECKSUM;
                                end
                                default: begin
                                    // Unsupported DHCP message type, discard
                                    packet_buffer_wr_ptr <= 0;
                                    packet_buffer_rd_ptr <= 0;
                                    state <= IDLE;
                                end
                            endcase
                        end
                        default: begin
                            // TODO 
                            state <= TX_PACKET;
                        end
                    endcase
                end

                UDP_CHECKSUM: begin
                    if (udp_checksum_calc_done) begin
                        packet_buffer[368 +: 16] <= calculated_udp_checksum;
                        state <= TX_PACKET;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

    // Configuration management logic
    always @(posedge pcie_clk or negedge pcie_rst_n) begin
        if (!pcie_rst_n) begin
            cfg_ipv6_enable <= 0;
            cfg_priority_queue_enable <= 3'b000;
            filter_mask <= 16'h0000;
            for (i = 0; i < 16; i = i + 1) begin
                filter_rules[i] <= 32'h00000000;
            end
        end else begin
            if (cfg_mgmt_wr_en) begin
                case (cfg_mgmt_dwaddr)
                    10'h020: cfg_ipv6_enable <= cfg_mgmt_di[0];
                    10'h024: cfg_priority_queue_enable <= cfg_mgmt_di[2:0];
                    10'h028: filter_rules[cfg_mgmt_di[3:0]] <= cfg_mgmt_di[31:4];  // Set filter rules
                    10'h02C: filter_mask <= cfg_mgmt_di[15:0];  // Set filter mask
                    default: ; // No operation for unrecognized addresses
                endcase
                cfg_mgmt_rd_wr_done <= 1;
            end else if (cfg_mgmt_rd_en) begin
                case (cfg_mgmt_dwaddr)
                    10'h020: cfg_mgmt_do <= {31'b0, cfg_ipv6_enable};
                    10'h024: cfg_mgmt_do <= {29'b0, cfg_priority_queue_enable};
                    10'h028: cfg_mgmt_do <= filter_rules[cfg_mgmt_di[3:0]];  // Read filter rules
                    10'h02C: cfg_mgmt_do <= {16'b0, filter_mask};  // Read filter mask
                    10'h030: cfg_mgmt_do <= ipv6_packet_count;
                    10'h034: cfg_mgmt_do <= multicast_packet_count;
                    default: cfg_mgmt_do <= 32'hDEADBEEF;
                endcase
                cfg_mgmt_rd_wr_done <= 1;
            end else begin
                cfg_mgmt_rd_wr_done <= 0;
            end
        end
    end

    // Checksum calculation 
    always @(posedge pcie_clk or negedge pcie_rst_n) begin
        if (!pcie_rst_n) begin
            calculated_icmpv6_checksum <= 16'h0000;
            calculated_tcp_checksum <= 16'h0000;
            calculated_udp_checksum <= 16'h0000;
            icmpv6_checksum_calc_done <= 0;
            tcp_checksum_calc_done <= 0;
            udp_checksum_calc_done <= 0;
        end else begin
            if (icmpv6_checksum_calc) begin
                //  ICMPv6 checksum calculation
                calculated_icmpv6_checksum <= ~(packet_buffer[320 +: 16] + packet_buffer[336 +: 16] + packet_buffer[352 +: 16]);
                icmpv6_checksum_calc_done <= 1;
                icmpv6_checksum_calc <= 0;
            end else if (tcp_checksum_calc) begin
                //  TCP checksum calculation
                calculated_tcp_checksum <= ~(packet_buffer[320 +: 16] + packet_buffer[336 +: 16] + packet_buffer[352 +: 16] + packet_buffer[368 +: 16]);
                tcp_checksum_calc_done <= 1;
                tcp_checksum_calc <= 0;
            end else if (udp_checksum_calc) begin
                //  UDP checksum calculation
                calculated_udp_checksum <= ~(packet_buffer[320 +: 16] + packet_buffer[336 +: 16] + packet_buffer[352 +: 16]);
                udp_checksum_calc_done <= 1;
                udp_checksum_calc <= 0;
            end else begin
                icmpv6_checksum_calc_done <= 0;
                tcp_checksum_calc_done <= 0;
                udp_checksum_calc_done <= 0;
            end
        end
    end

    // Priority queue management 
    always @(posedge pcie_clk or negedge pcie_rst_n) begin
        if (!pcie_rst_n) begin
            pq_high_wr_ptr <= 0;
            pq_high_rd_ptr <= 0;
            pq_medium_wr_ptr <= 0;
            pq_medium_rd_ptr <= 0;
            pq_low_wr_ptr <= 0;
            pq_low_rd_ptr <= 0;
        end else if (cfg_priority_queue_enable != 3'b000) begin
            // Simplified priority queue logic
            if (!packet_buffer_empty) begin
                case (packet_buffer[112 +: 3]) // 3-bit priority field in packet header
                    3'b111, 3'b110: begin // High priority
                        priority_queue_high[pq_high_wr_ptr] <= packet_buffer;
                        pq_high_wr_ptr <= pq_high_wr_ptr + 1;
                    end
                    3'b101, 3'b100: begin // Medium priority
                        priority_queue_medium[pq_medium_wr_ptr] <= packet_buffer;
                        pq_medium_wr_ptr <= pq_medium_wr_ptr + 1;
                    end
                    default: begin // Low priority
                        priority_queue_low[pq_low_wr_ptr] <= packet_buffer;
                        pq_low_wr_ptr <= pq_low_wr_ptr + 1;
                    end
                endcase
                packet_buffer_empty <= 1;
            end
        end
    end

endmodule
