//
// PCILeech FPGA.
//
// PCIe BAR PIO controller.
//
// The PCILeech BAR PIO controller allows for easy user-implementation on top
// of the PCILeech AXIS128 PCIe TLP streaming interface.
// The controller consists of a read engine and a write engine and pluggable
// user-implemented PCIe BAR implementations (found at bottom of the file).
//
// Considerations:
// - The core handles 1 DWORD read + 1 DWORD write per CLK max. If a lot of
//   data is written / read from the TLP streaming interface the core may
//   drop packet silently.
// - The core reads 1 DWORD of data (without byte enable) per CLK.
// - The core writes 1 DWORD of data (with byte enable) per CLK.
// - All user-implemented cores must have the same latency in CLKs for the
//   returned read data or else undefined behavior will take place.
// - 32-bit addresses are passed for read/writes. Larger BARs than 4GB are
//   not supported due to addressing constraints. Lower bits (LSBs) are the
//   BAR offset, Higher bits (MSBs) are the 32-bit base address of the BAR.
// - DO NOT edit read/write engines.
// - DO edit pcileech_tlps128_bar_controller (to swap bar implementations).
// - DO edit the bar implementations (at bottom of the file, if neccessary).
//
// Example implementations exists below, swap out any of the example cores
// against a core of your use case, or modify existing cores.
// Following test cores exist (see below in this file):
// - pcileech_bar_impl_zerowrite4k = zero-initialized read/write BAR.
//     It's possible to modify contents by use of .coe file.
// - pcileech_bar_impl_loopaddr = test core that loops back the 32-bit
//     address of the current read. Does not support writes.
// - pcileech_bar_impl_none = core without any reply.
// 
// (c) Ulf Frisk, 2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_tlps128_bar_controller(
    input                   rst,
    input                   clk,
    input                   bar_en,
    input [15:0]            pcie_id,
    input [31:0]            base_address_register,
    IfAXIS128.sink_lite     tlps_in,
    IfAXIS128.source        tlps_out
);
    
    // ------------------------------------------------------------------------
    // 1: TLP RECEIVE:
    // Receive incoming BAR requests from the TLP stream:
    // send them onwards to read and write FIFOs
    // ------------------------------------------------------------------------
    wire in_is_wr_ready;
    bit  in_is_wr_last;
    wire in_is_first    = tlps_in.tuser[0];
    wire in_is_bar      = bar_en && (tlps_in.tuser[8:2] != 0);
    wire in_is_rd       = (in_is_first && tlps_in.tlast && ((tlps_in.tdata[31:25] == 7'b0000000) || (tlps_in.tdata[31:25] == 7'b0010000) || (tlps_in.tdata[31:24] == 8'b00000010)));
    wire in_is_wr       = in_is_wr_last || (in_is_first && in_is_wr_ready && ((tlps_in.tdata[31:25] == 7'b0100000) || (tlps_in.tdata[31:25] == 7'b0110000) || (tlps_in.tdata[31:24] == 8'b01000010)));
    
    always @ ( posedge clk )
        if ( rst ) begin
            in_is_wr_last <= 0;
        end
        else if ( tlps_in.tvalid ) begin
            in_is_wr_last <= !tlps_in.tlast && in_is_wr;
        end
    
    wire [6:0]  wr_bar;
    wire [31:0] wr_addr;
    wire [3:0]  wr_be;
    wire [31:0] wr_data;
    wire        wr_valid;
    wire [87:0] rd_req_ctx;
    wire [6:0]  rd_req_bar;
    wire [31:0] rd_req_addr;
    wire        rd_req_valid;
    wire [87:0] rd_rsp_ctx;
    wire [31:0] rd_rsp_data;
    wire        rd_rsp_valid;
        
    pcileech_tlps128_bar_rdengine i_pcileech_tlps128_bar_rdengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .pcie_id        ( pcie_id                       ),
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_rd ),
        .tlps_out       ( tlps_out                      ),
        // BAR reads:
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_bar     ( rd_req_bar                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid                  ),
        .rd_rsp_ctx     ( rd_rsp_ctx                    ),
        .rd_rsp_data    ( rd_rsp_data                   ),
        .rd_rsp_valid   ( rd_rsp_valid                  )
    );

    pcileech_tlps128_bar_wrengine i_pcileech_tlps128_bar_wrengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_wr ),
        .tlps_in_ready  ( in_is_wr_ready                ),
        // outgoing BAR writes:
        .wr_bar         ( wr_bar                        ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid                      )
    );
    
    wire [87:0] bar_rsp_ctx[7];
    wire [31:0] bar_rsp_data[7];
    wire        bar_rsp_valid[7];
    
    assign rd_rsp_ctx = bar_rsp_valid[0] ? bar_rsp_ctx[0] :
                        bar_rsp_valid[1] ? bar_rsp_ctx[1] :
                        bar_rsp_valid[2] ? bar_rsp_ctx[2] :
                        bar_rsp_valid[3] ? bar_rsp_ctx[3] :
                        bar_rsp_valid[4] ? bar_rsp_ctx[4] :
                        bar_rsp_valid[5] ? bar_rsp_ctx[5] :
                        bar_rsp_valid[6] ? bar_rsp_ctx[6] : 0;
    assign rd_rsp_data = bar_rsp_valid[0] ? bar_rsp_data[0] :
                        bar_rsp_valid[1] ? bar_rsp_data[1] :
                        bar_rsp_valid[2] ? bar_rsp_data[2] :
                        bar_rsp_valid[3] ? bar_rsp_data[3] :
                        bar_rsp_valid[4] ? bar_rsp_data[4] :
                        bar_rsp_valid[5] ? bar_rsp_data[5] :
                        bar_rsp_valid[6] ? bar_rsp_data[6] : 0;
    assign rd_rsp_valid = bar_rsp_valid[0] || bar_rsp_valid[1] || bar_rsp_valid[2] || bar_rsp_valid[3] || bar_rsp_valid[4] || bar_rsp_valid[5] || bar_rsp_valid[6];
    
    pcileech_bar_impl_ar9287_wifi i_bar0(
        .rst                   ( rst                           ),
        .clk                   ( clk                           ),
        .wr_addr               ( wr_addr                       ),
        .wr_be                 ( wr_be                         ),
        .wr_data               ( wr_data                       ),
        .wr_valid              ( wr_valid && wr_bar[0]         ),
        .rd_req_ctx            ( rd_req_ctx                    ),
        .rd_req_addr           ( rd_req_addr                   ),
        .rd_req_valid          ( rd_req_valid && rd_req_bar[0] ),
        .base_address_register ( base_address_register         ),
        .rd_rsp_ctx            ( bar_rsp_ctx[0]                ),
        .rd_rsp_data           ( bar_rsp_data[0]               ),
        .rd_rsp_valid          ( bar_rsp_valid[0]              )
    );
    
    pcileech_bar_impl_loopaddr i_bar1(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[1]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[1] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[1]                ),
        .rd_rsp_data    ( bar_rsp_data[1]               ),
        .rd_rsp_valid   ( bar_rsp_valid[1]              )
    );
    
    pcileech_bar_impl_cool i_bar2(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[2]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[2] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[2]                ),
        .rd_rsp_data    ( bar_rsp_data[2]               ),
        .rd_rsp_valid   ( bar_rsp_valid[2]              )
    );
    
    pcileech_bar_impl_none i_bar3(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[3]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[3] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[3]                ),
        .rd_rsp_data    ( bar_rsp_data[3]               ),
        .rd_rsp_valid   ( bar_rsp_valid[3]              )
    );
    
    pcileech_bar_impl_cool i_bar4(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[4]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[4] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[4]                ),
        .rd_rsp_data    ( bar_rsp_data[4]               ),
        .rd_rsp_valid   ( bar_rsp_valid[4]              )
    );
    
    pcileech_bar_impl_cool i_bar5(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[5]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[5] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[5]                ),
        .rd_rsp_data    ( bar_rsp_data[5]               ),
        .rd_rsp_valid   ( bar_rsp_valid[5]              )
    );
    
    pcileech_bar_impl_cool i_bar6_optrom(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[6]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[6] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[6]                ),
        .rd_rsp_data    ( bar_rsp_data[6]               ),
        .rd_rsp_valid   ( bar_rsp_valid[6]              )
    );


endmodule



// ------------------------------------------------------------------------
// BAR WRITE ENGINE:
// Receives BAR WRITE TLPs and output BAR WRITE requests.
// Holds a 2048-byte buffer.
// Input flow rate is 16bytes/CLK (max).
// Output flow rate is 4bytes/CLK.
// If write engine overflows incoming TLP is completely discarded silently.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_wrengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    output                  tlps_in_ready,
    // outgoing BAR writes:
    output bit [6:0]        wr_bar,
    output bit [31:0]       wr_addr,
    output bit [3:0]        wr_be,
    output bit [31:0]       wr_data,
    output bit              wr_valid
);

    wire            f_rd_en;
    wire [127:0]    f_tdata;
    wire [3:0]      f_tkeepdw;
    wire [8:0]      f_tuser;
    wire            f_tvalid;
    
    bit [127:0]     tdata;
    bit [3:0]       tkeepdw;
    bit             tlast;
    
    bit [3:0]       be_first;
    bit [3:0]       be_last;
    bit             first_dw;
    bit [31:0]      addr;

    fifo_141_141_clk1_bar_wr i_fifo_141_141_clk1_bar_wr(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( {tlps_in.tuser[8:0], tlps_in.tkeepdw, tlps_in.tdata} ),
        .full           (                               ),
        .prog_empty     ( tlps_in_ready                 ),
        .rd_en          ( f_rd_en                       ),
        .dout           ( {f_tuser, f_tkeepdw, f_tdata} ),    
        .empty          (                               ),
        .valid          ( f_tvalid                      )
    );
    
    // STATE MACHINE:
    `define S_ENGINE_IDLE        3'h0
    `define S_ENGINE_FIRST       3'h1
    `define S_ENGINE_4DW_REQDATA 3'h2
    `define S_ENGINE_TX0         3'h4
    `define S_ENGINE_TX1         3'h5
    `define S_ENGINE_TX2         3'h6
    `define S_ENGINE_TX3         3'h7
    (* KEEP = "TRUE" *) bit [3:0] state = `S_ENGINE_IDLE;
    
    assign f_rd_en = (state == `S_ENGINE_IDLE) ||
                     (state == `S_ENGINE_4DW_REQDATA) ||
                     (state == `S_ENGINE_TX3) ||
                     ((state == `S_ENGINE_TX2 && !tkeepdw[3])) ||
                     ((state == `S_ENGINE_TX1 && !tkeepdw[2])) ||
                     ((state == `S_ENGINE_TX0 && !f_tkeepdw[1]));

    always @ ( posedge clk ) begin
        wr_addr     <= addr;
        wr_valid    <= ((state == `S_ENGINE_TX0) && f_tvalid) || (state == `S_ENGINE_TX1) || (state == `S_ENGINE_TX2) || (state == `S_ENGINE_TX3);
        
    end

    always @ ( posedge clk )
        if ( rst ) begin
            state <= `S_ENGINE_IDLE;
        end
        else case ( state )
            `S_ENGINE_IDLE: begin
                state   <= `S_ENGINE_FIRST;
            end
            `S_ENGINE_FIRST: begin
                if ( f_tvalid && f_tuser[0] ) begin
                    wr_bar      <= f_tuser[8:2];
                    tdata       <= f_tdata;
                    tkeepdw     <= f_tkeepdw;
                    tlast       <= f_tuser[1];
                    first_dw    <= 1;
                    be_first    <= f_tdata[35:32];
                    be_last     <= f_tdata[39:36];
                    if ( f_tdata[31:29] == 8'b010 ) begin       // 3 DW header, with data
                        addr    <= { f_tdata[95:66], 2'b00 };
                        state   <= `S_ENGINE_TX3;
                    end
                    else if ( f_tdata[31:29] == 8'b011 ) begin  // 4 DW header, with data
                        addr    <= { f_tdata[127:98], 2'b00 };
                        state   <= `S_ENGINE_4DW_REQDATA;
                    end 
                end
                else begin
                    state   <= `S_ENGINE_IDLE;
                end
            end 
            `S_ENGINE_4DW_REQDATA: begin
                state   <= `S_ENGINE_TX0;
            end
            `S_ENGINE_TX0: begin
                tdata       <= f_tdata;
                tkeepdw     <= f_tkeepdw;
                tlast       <= f_tuser[1];
                addr        <= addr + 4;
                wr_data     <= { f_tdata[0+00+:8], f_tdata[0+08+:8], f_tdata[0+16+:8], f_tdata[0+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (f_tkeepdw[1] ? 4'hf : be_last);
                state       <= f_tvalid ? (f_tkeepdw[1] ? `S_ENGINE_TX1 : `S_ENGINE_FIRST) : `S_ENGINE_IDLE;
            end
            `S_ENGINE_TX1: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[32+00+:8], tdata[32+08+:8], tdata[32+16+:8], tdata[32+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[2] ? 4'hf : be_last);
                state       <= tkeepdw[2] ? `S_ENGINE_TX2 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX2: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[64+00+:8], tdata[64+08+:8], tdata[64+16+:8], tdata[64+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[3] ? 4'hf : be_last);
                state       <= tkeepdw[3] ? `S_ENGINE_TX3 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX3: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[96+00+:8], tdata[96+08+:8], tdata[96+16+:8], tdata[96+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (!tlast ? 4'hf : be_last);
                state       <= !tlast ? `S_ENGINE_TX0 : `S_ENGINE_FIRST;
            end
        endcase

endmodule



// ------------------------------------------------------------------------
// BAR READ ENGINE:
// Receives BAR READ TLPs and output BAR READ requests.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_rdengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    input [15:0]            pcie_id,
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    IfAXIS128.source        tlps_out,
    // BAR reads:
    output [87:0]           rd_req_ctx,
    output [6:0]            rd_req_bar,
    output [31:0]           rd_req_addr,
    output                  rd_req_valid,
    input  [87:0]           rd_rsp_ctx,
    input  [31:0]           rd_rsp_data,
    input                   rd_rsp_valid
);

    // ------------------------------------------------------------------------
    // 1: PROCESS AND QUEUE INCOMING READ TLPs:
    // ------------------------------------------------------------------------
    wire [10:0] rd1_in_dwlen    = (tlps_in.tdata[9:0] == 0) ? 11'd1024 : {1'b0, tlps_in.tdata[9:0]};
    wire [6:0]  rd1_in_bar      = tlps_in.tuser[8:2];
    wire [15:0] rd1_in_reqid    = tlps_in.tdata[63:48];
    wire [7:0]  rd1_in_tag      = tlps_in.tdata[47:40];
    wire [31:0] rd1_in_addr     = { ((tlps_in.tdata[31:29] == 3'b000) ? tlps_in.tdata[95:66] : tlps_in.tdata[127:98]), 2'b00 };
    wire [73:0] rd1_in_data;
    assign rd1_in_data[73:63]   = rd1_in_dwlen;
    assign rd1_in_data[62:56]   = rd1_in_bar;   
    assign rd1_in_data[55:48]   = rd1_in_tag;
    assign rd1_in_data[47:32]   = rd1_in_reqid;
    assign rd1_in_data[31:0]    = rd1_in_addr;
    
    wire        rd1_out_rden;
    wire [73:0] rd1_out_data;
    wire        rd1_out_valid;
    
    fifo_74_74_clk1_bar_rd1 i_fifo_74_74_clk1_bar_rd1(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( rd1_in_data                   ),
        .full           (                               ),
        .rd_en          ( rd1_out_rden                  ),
        .dout           ( rd1_out_data                  ),    
        .empty          (                               ),
        .valid          ( rd1_out_valid                 )
    );
    
    // ------------------------------------------------------------------------
    // 2: PROCESS AND SPLIT READ TLPs INTO RESPONSE TLP READ REQUESTS AND QUEUE:
    //    (READ REQUESTS LARGER THAN 128-BYTES WILL BE SPLIT INTO MULTIPLE).
    // ------------------------------------------------------------------------
    
    wire [10:0] rd1_out_dwlen       = rd1_out_data[73:63];
    wire [4:0]  rd1_out_dwlen5      = rd1_out_data[67:63];
    wire [4:0]  rd1_out_addr5       = rd1_out_data[6:2];
    
    // 1st "instant" packet:
    wire [4:0]  rd2_pkt1_dwlen_pre  = ((rd1_out_addr5 + rd1_out_dwlen5 > 6'h20) || ((rd1_out_addr5 != 0) && (rd1_out_dwlen5 == 0))) ? (6'h20 - rd1_out_addr5) : rd1_out_dwlen5;
    wire [5:0]  rd2_pkt1_dwlen      = (rd2_pkt1_dwlen_pre == 0) ? 6'h20 : rd2_pkt1_dwlen_pre;
    wire [10:0] rd2_pkt1_dwlen_next = rd1_out_dwlen - rd2_pkt1_dwlen;
    wire        rd2_pkt1_large      = (rd1_out_dwlen > 32) || (rd1_out_dwlen != rd2_pkt1_dwlen);
    wire        rd2_pkt1_tiny       = (rd1_out_dwlen == 1);
    wire [11:0] rd2_pkt1_bc         = rd1_out_dwlen << 2;
    wire [85:0] rd2_pkt1;
    assign      rd2_pkt1[85:74]     = rd2_pkt1_bc;
    assign      rd2_pkt1[73:63]     = rd2_pkt1_dwlen;
    assign      rd2_pkt1[62:0]      = rd1_out_data[62:0];
    
    // Nth packet (if split should take place):
    bit  [10:0] rd2_total_dwlen;
    wire [10:0] rd2_total_dwlen_next = rd2_total_dwlen - 11'h20;
    
    bit  [85:0] rd2_pkt2;
    wire [10:0] rd2_pkt2_dwlen = rd2_pkt2[73:63];
    wire        rd2_pkt2_large = (rd2_total_dwlen > 11'h20);
    
    wire        rd2_out_rden;
    
    // STATE MACHINE:
    `define S2_ENGINE_REQDATA     1'h0
    `define S2_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state2 = `S2_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            state2 <= `S2_ENGINE_REQDATA;
        end
        else case ( state2 )
            `S2_ENGINE_REQDATA: begin
                if ( rd1_out_valid && rd2_pkt1_large ) begin
                    rd2_total_dwlen <= rd2_pkt1_dwlen_next;                             // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_pkt1_dwlen_next << 2;                        // byte-count
                    rd2_pkt2[73:63] <= (rd2_pkt1_dwlen_next > 11'h20) ? 11'h20 : rd2_pkt1_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd1_out_data[62:12];                             // various data
                    rd2_pkt2[11:0]  <= rd1_out_data[11:0] + (rd2_pkt1_dwlen << 2);      // base address (within 4k page)
                    state2 <= `S2_ENGINE_PROCESSING;
                end
            end
            `S2_ENGINE_PROCESSING: begin
                if ( rd2_out_rden ) begin
                    rd2_total_dwlen <= rd2_total_dwlen_next;                                // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_total_dwlen_next << 2;                           // byte-count
                    rd2_pkt2[73:63] <= (rd2_total_dwlen_next > 11'h20) ? 11'h20 : rd2_total_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd2_pkt2[62:12];                                     // various data
                    rd2_pkt2[11:0]  <= rd2_pkt2[11:0] + (rd2_pkt2_dwlen << 2);              // base address (within 4k page)
                    if ( !rd2_pkt2_large ) begin
                        state2 <= `S2_ENGINE_REQDATA;
                    end
                end
            end
        endcase
    
    assign rd1_out_rden = rd2_out_rden && (((state2 == `S2_ENGINE_REQDATA) && (!rd1_out_valid || rd2_pkt1_tiny)) || ((state2 == `S2_ENGINE_PROCESSING) && !rd2_pkt2_large));

    wire [85:0] rd2_in_data  = (state2 == `S2_ENGINE_REQDATA) ? rd2_pkt1 : rd2_pkt2;
    wire        rd2_in_valid = rd1_out_valid || ((state2 == `S2_ENGINE_PROCESSING) && rd2_out_rden);

    bit  [85:0] rd2_out_data;
    bit         rd2_out_valid;
    always @ ( posedge clk ) begin
        rd2_out_data    <= rd2_in_valid ? rd2_in_data : rd2_out_data;
        rd2_out_valid   <= rd2_in_valid && !rst;
    end

    // ------------------------------------------------------------------------
    // 3: PROCESS EACH READ REQUEST PACKAGE PER INDIVIDUAL 32-bit READ DWORDS:
    // ------------------------------------------------------------------------

    wire [4:0]  rd2_out_dwlen   = rd2_out_data[67:63];
    wire        rd2_out_last    = (rd2_out_dwlen == 1);
    wire [9:0]  rd2_out_dwaddr  = rd2_out_data[11:2];
    
    wire        rd3_enable;
    
    bit         rd3_process_valid;
    bit         rd3_process_first;
    bit         rd3_process_last;
    bit [4:0]   rd3_process_dwlen;
    bit [9:0]   rd3_process_dwaddr;
    bit [85:0]  rd3_process_data;
    wire        rd3_process_next_last = (rd3_process_dwlen == 2);
    wire        rd3_process_nextnext_last = (rd3_process_dwlen <= 3);
    
    assign rd_req_ctx   = { rd3_process_first, rd3_process_last, rd3_process_data };
    assign rd_req_bar   = rd3_process_data[62:56];
    assign rd_req_addr  = { rd3_process_data[31:12], rd3_process_dwaddr, 2'b00 };
    assign rd_req_valid = rd3_process_valid;
    
    // STATE MACHINE:
    `define S3_ENGINE_REQDATA     1'h0
    `define S3_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state3 = `S3_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            rd3_process_valid   <= 1'b0;
            state3              <= `S3_ENGINE_REQDATA;
        end
        else case ( state3 )
            `S3_ENGINE_REQDATA: begin
                if ( rd2_out_valid ) begin
                    rd3_process_valid       <= 1'b1;
                    rd3_process_first       <= 1'b1;                    // FIRST
                    rd3_process_last        <= rd2_out_last;            // LAST (low 5 bits of dwlen == 1, [max pktlen = 0x20))
                    rd3_process_dwlen       <= rd2_out_dwlen;           // PKT LENGTH IN DW
                    rd3_process_dwaddr      <= rd2_out_dwaddr;          // DWADDR OF THIS DWORD
                    rd3_process_data[85:0]  <= rd2_out_data[85:0];      // FORWARD / SAVE DATA
                    if ( !rd2_out_last ) begin
                        state3 <= `S3_ENGINE_PROCESSING;
                    end
                end
                else begin
                    rd3_process_valid       <= 1'b0;
                end
            end
            `S3_ENGINE_PROCESSING: begin
                rd3_process_first           <= 1'b0;                    // FIRST
                rd3_process_last            <= rd3_process_next_last;   // LAST
                rd3_process_dwlen           <= rd3_process_dwlen - 1;   // LEN DEC
                rd3_process_dwaddr          <= rd3_process_dwaddr + 1;  // ADDR INC
                if ( rd3_process_next_last ) begin
                    state3 <= `S3_ENGINE_REQDATA;
                end
            end
        endcase

    assign rd2_out_rden = rd3_enable && (
        ((state3 == `S3_ENGINE_REQDATA) && (!rd2_out_valid || rd2_out_last)) ||
        ((state3 == `S3_ENGINE_PROCESSING) && rd3_process_nextnext_last));
    
    // ------------------------------------------------------------------------
    // 4: PROCESS RESPONSES:
    // ------------------------------------------------------------------------
    
    wire        rd_rsp_first    = rd_rsp_ctx[87];
    wire        rd_rsp_last     = rd_rsp_ctx[86];
    
    wire [9:0]  rd_rsp_dwlen    = rd_rsp_ctx[72:63];
    wire [11:0] rd_rsp_bc       = rd_rsp_ctx[85:74];
    wire [15:0] rd_rsp_reqid    = rd_rsp_ctx[47:32];
    wire [7:0]  rd_rsp_tag      = rd_rsp_ctx[55:48];
    wire [6:0]  rd_rsp_lowaddr  = rd_rsp_ctx[6:0];
    wire [31:0] rd_rsp_addr     = rd_rsp_ctx[31:0];
    wire [31:0] rd_rsp_data_bs  = { rd_rsp_data[7:0], rd_rsp_data[15:8], rd_rsp_data[23:16], rd_rsp_data[31:24] };
    
    // 1: 32-bit -> 128-bit state machine:
    bit [127:0] tdata;
    bit [3:0]   tkeepdw = 0;
    bit         tlast;
    bit         first   = 1;
    wire        tvalid  = tlast || tkeepdw[3];
    
    always @ ( posedge clk )
        if ( rst ) begin
            tkeepdw <= 0;
            tlast   <= 0;
            first   <= 0;
        end
        else if ( rd_rsp_valid && rd_rsp_first ) begin
            tkeepdw         <= 4'b1111;
            tlast           <= rd_rsp_last;
            first           <= 1'b1;
            tdata[31:0]     <= { 22'b0100101000000000000000, rd_rsp_dwlen };            // format, type, length
            tdata[63:32]    <= { pcie_id[7:0], pcie_id[15:8], 4'b0, rd_rsp_bc };        // pcie_id, byte_count
            tdata[95:64]    <= { rd_rsp_reqid, rd_rsp_tag, 1'b0, rd_rsp_lowaddr };      // req_id, tag, lower_addr
            tdata[127:96]   <= rd_rsp_data_bs;
        end
        else begin
            tlast   <= rd_rsp_valid && rd_rsp_last;
            tkeepdw <= tvalid ? (rd_rsp_valid ? 4'b0001 : 4'b0000) : (rd_rsp_valid ? ((tkeepdw << 1) | 1'b1) : tkeepdw);
            first   <= 0;
            if ( rd_rsp_valid ) begin
                if ( tvalid || !tkeepdw[0] )
                    tdata[31:0]   <= rd_rsp_data_bs;
                if ( !tkeepdw[1] )
                    tdata[63:32]  <= rd_rsp_data_bs;
                if ( !tkeepdw[2] )
                    tdata[95:64]  <= rd_rsp_data_bs;
                if ( !tkeepdw[3] )
                    tdata[127:96] <= rd_rsp_data_bs;   
            end
        end
    
    // 2.1 - submit to output fifo - will feed into mux/pcie core.
    fifo_134_134_clk1_bar_rdrsp i_fifo_134_134_clk1_bar_rdrsp(
        .srst           ( rst                       ),
        .clk            ( clk                       ),
        .din            ( { first, tlast, tkeepdw, tdata } ),
        .wr_en          ( tvalid                    ),
        .rd_en          ( tlps_out.tready           ),
        .dout           ( { tlps_out.tuser[0], tlps_out.tlast, tlps_out.tkeepdw, tlps_out.tdata } ),
        .full           (                           ),
        .empty          (                           ),
        .prog_empty     ( rd3_enable                ),
        .valid          ( tlps_out.tvalid           )
    );
    
    assign tlps_out.tuser[1] = tlps_out.tlast;
    assign tlps_out.tuser[8:2] = 0;
    
    // 2.2 - packet count:
    bit [10:0]  pkt_count       = 0;
    wire        pkt_count_dec   = tlps_out.tvalid && tlps_out.tlast;
    wire        pkt_count_inc   = tvalid && tlast;
    wire [10:0] pkt_count_next  = pkt_count + pkt_count_inc - pkt_count_dec;
    assign tlps_out.has_data    = (pkt_count_next > 0);
    
    always @ ( posedge clk ) begin
        pkt_count <= rst ? 0 : pkt_count_next;
    end

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation that does nothing but drop any read/writes
// silently without generating a response.
// This is only recommended for placeholder designs.
// Latency = N/A.
// ------------------------------------------------------------------------
module pcileech_bar_impl_cool(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    initial rd_rsp_ctx = 0;
    initial rd_rsp_data = 0;
    initial rd_rsp_valid = 0;

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation of "address loopback" which can be useful
// for testing. Any read to a specific BAR address will result in the
// address as response.
// Latency = 2CLKs.
// ------------------------------------------------------------------------
module pcileech_bar_impl_loopaddr(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input [87:0]        rd_req_ctx,
    input [31:0]        rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    bit [87:0]      rd_req_ctx_1;
    bit [31:0]      rd_req_addr_1;
    bit             rd_req_valid_1;
    
    always @ ( posedge clk ) begin
        rd_req_ctx_1    <= rd_req_ctx;
        rd_req_addr_1   <= rd_req_addr;
        rd_req_valid_1  <= rd_req_valid;
        rd_rsp_ctx      <= rd_req_ctx_1;
        rd_rsp_data     <= rd_req_addr_1;
        rd_rsp_valid    <= rd_req_valid_1;
    end    

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation of a 4kB writable initial-zero BAR.
// Latency = 2CLKs.
// ------------------------------------------------------------------------
module pcileech_bar_impl_none( 
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    bit [87:0]  drd_req_ctx;
    bit         drd_req_valid;
    wire [31:0] doutb;
    
    always @ ( posedge clk ) begin
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        rd_rsp_data     <= doutb; 
    end
    
    bram_bar_zero4k i_bram_bar_zero4k(
        // Port A - write:
        .addra  ( wr_addr[11:2]     ),
        .clka   ( clk               ),
        .dina   ( wr_data           ),
        .ena    ( wr_valid          ),
        .wea    ( wr_be             ),
        // Port A - read (2 CLK latency):
        .addrb  ( rd_req_addr[11:2] ),
        .clkb   ( clk               ),
        .doutb  ( doutb             ),
        .enb    ( rd_req_valid      )
    );

endmodule



// ------------------------------------------------------------------------
// hi
// hi 
// ------------------------------------------------------------------------
module pcileech_bar_impl_ar9287_wifi(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    input  [31:0]       base_address_register,
    // outgoing BAR read replies:
    output reg [87:0]   rd_rsp_ctx,
    output reg [31:0]   rd_rsp_data,
    output reg          rd_rsp_valid
);

    reg [87:0]      drd_req_ctx;
    reg [31:0]      drd_req_addr;
    reg             drd_req_valid;

    reg [31:0]      dwr_addr;
    reg [31:0]      dwr_data;
    reg             dwr_valid;

    reg [31:0]      data_32;

    time number = 0;

    always @ (posedge clk) begin
        if (rst)
            number <= 0;

        number          <= number + 1;
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        dwr_valid       <= wr_valid;
        drd_req_addr    <= rd_req_addr;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        dwr_addr        <= wr_addr;
        dwr_data        <= wr_data;

        if (drd_req_valid) begin
            case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & ~32'h4)) & 32'hFFFF)
                32'h0604 : rd_rsp_data <= 32'h327CD9D0; // <- specal address register  // 16'h0604 : rd_rsp_data <= 32'h327CD9D0; | DO NOT UPDATE
                32'h0608 : rd_rsp_data <= 32'hAB2C04FA; // <- specal address register  // 16'h0608 : rd_rsp_data <= 32'hAB2C04FA; | DO NOT UPDATE
			    default : begin
                    case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & ~32'h4)) & 32'h00FF)
							8'h00 : begin
							
								rd_rsp_data[7:0]   <= 8'h00;  // mac prefix <- changed to intel 
								rd_rsp_data[15:8]  <= 8'h02;  // mac prefix <- changed to intel 
								rd_rsp_data[23:16] <= 8'hB3;  // mac prefix <- changed to intel 
                            rd_rsp_data[31:24] <= ((0 + (number) % (15 + 1 - 0)) << 4) | (0 + (number + 3) % (15 + 1 - 0));
							end
									8'h04 : begin
									rd_rsp_data[7:0]   <= ((0 + (number + 6) % (15 + 1 - 0)) << 4) | (0 + (number + 9) % (15 + 1 - 0));
									rd_rsp_data[15:8]  <= ((0 + (number + 12) % (15 + 1 - 0)) << 4) | (0 + (number + 15) % (15 + 1 - 0));
									rd_rsp_data[31:16] <= 16'h0000;
									end
             // 16'h0000 : rd_rsp_data <= 32'h581C0641;  //
             // 16'h0004 : rd_rsp_data <= 32'h581C0641;  //
                16'h0008 : rd_rsp_data <= 32'h40380683;
                16'h0010 : rd_rsp_data <= 32'h04082B40;
                16'h0014 : rd_rsp_data <= 32'h991C046F;
                16'h0018 : rd_rsp_data <= 32'h10000040;
                16'h001C : rd_rsp_data <= 32'h000A0042;
                16'h0020 : rd_rsp_data <= 32'h1805C5E1;
                16'h0028 : rd_rsp_data <= 32'h00C28001;
                16'h002C : rd_rsp_data <= 32'h00000100;
                16'h0030 : rd_rsp_data <= 32'h00008808;
                16'h0034 : rd_rsp_data <= 32'h00000200;
                16'h0038 : rd_rsp_data <= 32'h00008100;
                16'h0040 : rd_rsp_data <= 32'h70D35ED8;
                16'h0044 : rd_rsp_data <= 32'h8004703D;
                16'h00C0 : rd_rsp_data <= 32'h00000081;
                16'h00C4 : rd_rsp_data <= 32'h00008100;
                16'h00D0 : rd_rsp_data <= 32'h4044000C;
                16'h00D8 : rd_rsp_data <= 32'h4044000C;
                16'h0100 : rd_rsp_data <= 32'h04408002;
                16'h0110 : rd_rsp_data <= 32'hC3134000;
                16'h0114 : rd_rsp_data <= 32'h0000000B;
                16'h0118 : rd_rsp_data <= 32'h00004000;
                16'h0120 : rd_rsp_data <= 32'h00000108;
                16'h0128 : rd_rsp_data <= 32'h00000165;
                16'h0160 : rd_rsp_data <= 32'h0000E660;
                16'h0168 : rd_rsp_data <= 32'h0000CCC0;
                16'h0170 : rd_rsp_data <= 32'h00000680;
                16'h01A0 : rd_rsp_data <= 32'h00000038;
                16'h0200 : rd_rsp_data <= 32'h00010000;
                16'h0204 : rd_rsp_data <= 32'h00010000;
                16'h0218 : rd_rsp_data <= 32'h00000001;
                16'h021C : rd_rsp_data <= 32'h00000001;
                16'h03F0 : rd_rsp_data <= 32'h00200000;
                16'h03F4 : rd_rsp_data <= 32'h00018000;
                16'h03F8 : rd_rsp_data <= 32'h00000001;
                16'h0400 : rd_rsp_data <= 32'hA403F0FA;
                16'h0404 : rd_rsp_data <= 32'h00010840;
                16'h0410 : rd_rsp_data <= 32'h00601008;
                16'h041C : rd_rsp_data <= 32'h00000003;
                16'h0420 : rd_rsp_data <= 32'hC312F000;
                16'h0424 : rd_rsp_data <= 32'h0000000B;
                16'h0428 : rd_rsp_data <= 32'h00004000;
                16'h0430 : rd_rsp_data <= 32'h00000304;
                16'h0438 : rd_rsp_data <= 32'h00000304;
                16'h0448 : rd_rsp_data <= 32'h00002000;
                16'h0600 : rd_rsp_data <= 32'h00000001;
                16'h060C : rd_rsp_data <= 32'h216B7D9E;
                16'h0610 : rd_rsp_data <= 32'h7C664832;
                16'h0614 : rd_rsp_data <= 32'h5B458367;
                16'h0618 : rd_rsp_data <= 32'h2E136D16;
                16'h061C : rd_rsp_data <= 32'h7D6596C4;
                16'h0620 : rd_rsp_data <= 32'h3EFE2ADA;
                16'h0624 : rd_rsp_data <= 32'h7A8E5D7C;
                16'h0628 : rd_rsp_data <= 32'hC8E2B6C8;
                16'h062C : rd_rsp_data <= 32'h1A321B6F;
                16'h0630 : rd_rsp_data <= 32'h3F00A35C;
                16'h0634 : rd_rsp_data <= 32'h6E56045A;
                16'h0638 : rd_rsp_data <= 32'h68373EAB;
                16'h063C : rd_rsp_data <= 32'h8E961CBC;
                16'h0640 : rd_rsp_data <= 32'h3E27C39F;
                16'h0644 : rd_rsp_data <= 32'hE3B9A8E3;
                16'h0648 : rd_rsp_data <= 32'hCD2ABFEB;
                16'h064C : rd_rsp_data <= 32'hF3E7969D;
                16'h0650 : rd_rsp_data <= 32'hD2D8BDCB;
                16'h0654 : rd_rsp_data <= 32'h58BD8EC8;
                16'h0658 : rd_rsp_data <= 32'h1BFA2F19;
                16'h065C : rd_rsp_data <= 32'hD60B3D1B;
                16'h0660 : rd_rsp_data <= 32'hB2198A4B;
                16'h0664 : rd_rsp_data <= 32'hEAF26D59;
                16'h0668 : rd_rsp_data <= 32'h3509E6C4;
                16'h066C : rd_rsp_data <= 32'h38419CA8;
                16'h0670 : rd_rsp_data <= 32'h69F22490;
                16'h0674 : rd_rsp_data <= 32'h88619588;
                16'h0678 : rd_rsp_data <= 32'h2061BA34;
                16'h067C : rd_rsp_data <= 32'h9873A4D5;
                16'h0680 : rd_rsp_data <= 32'h23683824;
                16'h0684 : rd_rsp_data <= 32'h4322314E;
                16'h0688 : rd_rsp_data <= 32'h7552A093;
                16'h068C : rd_rsp_data <= 32'hF82E99A9;
                16'h0690 : rd_rsp_data <= 32'h1B4A9DD6;
                16'h0694 : rd_rsp_data <= 32'h19AEE2BD;
                16'h0698 : rd_rsp_data <= 32'h604BEC09;
                16'h069C : rd_rsp_data <= 32'h1146EA36;
                16'h06A0 : rd_rsp_data <= 32'h4A6DE12D;
                16'h06A4 : rd_rsp_data <= 32'hFFF5C978;
                16'h06A8 : rd_rsp_data <= 32'h5C1B6AD1;
                16'h06AC : rd_rsp_data <= 32'hF8C845DB;
                16'h06B0 : rd_rsp_data <= 32'h32984E6B;
                16'h06B4 : rd_rsp_data <= 32'h49C4A5A5;
                16'h06B8 : rd_rsp_data <= 32'h11C9436A;
                16'h06BC : rd_rsp_data <= 32'h539010BC;
                16'h06C0 : rd_rsp_data <= 32'hC83CF139;
                16'h06C4 : rd_rsp_data <= 32'h6D88DBB4;
                16'h06C8 : rd_rsp_data <= 32'hA20788BA;
                16'h06CC : rd_rsp_data <= 32'hABC0D07D;
                16'h06D0 : rd_rsp_data <= 32'h2C9F4AED;
                16'h06D4 : rd_rsp_data <= 32'h7ACD3C2D;
                16'h06D8 : rd_rsp_data <= 32'h9E8FCF1C;
                16'h06DC : rd_rsp_data <= 32'hD36AA3BD;
                16'h06E0 : rd_rsp_data <= 32'h33A23DCE;
                16'h06E4 : rd_rsp_data <= 32'h8589F6FB;
                16'h06E8 : rd_rsp_data <= 32'hA0E83073;
                16'h06EC : rd_rsp_data <= 32'h6B1345BF;
                16'h06F0 : rd_rsp_data <= 32'h888FF0AD;
                16'h06F4 : rd_rsp_data <= 32'h401942ED;
                16'h06F8 : rd_rsp_data <= 32'h75D519BD;
                16'h06FC : rd_rsp_data <= 32'h6F98C7F4;
                16'h0700 : rd_rsp_data <= 32'hC8EA7ABC;
                16'h0704 : rd_rsp_data <= 32'h2901DCD7;
                16'h0708 : rd_rsp_data <= 32'h8987820B;
                16'h070C : rd_rsp_data <= 32'h42E52AE0;
                16'h0710 : rd_rsp_data <= 32'hAFEC189B;
                16'h0714 : rd_rsp_data <= 32'h4CDB3830;
                16'h0718 : rd_rsp_data <= 32'h979A2418;
                16'h071C : rd_rsp_data <= 32'h48C5327E;
                16'h0720 : rd_rsp_data <= 32'hA706240D;
                16'h0724 : rd_rsp_data <= 32'h1771EA48;
                16'h0728 : rd_rsp_data <= 32'hEA90C84F;
                16'h072C : rd_rsp_data <= 32'h60C87316;
                16'h0730 : rd_rsp_data <= 32'hBBB6EA2F;
                16'h0734 : rd_rsp_data <= 32'hF1F614A5;
                16'h0738 : rd_rsp_data <= 32'hC193FB23;
                16'h073C : rd_rsp_data <= 32'hE2709879;
                16'h0740 : rd_rsp_data <= 32'h3B3C1C12;
                16'h0744 : rd_rsp_data <= 32'h07F754E3;
                16'h0748 : rd_rsp_data <= 32'h8A01A89E;
                16'h074C : rd_rsp_data <= 32'hA5EF2E1D;
                16'h0750 : rd_rsp_data <= 32'h78CF5FE9;
                16'h0754 : rd_rsp_data <= 32'h128C54E6;
                16'h0758 : rd_rsp_data <= 32'h660AABAA;
                16'h075C : rd_rsp_data <= 32'hE3EA21A1;
                16'h0760 : rd_rsp_data <= 32'h5D60199E;
                16'h0764 : rd_rsp_data <= 32'h11EB4212;
                16'h0768 : rd_rsp_data <= 32'hACC5EB06;
                16'h076C : rd_rsp_data <= 32'h9FBECAA1;
                16'h0770 : rd_rsp_data <= 32'h6BB2A30C;
                16'h0774 : rd_rsp_data <= 32'h54C29CEF;
                16'h0778 : rd_rsp_data <= 32'h20299F99;
                16'h077C : rd_rsp_data <= 32'hCC25672C;
                16'h0780 : rd_rsp_data <= 32'hD376BF18;
                16'h0784 : rd_rsp_data <= 32'hA879F1B6;
                16'h0788 : rd_rsp_data <= 32'h8C26EAED;
                16'h078C : rd_rsp_data <= 32'h03BA3238;
                16'h0790 : rd_rsp_data <= 32'hF9FBA9F3;
                16'h0794 : rd_rsp_data <= 32'hA324A570;
                16'h0798 : rd_rsp_data <= 32'h7204AA4D;
                16'h079C : rd_rsp_data <= 32'h966A383E;
                16'h07A0 : rd_rsp_data <= 32'h892E5B88;
                16'h07A4 : rd_rsp_data <= 32'hF88545FA;
                16'h07A8 : rd_rsp_data <= 32'h1A69A9ED;
                16'h07AC : rd_rsp_data <= 32'h826F28F0;
                16'h07B0 : rd_rsp_data <= 32'hB652A6FE;
                16'h07B4 : rd_rsp_data <= 32'h9ACAB7D3;
                16'h07B8 : rd_rsp_data <= 32'hB9A38A11;
                16'h07BC : rd_rsp_data <= 32'hD8FE8708;
                16'h07C0 : rd_rsp_data <= 32'hF4A55531;
                16'h07C4 : rd_rsp_data <= 32'hACA8DE70;
                16'h07C8 : rd_rsp_data <= 32'h3ED8034F;
                16'h07CC : rd_rsp_data <= 32'h0B61B18A;
                16'h07D0 : rd_rsp_data <= 32'hE4DA2939;
                16'h07D4 : rd_rsp_data <= 32'h1770C3CE;
                16'h07D8 : rd_rsp_data <= 32'hB7880157;
                16'h07DC : rd_rsp_data <= 32'hDE121696;
                16'h07E0 : rd_rsp_data <= 32'h3A600786;
                16'h07E4 : rd_rsp_data <= 32'h11715E85;
                16'h07E8 : rd_rsp_data <= 32'h6BE8CF42;
                16'h07EC : rd_rsp_data <= 32'hD241E90C;
                16'h07F0 : rd_rsp_data <= 32'h9E0FCFE9;
                16'h07F4 : rd_rsp_data <= 32'h5D402F0C;
                16'h07F8 : rd_rsp_data <= 32'h0656C0BE;
                16'h07FC : rd_rsp_data <= 32'h83932B5F;
                16'h0E00 : rd_rsp_data <= 32'h00840827;
                16'h0E0C : rd_rsp_data <= 32'h00000083;
                16'h0E14 : rd_rsp_data <= 32'h00044090;
                16'h0E20 : rd_rsp_data <= 32'hFD600B26;
                16'h0E24 : rd_rsp_data <= 32'hFFF30005;
                16'h0E28 : rd_rsp_data <= 32'h0006401B;
                16'h0E2C : rd_rsp_data <= 32'h0000003B;
                16'h0E30 : rd_rsp_data <= 32'h00060000;
                16'h0E34 : rd_rsp_data <= 32'h0BC03C21;
                16'h0E38 : rd_rsp_data <= 32'h00000001;
                16'h0E3C : rd_rsp_data <= 32'h0000003B;
                16'h1010 : rd_rsp_data <= 32'h0000001F;
                16'h1014 : rd_rsp_data <= 32'h0000001F;
                16'h1018 : rd_rsp_data <= 32'h0000001F;
                16'h101C : rd_rsp_data <= 32'h0000001F;
                16'h1020 : rd_rsp_data <= 32'h0000001F;
                16'h1024 : rd_rsp_data <= 32'h0000001F;
                16'h102C : rd_rsp_data <= 32'h00000046;
                16'h1038 : rd_rsp_data <= 32'h0000001F;
                16'h103C : rd_rsp_data <= 32'h0000001F;
                16'h1040 : rd_rsp_data <= 32'h02020000;
                16'h1048 : rd_rsp_data <= 32'h66EABAD5;
                16'h1054 : rd_rsp_data <= 32'hDE371C00;
                16'h1058 : rd_rsp_data <= 32'h0000122D;
                16'h1080 : rd_rsp_data <= 32'h01018808;
                16'h1088 : rd_rsp_data <= 32'h0000000F;
                16'h112C : rd_rsp_data <= 32'h00000046;
                16'h1140 : rd_rsp_data <= 32'h02020000;
                16'h1148 : rd_rsp_data <= 32'h66EABD2C;
                16'h1154 : rd_rsp_data <= 32'hDE371C00;
                16'h1158 : rd_rsp_data <= 32'h0000149F;
                16'h1180 : rd_rsp_data <= 32'h01018808;
                16'h1188 : rd_rsp_data <= 32'h0000000F;
                16'h122C : rd_rsp_data <= 32'h00000046;
                16'h1240 : rd_rsp_data <= 32'h02020000;
                16'h1248 : rd_rsp_data <= 32'h66EABFD7;
                16'h1254 : rd_rsp_data <= 32'hDE371C00;
                16'h1258 : rd_rsp_data <= 32'h00001717;
                16'h1280 : rd_rsp_data <= 32'h01018808;
                16'h1288 : rd_rsp_data <= 32'h0000000F;
                16'h132C : rd_rsp_data <= 32'h00000046;
                16'h1340 : rd_rsp_data <= 32'h02020000;
                16'h1348 : rd_rsp_data <= 32'h66EAC218;
                16'h1354 : rd_rsp_data <= 32'hDE371C00;
                16'h1358 : rd_rsp_data <= 32'h0000193F;
                16'h1380 : rd_rsp_data <= 32'h01018808;
                16'h1388 : rd_rsp_data <= 32'h0000000F;
                16'h142C : rd_rsp_data <= 32'h00000046;
                16'h1440 : rd_rsp_data <= 32'h02020000;
                16'h1448 : rd_rsp_data <= 32'h66EAC459;
                16'h1454 : rd_rsp_data <= 32'hDE371C00;
                16'h1458 : rd_rsp_data <= 32'h00001B6A;
                16'h1480 : rd_rsp_data <= 32'h01018808;
                16'h1488 : rd_rsp_data <= 32'h0000000F;
                16'h1508 : rd_rsp_data <= 32'h4044000C;
                16'h150C : rd_rsp_data <= 32'h4044000C;
                16'h1514 : rd_rsp_data <= 32'hC0000011;
                16'h1524 : rd_rsp_data <= 32'h0000001F;
                16'h1528 : rd_rsp_data <= 32'h0000001F;
                16'h152C : rd_rsp_data <= 32'h0000001F;
                16'h1530 : rd_rsp_data <= 32'h0000001F;
                16'h1680 : rd_rsp_data <= 32'h00008100;
                16'h1684 : rd_rsp_data <= 32'h00008100;
                16'h1688 : rd_rsp_data <= 32'h00008100;
                16'h168C : rd_rsp_data <= 32'h00008100;
                16'h1690 : rd_rsp_data <= 32'h00008100;
                16'h1700 : rd_rsp_data <= 32'h00008181;
                16'h1740 : rd_rsp_data <= 32'h00008000;
                16'h2160 : rd_rsp_data <= 32'h0000CCC0;
                16'h2168 : rd_rsp_data <= 32'h0000E660;
                16'h2404 : rd_rsp_data <= 32'h000000A2;
                16'h2458 : rd_rsp_data <= 32'h00100000;
                16'h245C : rd_rsp_data <= 32'h00000001;
                16'h2464 : rd_rsp_data <= 32'h00000004;
                16'h24C0 : rd_rsp_data <= 32'h000006F4;
                16'h24C4 : rd_rsp_data <= 32'h000006F3;
                16'h24C8 : rd_rsp_data <= 32'h000006F3;
                16'h24CC : rd_rsp_data <= 32'h000006F3;
                16'h24E8 : rd_rsp_data <= 32'h8888D88F;
                16'h2508 : rd_rsp_data <= 32'h3000C020;
                16'h2514 : rd_rsp_data <= 32'h00002020;
                16'h25C0 : rd_rsp_data <= 32'h00002084;
                16'h25C4 : rd_rsp_data <= 32'h0000FEEC;
                16'h25E8 : rd_rsp_data <= 32'h00080000;
                16'h25FC : rd_rsp_data <= 32'h00000480;
                16'h2800 : rd_rsp_data <= 32'hC3134000;
                16'h2804 : rd_rsp_data <= 32'h0000000B;
                16'h2808 : rd_rsp_data <= 32'h00004000;
                16'h280C : rd_rsp_data <= 32'h02000402;
                16'h2810 : rd_rsp_data <= 32'h00000108;
                16'h2814 : rd_rsp_data <= 32'h0000A200;
                16'h2818 : rd_rsp_data <= 32'h00000165;
                16'h2828 : rd_rsp_data <= 32'h02010A0C;
                16'h2830 : rd_rsp_data <= 32'h00000002;
                16'h290C : rd_rsp_data <= 32'h80000400;
                16'h2914 : rd_rsp_data <= 32'h0000A200;
                16'h2928 : rd_rsp_data <= 32'h00010A0C;
                16'h2A0C : rd_rsp_data <= 32'h80000400;
                16'h2A14 : rd_rsp_data <= 32'h0000A200;
                16'h2A28 : rd_rsp_data <= 32'h00010A0C;
                16'h2B0C : rd_rsp_data <= 32'h80000400;
                16'h2B14 : rd_rsp_data <= 32'h0000A200;
                16'h2B28 : rd_rsp_data <= 32'h00010A0C;
                16'h3000 : rd_rsp_data <= 32'h00007736;
                16'h3308 : rd_rsp_data <= 32'h0000000F;
                16'h331C : rd_rsp_data <= 32'h3B9ACA00;
                16'h3354 : rd_rsp_data <= 32'h000300E4;
                16'h3400 : rd_rsp_data <= 32'h00000006;
                16'h3404 : rd_rsp_data <= 32'h04000014;
                16'h3504 : rd_rsp_data <= 32'h00018020;
                16'h3508 : rd_rsp_data <= 32'h04000000;
                16'h350C : rd_rsp_data <= 32'h00000001;
                16'h3540 : rd_rsp_data <= 32'h00000010;
                16'h354C : rd_rsp_data <= 32'h00008600;
                16'h3550 : rd_rsp_data <= 32'h000000E4;
                16'h355C : rd_rsp_data <= 32'h00000098;
                16'h3560 : rd_rsp_data <= 32'h0000004C;
                16'h3568 : rd_rsp_data <= 32'hD54C99B2;
                16'h3578 : rd_rsp_data <= 32'h80000000;
                16'h3580 : rd_rsp_data <= 32'h3FE9C100;
                16'h3584 : rd_rsp_data <= 32'h0000004B;
                16'h3590 : rd_rsp_data <= 32'h00000080;
                16'h359C : rd_rsp_data <= 32'h0F760FF6;
                16'h35A0 : rd_rsp_data <= 32'h00000F7F;
                16'h35A4 : rd_rsp_data <= 32'h00400004;
                16'h35C0 : rd_rsp_data <= 32'h00002084;
                16'h35C4 : rd_rsp_data <= 32'h004E7EEC;
                16'h35E8 : rd_rsp_data <= 32'h00080000;
                16'h35FC : rd_rsp_data <= 32'h00000480;
                16'h36CC : rd_rsp_data <= 32'h0000004C;
                16'h3800 : rd_rsp_data <= 32'hC312F000;
                16'h3804 : rd_rsp_data <= 32'h0000000B;
                16'h3808 : rd_rsp_data <= 32'h00004000;
                16'h3810 : rd_rsp_data <= 32'h00000306;
                16'h3814 : rd_rsp_data <= 32'h00002A00;
                16'h3818 : rd_rsp_data <= 32'h00000306;
                16'h3828 : rd_rsp_data <= 32'h02040000;
                16'h3914 : rd_rsp_data <= 32'h00002A00;
                16'h3A14 : rd_rsp_data <= 32'h00002A00;
                16'h3B14 : rd_rsp_data <= 32'h00002A00;
                16'h3F0C : rd_rsp_data <= 32'h00004828;
                16'h3F24 : rd_rsp_data <= 32'h00022000;
                16'h3F30 : rd_rsp_data <= 32'h0000001F;
                16'h3F34 : rd_rsp_data <= 32'h0000001F;
                16'h3F38 : rd_rsp_data <= 32'h0000001F;
                16'h3F3C : rd_rsp_data <= 32'h0000001F;
                16'h3F40 : rd_rsp_data <= 32'h0000001F;
                16'h3F44 : rd_rsp_data <= 32'h0000001F;
                16'h3F48 : rd_rsp_data <= 32'h0000001F;
                16'h3F4C : rd_rsp_data <= 32'h0000001F;
                16'h3F50 : rd_rsp_data <= 32'h0000001F;
                16'h3F54 : rd_rsp_data <= 32'h0000001F;
                16'h3F58 : rd_rsp_data <= 32'h0000001F;
                16'h3F5C : rd_rsp_data <= 32'h0000001F;
                16'h3F60 : rd_rsp_data <= 32'h0000001F;
                16'h3F64 : rd_rsp_data <= 32'h0000001F;
                16'h3F68 : rd_rsp_data <= 32'h0000001F;
                16'h3F6C : rd_rsp_data <= 32'h0000001F;
                16'h3F70 : rd_rsp_data <= 32'h0000001F;
                16'h3F74 : rd_rsp_data <= 32'h0000001F;
                16'h3F78 : rd_rsp_data <= 32'h0000001F;
                16'h3F7C : rd_rsp_data <= 32'h0000001F;
                16'h3F80 : rd_rsp_data <= 32'h0000001F;
                16'h3F84 : rd_rsp_data <= 32'h0000001F;
                16'h3F88 : rd_rsp_data <= 32'h0000001F;
                16'h3F8C : rd_rsp_data <= 32'h0000001F;
                16'h3F90 : rd_rsp_data <= 32'h0000001F;
                16'h3F94 : rd_rsp_data <= 32'h0000001F;
                16'h3F98 : rd_rsp_data <= 32'h0000001F;
                16'h3F9C : rd_rsp_data <= 32'h0000001F;
                16'h3FA0 : rd_rsp_data <= 32'h0000001F;
                16'h3FA4 : rd_rsp_data <= 32'h0000001F;
                16'h3FA8 : rd_rsp_data <= 32'h0000001F;
                16'h3FAC : rd_rsp_data <= 32'h0000001F;
                16'h3FB0 : rd_rsp_data <= 32'h0000001F;
                16'h3FB4 : rd_rsp_data <= 32'h0000001F;
                16'h3FB8 : rd_rsp_data <= 32'h0000001F;
                16'h3FBC : rd_rsp_data <= 32'h0000001F;
                16'h3FC0 : rd_rsp_data <= 32'h0000001F;
                16'h3FC4 : rd_rsp_data <= 32'h0000001F;
                16'h3FC8 : rd_rsp_data <= 32'h0000001F;
                16'h3FCC : rd_rsp_data <= 32'h0000001F;
                16'h3FD0 : rd_rsp_data <= 32'h0000001F;
                16'h3FD4 : rd_rsp_data <= 32'h0000001F;
                16'h3FD8 : rd_rsp_data <= 32'h0000001F;
                16'h3FDC : rd_rsp_data <= 32'h0000001F;
                16'h3FE0 : rd_rsp_data <= 32'h0000001F;
                16'h3FE4 : rd_rsp_data <= 32'h0000001F;
                16'h3FE8 : rd_rsp_data <= 32'h0000001F;
                16'h3FEC : rd_rsp_data <= 32'h0000001F;
                16'h3FF0 : rd_rsp_data <= 32'h0000001F;
                16'h3FF4 : rd_rsp_data <= 32'h0000001F;
                16'h3FF8 : rd_rsp_data <= 32'h0000001F;
                16'h3FFC : rd_rsp_data <= 32'h0000001F;
                default : rd_rsp_data <= 32'h00000000;
                    endcase
                end
            endcase
        end else if (dwr_valid) begin
             case (({dwr_addr[31:24], dwr_addr[23:16], dwr_addr[15:08], dwr_addr[07:00]} - (base_address_register & ~32'h4)) & 32'h00FF) //
            endcase
        end else begin
                            rd_rsp_data[7:0]   <= ((0 + (number) % (15 + 1 - 0)) << 4) | (0 + (number + 3) % (15 + 1 - 0));
                            rd_rsp_data[15:8]  <= ((0 + (number + 6) % (15 + 1 - 0)) << 4) | (0 + (number + 9) % (15 + 1 - 0));
                            rd_rsp_data[23:16] <= ((0 + (number + 12) % (15 + 1 - 0)) << 4) | (0 + (number + 15) % (15 + 1 - 0));
                            rd_rsp_data[31:24] <= ((0 + (number) % (15 + 1 - 0)) << 4) | (0 + (number + 3) % (15 + 1 - 0));
        end
    end

endmodule
