module pcie_dma_ctrl(
    input  wire             clk,
    input  wire             pix_clk_out,
    input  wire             rstn,

    // TLPs received by the PCIe IP (RC -> EP).
    input  wire             axis_master_tvalid /* synthesis PAP_MARK_DEBUG="1" */,
    output wire             axis_master_tready /* synthesis PAP_MARK_DEBUG="1" */,
    input  wire [127:0]     axis_master_tdata  /* synthesis PAP_MARK_DEBUG="1" */,
    input  wire [3:0]       axis_master_tkeep  /* synthesis PAP_MARK_DEBUG="1" */,
    input  wire             axis_master_tlast  /* synthesis PAP_MARK_DEBUG="1" */,
    input  wire [7:0]       axis_master_tuser  /* synthesis PAP_MARK_DEBUG="1" */,

    input  wire [7:0]       ep_bus_num,
    input  wire [4:0]       ep_dev_num,

    // TLPs sent to the PCIe IP (EP -> RC).
    input  wire             AXIS_S_TREADY /* synthesis PAP_MARK_DEBUG="1" */,
    output wire             AXIS_S_TVALID /* synthesis PAP_MARK_DEBUG="1" */,
    output wire [127:0]     AXIS_S_TDATA  /* synthesis PAP_MARK_DEBUG="1" */,
    output wire             AXIS_S_TLAST  /* synthesis PAP_MARK_DEBUG="1" */,
    output wire             AXIS_S_TUSER  /* synthesis PAP_MARK_DEBUG="1" */,

    // HDMI RGB565 input.
    input  wire [15:0]      hdmi_data_in  /* synthesis PAP_MARK_DEBUG="1" */,
    input  wire             vs_in        /* synthesis PAP_MARK_DEBUG="1" */,
    input  wire             de_in        /* synthesis PAP_MARK_DEBUG="1" */,

    output reg  [7:0]       video_enhance_lightdown_num /* synthesis PAP_MARK_DEBUG="1" */,
    output reg              video_enhance_lightdown_sw  /* synthesis PAP_MARK_DEBUG="1" */,
    output reg  [7:0]       video_enhance_darkup_num    /* synthesis PAP_MARK_DEBUG="1" */,
    output reg              video_enhance_darkup_sw     /* synthesis PAP_MARK_DEBUG="1" */
);

// PCIE_FIX_20260912:
//   1. Decode received commands with an explicit HEADER/PAYLOAD state machine.
//   2. Require BAR1, MWr32, Length=1DW, valid DW0 and TLAST for a command.
//   3. Buffer an entire 64-byte payload before presenting a transmit TLP.
//   4. Advance all TX state only on TVALID && TREADY handshakes.
//   5. Hold TVALID/TDATA/TLAST stable while the PCIe core applies backpressure.
//   6. Select a valid frame buffer only at a VS frame boundary.
//   7. Emit one constant 64-byte marker per frame and increment it once/frame.

localparam [7:0] MWR_32 = 8'h40;

localparam [11:0] DMA_CMD_L_ADDR               = 12'h110;
localparam [11:0] DMA_CMD_CLEAR_ADDR           = 12'h130;
localparam [11:0] DMA_CMD_VIDEO_HIGHTLIGHT_SUB = 12'h140;
localparam [11:0] DMA_CMD_VIDEO_LOWLIGHT_ADD   = 12'h150;
localparam [11:0] DMA_CMD_VIDEO_ENHANCE_CLEAR  = 12'h160;

localparam [1:0] RX_HEADER  = 2'd0;
localparam [1:0] RX_PAYLOAD = 2'd1;

localparam [2:0] TX_WAIT          = 3'd0;
localparam [2:0] TX_FETCH_REQUEST = 3'd1;
localparam [2:0] TX_FETCH_WAIT    = 3'd2;
localparam [2:0] TX_FETCH_CAPTURE = 3'd3;
localparam [2:0] TX_HEADER        = 3'd4;
localparam [2:0] TX_DATA          = 3'd5;

localparam [9:0]  TLP_LENGTH_DW    = 10'd16;
localparam [15:0] IMAGE_TLP_COUNT  = 16'd64800;
localparam [15:0] TOTAL_TLP_COUNT  = 16'd64801;

wire [12:0]  rd_water_level /* synthesis PAP_MARK_DEBUG="1" */;
wire [127:0] pcie_dma_data  /* synthesis PAP_MARK_DEBUG="1" */;
wire         fifo_wr_full   /* synthesis PAP_MARK_DEBUG="1" */;
wire         fifo_rd_empty  /* synthesis PAP_MARK_DEBUG="1" */;

reg [1:0]  rx_state /* synthesis PAP_MARK_DEBUG="1" */;
reg [11:0] rx_cmd_addr /* synthesis PAP_MARK_DEBUG="1" */;

reg [1:0]  alloc_addr_state /* synthesis PAP_MARK_DEBUG="1" */;
reg [31:0] dma_addr0 /* synthesis PAP_MARK_DEBUG="1" */;
reg [31:0] dma_addr1 /* synthesis PAP_MARK_DEBUG="1" */;
reg [31:0] dma_addr2 /* synthesis PAP_MARK_DEBUG="1" */;
reg [31:0] dma_addr3 /* synthesis PAP_MARK_DEBUG="1" */;
reg        rc_cfg_ep_flag /* synthesis PAP_MARK_DEBUG="1" */;
reg        dma_clear_pulse /* synthesis PAP_MARK_DEBUG="1" */;

reg vs_in_meta /* synthesis PAP_MARK_DEBUG="1" */;
reg vs_in_sync /* synthesis PAP_MARK_DEBUG="1" */;
reg vs_in_prev /* synthesis PAP_MARK_DEBUG="1" */;

reg pix_rstn_meta;
reg pix_rstn_sync;
reg frame_active_pix_meta;
reg frame_active_pix;

reg [2:0]   tx_state /* synthesis PAP_MARK_DEBUG="1" */;
reg         r_axis_s_tvalid /* synthesis PAP_MARK_DEBUG="1" */;
reg [127:0] r_axis_s_tdata  /* synthesis PAP_MARK_DEBUG="1" */;
reg         r_axis_s_tlast  /* synthesis PAP_MARK_DEBUG="1" */;
reg         pcie_rd_en      /* synthesis PAP_MARK_DEBUG="1" */;

reg [511:0] payload_buffer /* synthesis PAP_MARK_DEBUG="1" */;
reg [1:0]   fetch_index /* synthesis PAP_MARK_DEBUG="1" */;
reg [1:0]   data_index  /* synthesis PAP_MARK_DEBUG="1" */;
reg [15:0]  dma_cnt     /* synthesis PAP_MARK_DEBUG="1" */;
reg [7:0]   frame_counter /* synthesis PAP_MARK_DEBUG="1" */;

reg [1:0]  addr_page /* synthesis PAP_MARK_DEBUG="1" */;
reg [31:0] current_addr /* synthesis PAP_MARK_DEBUG="1" */;
reg [31:0] packet_addr  /* synthesis PAP_MARK_DEBUG="1" */;
reg        frame_addr_valid /* synthesis PAP_MARK_DEBUG="1" */;
reg        frame_active /* synthesis PAP_MARK_DEBUG="1" */;
reg        abort_pending /* synthesis PAP_MARK_DEBUG="1" */;
reg        stop_pending  /* synthesis PAP_MARK_DEBUG="1" */;

wire rx_handshake;
wire tx_handshake;
wire vs_rising;
wire vs_falling;
wire tx_tlp_in_progress;
wire fifo_wr_en;
wire fifo_wr_rst;
wire fifo_rd_rst;

assign axis_master_tready = 1'b1;
assign rx_handshake = axis_master_tvalid && axis_master_tready;

assign AXIS_S_TVALID = r_axis_s_tvalid;
assign AXIS_S_TDATA  = r_axis_s_tdata;
assign AXIS_S_TLAST  = r_axis_s_tlast;
assign AXIS_S_TUSER  = 1'b0;
assign tx_handshake  = r_axis_s_tvalid && AXIS_S_TREADY;

assign vs_rising  =  vs_in_sync && !vs_in_prev;
assign vs_falling = !vs_in_sync &&  vs_in_prev;

assign tx_tlp_in_progress =
    ((tx_state == TX_HEADER) && r_axis_s_tvalid) ||
    (tx_state == TX_DATA);

assign fifo_wr_en  = de_in && frame_active_pix && !fifo_wr_full;
assign fifo_wr_rst = vs_in || !pix_rstn_sync || !frame_active_pix;
assign fifo_rd_rst = vs_in_sync || !rstn || !frame_active;

function [31:0] endian32;
    input [31:0] data_in;
    begin
        endian32 = {data_in[7:0], data_in[15:8],
                    data_in[23:16], data_in[31:24]};
    end
endfunction

function [127:0] endian128;
    input [127:0] data_in;
    begin
        endian128 = {
            data_in[103:96], data_in[111:104], data_in[119:112], data_in[127:120],
            data_in[71:64],  data_in[79:72],   data_in[87:80],   data_in[95:88],
            data_in[39:32],  data_in[47:40],   data_in[55:48],   data_in[63:56],
            data_in[7:0],    data_in[15:8],    data_in[23:16],   data_in[31:24]
        };
    end
endfunction

function [127:0] make_mwr32_header;
    input [31:0] address;
    input [7:0]  bus_number;
    input [4:0]  device_number;
    reg [127:0] header;
    begin
        header = 128'd0;
        header[9:0]   = TLP_LENGTH_DW;
        header[31:29] = 3'b010;
        header[28:24] = 5'b00000;
        header[35:32] = 4'hf;
        header[39:36] = 4'hf;
        header[47:40] = 8'h01;
        header[63:48] = {bus_number, device_number, 3'b000};
        header[95:64] = address;
        make_mwr32_header = header;
    end
endfunction

// PCIE_FIX_20260912: synchronize the pixel-domain VS signal before edge use in
// the pclk_div2 domain. The FIFO itself remains the data-domain crossing point.
always @(posedge clk) begin
    if (!rstn) begin
        vs_in_meta <= 1'b0;
        vs_in_sync <= 1'b0;
        vs_in_prev <= 1'b0;
    end
    else begin
        vs_in_meta <= vs_in;
        vs_in_sync <= vs_in_meta;
        vs_in_prev <= vs_in_sync;
    end
end

// Create a clean pixel-clock reset release and synchronize frame_active back to
// the FIFO write domain. frame_active changes only around a synchronized VS edge.
always @(posedge pix_clk_out or negedge rstn) begin
    if (!rstn) begin
        pix_rstn_meta         <= 1'b0;
        pix_rstn_sync         <= 1'b0;
        frame_active_pix_meta <= 1'b0;
        frame_active_pix      <= 1'b0;
    end
    else begin
        pix_rstn_meta         <= 1'b1;
        pix_rstn_sync         <= pix_rstn_meta;
        frame_active_pix_meta <= frame_active;
        frame_active_pix      <= frame_active_pix_meta;
    end
end

// PCIE_FIX_20260912: explicit receive framing replaces TVALID rising-edge
// detection. Back-to-back TLPs are accepted and commands are restricted to BAR1.
always @(posedge clk) begin
    if (!rstn) begin
        rx_state                    <= RX_HEADER;
        rx_cmd_addr                 <= 12'd0;
        alloc_addr_state            <= 2'd0;
        dma_addr0                   <= 32'd0;
        dma_addr1                   <= 32'd0;
        dma_addr2                   <= 32'd0;
        dma_addr3                   <= 32'd0;
        rc_cfg_ep_flag              <= 1'b0;
        dma_clear_pulse             <= 1'b0;
        video_enhance_lightdown_num <= 8'd0;
        video_enhance_lightdown_sw  <= 1'b0;
        video_enhance_darkup_num    <= 8'd0;
        video_enhance_darkup_sw     <= 1'b0;
    end
    else begin
        dma_clear_pulse <= 1'b0;

        case (rx_state)
            RX_HEADER: begin
                if (rx_handshake) begin
                    if (({axis_master_tdata[31:29], axis_master_tdata[28:24]} == MWR_32) &&
                        (axis_master_tdata[9:0] == 10'd1) &&
                        (axis_master_tuser[5:4] == 2'b01) &&
                        !axis_master_tlast) begin
                        rx_cmd_addr <= axis_master_tdata[75:64];
                        rx_state    <= RX_PAYLOAD;
                    end
                    else begin
                        rx_state <= RX_HEADER;
                    end
                end
            end

            RX_PAYLOAD: begin
                if (rx_handshake) begin
                    if (axis_master_tlast && axis_master_tkeep[0]) begin
                        case (rx_cmd_addr)
                            DMA_CMD_L_ADDR: begin
                                // Once all four addresses are committed, ignore
                                // further address writes until CLEAR prevents an
                                // accidental fifth write from rotating the list.
                                if (!rc_cfg_ep_flag) begin
                                    case (alloc_addr_state)
                                        2'd0: begin
                                            dma_addr0        <= endian32(axis_master_tdata[31:0]);
                                            alloc_addr_state <= 2'd1;
                                        end
                                        2'd1: begin
                                            dma_addr1        <= endian32(axis_master_tdata[31:0]);
                                            alloc_addr_state <= 2'd2;
                                        end
                                        2'd2: begin
                                            dma_addr2        <= endian32(axis_master_tdata[31:0]);
                                            alloc_addr_state <= 2'd3;
                                        end
                                        default: begin
                                            dma_addr3        <= endian32(axis_master_tdata[31:0]);
                                            alloc_addr_state <= 2'd0;
                                            if ((dma_addr0 != 32'd0) &&
                                                (dma_addr1 != 32'd0) &&
                                                (dma_addr2 != 32'd0) &&
                                                (endian32(axis_master_tdata[31:0]) != 32'd0)) begin
                                                rc_cfg_ep_flag <= 1'b1;
                                            end
                                        end
                                    endcase
                                end
                            end

                            DMA_CMD_CLEAR_ADDR: begin
                                alloc_addr_state <= 2'd0;
                                dma_addr0        <= 32'd0;
                                dma_addr1        <= 32'd0;
                                dma_addr2        <= 32'd0;
                                dma_addr3        <= 32'd0;
                                rc_cfg_ep_flag   <= 1'b0;
                                dma_clear_pulse  <= 1'b1;
                            end

                            DMA_CMD_VIDEO_LOWLIGHT_ADD: begin
                                // Low byte after endian32() equals the received
                                // AXIS DW's upper byte.
                                video_enhance_darkup_num <= axis_master_tdata[31:24];
                                video_enhance_darkup_sw  <= 1'b1;
                            end

                            DMA_CMD_VIDEO_HIGHTLIGHT_SUB: begin
                                video_enhance_lightdown_num <= axis_master_tdata[31:24];
                                video_enhance_lightdown_sw  <= 1'b1;
                            end

                            DMA_CMD_VIDEO_ENHANCE_CLEAR: begin
                                video_enhance_lightdown_num <= 8'd0;
                                video_enhance_lightdown_sw  <= 1'b0;
                                video_enhance_darkup_num    <= 8'd0;
                                video_enhance_darkup_sw     <= 1'b0;
                            end

                            default: begin
                            end
                        endcase
                    end

                    if (axis_master_tlast)
                        rx_state <= RX_HEADER;
                end
            end

            default: rx_state <= RX_HEADER;
        endcase
    end
end

// PCIE_FIX_20260912: a complete 64-byte payload is staged locally before TVALID
// is asserted. No FIFO read or packet counter can advance during backpressure.
always @(posedge clk) begin
    if (!rstn) begin
        tx_state        <= TX_WAIT;
        r_axis_s_tvalid <= 1'b0;
        r_axis_s_tdata  <= 128'd0;
        r_axis_s_tlast  <= 1'b0;
        pcie_rd_en      <= 1'b0;
        payload_buffer  <= 512'd0;
        fetch_index     <= 2'd0;
        data_index      <= 2'd0;
        dma_cnt         <= 16'd0;
        frame_counter   <= 8'd1;
        addr_page       <= 2'd0;
        current_addr    <= 32'd0;
        packet_addr     <= 32'd0;
        frame_addr_valid <= 1'b0;
        frame_active    <= 1'b0;
        abort_pending   <= 1'b0;
        stop_pending    <= 1'b0;
    end
    else begin
        pcie_rd_en <= 1'b0;

        if (dma_clear_pulse) begin
            frame_active     <= 1'b0;
            frame_addr_valid <= 1'b0;
            addr_page        <= 2'd0;
            current_addr     <= 32'd0;
            dma_cnt          <= 16'd0;

            // A TLP whose header is already visible must finish according to
            // AXIS. If nothing is on the wire, stop immediately.
            if (tx_tlp_in_progress) begin
                abort_pending <= 1'b1;
                stop_pending  <= 1'b1;
            end
            else begin
                tx_state        <= TX_WAIT;
                r_axis_s_tvalid <= 1'b0;
                r_axis_s_tdata  <= 128'd0;
                r_axis_s_tlast  <= 1'b0;
                fetch_index     <= 2'd0;
                data_index      <= 2'd0;
                abort_pending   <= 1'b0;
                stop_pending    <= 1'b0;
            end
        end
        else if (vs_rising) begin
            // Arm exactly one known buffer for the next frame. Configuration
            // completed after this edge waits for the following full frame.
            frame_active <= 1'b0;
            dma_cnt      <= 16'd0;

            if (rc_cfg_ep_flag) begin
                frame_addr_valid <= 1'b1;
                case (addr_page)
                    2'd0: current_addr <= dma_addr0;
                    2'd1: current_addr <= dma_addr1;
                    2'd2: current_addr <= dma_addr2;
                    default: current_addr <= dma_addr3;
                endcase
                addr_page <= addr_page + 2'd1;
            end
            else begin
                frame_addr_valid <= 1'b0;
                current_addr     <= 32'd0;
                addr_page        <= 2'd0;
            end

            if (tx_tlp_in_progress) begin
                // Finish the already presented TLP, then discard the remainder
                // of the late frame. The new buffer address is not used by it;
                // packet_addr was latched before its header was presented.
                abort_pending <= 1'b1;
            end
            else begin
                tx_state        <= TX_WAIT;
                r_axis_s_tvalid <= 1'b0;
                r_axis_s_tdata  <= 128'd0;
                r_axis_s_tlast  <= 1'b0;
                fetch_index     <= 2'd0;
                data_index      <= 2'd0;
                abort_pending   <= 1'b0;
                stop_pending    <= 1'b0;
            end
        end
        else begin
            if (vs_falling && rc_cfg_ep_flag && frame_addr_valid &&
                (tx_state == TX_WAIT) && !abort_pending && !stop_pending) begin
                frame_active <= 1'b1;
                dma_cnt      <= 16'd0;
            end

            case (tx_state)
                TX_WAIT: begin
                    r_axis_s_tvalid <= 1'b0;
                    r_axis_s_tdata  <= 128'd0;
                    r_axis_s_tlast  <= 1'b0;
                    data_index      <= 2'd0;

                    if (frame_active && rc_cfg_ep_flag) begin
                        if (dma_cnt == IMAGE_TLP_COUNT) begin
                            // One stable marker value fills the entire 64-byte
                            // marker packet; it increments only after TLAST is
                            // accepted by the PCIe core.
                            payload_buffer <= {64{frame_counter}};
                            packet_addr    <= current_addr;
                            tx_state       <= TX_HEADER;
                        end
                        else if ((rd_water_level >= 13'd4) && !fifo_rd_empty) begin
                            fetch_index <= 2'd0;
                            tx_state    <= TX_FETCH_REQUEST;
                        end
                    end
                end

                TX_FETCH_REQUEST: begin
                    if (!frame_active) begin
                        tx_state <= TX_WAIT;
                    end
                    else if (!fifo_rd_empty) begin
                        pcie_rd_en <= 1'b1;
                        tx_state   <= TX_FETCH_WAIT;
                    end
                end

                // pcie_rd_en is registered. The FIFO consumes it at the next
                // rd_clk edge and updates rd_data after that edge, so capture
                // one further clock later.
                TX_FETCH_WAIT: begin
                    if (!frame_active)
                        tx_state <= TX_WAIT;
                    else
                        tx_state <= TX_FETCH_CAPTURE;
                end

                TX_FETCH_CAPTURE: begin
                    if (!frame_active) begin
                        tx_state <= TX_WAIT;
                    end
                    else begin
                        case (fetch_index)
                            2'd0: payload_buffer[127:0]   <= endian128(pcie_dma_data);
                            2'd1: payload_buffer[255:128] <= endian128(pcie_dma_data);
                            2'd2: payload_buffer[383:256] <= endian128(pcie_dma_data);
                            default: payload_buffer[511:384] <= endian128(pcie_dma_data);
                        endcase

                        if (fetch_index == 2'd3) begin
                            packet_addr <= current_addr;
                            tx_state    <= TX_HEADER;
                        end
                        else begin
                            fetch_index <= fetch_index + 2'd1;
                            tx_state    <= TX_FETCH_REQUEST;
                        end
                    end
                end

                TX_HEADER: begin
                    if (!r_axis_s_tvalid) begin
                        r_axis_s_tdata  <= make_mwr32_header(packet_addr, ep_bus_num, ep_dev_num);
                        r_axis_s_tvalid <= 1'b1;
                        r_axis_s_tlast  <= 1'b0;
                    end
                    else if (tx_handshake) begin
                        // Header was accepted. Present payload beat 0 and keep
                        // TVALID asserted for a gap-free transfer.
                        r_axis_s_tdata  <= payload_buffer[127:0];
                        r_axis_s_tvalid <= 1'b1;
                        r_axis_s_tlast  <= 1'b0;
                        data_index      <= 2'd0;
                        tx_state        <= TX_DATA;
                    end
                end

                TX_DATA: begin
                    if (tx_handshake) begin
                        if (r_axis_s_tlast) begin
                            r_axis_s_tvalid <= 1'b0;
                            r_axis_s_tdata  <= 128'd0;
                            r_axis_s_tlast  <= 1'b0;
                            data_index      <= 2'd0;
                            tx_state        <= TX_WAIT;

                            if (abort_pending || stop_pending) begin
                                dma_cnt        <= 16'd0;
                                abort_pending <= 1'b0;
                                stop_pending  <= 1'b0;
                            end
                            else if (dma_cnt == (TOTAL_TLP_COUNT - 16'd1)) begin
                                dma_cnt      <= 16'd0;
                                frame_active <= 1'b0;
                                current_addr <= current_addr + 32'd64;
                                if (frame_counter == 8'hff)
                                    frame_counter <= 8'd1;
                                else
                                    frame_counter <= frame_counter + 8'd1;
                            end
                            else begin
                                dma_cnt      <= dma_cnt + 16'd1;
                                current_addr <= current_addr + 32'd64;
                            end
                        end
                        else begin
                            data_index <= data_index + 2'd1;
                            case (data_index)
                                2'd0: r_axis_s_tdata <= payload_buffer[255:128];
                                2'd1: r_axis_s_tdata <= payload_buffer[383:256];
                                default: r_axis_s_tdata <= payload_buffer[511:384];
                            endcase

                            // The next beat is beat 3, the final 128 bits.
                            if (data_index == 2'd2)
                                r_axis_s_tlast <= 1'b1;
                        end
                    end
                end

                default: begin
                    tx_state        <= TX_WAIT;
                    r_axis_s_tvalid <= 1'b0;
                    r_axis_s_tdata  <= 128'd0;
                    r_axis_s_tlast  <= 1'b0;
                end
            endcase
        end
    end
end

// PCIE_FIX_20260912: full/empty flags are now connected. In the expected design
// point neither should assert during active video, but the guards prevent an
// illegal FIFO access if PCIe throughput or clock startup differs from nominal.
hdmi_pcie_fifo u_hdmi_pcie_fifo (
    .wr_clk         (pix_clk_out),
    .wr_rst         (fifo_wr_rst),
    .wr_en          (fifo_wr_en),
    .wr_data        (hdmi_data_in),
    .wr_full        (fifo_wr_full),
    .wr_water_level (),
    .almost_full    (),
    .rd_clk         (clk),
    .rd_rst         (fifo_rd_rst),
    .rd_en          (pcie_rd_en),
    .rd_data        (pcie_dma_data),
    .rd_empty       (fifo_rd_empty),
    .rd_water_level (rd_water_level),
    .almost_empty   ()
);

endmodule
