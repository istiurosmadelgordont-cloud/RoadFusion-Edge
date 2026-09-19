`timescale 1ns/1ps
// PCIE_HS_20260919: four-buffer ownership protocol v2.
// Change map: docs/CODE_NOTES.md; host contract: docs/RC_HANDSHAKE_GUIDE.md.
module pcie_dma_ctrl #(
    parameter [15:0] IMAGE_TLP_COUNT = 16'd64800
)(
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


// PCIE_HS_20260919 [H1]: Register protocol. All command values use endian32.
localparam [7:0] MWR_32 = 8'h40;

localparam [11:0] CMD_ADDR         = 12'h110;
localparam [11:0] CMD_STOP         = 12'h130;
localparam [11:0] CMD_LIGHT        = 12'h140;
localparam [11:0] CMD_DARK         = 12'h150;
localparam [11:0] CMD_ENH_CLEAR    = 12'h160;
localparam [11:0] CMD_RELEASE      = 12'h170;
localparam [11:0] CMD_STATUS       = 12'h180;
localparam [11:0] CMD_START        = 12'h190;
localparam [11:0] CMD_RESET_CONFIG = 12'h1a0;

localparam [1:0] RX_HEADER  = 0;
localparam [1:0] RX_PAYLOAD = 1;
localparam [1:0] RX_DROP    = 2;

localparam [2:0] TX_WAIT          = 0;
localparam [2:0] TX_FETCH_REQUEST = 1;
localparam [2:0] TX_FETCH_WAIT    = 2;
localparam [2:0] TX_FETCH_CAPTURE = 3;
localparam [2:0] TX_HEADER        = 4;
localparam [2:0] TX_DATA          = 5;

localparam [1:0] BUF_FREE    = 0;
localparam [1:0] BUF_WRITING = 1;
localparam [1:0] BUF_HELD    = 2;

localparam [1:0] PK_IMAGE      = 0;
localparam [1:0] PK_FRAME_DONE = 1;
localparam [1:0] PK_STOP_DONE  = 2;

localparam [9:0]  TLP_LENGTH_DW = 10'd16;
localparam [32:0] BUFFER_BYTES  = ({17'd0, IMAGE_TLP_COUNT} + 33'd1) << 6;

reg [1:0]  rx_state;
reg [1:0]  alloc_addr_state;
reg [11:0] rx_cmd_addr;
reg [31:0] dma_addr0;
reg [31:0] dma_addr1;
reg [31:0] dma_addr2;
reg [31:0] dma_addr3;
reg [31:0] status_addr;
reg        rc_cfg_ep_flag;
reg        status_addr_valid;
reg        start_pulse;
reg        stop_pulse;
reg        release_pulse;
reg [31:0] release_token;
reg [31:0] stop_cookie;

reg [2:0]   tx_state;
reg         r_axis_s_tvalid;
reg         r_axis_s_tlast;
reg         pcie_rd_en;
reg [127:0] r_axis_s_tdata;
reg [511:0] payload_buffer;
reg [1:0]   fetch_index;
reg [1:0]   data_index;
reg [1:0]   packet_kind;
reg [15:0]  dma_cnt;
reg [31:0]  current_addr;
reg [31:0]  packet_addr;
reg         frame_addr_valid;
reg         frame_active;
reg         abort_pending;
reg         stop_pending;
reg         running;

// [H2] Sole writer of these ownership registers is the TX always block.
reg [1:0]  buffer_state [0:3];
reg [31:0] buffer_token [0:3];
reg [29:0] allocation_sequence;
reg [1:0]  active_slot;
reg [1:0]  addr_page;
reg [31:0] active_token;
// Preserve diagnostic counters for PDS/ILA even without a BAR readback path.
reg [31:0] dropped_frames /* synthesis PAP_MARK_DEBUG="1" */;
reg [31:0] invalid_releases /* synthesis PAP_MARK_DEBUG="1" */;
reg       free_found;
reg [1:0] free_slot;
reg [1:0] scan_slot;
integer   scan_i;
integer   reset_i;

reg vs_in_meta;
reg vs_in_sync;
reg vs_in_prev;
reg pix_rstn_meta;
reg pix_rstn_sync;
reg frame_active_pix_meta;
reg frame_active_pix;
reg overflow_pix;
reg overflow_meta;
reg overflow_sync;

wire [12:0]  rd_water_level;
wire [127:0] pcie_dma_data;
wire         fifo_wr_full /* synthesis PAP_MARK_DEBUG="1" */;
wire         fifo_rd_empty;
wire [31:0]  command_value;
wire         rx_handshake;
wire         tx_handshake;
wire         vs_rising;
wire         vs_falling;
wire         tx_tlp_in_progress;
wire         fifo_wr_en;
wire         fifo_wr_rst;
wire         fifo_rd_rst;
wire         idle_control;
wire         cancel_frame;

assign axis_master_tready = 1'b1;
assign rx_handshake       = axis_master_tvalid && axis_master_tready;

assign AXIS_S_TVALID = r_axis_s_tvalid;
assign AXIS_S_TDATA  = r_axis_s_tdata;
assign AXIS_S_TLAST  = r_axis_s_tlast;
assign AXIS_S_TUSER  = 1'b0;
assign tx_handshake  = r_axis_s_tvalid && AXIS_S_TREADY;

assign command_value = endian32(axis_master_tdata[31:0]);
assign idle_control  = !running && !stop_pending && (tx_state == TX_WAIT);
assign vs_rising     =  vs_in_sync && !vs_in_prev;
assign vs_falling    = !vs_in_sync &&  vs_in_prev;
assign cancel_frame  = vs_rising || stop_pulse || overflow_sync;

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


// [H3] 33-bit endpoint arithmetic rejects wrap and overlapping DMA ranges.
function address_valid;
    input [31:0] address;
    begin
        address_valid = (address != 0) &&
                        (address[5:0] == 0) &&
                        ({1'b0, address} + BUFFER_BYTES <= 33'h100000000);
    end
endfunction

function ranges_separate;
    input [31:0] a;
    input [31:0] b;
    begin
        ranges_separate = ({1'b0, a} + BUFFER_BYTES <= {1'b0, b}) ||
                          ({1'b0, b} + BUFFER_BYTES <= {1'b0, a});
    end
endfunction

function status_separate;
    input [31:0] a;
    input [31:0] b;
    begin
        status_separate = ({1'b0, a} + BUFFER_BYTES <= {1'b0, b}) ||
                          ({1'b0, b} + 33'd64 <= {1'b0, a});
    end
endfunction

// [H4] Round-robin arbitration skips buffers held by any RC consumer.
always @* begin
    free_found = 0;
    free_slot  = addr_page;
    scan_slot  = addr_page;

    for (scan_i = 0; scan_i < 4; scan_i = scan_i + 1) begin
        scan_slot = addr_page + scan_i;
        if (!free_found && (buffer_state[scan_slot] == BUF_FREE)) begin
            free_found = 1;
            free_slot  = scan_slot;
        end
    end
end

// Preserve the existing VS crossing and FIFO clock/reset structure.
// [H5] Overflow becomes sticky for this frame; never report it as complete.
always @(posedge pix_clk_out or negedge rstn) begin
    if (!rstn) begin
        pix_rstn_meta         <= 0;
        pix_rstn_sync         <= 0;
        frame_active_pix_meta <= 0;
        frame_active_pix      <= 0;
        overflow_pix          <= 0;
    end
    else begin
        pix_rstn_meta         <= 1;
        pix_rstn_sync         <= pix_rstn_meta;
        frame_active_pix_meta <= frame_active;
        frame_active_pix      <= frame_active_pix_meta;

        if (vs_in || !frame_active_pix)
            overflow_pix <= 0;
        else if (de_in && fifo_wr_full)
            overflow_pix <= 1;
    end
end

always @(posedge clk) begin
    if (!rstn) begin
        vs_in_meta   <= 0;
        vs_in_sync   <= 0;
        vs_in_prev   <= 0;
        overflow_meta <= 0;
        overflow_sync <= 0;
    end
    else begin
        vs_in_meta   <= vs_in;
        vs_in_sync   <= vs_in_meta;
        vs_in_prev   <= vs_in_sync;
        overflow_meta <= overflow_pix;
        overflow_sync <= overflow_meta;
    end
end

// [H6] Explicit RX framing. Unsupported TLPs drain to TLAST.
always @(posedge clk) begin
    if (!rstn) begin
        rx_state                    <= RX_HEADER;
        rx_cmd_addr                 <= 0;
        alloc_addr_state            <= 0;
        dma_addr0                   <= 0;
        dma_addr1                   <= 0;
        dma_addr2                   <= 0;
        dma_addr3                   <= 0;
        status_addr                 <= 0;
        status_addr_valid           <= 0;
        rc_cfg_ep_flag              <= 0;
        start_pulse                 <= 0;
        stop_pulse                  <= 0;
        release_pulse               <= 0;
        release_token               <= 0;
        stop_cookie                 <= 0;
        video_enhance_lightdown_num <= 0;
        video_enhance_lightdown_sw  <= 0;
        video_enhance_darkup_num    <= 0;
        video_enhance_darkup_sw     <= 0;
    end
    else begin
        start_pulse   <= 0;
        stop_pulse    <= 0;
        release_pulse <= 0;

        if (rx_handshake) begin
            case (rx_state)
                RX_HEADER: begin
                    if ((axis_master_tdata[31:24] == MWR_32) &&
                        (axis_master_tdata[9:0] == 1) &&
                        (axis_master_tuser[5:4] == 2'b01) &&
                        (axis_master_tkeep[2:0] == 3'b111) &&
                        (axis_master_tdata[39:32] == 8'h0f) &&
                        (axis_master_tdata[65:64] == 0) &&
                        !axis_master_tlast) begin
                        rx_cmd_addr <= axis_master_tdata[75:64];
                        rx_state    <= RX_PAYLOAD;
                    end
                    else if (!axis_master_tlast) begin
                        rx_state <= RX_DROP;
                    end
                end

                RX_PAYLOAD: begin
                    if (axis_master_tlast && axis_master_tkeep[0]) begin
                        case (rx_cmd_addr)
                            CMD_ADDR: begin
                                if (idle_control && !rc_cfg_ep_flag &&
                                    address_valid(command_value)) begin
                                    case (alloc_addr_state)
                                        0: begin
                                            dma_addr0        <= command_value;
                                            alloc_addr_state <= 1;
                                        end
                                        1: begin
                                            dma_addr1        <= command_value;
                                            alloc_addr_state <= 2;
                                        end
                                        2: begin
                                            dma_addr2        <= command_value;
                                            alloc_addr_state <= 3;
                                        end
                                        3: begin
                                            dma_addr3        <= command_value;
                                            alloc_addr_state <= 0;
                                            if (ranges_separate(dma_addr0, dma_addr1) &&
                                                ranges_separate(dma_addr0, dma_addr2) &&
                                                ranges_separate(dma_addr1, dma_addr2) &&
                                                ranges_separate(dma_addr0, command_value) &&
                                                ranges_separate(dma_addr1, command_value) &&
                                                ranges_separate(dma_addr2, command_value)) begin
                                                rc_cfg_ep_flag <= 1;
                                            end
                                        end
                                    endcase
                                end
                            end

                            CMD_STATUS: begin
                                if (idle_control && rc_cfg_ep_flag &&
                                    (command_value != 0) &&
                                    (command_value[5:0] == 0) &&
                                    status_separate(dma_addr0, command_value) &&
                                    status_separate(dma_addr1, command_value) &&
                                    status_separate(dma_addr2, command_value) &&
                                    status_separate(dma_addr3, command_value)) begin
                                    status_addr       <= command_value;
                                    status_addr_valid <= 1;
                                end
                            end

                            CMD_START: begin
                                if (idle_control && (command_value == 1) &&
                                    rc_cfg_ep_flag && status_addr_valid) begin
                                    start_pulse <= 1;
                                end
                            end

                            CMD_STOP: begin
                                if (status_addr_valid && !stop_pending &&
                                    (command_value != 0)) begin
                                    stop_cookie <= command_value;
                                    stop_pulse  <= 1;
                                end
                            end

                            CMD_RELEASE: begin
                                release_token <= command_value;
                                release_pulse <= 1;
                            end

                            CMD_RESET_CONFIG: begin
                                if (idle_control && (command_value == 1)) begin
                                    rc_cfg_ep_flag   <= 0;
                                    alloc_addr_state <= 0;
                                    status_addr_valid <= 0;
                                end
                            end

                            CMD_DARK: begin
                                video_enhance_darkup_num <= command_value[7:0];
                                video_enhance_darkup_sw  <= 1;
                            end

                            CMD_LIGHT: begin
                                video_enhance_lightdown_num <= command_value[7:0];
                                video_enhance_lightdown_sw  <= 1;
                            end

                            CMD_ENH_CLEAR: begin
                                video_enhance_lightdown_num <= 0;
                                video_enhance_lightdown_sw  <= 0;
                                video_enhance_darkup_num    <= 0;
                                video_enhance_darkup_sw     <= 0;
                            end

                        endcase
                    end

                    if (axis_master_tlast)
                        rx_state <= RX_HEADER;
                    else
                        rx_state <= RX_DROP;
                end

                RX_DROP: begin
                    if (axis_master_tlast)
                        rx_state <= RX_HEADER;
                end

                default: begin
                    rx_state <= RX_HEADER;
                end
            endcase
        end
    end
end

// [H7] Process AXIS handshake even if VS/STOP arrives at the same edge.
// All in-flight packet fields remain stable under backpressure.
always @(posedge clk) begin
    if (!rstn) begin
        tx_state            <= TX_WAIT;
        r_axis_s_tvalid     <= 0;
        r_axis_s_tdata      <= 0;
        r_axis_s_tlast      <= 0;
        pcie_rd_en          <= 0;
        payload_buffer      <= 0;
        fetch_index         <= 0;
        data_index          <= 0;
        dma_cnt             <= 0;
        addr_page           <= 0;
        active_slot         <= 0;
        active_token        <= 0;
        current_addr        <= 0;
        packet_addr         <= 0;
        packet_kind         <= PK_IMAGE;
        frame_addr_valid    <= 0;
        frame_active        <= 0;
        abort_pending       <= 0;
        stop_pending        <= 0;
        running             <= 0;
        allocation_sequence <= 0;
        dropped_frames      <= 0;
        invalid_releases    <= 0;

        for (reset_i = 0; reset_i < 4; reset_i = reset_i + 1) begin
            buffer_state[reset_i] <= BUF_FREE;
            buffer_token[reset_i] <= 0;
        end
    end
    else begin
        pcie_rd_en <= 0;

        // [H8] Exact token match prevents duplicate/stale releases from
        // freeing a later use of this slot. WRITING slots cannot be released.
        if (release_pulse) begin
            if (running &&
                (buffer_state[release_token[1:0]] == BUF_HELD) &&
                (buffer_token[release_token[1:0]] == release_token) &&
                (release_token[31:2] != 0)) begin
                buffer_state[release_token[1:0]] <= BUF_FREE;
            end
            else begin
                invalid_releases <= invalid_releases + 1'b1;
            end
        end

        if (start_pulse && idle_control) begin
            running          <= 1;
            addr_page        <= 0;
            frame_addr_valid <= 0;
            frame_active     <= 0;
            abort_pending    <= 0;

            // Host must finish every previous consumer before restarting.
            for (reset_i = 0; reset_i < 4; reset_i = reset_i + 1)
                buffer_state[reset_i] <= BUF_FREE;
        end

        if (stop_pulse) begin
            running      <= 0;
            stop_pending <= 1;
        end

        // [H9] Reserve at VS rising, capture at VS falling.
        // A late in-flight packet makes us skip this entire incoming frame.
        if (cancel_frame) begin
            frame_active     <= 0;
            frame_addr_valid <= 0;

            if (tx_tlp_in_progress && (packet_kind != PK_STOP_DONE))
                abort_pending <= 1;

            if (!tx_tlp_in_progress &&
                (buffer_state[active_slot] == BUF_WRITING)) begin
                buffer_state[active_slot] <= BUF_FREE;
            end
        end

        if (vs_rising && running && !stop_pending && !stop_pulse) begin
            if (!tx_tlp_in_progress && !abort_pending && free_found &&
                (allocation_sequence != 30'h3fffffff)) begin
                active_slot            <= free_slot;
                active_token           <= {allocation_sequence + 30'd1, free_slot};
                buffer_token[free_slot] <= {allocation_sequence + 30'd1, free_slot};
                allocation_sequence    <= allocation_sequence + 1'b1;
                buffer_state[free_slot] <= BUF_WRITING;
                addr_page              <= free_slot + 1'b1;
                frame_addr_valid       <= 1;
                dma_cnt                <= 0;

                case (free_slot)
                    0: current_addr <= dma_addr0;
                    1: current_addr <= dma_addr1;
                    2: current_addr <= dma_addr2;
                    3: current_addr <= dma_addr3;
                endcase
            end
            else begin
                dropped_frames <= dropped_frames + 1'b1;
            end
        end

        if (vs_falling && running && frame_addr_valid && !stop_pending &&
            !stop_pulse && !overflow_sync && (tx_state == TX_WAIT) &&
            !abort_pending) begin
            frame_active <= 1;
        end

        case (tx_state)
            TX_WAIT: begin
                r_axis_s_tvalid <= 0;
                r_axis_s_tlast  <= 0;

                if (stop_pending) begin
                    // [H10] Echo STOP cookie only after preceding writes drain.
                    payload_buffer <= {16{endian32(stop_cookie)}};
                    packet_addr    <= status_addr;
                    packet_kind    <= PK_STOP_DONE;
                    tx_state       <= TX_HEADER;
                end
                else if (frame_active && running && !cancel_frame) begin
                    if (dma_cnt == IMAGE_TLP_COUNT) begin
                        // [H11] Frame tail: 16 identical LE32 ownership tokens.
                        payload_buffer <= {16{endian32(active_token)}};
                        packet_addr    <= current_addr;
                        packet_kind    <= PK_FRAME_DONE;
                        tx_state       <= TX_HEADER;
                    end
                    else if ((rd_water_level >= 4) && !fifo_rd_empty) begin
                        fetch_index <= 0;
                        packet_kind <= PK_IMAGE;
                        tx_state    <= TX_FETCH_REQUEST;
                    end
                end
            end

            TX_FETCH_REQUEST: begin
                if (cancel_frame || !frame_active) begin
                    tx_state <= TX_WAIT;
                end
                else if (!fifo_rd_empty) begin
                    pcie_rd_en <= 1;
                    tx_state   <= TX_FETCH_WAIT;
                end
            end

            TX_FETCH_WAIT: begin
                if (cancel_frame || !frame_active)
                    tx_state <= TX_WAIT;
                else
                    tx_state <= TX_FETCH_CAPTURE;
            end

            TX_FETCH_CAPTURE: begin
                if (cancel_frame || !frame_active) begin
                    tx_state <= TX_WAIT;
                end
                else begin
                    case (fetch_index)
                        0: payload_buffer[127:0]   <= endian128(pcie_dma_data);
                        1: payload_buffer[255:128] <= endian128(pcie_dma_data);
                        2: payload_buffer[383:256] <= endian128(pcie_dma_data);
                        3: payload_buffer[511:384] <= endian128(pcie_dma_data);
                    endcase

                    if (fetch_index == 3) begin
                        packet_addr <= current_addr;
                        tx_state    <= TX_HEADER;
                    end
                    else begin
                        fetch_index <= fetch_index + 1'b1;
                        tx_state    <= TX_FETCH_REQUEST;
                    end
                end
            end

            TX_HEADER: begin
                if (!r_axis_s_tvalid) begin
                    if ((cancel_frame || !frame_active) &&
                        (packet_kind != PK_STOP_DONE)) begin
                        tx_state <= TX_WAIT;
                    end
                    else begin
                        r_axis_s_tdata  <= make_mwr32_header(
                            packet_addr, ep_bus_num, ep_dev_num
                        );
                        r_axis_s_tvalid <= 1;
                        r_axis_s_tlast  <= 0;
                    end
                end
                else if (tx_handshake) begin
                    r_axis_s_tdata <= payload_buffer[127:0];
                    r_axis_s_tlast <= 0;
                    data_index     <= 0;
                    tx_state       <= TX_DATA;
                end
            end

            TX_DATA: begin
                if (tx_handshake) begin
                    if (r_axis_s_tlast) begin
                        r_axis_s_tvalid <= 0;
                        r_axis_s_tlast  <= 0;
                        tx_state        <= TX_WAIT;
                        abort_pending   <= 0;

                        if (packet_kind == PK_STOP_DONE) begin
                            stop_pending     <= 0;
                            frame_active     <= 0;
                            frame_addr_valid <= 0;

                            for (reset_i = 0; reset_i < 4; reset_i = reset_i + 1)
                                buffer_state[reset_i] <= BUF_FREE;
                        end
                        else if (packet_kind == PK_FRAME_DONE) begin
                        // A presented completion TLP finishes even at VS/STOP.
                            buffer_state[active_slot] <= BUF_HELD;
                            frame_active              <= 0;
                            frame_addr_valid          <= 0;
                        end
                        else if (abort_pending || cancel_frame || !running) begin
                            buffer_state[active_slot] <= BUF_FREE;
                            frame_active              <= 0;
                            frame_addr_valid          <= 0;
                            dma_cnt                   <= 0;
                        end
                        else begin
                            dma_cnt      <= dma_cnt + 1'b1;
                            current_addr <= current_addr + 32'd64;
                        end
                    end
                    else begin
                        data_index <= data_index + 1'b1;
                        case (data_index)
                            0: r_axis_s_tdata <= payload_buffer[255:128];
                            1: r_axis_s_tdata <= payload_buffer[383:256];
                            default: r_axis_s_tdata <= payload_buffer[511:384];
                        endcase

                        if (data_index == 2)
                            r_axis_s_tlast <= 1;
                    end
                end
            end

            default: begin
                tx_state <= TX_WAIT;
            end
        endcase
    end
end

// Original IP instance and pixel layout are preserved.
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
