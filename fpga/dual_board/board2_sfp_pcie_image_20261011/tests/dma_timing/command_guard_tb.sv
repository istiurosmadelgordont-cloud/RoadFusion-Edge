`timescale 1ns / 1ps
// Command-only equivalence regression. The local FIFO is a controller fixture;
// true vendor FIFO data/CDC validation is in dma_timing_vendor_tb.sv.
module command_guard_tb #(
    parameter integer IMAGE_PACKETS = 2
);

    localparam [32 : 0] BUFFER_BYTES = (IMAGE_PACKETS + 33'd1) << 6;
    localparam [31 : 0] LOW_BASE     = 32'h01000000;
    localparam [31 : 0] MAX_BASE     = 33'h100000000 - BUFFER_BYTES;

    reg                     clk         = 0;
    reg                     pix_clk     = 0;
    reg                     rstn        = 0;
    reg                     rx_valid    = 0;
    reg     [127 : 0]       rx_data     = 0;
    reg     [3 : 0]         rx_keep     = 0;
    reg                     rx_last     = 0;
    reg     [7 : 0]         rx_user     = 0;
    reg                     tx_ready    = 1;
    reg     [31 : 0]        ack_addr    = 0;
    reg                     ack_valid   = 0;
    reg                     test_passed = 0;
    reg     [31 : 0]        random_seed = 32'h6ad3812f;
    integer                 cycles      = 0;
    integer                 comparisons = 0;
    integer                 cmd_pulses  = 0;
    integer                 drop_cycles = 0;
    integer                 late_ack    = 0;
    integer                 directed    = 0;
    integer                 group_i;
    integer                 pair_i;
    integer                 pair_j;
    integer                 slot_i;
    reg     [31 : 0]        address;
    reg     [31 : 0]        random_word;
    reg     [31 : 0]        random_ack;
    reg                     random_valid;
    reg     [31 : 0]        addresses [0 : 3];

    always #4 clk = !clk;
    always #3.367 pix_clk = !pix_clk;

    wire                       axis_master_tready_dut;
    wire                       AXIS_S_TVALID_dut;
    wire    [127 : 0]          AXIS_S_TDATA_dut;
    wire                       AXIS_S_TLAST_dut;
    wire                       AXIS_S_TUSER_dut;
    wire                       img_cmd_valid_dut;
    wire    [11 : 0]           img_cmd_addr_dut;
    wire    [31 : 0]           img_cmd_data_dut;
    wire    [31 : 0]           img_dma_addr0_dut;
    wire    [31 : 0]           img_dma_addr1_dut;
    wire    [31 : 0]           img_dma_addr2_dut;
    wire    [31 : 0]           img_dma_addr3_dut;
    wire    [31 : 0]           img_dma_status_addr_dut;
    wire                       img_dma_config_valid_dut;
    wire                       img_dma_status_valid_dut;
    wire    [7 : 0]            video_enhance_lightdown_num_dut;
    wire                       video_enhance_lightdown_sw_dut;
    wire    [7 : 0]            video_enhance_darkup_num_dut;
    wire                       video_enhance_darkup_sw_dut;

    wire                       axis_master_tready_ref;
    wire                       AXIS_S_TVALID_ref;
    wire    [127 : 0]          AXIS_S_TDATA_ref;
    wire                       AXIS_S_TLAST_ref;
    wire                       AXIS_S_TUSER_ref;
    wire                       img_cmd_valid_ref;
    wire    [11 : 0]           img_cmd_addr_ref;
    wire    [31 : 0]           img_cmd_data_ref;
    wire    [31 : 0]           img_dma_addr0_ref;
    wire    [31 : 0]           img_dma_addr1_ref;
    wire    [31 : 0]           img_dma_addr2_ref;
    wire    [31 : 0]           img_dma_addr3_ref;
    wire    [31 : 0]           img_dma_status_addr_ref;
    wire                       img_dma_config_valid_ref;
    wire                       img_dma_status_valid_ref;
    wire    [7 : 0]            video_enhance_lightdown_num_ref;
    wire                       video_enhance_lightdown_sw_ref;
    wire    [7 : 0]            video_enhance_darkup_num_ref;
    wire                       video_enhance_darkup_sw_ref;

    pcie_dma_ctrl #(
        .IMAGE_TLP_COUNT      (IMAGE_PACKETS),
        .IMAGE_CONTROL_ENABLE (1)
    ) dut (
        .clk                          (clk),
        .pix_clk_out                  (pix_clk),
        .rstn                         (rstn),
        .axis_master_tvalid           (rx_valid),
        .axis_master_tdata            (rx_data),
        .axis_master_tkeep            (rx_keep),
        .axis_master_tlast            (rx_last),
        .axis_master_tuser            (rx_user),
        .ep_bus_num                   (8'h35),
        .ep_dev_num                   (5'h0a),
        .AXIS_S_TREADY                (tx_ready),
        .img_ack_addr                 (ack_addr),
        .img_ack_addr_valid           (ack_valid),
        .hdmi_data_in                 (16'd0),
        .vs_in                        (1'b0),
        .de_in                        (1'b0),
        .axis_master_tready           (axis_master_tready_dut),
        .AXIS_S_TVALID                (AXIS_S_TVALID_dut),
        .AXIS_S_TDATA                 (AXIS_S_TDATA_dut),
        .AXIS_S_TLAST                 (AXIS_S_TLAST_dut),
        .AXIS_S_TUSER                 (AXIS_S_TUSER_dut),
        .img_cmd_valid                (img_cmd_valid_dut),
        .img_cmd_addr                 (img_cmd_addr_dut),
        .img_cmd_data                 (img_cmd_data_dut),
        .img_dma_addr0                (img_dma_addr0_dut),
        .img_dma_addr1                (img_dma_addr1_dut),
        .img_dma_addr2                (img_dma_addr2_dut),
        .img_dma_addr3                (img_dma_addr3_dut),
        .img_dma_status_addr          (img_dma_status_addr_dut),
        .img_dma_config_valid         (img_dma_config_valid_dut),
        .img_dma_status_valid         (img_dma_status_valid_dut),
        .video_enhance_lightdown_num  (video_enhance_lightdown_num_dut),
        .video_enhance_lightdown_sw   (video_enhance_lightdown_sw_dut),
        .video_enhance_darkup_num     (video_enhance_darkup_num_dut),
        .video_enhance_darkup_sw      (video_enhance_darkup_sw_dut)
    );

    pcie_dma_ctrl_timing_reference #(
        .IMAGE_TLP_COUNT      (IMAGE_PACKETS),
        .IMAGE_CONTROL_ENABLE (1)
    ) oracle (
        .clk                          (clk),
        .pix_clk_out                  (pix_clk),
        .rstn                         (rstn),
        .axis_master_tvalid           (rx_valid),
        .axis_master_tdata            (rx_data),
        .axis_master_tkeep            (rx_keep),
        .axis_master_tlast            (rx_last),
        .axis_master_tuser            (rx_user),
        .ep_bus_num                   (8'h35),
        .ep_dev_num                   (5'h0a),
        .AXIS_S_TREADY                (tx_ready),
        .img_ack_addr                 (ack_addr),
        .img_ack_addr_valid           (ack_valid),
        .hdmi_data_in                 (16'd0),
        .vs_in                        (1'b0),
        .de_in                        (1'b0),
        .axis_master_tready           (axis_master_tready_ref),
        .AXIS_S_TVALID                (AXIS_S_TVALID_ref),
        .AXIS_S_TDATA                 (AXIS_S_TDATA_ref),
        .AXIS_S_TLAST                 (AXIS_S_TLAST_ref),
        .AXIS_S_TUSER                 (AXIS_S_TUSER_ref),
        .img_cmd_valid                (img_cmd_valid_ref),
        .img_cmd_addr                 (img_cmd_addr_ref),
        .img_cmd_data                 (img_cmd_data_ref),
        .img_dma_addr0                (img_dma_addr0_ref),
        .img_dma_addr1                (img_dma_addr1_ref),
        .img_dma_addr2                (img_dma_addr2_ref),
        .img_dma_addr3                (img_dma_addr3_ref),
        .img_dma_status_addr          (img_dma_status_addr_ref),
        .img_dma_config_valid         (img_dma_config_valid_ref),
        .img_dma_status_valid         (img_dma_status_valid_ref),
        .video_enhance_lightdown_num  (video_enhance_lightdown_num_ref),
        .video_enhance_lightdown_sw   (video_enhance_lightdown_sw_ref),
        .video_enhance_darkup_num     (video_enhance_darkup_num_ref),
        .video_enhance_darkup_sw      (video_enhance_darkup_sw_ref)
    );

    wire [511 : 0] public_dut = {
        axis_master_tready_dut,
        AXIS_S_TVALID_dut,
        AXIS_S_TDATA_dut,
        AXIS_S_TLAST_dut,
        AXIS_S_TUSER_dut,
        img_cmd_valid_dut,
        img_cmd_addr_dut,
        img_cmd_data_dut,
        img_dma_addr0_dut,
        img_dma_addr1_dut,
        img_dma_addr2_dut,
        img_dma_addr3_dut,
        img_dma_status_addr_dut,
        img_dma_config_valid_dut,
        img_dma_status_valid_dut,
        video_enhance_lightdown_num_dut,
        video_enhance_lightdown_sw_dut,
        video_enhance_darkup_num_dut,
        video_enhance_darkup_sw_dut
    };

    wire [2047 : 0] state_dut = {
        dut.rx_state,
        dut.rx_cmd_addr,
        dut.alloc_addr_state,
        dut.start_pulse,
        dut.stop_pulse,
        dut.release_pulse,
        dut.release_token,
        dut.stop_cookie,
        dut.running,
        dut.stop_pending,
        dut.tx_state,
        dut.frame_active,
        dut.abort_pending,
        dut.allocation_sequence,
        dut.active_slot,
        dut.addr_page,
        dut.active_token,
        dut.dropped_frames,
        dut.invalid_releases,
        dut.r_axis_s_tvalid,
        dut.r_axis_s_tlast,
        dut.r_axis_s_tdata,
        dut.pcie_rd_en,
        dut.payload_buffer,
        dut.fetch_index,
        dut.data_index,
        dut.packet_kind,
        dut.dma_cnt,
        dut.current_addr,
        dut.packet_addr,
        dut.frame_addr_valid,
        dut.vs_in_meta,
        dut.vs_in_sync,
        dut.vs_in_prev,
        dut.pix_rstn_meta,
        dut.pix_rstn_sync,
        dut.frame_active_pix_meta,
        dut.frame_active_pix,
        dut.overflow_pix,
        dut.overflow_meta,
        dut.overflow_sync,
        dut.fifo_wr_rst,
        dut.fifo_wr_en,
        dut.fifo_rd_rst,
        dut.buffer_state[0],
        dut.buffer_state[1],
        dut.buffer_state[2],
        dut.buffer_state[3],
        dut.buffer_token[0],
        dut.buffer_token[1],
        dut.buffer_token[2],
        dut.buffer_token[3]
    };

    wire [511 : 0] public_ref = {
        axis_master_tready_ref,
        AXIS_S_TVALID_ref,
        AXIS_S_TDATA_ref,
        AXIS_S_TLAST_ref,
        AXIS_S_TUSER_ref,
        img_cmd_valid_ref,
        img_cmd_addr_ref,
        img_cmd_data_ref,
        img_dma_addr0_ref,
        img_dma_addr1_ref,
        img_dma_addr2_ref,
        img_dma_addr3_ref,
        img_dma_status_addr_ref,
        img_dma_config_valid_ref,
        img_dma_status_valid_ref,
        video_enhance_lightdown_num_ref,
        video_enhance_lightdown_sw_ref,
        video_enhance_darkup_num_ref,
        video_enhance_darkup_sw_ref
    };

    wire [2047 : 0] state_ref = {
        oracle.rx_state,
        oracle.rx_cmd_addr,
        oracle.alloc_addr_state,
        oracle.start_pulse,
        oracle.stop_pulse,
        oracle.release_pulse,
        oracle.release_token,
        oracle.stop_cookie,
        oracle.running,
        oracle.stop_pending,
        oracle.tx_state,
        oracle.frame_active,
        oracle.abort_pending,
        oracle.allocation_sequence,
        oracle.active_slot,
        oracle.addr_page,
        oracle.active_token,
        oracle.dropped_frames,
        oracle.invalid_releases,
        oracle.r_axis_s_tvalid,
        oracle.r_axis_s_tlast,
        oracle.r_axis_s_tdata,
        oracle.pcie_rd_en,
        oracle.payload_buffer,
        oracle.fetch_index,
        oracle.data_index,
        oracle.packet_kind,
        oracle.dma_cnt,
        oracle.current_addr,
        oracle.packet_addr,
        oracle.frame_addr_valid,
        oracle.vs_in_meta,
        oracle.vs_in_sync,
        oracle.vs_in_prev,
        oracle.pix_rstn_meta,
        oracle.pix_rstn_sync,
        oracle.frame_active_pix_meta,
        oracle.frame_active_pix,
        oracle.overflow_pix,
        oracle.overflow_meta,
        oracle.overflow_sync,
        oracle.fifo_wr_rst,
        oracle.fifo_wr_en,
        oracle.fifo_rd_rst,
        oracle.buffer_state[0],
        oracle.buffer_state[1],
        oracle.buffer_state[2],
        oracle.buffer_state[3],
        oracle.buffer_token[0],
        oracle.buffer_token[1],
        oracle.buffer_token[2],
        oracle.buffer_token[3]
    };

    // Observe settled NBA results on every PCIe cycle, including reset and
    // malformed/drop traffic. No tolerance for a one-cycle public delay shift.
    always@(posedge clk)begin
        #1;
        cycles = cycles + 1;
        if(cycles > 2)begin
            comparisons = comparisons + 1;
            if(public_dut !== public_ref)begin
                $display("PUBLIC MISMATCH cycle=%0d rx_state=%0d command=%03h data=%08h",
                         cycles, dut.rx_state, dut.img_cmd_addr, dut.img_cmd_data);
                $fatal(1, "All public controller outputs must match on the same cycle");
            end
            if(state_dut !== state_ref)begin
                $display("STATE MISMATCH cycle=%0d alloc=%0d/%0d rx=%0d/%0d",
                         cycles, dut.alloc_addr_state, oracle.alloc_addr_state,
                         dut.rx_state, oracle.rx_state);
                $fatal(1, "Parser, TX, ownership, reset and CDC state differ");
            end
            if(dut.fifo_write_ready !== !dut.fifo_wr_rst)begin
                $fatal(1, "Preserved write-ready register lost complementary reset relation");
            end
            if(dut.img_cmd_valid)begin
                cmd_pulses = cmd_pulses + 1;
            end
            if(dut.rx_state == 2)begin
                drop_cycles = drop_cycles + 1;
            end
            if(dut.AXIS_S_TVALID || oracle.AXIS_S_TVALID)begin
                $fatal(1, "Command-only fixture must not emit an image/status TLP");
            end
        end
    end

    always@(posedge pix_clk)begin
        #1;
        if(cycles > 2)begin
            if({dut.fifo_wr_rst, dut.fifo_wr_en, dut.frame_active_pix, dut.overflow_pix} !==
               {oracle.fifo_wr_rst, oracle.fifo_wr_en, oracle.frame_active_pix, oracle.overflow_pix})begin
                $fatal(1, "Pixel-side write/reset differs with no active image");
            end
        end
    end

    function [31 : 0] endian32;
        input [31 : 0] value;
        begin
            endian32 = {value[7 : 0], value[15 : 8], value[23 : 16], value[31 : 24]};
        end
    endfunction

    function [127 : 0] header;
        input [11 : 0] register_address;
        reg   [127 : 0] value;
        begin
            value = 0;
            value[31 : 24] = 8'h40;
            value[9 : 0]   = 1;
            value[39 : 32] = 8'h0f;
            value[75 : 64] = register_address;
            header = value;
        end
    endfunction

    function [31 : 0] next_random;
        reg [31 : 0] value;
        begin
            value = random_seed ^ (random_seed << 13);
            value = value ^ (value >> 17);
            value = value ^ (value << 5);
            random_seed = value;
            next_random = value;
        end
    endfunction

    task idle;
        begin
            @(negedge clk);
            rx_valid = 0;
            rx_data  = 0;
            rx_keep  = 0;
            rx_last  = 0;
            rx_user  = 0;
        end
    endtask

    task flush;
        begin
            idle;
            repeat(4) @(negedge clk);
        end
    endtask

    // Calling this task consecutively yields header,payload,header,payload,
    // without an idle cycle. The real strict MWr32 BAR1 encoding is exercised.
    task command;
        input [11 : 0] register_address;
        input [31 : 0] value;
        begin
            @(negedge clk);
            rx_valid = 1;
            rx_data  = header(register_address);
            rx_keep  = 4'h7;
            rx_last  = 0;
            rx_user  = 8'h10;
            @(negedge clk);
            rx_data  = {96'd0, endian32(value)};
            rx_keep  = 4'h1;
            rx_last  = 1;
        end
    endtask

    task clear_config;
        begin
            command(12'h1a0, 1);
            flush;
            if(dut.alloc_addr_state != 0 || dut.rc_cfg_ep_flag || dut.status_addr_valid)begin
                $fatal(1, "RESET_CONFIG must clear the idle allocation state");
            end
        end
    endtask

    task hardware_reset;
        begin
            idle;
            rstn = 0;
            repeat(5) @(negedge clk);
            ack_addr  = 0;
            ack_valid = 0;
            rstn = 1;
            repeat(6) @(negedge clk);
        end
    endtask

    task expect_alloc;
        input [1 : 0] expected;
        begin
            if(dut.alloc_addr_state !== expected)begin
                $display("ALLOC expected=%0d got=%0d A=%08h ACK=%08h valid=%0d",
                         expected, dut.alloc_addr_state, dut.dma_addr0, ack_addr, ack_valid);
                $fatal(1, "Directed address validation expectation failed");
            end
            directed = directed + 1;
        end
    endtask

    task single_address;
        input [31 : 0] value;
        input          accepted;
        begin
            clear_config;
            command(12'h110, value);
            flush;
            expect_alloc(accepted ? 1 : 0);
        end
    endtask

    task configured_low_buffers;
        begin
            ack_valid = 0;
            clear_config;
            command(12'h110, LOW_BASE);
            command(12'h110, LOW_BASE + BUFFER_BYTES);
            command(12'h110, LOW_BASE + BUFFER_BYTES * 2);
            command(12'h110, LOW_BASE + BUFFER_BYTES * 3);
            flush;
            if(!dut.rc_cfg_ep_flag)begin
                $fatal(1, "Four exactly touching buffers must configure successfully");
            end
            directed = directed + 1;
        end
    endtask

    task status_case;
        input [31 : 0] value;
        input          accepted;
        begin
            configured_low_buffers;
            command(12'h180, value);
            flush;
            if(dut.status_addr_valid !== accepted)begin
                $fatal(1, "Directed 64-byte status boundary expectation failed");
            end
            if(accepted && dut.status_addr !== value)begin
                $fatal(1, "Accepted status address data changed");
            end
            directed = directed + 1;
        end
    endtask

    // ACK changes after the payload has been clocked into rx_stage_tdata.
    // The parser consumes that staged payload on the following positive edge.
    // Any refactor that sampled ACK together with payload would fail here.
    task late_ack_command;
        input [11 : 0] register_address;
        input [31 : 0] value;
        input [31 : 0] new_ack;
        input          new_valid;
        begin
            command(register_address, value);
            idle;
            if(dut.rx_stage_tdata[31 : 0] !== endian32(value) || !dut.rx_stage_tvalid)begin
                $fatal(1, "Late ACK test did not reach the payload input-stage boundary");
            end
            ack_addr  = new_ack;
            ack_valid = new_valid;
            late_ack  = late_ack + 1;
            repeat(4) @(negedge clk);
        end
    endtask

    task malformed;
        input integer mode;
        reg [127 : 0] bad_header;
        begin
            bad_header = header(12'h110);
            @(negedge clk);
            rx_valid = 1;
            rx_data  = bad_header;
            rx_keep  = 7;
            rx_last  = 0;
            rx_user  = 8'h10;
            case(mode)
                0: rx_keep = 3;
                1: rx_data[31 : 24] = 8'h00;
                2: rx_data[9 : 0] = 2;
                3: rx_user[5 : 4] = 0;
                4: rx_data[39 : 32] = 8'h07;
                5: rx_data[65 : 64] = 2;
                6: rx_last = 1;
                default: begin end
            endcase
            @(negedge clk);
            rx_data = {96'd0, endian32(LOW_BASE)};
            rx_keep = mode == 7 ? 0 : 1;
            rx_last = mode == 8 ? 0 : 1;
            if(mode == 8 || mode <= 5)begin
                // Drain body containing a plausible header and payload. Neither
                // may escape RX_DROP and be interpreted as a new command.
                @(negedge clk);
                rx_data = header(12'h150);
                rx_keep = 7;
                rx_last = 0;
                @(negedge clk);
                rx_data = {96'd0, endian32(32'h1234)};
                rx_keep = 1;
                rx_last = 1;
            end
            flush;
        end
    endtask

    initial begin
        hardware_reset;

        // Zero, byte alignment, exact 4 GiB endpoint and overflow boundaries.
        single_address(0, 0);
        single_address(32'h40, 1);
        single_address(LOW_BASE + 1, 0);
        single_address(LOW_BASE + 63, 0);
        single_address(MAX_BASE, 1);
        single_address(MAX_BASE + 64, 0);
        single_address(32'hffffffc0, 0);

        // ACK half-open interval adjacency and high ACK endpoint carry.
        ack_valid = 1;
        ack_addr = LOW_BASE + BUFFER_BYTES;
        single_address(LOW_BASE, 1);
        ack_addr = LOW_BASE - 64;
        single_address(LOW_BASE, 1);
        ack_addr = LOW_BASE;
        single_address(LOW_BASE, 0);
        ack_addr = LOW_BASE + 64;
        single_address(LOW_BASE, 0);
        ack_addr = 32'hffffffe0;
        single_address(MAX_BASE, 0);
        ack_addr = 32'hffffffc0;
        single_address(LOW_BASE, 1);
        ack_valid = 0;

        // Every one of the six pairwise overlap checks must block publishing
        // the four-address configuration. Also test exactly touching ranges.
        configured_low_buffers;
        for(pair_i = 0; pair_i < 4; pair_i = pair_i + 1)begin
            for(pair_j = pair_i + 1; pair_j < 4; pair_j = pair_j + 1)begin
                clear_config;
                for(slot_i = 0; slot_i < 4; slot_i = slot_i + 1)begin
                    addresses[slot_i] = LOW_BASE + BUFFER_BYTES * slot_i;
                end
                addresses[pair_j] = addresses[pair_i] + BUFFER_BYTES - 64;
                for(slot_i = 0; slot_i < 4; slot_i = slot_i + 1)begin
                    command(12'h110, addresses[slot_i]);
                end
                flush;
                if(dut.rc_cfg_ep_flag)begin
                    $fatal(1, "Overlapping four-buffer allocation was accepted");
                end
                directed = directed + 1;
            end
        end

        status_case(0, 0);
        status_case(LOW_BASE + 1, 0);
        status_case(LOW_BASE, 0);
        status_case(LOW_BASE - 64, 1);
        status_case(LOW_BASE + BUFFER_BYTES * 4, 1);
        status_case(32'hffffffc0, 1);
        configured_low_buffers;
        ack_addr  = 32'hffffffe0;
        ack_valid = 1;
        command(12'h180, 32'hffffffc0);
        flush;
        if(dut.status_addr_valid)begin
            $fatal(1, "High overflowing ACK page overlapped status page");
        end
        directed = directed + 1;

        // Live ACK instead of payload-time ACK, tested in both directions,
        // including changes of ACK-valid and a 33-bit endpoint carry.
        ack_valid = 0;
        clear_config;
        ack_addr  = LOW_BASE;
        ack_valid = 1;
        late_ack_command(12'h110, LOW_BASE, LOW_BASE + BUFFER_BYTES, 1);
        expect_alloc(1);
        clear_config;
        ack_addr = LOW_BASE + BUFFER_BYTES;
        late_ack_command(12'h110, LOW_BASE, LOW_BASE, 1);
        expect_alloc(0);
        clear_config;
        ack_addr  = LOW_BASE;
        ack_valid = 0;
        late_ack_command(12'h110, LOW_BASE, LOW_BASE, 1);
        expect_alloc(0);
        clear_config;
        ack_valid = 1;
        late_ack_command(12'h110, LOW_BASE, LOW_BASE, 0);
        expect_alloc(1);
        clear_config;
        ack_valid = 0;
        late_ack_command(12'h110, MAX_BASE, 32'hffffffe0, 1);
        expect_alloc(0);
        configured_low_buffers;
        ack_addr  = LOW_BASE - 64;
        ack_valid = 1;
        late_ack_command(12'h180, LOW_BASE - 64, 32'hffffffc0, 1);
        if(!dut.status_addr_valid)begin
            $fatal(1, "Live ACK moved away from status page but status was rejected");
        end
        directed = directed + 1;
        configured_low_buffers;
        late_ack_command(12'h180, 32'hffffffc0, 32'hffffffe0, 1);
        if(dut.status_addr_valid)begin
            $fatal(1, "Live high ACK overlap failed to reject status page");
        end
        directed = directed + 1;

        ack_valid = 0;
        for(slot_i = 0; slot_i < 9; slot_i = slot_i + 1)begin
            clear_config;
            malformed(slot_i);
            expect_alloc(0);
            if(dut.rx_state != 0)begin
                $fatal(1, "Malformed TLP did not return to HEADER after drain");
            end
        end

        // 2,000 deterministic randomized groups. Includes late live ACK,
        // unaligned/address-wrap candidates, four buffers, status, unknown
        // image-control registers, enhancement controls and malformed drains.
        for(group_i = 0; group_i < 2000; group_i = group_i + 1)begin
            clear_config;
            random_word = next_random();
            address = random_word & 32'hffffffc0;
            case(group_i % 7)
                0: address = MAX_BASE;
                1: address = MAX_BASE + 64;
                2: address = 0;
                3: address = random_word;
                default: begin end
            endcase
            ack_addr = next_random();
            ack_valid = random_word[0];
            random_ack = next_random();
            random_valid = random_word[1];
            if(group_i % 4 == 0)begin
                random_ack = address;
                random_valid = 1;
            end
            late_ack_command(12'h110, address, random_ack, random_valid);

            configured_low_buffers;
            random_word = next_random();
            case(group_i % 8)
                0: address = LOW_BASE - 64;
                1: address = LOW_BASE;
                2: address = LOW_BASE + BUFFER_BYTES * 4;
                3: address = 32'hffffffc0;
                4: address = 0;
                default: address = random_word;
            endcase
            ack_addr = next_random();
            ack_valid = random_word[0];
            random_ack = next_random();
            random_valid = random_word[1];
            if(group_i % 5 == 0)begin
                random_ack = address;
                random_valid = 1;
            end
            late_ack_command(12'h180, address, random_ack, random_valid);

            // Invalid START values and zero STOP keep this regression command
            // only. Real start/stop/status data paths use the vendor FIFO test.
            command(12'h190, 2);
            command(12'h130, 0);
            command(12'h170, next_random());
            command(12'h140, next_random());
            command(12'h150, next_random());
            command(12'h2f0, next_random());
            command(12'h160, 1);
            flush;
            malformed(group_i % 9);
            if(group_i % 127 == 0)begin
                // Reset while a valid header, payload or DROP is in flight.
                @(negedge clk);
                rx_valid = 1;
                rx_data  = header(12'h110);
                rx_keep  = group_i % 3 == 0 ? 0 : 7;
                rx_last  = 0;
                rx_user  = 8'h10;
                if(group_i % 3 == 1)begin
                    @(negedge clk);
                    rx_data = {96'd0, endian32(LOW_BASE)};
                    rx_keep = 1;
                    rx_last = 1;
                end
                hardware_reset;
            end
        end
        flush;
        if(comparisons < 50000 || cmd_pulses < 20000 || late_ack != 4007 || drop_cycles == 0)begin
            $fatal(1, "Coverage counters did not reach required guard regression workload");
        end
        test_passed = 1;
        $display("TEST PASSED command_guard IMAGE_PACKETS=%0d cycles=%0d comparisons=%0d commands=%0d",
                 IMAGE_PACKETS, cycles, comparisons, cmd_pulses);
        $display("TEST PASSED directed=%0d late_ACK=%0d random_groups=2000 drop_cycles=%0d",
                 directed, late_ack, drop_cycles);
        $finish;
    end

    initial begin
        #20000000;
        $fatal(1, "Command guard regression timed out");
    end

endmodule
