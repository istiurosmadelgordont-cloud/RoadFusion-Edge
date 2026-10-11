`timescale 1ns / 1ps
module pcie_image_control_tb;
    localparam [159 : 0] RESET = 160'h0080c040020301900000410000f38034000001c0;
    localparam [159 : 0] FIRST = 160'h0080c04002030190c040410000f38034000019e1;
    localparam [159 : 0] SECOND = 160'h0080c04002030190d060410000e0805000001882;
    reg clk = 0;
    reg pix = 0;
    reg rst_n = 0;
    reg image_rst_n = 0;
    reg image_ready = 0;
    reg frame_start = 0;
    reg vs = 0;
    reg de = 0;
    always #5 clk = ~clk;
    always #7 pix = ~pix;

    reg rx_valid = 0;
    reg rx_last = 0;
    reg [127 : 0] rx_data = 0;
    reg [3 : 0] rx_keep = 0;
    reg [7 : 0] rx_user = 0;
    wire rx_ready;
    wire command_valid;
    wire [11 : 0] command_addr;
    wire [31 : 0] command_data;
    wire [31 : 0] addr0;
    wire [31 : 0] addr1;
    wire [31 : 0] addr2;
    wire [31 : 0] addr3;
    wire [31 : 0] status_addr;
    wire dma_config_valid;
    wire dma_status_valid;
    wire [31 : 0] ack_addr;
    wire ack_addr_valid;
    wire [159 : 0] cfg_data;
    wire cfg_apply;
    wire cfg_busy;
    wire [159 : 0] image_active;
    wire image_ack;
    wire image_pending;
    wire [159 : 0] frame_config;
    wire dma_valid;
    wire dma_ready;
    wire [127 : 0] dma_data;
    wire dma_last;
    wire dma_user;
    wire ack_valid;
    wire ack_ready;
    wire [127 : 0] ack_data;
    wire ack_last;
    wire ack_user;
    wire tx_valid;
    wire [127 : 0] tx_data;
    wire tx_last;
    wire tx_user;
    reg tx_ready = 1;
    reg random_stalls = 0;
    reg [31 : 0] random_state = 32'h839ca157;
    always@(negedge clk)begin
        if(random_stalls)begin
            random_state = {random_state[30 : 0],
                random_state[31] ^ random_state[21] ^ random_state[1] ^ random_state[0]};
            tx_ready = random_state[2 : 0] != 0;
        end
    end

    pcie_dma_ctrl#(
        .IMAGE_TLP_COUNT(2),
        .IMAGE_CONTROL_ENABLE(1)
    )u_dma(
        .clk(clk),
        .pix_clk_out(pix),
        .rstn(rst_n),
        .axis_master_tvalid(rx_valid),
        .axis_master_tready(rx_ready),
        .axis_master_tdata(rx_data),
        .axis_master_tkeep(rx_keep),
        .axis_master_tlast(rx_last),
        .axis_master_tuser(rx_user),
        .ep_bus_num(8'h12),
        .ep_dev_num(5'h03),
        .AXIS_S_TREADY(dma_ready),
        .AXIS_S_TVALID(dma_valid),
        .AXIS_S_TDATA(dma_data),
        .AXIS_S_TLAST(dma_last),
        .AXIS_S_TUSER(dma_user),
        .img_cmd_valid(command_valid),
        .img_cmd_addr(command_addr),
        .img_cmd_data(command_data),
        .img_dma_addr0(addr0),
        .img_dma_addr1(addr1),
        .img_dma_addr2(addr2),
        .img_dma_addr3(addr3),
        .img_dma_status_addr(status_addr),
        .img_dma_config_valid(dma_config_valid),
        .img_dma_status_valid(dma_status_valid),
        .img_ack_addr(ack_addr),
        .img_ack_addr_valid(ack_addr_valid),
        .hdmi_data_in(16'h1234),
        .vs_in(vs),
        .de_in(de),
        .video_enhance_lightdown_num(),
        .video_enhance_lightdown_sw(),
        .video_enhance_darkup_num(),
        .video_enhance_darkup_sw()
    );

    pcie_image_control#(
        .IMAGE_TLP_COUNT(2)
    )u_control(
        .clk(clk),
        .rst_n(rst_n),
        .cmd_valid(command_valid),
        .cmd_addr(command_addr),
        .cmd_data(command_data),
        .dma_addr0(addr0),
        .dma_addr1(addr1),
        .dma_addr2(addr2),
        .dma_addr3(addr3),
        .dma_status_addr(status_addr),
        .dma_config_valid(dma_config_valid),
        .dma_status_valid(dma_status_valid),
        .ep_bus_num(8'h12),
        .ep_dev_num(5'h03),
        .image_rst_n(image_rst_n),
        .image_ready(image_ready),
        .image_cfg_ack(image_ack),
        .image_cfg_pending(image_pending),
        .image_cfg_active(image_active),
        .cfg_data(cfg_data),
        .cfg_apply(cfg_apply),
        .ack_addr(ack_addr),
        .ack_addr_valid(ack_addr_valid),
        .cfg_busy(cfg_busy),
        .tx_tvalid(ack_valid),
        .tx_tdata(ack_data),
        .tx_tlast(ack_last),
        .tx_tuser(ack_user),
        .tx_tready(ack_ready)
    );

    pcie_tlp_arbiter u_arbiter(
        .clk(clk),
        .rst_n(rst_n),
        .dma_tvalid(dma_valid),
        .dma_tdata(dma_data),
        .dma_tlast(dma_last),
        .dma_tuser(dma_user),
        .dma_tready(dma_ready),
        .ack_tvalid(ack_valid),
        .ack_tdata(ack_data),
        .ack_tlast(ack_last),
        .ack_tuser(ack_user),
        .ack_tready(ack_ready),
        .tx_tvalid(tx_valid),
        .tx_tdata(tx_data),
        .tx_tlast(tx_last),
        .tx_tuser(tx_user),
        .tx_tready(tx_ready)
    );

    image_runtime_control u_pixel_control(
        .clk(pix),
        .rst_n(image_rst_n),
        .frame_start(frame_start),
        .requested_config(cfg_data),
        .apply_toggle(cfg_apply),
        .frame_config(frame_config),
        .active_config(image_active),
        .pending(image_pending),
        .ack_toggle(image_ack)
    );

    function automatic [31 : 0] swap(input [31 : 0] value);
        swap = {value[7 : 0], value[15 : 8], value[23 : 16], value[31 : 24]};
    endfunction

    task automatic tick(input integer cycles);
        repeat(cycles) @(negedge clk);
    endtask

    task automatic check(input bit condition, input string message);
        if(!condition)
            $fatal(1, "%s at %t", message, $time);
    endtask

    task automatic write_command(input [11 : 0] address, input [31 : 0] value);
        @(negedge clk);
        rx_valid = 1;
        rx_last = 0;
        rx_keep = 7;
        rx_user = 8'h10;
        rx_data = 0;
        rx_data[31 : 24] = 8'h40;
        rx_data[9 : 0] = 1;
        rx_data[39 : 32] = 8'h0f;
        rx_data[75 : 64] = address;
        @(negedge clk);
        rx_last = 1;
        rx_keep = 1;
        rx_data = {96'd0, swap(value)};
        @(negedge clk);
        rx_valid = 0;
        rx_last = 0;
        tick(2);
    endtask

    // These drivers keep VALID continuously asserted across packet boundaries
    // when called header/payload/header/payload without an intervening idle.
    task automatic rx_header_beat(input [11 : 0] address);
        @(negedge clk);
        rx_valid = 1;
        rx_last = 0;
        rx_keep = 7;
        rx_user = 8'h10;
        rx_data = 0;
        rx_data[31 : 24] = 8'h40;
        rx_data[9 : 0] = 1;
        rx_data[39 : 32] = 8'h0f;
        rx_data[75 : 64] = address;
    endtask

    task automatic rx_payload_beat(input [31 : 0] value);
        @(negedge clk);
        rx_valid = 1;
        rx_last = 1;
        rx_keep = 1;
        rx_data = {96'd0, swap(value)};
    endtask

    task automatic rx_idle_beat;
        @(negedge clk);
        rx_valid = 0;
        // Invalid cycles deliberately carry misleading packet metadata.
        rx_last = 1;
        rx_keep = 0;
        rx_user = 0;
        rx_data = {128{1'b1}};
    endtask

    task automatic illegal_command(input integer variant);
        @(negedge clk);
        rx_valid = 1;
        rx_last = 0;
        rx_keep = 7;
        rx_user = 8'h10;
        rx_data = 0;
        rx_data[31 : 24] = 8'h40;
        rx_data[9 : 0] = 1;
        rx_data[39 : 32] = 8'h0f;
        rx_data[75 : 64] = 12'h214;
        case(variant)
            0:rx_data[9 : 0] = 2;
            1:rx_user = 0;
            2:rx_keep = 3;
            3:rx_data[65 : 64] = 1;
            4:rx_data[31 : 24] = 8'h00;
            5:rx_last = 1;
            default:begin
            end
        endcase
        @(negedge clk);
        rx_last = 1;
        rx_keep = variant == 6 ? 0 : 1;
        rx_data = {96'd0, swap(32'hdead1111)};
        @(negedge clk);
        rx_valid = 0;
        rx_last = 0;
        tick(8);
    endtask

    task automatic stage_packet(input [159 : 0] packet);
        for(integer j = 0; j < 5; j = j + 1)
            write_command(12'h200 + j * 4, packet[j * 32 +: 32]);
    endtask

    task automatic frame_edge;
        repeat(12) @(negedge pix);
        frame_start = 1;
        @(negedge pix);
        frame_start = 0;
        repeat(12) @(negedge pix);
    endtask

    reg [511 : 0] received [0 : 255];
    reg [511 : 0] packet_payload;
    reg [31 : 0] packet_address;
    reg [127 : 0] held_data;
    reg held_last;
    reg held_user;
    reg held_valid = 0;
    integer beat = 0;
    integer replies = 0;
    integer stops = 0;
    reg test_passed = 0;
    integer dma_packets = 0;
    integer commands = 0;
    reg sequence_check = 0;
    integer sequence_expected = 0;
    integer sequence_received = 0;
    reg [11 : 0] expected_address [0 : 15];
    reg [31 : 0] expected_data [0 : 15];
    integer owner_on_header = 0;
    reg [31 : 0] last_stop;
    always@(posedge clk)begin
        if(!rst_n)begin
            held_valid = 0;
            beat = 0;
        end
        else begin
            if(command_valid)begin
                if(sequence_check)begin
                    check(sequence_received < sequence_expected,
                        "RX stage duplicated a decoded command");
                    check(command_addr === expected_address[sequence_received] &&
                        command_data === expected_data[sequence_received],
                        $sformatf("RX command order/data wrong at item %0d: addr=%h data=%h",
                            sequence_received, command_addr, command_data));
                    sequence_received = sequence_received + 1;
                end
                commands = commands + 1;
            end
            if(held_valid)begin
                check(tx_valid && tx_data === held_data && tx_last === held_last && tx_user === held_user,
                    "TLP changed under backpressure");
            end
            held_valid = tx_valid && !tx_ready;
            held_data = tx_data;
            held_last = tx_last;
            held_user = tx_user;
            if(tx_valid && tx_ready)begin
                if(beat == 0)begin
                    check(!tx_last && tx_data[31 : 24] == 8'h40 && tx_data[9 : 0] == 16,
                        "Bad posted-write header");
                    check(tx_data[63 : 48] == {8'h12, 5'h03, 3'd0}, "Wrong requester ID");
                    packet_address = tx_data[95 : 64];
                    packet_payload = 0;
                    owner_on_header = u_arbiter.selected_owner;
                    beat = 1;
                end
                else begin
                    check(u_arbiter.selected_owner == owner_on_header, "TLP source changed before TLAST");
                    check(tx_last == (beat == 4), "Incorrect ACK TLAST");
                    for(integer j = 0; j < 4; j = j + 1)
                        packet_payload[(beat - 1) * 128 + j * 32 +: 32] = swap(tx_data[j * 32 +: 32]);
                    if(tx_last)begin
                        if(packet_address == 32'h09000000)begin
                            check(packet_payload[31 : 0] == 32'h494d4743, "ACK magic corrupt");
                            check(packet_payload[63 : 32] != 0, "ACK sequence is zero");
                            check(packet_payload[511 : 480] == ~packet_payload[63 : 32], "ACK sequence footer corrupt");
                            received[replies] = packet_payload;
                            replies = replies + 1;
                        end
                        else if(packet_address == 32'h08000000)begin
                            last_stop = packet_payload[31 : 0];
                            for(integer j = 1; j < 16; j = j + 1)
                                check(packet_payload[j * 32 +: 32] == last_stop, "STOP payload interleaved");
                            stops = stops + 1;
                        end
                        else begin
                            dma_packets = dma_packets + 1;
                        end
                        beat = 0;
                    end
                    else begin
                        beat = beat + 1;
                    end
                end
            end
        end
    end

    task automatic wait_reply(input [31 : 0] cookie, input [31 : 0] status);
        integer timeout;
        bit found;
        begin
            timeout = 0;
            found = 0;
            while(!found && timeout < 1500)begin
                for(integer j = 0; j < replies; j = j + 1)
                    if(received[j][95 : 64] == cookie && received[j][127 : 96] == status)
                        found = 1;
                tick(1);
                timeout = timeout + 1;
            end
            check(found, $sformatf("Missing ACK cookie=%h status=%0d", cookie, status));
        end
    endtask

    initial begin : main
        integer before_count;
        integer before_stops;
        tick(5);
        rst_n = 1;
        tick(8);
        write_command(12'h1a0, 1);
        for(integer j = 0; j < 4; j = j + 1)
            write_command(12'h110, 32'h01000000 + j * 32'h01000000);
        write_command(12'h180, 32'h08000000);
        check(dma_config_valid && dma_status_valid, "Initial DMA registration failed");
        write_command(12'h220, 32'h01000000);
        check(!ack_addr_valid, "ACK address overlapped frame buffer");
        write_command(12'h220, 32'h08000000);
        check(!ack_addr_valid, "ACK address overlapped STOP page");
        write_command(12'h220, 32'h09000001);
        check(!ack_addr_valid, "Unaligned ACK address accepted");
        write_command(12'h220, 32'h09000000);
        check(ack_addr_valid && ack_addr == 32'h09000000, "ACK address not accepted");

        stage_packet(FIRST);
        write_command(12'h214, 32'h11112222);
        wait_reply(32'h11112222, 1);
        check(cfg_data == FIRST && u_control.in_flight, "Commit snapshot was not held without video");
        stage_packet(SECOND);
        write_command(12'h214, 32'h33334444);
        wait_reply(32'h33334444, 3);
        check(cfg_data == FIRST, "Busy commit overwrote in-flight snapshot");
        write_command(12'h224, 32'h10101010);
        wait_reply(32'h10101010, 7);

        write_command(12'h190, 1);
        before_stops = stops;
        write_command(12'h130, 32'hcafebabe);
        tick(100);
        check(stops == before_stops + 1 && last_stop == 32'hcafebabe,
            "No-video image commit blocked DMA STOP");
        check(!u_dma.running && !u_dma.stop_pending, "DMA STOP did not quiesce");

        image_rst_n = 1;
        image_ready = 1;
        tick(25);
        check(image_active == RESET, "Image configuration changed before frame boundary");
        frame_edge();
        wait_reply(32'h11112222, 2);
        check(image_active == FIRST && u_control.active_snapshot == FIRST, "First active packet mismatch");
        check(cfg_apply == 1, "First transaction toggle wrong");

        random_stalls = 1;
        write_command(12'h214, 32'h55556666);
        tick(25);
        check(image_active == FIRST, "Second packet applied mid-frame");
        frame_edge();
        wait_reply(32'h55556666, 2);
        check(image_active == SECOND && cfg_apply == 0, "Second packet or even toggle wrong");

        image_rst_n = 0;
        image_ready = 0;
        repeat(8) @(negedge pix);
        check(cfg_apply == 1 && u_control.in_flight && u_control.restoring,
            "Independent image reset did not re-arm even-toggle packet");
        check(image_active == RESET, "Image reset did not restore reset defaults");
        image_rst_n = 1;
        image_ready = 1;
        frame_edge();
        tick(100);
        check(image_active == SECOND && u_control.applied_cookie == 32'h55556666,
            "Pixel reset silently lost last applied configuration");
        write_command(12'h224, 32'h20202020);
        wait_reply(32'h20202020, 2);

        random_stalls = 0;
        tx_ready = 0;
        before_count = u_control.overflow_count;
        for(integer j = 0; j < 20; j = j + 1)
            write_command(12'h224, 32'h30000000 + j);
        check(u_control.overflow_count > before_count, "ACK overflow not recorded");
        before_count = u_dma.invalid_releases;
        write_command(12'h170, 32'h76543210);
        tick(5);
        check(u_dma.invalid_releases == before_count + 1, "Full config queue blocked RELEASE parsing");
        before_stops = stops;
        write_command(12'h130, 32'habcd1234);
        check(u_dma.stop_pending, "Full config queue blocked STOP parsing");
        tx_ready = 1;
        tick(500);
        check(stops == before_stops + 1 && last_stop == 32'habcd1234, "STOP lost behind ACK backpressure");
        write_command(12'h224, 32'h40404040);
        wait_reply(32'h40404040, 6);
        check(received[replies - 1][447 : 416] > 0 && received[replies - 1][479 : 448] != 0,
            "QUERY hid ACK overflow/last rejected cookie");

        write_command(12'h220, 32'h01000000);
        check(ack_addr == 32'h09000000, "Invalid re-registration replaced safe ACK page");
        tick(60);
        write_command(12'h1a0, 1);
        write_command(12'h110, 32'h09000000);
        check(u_dma.alloc_addr_state == 0, "Reconfigured frame buffer overlapped reserved ACK page");
        for(integer j = 0; j < 4; j = j + 1)
            write_command(12'h110, 32'h01000000 + j * 32'h01000000);
        write_command(12'h180, 32'h09000000);
        check(!dma_status_valid, "Reconfigured STOP page overlapped reserved ACK page");
        write_command(12'h180, 32'h08000000);

        before_count = commands;
        @(negedge clk);
        rx_valid = 1;
        rx_last = 0;
        rx_keep = 7;
        rx_user = 8'h10;
        rx_data = 0;
        rx_data[31 : 24] = 8'h40;
        rx_data[9 : 0] = 1;
        rx_data[39 : 32] = 8'h07;
        rx_data[75 : 64] = 12'h214;
        @(negedge clk);
        rx_last = 1;
        rx_keep = 1;
        rx_data = {96'd0, swap(32'h80808080)};
        @(negedge clk);
        rx_valid = 0;
        rx_last = 0;
        tick(8);
        check(commands == before_count, "Partial byte-enable escaped strict command parser");

        before_count = commands;
        for(integer j = 0; j < 7; j = j + 1)
            illegal_command(j);
        check(commands == before_count, "Illegal Length/BAR/keep/alignment/fmt/header-last/payload-keep accepted");

        // Real image DMA and configuration ACKs contend at packet boundaries.
        random_stalls = 1;
        before_count = dma_packets;
        write_command(12'h190, 1);
        @(negedge pix);
        vs = 1;
        tick(20);
        @(negedge pix);
        vs = 0;
        fork
            begin
                for(integer j = 0; j < 8; j = j + 1)
                    write_command(12'h224, 32'h60000000 + j);
            end
            begin
                tick(300);
            end
        join
        check(dma_packets >= before_count + 3, "ACK traffic starved real image DMA/frame tail");
        wait_reply(32'h60000007, 6);
        write_command(12'h130, 32'hf00dd00d);
        tick(120);
        check(last_stop == 32'hf00dd00d, "Concurrent DMA/ACK STOP lost");
        random_stalls = 0;
        tx_ready = 1;

        image_rst_n = 0;
        image_ready = 0;
        rst_n = 0;
        tick(8);
        rst_n = 1;
        tick(8);
        check(cfg_data == RESET && cfg_apply == 0 && !ack_addr_valid && !cfg_busy,
            "New PCIe session retained stale configuration or ACK memory");

        sequence_expected = 8;
        sequence_received = 0;
        sequence_check = 1;
        before_count = commands;
        for(integer j = 0; j < sequence_expected; j = j + 1)begin
            expected_address[j] = 12'h200 + (j % 5) * 4;
            expected_data[j] = 32'ha1020304 + j * 32'h01020304;
        end
        for(integer j = 0; j < sequence_expected; j = j + 1)begin
            rx_header_beat(expected_address[j]);
            rx_payload_beat(expected_data[j]);
        end
        rx_idle_beat();
        tick(8);
        check(sequence_received == sequence_expected && commands == before_count + sequence_expected,
            "Continuous RX packet stream lost or repeated a command");
        sequence_check = 0;
        $display("PASS RX no-idle stream: eight Header/Payload packets, exact address/value order");

        sequence_expected = 1;
        sequence_received = 0;
        sequence_check = 1;
        expected_address[0] = 12'h204;
        expected_data[0] = 32'h1234abcd;
        before_count = commands;
        rx_header_beat(expected_address[0]);
        rx_idle_beat();
        tick(3);
        check(sequence_received == 0 && commands == before_count && u_dma.rx_state == 1,
            "VALID bubble lost the pending header or fabricated a command");
        rx_payload_beat(expected_data[0]);
        rx_idle_beat();
        tick(8);
        check(sequence_received == 1 && commands == before_count + 1,
            "RX VALID bubble corrupted Header/Payload alignment");
        sequence_check = 0;
        $display("PASS RX VALID bubbles: invalid metadata ignored; pending header preserved");

        before_count = commands;
        rx_header_beat(12'h200);
        rx_idle_beat();
        tick(2);
        check(u_dma.rx_state == 1, "Half-packet reset did not target parsed-header state");
        @(negedge clk);
        rst_n = 0;
        tick(5);
        rst_n = 1;
        tick(8);
        rx_payload_beat(32'habcd1234);
        rx_idle_beat();
        tick(8);
        check(commands == before_count && u_dma.rx_state == 0 && !command_valid,
            "Orphan payload after reset reused a pre-reset header");

        rx_header_beat(12'h208);
        rx_payload_beat(32'h76543210);
        @(negedge clk);
        check(u_dma.rx_stage_tvalid && u_dma.rx_stage_tlast && u_dma.rx_state == 1,
            "Reset did not target a buffered payload awaiting parser execution");
        rx_valid = 0;
        rx_last = 0;
        rst_n = 0;
        tick(5);
        rst_n = 1;
        tick(8);
        check(commands == before_count && !command_valid && !u_dma.rx_stage_tvalid && u_dma.rx_state == 0,
            "Reset replayed a buffered pre-reset payload");
        write_command(12'h20c, 32'h10293847);
        check(commands == before_count + 1 && u_control.shadow_config[127 : 96] == 32'h10293847,
            "Fresh command did not recover after interrupted packet reset");
        $display("PASS RX half-packet reset: parsed header and buffered payload discarded; fresh command recovered");

        test_passed = 1;
        $display("PASS PCIe image control: %0d replies, %0d STOP packets; snapshot/frame atomicity, CDC, even-toggle pixel reset, overflow, mutual address guards and TLP backpressure", replies, stops);
        $finish;
    end

    initial begin
        #1000000;
        $fatal(1, "Test timeout");
    end
endmodule
