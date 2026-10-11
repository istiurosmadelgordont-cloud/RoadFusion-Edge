`timescale 1ns/1ps
module accounting_equivalence_tb;
    reg                     clk                      = 0;
    reg                     rst_n                    = 0;
    reg                     cmd_valid                = 0;
    reg     [11 : 0]        cmd_addr                 = 0;
    reg     [31 : 0]        cmd_data                 = 0;
    reg                     image_rst_n              = 0;
    reg                     image_ready              = 0;
    reg                     image_cfg_ack            = 0;
    reg     [159 : 0]       image_cfg_active         = 0;
    reg                     tx_ready                 = 1;
    reg                     auto_image               = 1;
    reg                     test_passed              = 0;
    integer                 image_clock              = 0;
    integer                 checked_cycles           = 0;
    integer                 reply_beats              = 0;
    integer                 reject_loss_collisions   = 0;
    integer                 double_losses            = 0;
    integer                 adjacent_queries         = 0;
    reg                     previous_rejection       = 0;
    reg     [31 : 0]        random_state             = 32'h781a2093;

    always #4 clk = !clk;
    wire    [159 : 0]        cfg_data_new             ;
    wire                     cfg_apply_new            ;
    wire    [31 : 0]         ack_addr_new             ;
    wire                     ack_addr_valid_new       ;
    wire                     cfg_busy_new             ;
    wire                     tx_valid_new             ;
    wire    [127 : 0]        tx_data_new              ;
    wire                     tx_last_new              ;
    wire                     tx_user_new              ;

    pcie_image_control#(
        .IMAGE_TLP_COUNT                (16'd3)
    )u_new(
        .clk                           (clk),
        .rst_n                         (rst_n),
        .cmd_valid                     (cmd_valid),
        .cmd_addr                      (cmd_addr),
        .cmd_data                      (cmd_data),
        .dma_addr0                     (32'h01000000),
        .dma_addr1                     (32'h02000000),
        .dma_addr2                     (32'h03000000),
        .dma_addr3                     (32'h04000000),
        .dma_status_addr               (32'h08000000),
        .dma_config_valid              (1'b1),
        .dma_status_valid              (1'b1),
        .ep_bus_num                    (8'h12),
        .ep_dev_num                    (5'h03),
        .image_rst_n                   (image_rst_n),
        .image_ready                   (image_ready),
        .image_cfg_ack                 (image_cfg_ack),
        .image_cfg_pending             (1'b0),
        .image_cfg_active              (image_cfg_active),
        .cfg_data                      (cfg_data_new),
        .cfg_apply                     (cfg_apply_new),
        .ack_addr                      (ack_addr_new),
        .ack_addr_valid                (ack_addr_valid_new),
        .cfg_busy                      (cfg_busy_new),
        .tx_tvalid                     (tx_valid_new),
        .tx_tdata                      (tx_data_new),
        .tx_tlast                      (tx_last_new),
        .tx_tuser                      (tx_user_new),
        .tx_tready                     (tx_ready)
    );
    wire    [159 : 0]        cfg_data_ref             ;
    wire                     cfg_apply_ref            ;
    wire    [31 : 0]         ack_addr_ref             ;
    wire                     ack_addr_valid_ref       ;
    wire                     cfg_busy_ref             ;
    wire                     tx_valid_ref             ;
    wire    [127 : 0]        tx_data_ref              ;
    wire                     tx_last_ref              ;
    wire                     tx_user_ref              ;

    pcie_image_control_reference#(
        .IMAGE_TLP_COUNT                (16'd3)
    )u_reference(
        .clk                           (clk),
        .rst_n                         (rst_n),
        .cmd_valid                     (cmd_valid),
        .cmd_addr                      (cmd_addr),
        .cmd_data                      (cmd_data),
        .dma_addr0                     (32'h01000000),
        .dma_addr1                     (32'h02000000),
        .dma_addr2                     (32'h03000000),
        .dma_addr3                     (32'h04000000),
        .dma_status_addr               (32'h08000000),
        .dma_config_valid              (1'b1),
        .dma_status_valid              (1'b1),
        .ep_bus_num                    (8'h12),
        .ep_dev_num                    (5'h03),
        .image_rst_n                   (image_rst_n),
        .image_ready                   (image_ready),
        .image_cfg_ack                 (image_cfg_ack),
        .image_cfg_pending             (1'b0),
        .image_cfg_active              (image_cfg_active),
        .cfg_data                      (cfg_data_ref),
        .cfg_apply                     (cfg_apply_ref),
        .ack_addr                      (ack_addr_ref),
        .ack_addr_valid                (ack_addr_valid_ref),
        .cfg_busy                      (cfg_busy_ref),
        .tx_tvalid                     (tx_valid_ref),
        .tx_tdata                      (tx_data_ref),
        .tx_tlast                      (tx_last_ref),
        .tx_tuser                      (tx_user_ref),
        .tx_tready                     (tx_ready)
    );

    task automatic check(input bit good, input string message);
        if(!good)begin
            $display("FAIL at %0t: %s", $time, message);
            $fatal(1);
        end
    endtask

    task automatic tick(input integer clocks);
        repeat(clocks) @(negedge clk);
    endtask

    task automatic command(input [11 : 0] address, input [31 : 0] value);
        @(negedge clk);
        cmd_valid = 1;
        cmd_addr = address;
        cmd_data = value;
    endtask

    task automatic idle(input integer clocks);
        @(negedge clk);
        cmd_valid = 0;
        tick(clocks);
    endtask

    always@(posedge clk)begin
        image_clock <= image_clock + 1;
        if(!image_rst_n)begin
            image_cfg_ack <= 0;
            image_cfg_active <= 0;
        end
        else if(auto_image && image_ready && image_clock % 13 == 0)begin
            image_cfg_ack <= cfg_apply_new;
            image_cfg_active <= cfg_data_new;
        end
        if(rst_n)begin
            if(u_new.command_rejected && u_new.lost_records != 0)
                reject_loss_collisions = reject_loss_collisions + 1;
            if(u_new.lost_records == 2)
                double_losses = double_losses + 1;
            if(previous_rejection && cmd_valid && cmd_addr == 12'h224)
                adjacent_queries = adjacent_queries + 1;
            previous_rejection = u_new.command_rejected;
            if(tx_valid_new && tx_ready)
                reply_beats = reply_beats + 1;
        end
        else begin
            previous_rejection = 0;
        end
    end

    // Sample after inputs and combinational visible totals settle. All public
    // outputs and all valid pre-queue reply records must match the original each clock.
    always@(negedge clk)begin
        #1;
        if(rst_n)begin
            check({cfg_data_new, cfg_apply_new, ack_addr_new, ack_addr_valid_new, cfg_busy_new,
                tx_valid_new, tx_data_new, tx_last_new, tx_user_new} ===
                {cfg_data_ref, cfg_apply_ref, ack_addr_ref, ack_addr_valid_ref, cfg_busy_ref,
                tx_valid_ref, tx_data_ref, tx_last_ref, tx_user_ref}, "Public output/latency mismatch");
            check(u_new.rejected_count === u_reference.rejected_count, "Visible rejected count mismatch");
            check(u_new.overflow_count === u_reference.overflow_count, "Visible overflow count mismatch");
            check(u_new.last_rejected_cookie === u_reference.last_rejected_cookie,
                "Visible last cookie or simultaneous-loss priority mismatch");
            check({u_new.command_reply_s1, u_new.applied_reply_s1} ===
                {u_reference.command_reply_s1, u_reference.applied_reply_s1}, "S1 event latency mismatch");
            if(u_new.command_reply_s1)
                check(u_new.command_record_s1 === u_reference.command_record_s1,
                    "Queued command record count/cookie mismatch");
            if(u_new.applied_reply_s1)
                check(u_new.applied_record_s1 === u_reference.applied_record_s1,
                    "Queued APPLIED record count/cookie mismatch");
            checked_cycles = checked_cycles + 1;
        end
    end

    initial begin : stimulus
        tick(6);
        rst_n = 1;
        idle(8);
        command(12'h220, 32'h01000000);
        command(12'h220, 32'h08000000);
        command(12'h220, 32'h09000001);
        command(12'h220, 32'h09000000);
        idle(20);
        check(u_new.rejected_count == 3, "No-ACK rejected commands not counted");

        command(12'h220, 32'h01000000);
        command(12'h224, 32'h10000001);
        idle(30);
        command(12'h220, 32'h09000001);
        command(12'h224, 32'h10000002);
        idle(30);
        command(12'h214, 32'h00000000);
        command(12'h224, 32'h10000003);
        command(12'h224, 32'h00000000);
        command(12'h224, 32'h10000004);
        idle(40);

        // Two consecutive rejection events must wrap the 32-bit counter exactly.
        @(negedge clk);
        u_new.rejected_count_base = 32'hfffffffe;
        u_reference.rejected_count = 32'hfffffffe;
        command(12'h224, 0);
        command(12'h224, 0);
        command(12'h224, 32'h10000005);
        idle(35);
        check(u_new.rejected_count == 0, "Adjacent rejection wrap incorrect");

        // Stall ACK transmission, fill the queue, and combine loss with new
        // rejected commands. Applying a held commit also produces two losses.
        tx_ready = 0;
        command(12'h214, 32'h11223344);
        command(12'h214, 32'h55667788);
        command(12'h224, 32'h2000abcd);
        for(integer j = 0; j < 16; j = j + 1)
            command(12'h224, 32'h20000000 + j);
        for(integer j = 0; j < 16; j = j + 1)begin
            command(12'h220, 32'h01000000);
            command(12'h224, 32'h20001000 + j);
        end
        image_rst_n = 1;
        image_ready = 1;
        for(integer j = 0; j < 60; j = j + 1)
            command(12'h224, 32'h20002000 + j);
        idle(5);
        check(reject_loss_collisions > 0, "No true simultaneous rejection/queue-loss coverage");
        check(double_losses > 0, "No simultaneous command/APPLIED queue-loss coverage");
        tx_ready = 1;
        idle(180);

        // Random continuous commands, short/long TX stalls and independent
        // image resets exercise record ordering and pending accounting overlap.
        for(integer j = 0; j < 10000; j = j + 1)begin
            @(negedge clk);
            random_state = {random_state[30 : 0],
                random_state[31] ^ random_state[21] ^ random_state[1] ^ random_state[0]};
            tx_ready = j % 220 < 130 ? 0 : random_state[4];
            if(j % 997 == 0)begin
                image_rst_n = 0;
                image_ready = 0;
            end
            else if(j % 997 == 17)begin
                image_rst_n = 1;
                image_ready = 1;
            end
            cmd_valid = 1;
            case(random_state[3 : 0])
                0:begin cmd_addr = 12'h200; cmd_data = random_state; end
                1:begin cmd_addr = 12'h204; cmd_data = random_state; end
                2:begin cmd_addr = 12'h208; cmd_data = random_state; end
                3:begin cmd_addr = 12'h20c; cmd_data = random_state; end
                4:begin cmd_addr = 12'h210; cmd_data = random_state; end
                5:begin cmd_addr = 12'h214; cmd_data = random_state; end
                6:begin cmd_addr = 12'h220; cmd_data = 32'h01000000; end
                7:begin cmd_addr = 12'h220; cmd_data = 32'h09000001; end
                8:begin cmd_addr = 12'h224; cmd_data = random_state; end
                9:begin cmd_addr = 12'h224; cmd_data = 0; end
                10:begin cmd_addr = 12'h220; cmd_data = 32'h09000000; end
                11:begin cmd_addr = 12'h214; cmd_data = 0; end
                12:begin cmd_addr = 12'h224; cmd_data = random_state; end
                13:begin cmd_addr = 12'h224; cmd_data = random_state; end
                default:begin cmd_addr = 12'h230; cmd_data = random_state; cmd_valid = 0; end
            endcase
        end
        idle(5);
        tx_ready = 1;
        idle(200);
        check(adjacent_queries > 10, "Insufficient adjacent invalid/QUERY coverage");
        test_passed = 1;
        $display("PASS accounting equivalence: %0d cycles, %0d TX beats, %0d rejected+lost collisions, %0d double losses, %0d adjacent QUERY",
            checked_cycles, reply_beats, reject_loss_collisions, double_losses, adjacent_queries);
        $finish;
    end

    initial begin
        #200000;
        $fatal(1, "Accounting equivalence timeout");
    end
endmodule
