`timescale 1ns/1ps

module sky_carry_tb;

    reg                     clk                         =   0;
    reg                     rst_n                       =   0;
    reg                     vs                          =   1;
    reg                     href                        =   0;
    reg                     de                          =   0;
    reg     [23 : 0]        rgb                         =   0;
    reg     [159 : 0]       cfg                         =   0;
    reg     [7 : 0]         tx                          =   0;
    wire    [7 : 0]         floor_new;
    wire    [7 : 0]         floor_before;
    wire    [7 : 0]         floor_original;
    wire    [8 : 0]         weight_new;
    wire    [8 : 0]         weight_before;
    wire    [8 : 0]         weight_original;
    wire    [7 : 0]         tx_new;
    wire    [7 : 0]         tx_before;
    wire    [7 : 0]         tx_original;
    integer                 checks                      =   0;
    integer                 original_checks             =   0;
    integer                 age                         =   0;
    integer                 n;
    integer                 left_value;
    integer                 right_value;
    integer                 frame;
    integer                 seed                        =   32'h134acd76;
    reg                     test_passed                 =   0;

    always #5 clk = ~clk;

    sky_protect#(
        .RUNTIME_CONTROL                (1)
    )dut(
        .cfg_data                       (cfg),
        .clk                            (clk),
        .rst_n                          (rst_n),
        .pre_frame_vsync                (vs),
        .pre_frame_href                 (href),
        .pre_frame_clken                (de),
        .pre_img                        (rgb),
        .aligned_tx_img                 (tx),
        .post_tx_img                    (tx_new),
        .post_tx_floor                  (floor_new),
        .post_weight                    (weight_new)
    );

    sky_before_carry#(
        .RUNTIME_CONTROL                (1)
    )reference_before(
        .cfg_data                       (cfg),
        .clk                            (clk),
        .rst_n                          (rst_n),
        .pre_frame_vsync                (vs),
        .pre_frame_href                 (href),
        .pre_frame_clken                (de),
        .pre_img                        (rgb),
        .aligned_tx_img                 (tx),
        .post_tx_img                    (tx_before),
        .post_tx_floor                  (floor_before),
        .post_weight                    (weight_before)
    );

    sky_reference#(
        .RUNTIME_CONTROL                (1)
    )original(
        .cfg_data                       (cfg),
        .clk                            (clk),
        .rst_n                          (rst_n),
        .pre_frame_vsync                (vs),
        .pre_frame_href                 (href),
        .pre_frame_clken                (de),
        .pre_img                        (rgb),
        .aligned_tx_img                 (tx),
        .post_tx_img                    (tx_original),
        .post_tx_floor                  (floor_original),
        .post_weight                    (weight_original)
    );

    always@(posedge clk)begin
        #1;
        if(!rst_n)
            age = 0;
        else begin
            age = age + 1;
            if({floor_new, weight_new, tx_new} !== {floor_before, weight_before, tx_before})
                $fatal(1, "Current six-clock sky oracle differs at cycle %0d", checks);
            checks = checks + 1;
            if(age > 16)begin
                if({floor_new, weight_new, tx_new} !== {floor_original, weight_original, tx_original})
                    $fatal(1, "Original six-clock sky oracle differs at cycle %0d", checks);
                original_checks = original_checks + 1;
            end
        end
    end

    task cycle;
        input [23 : 0] value;
        input active;
        begin
            @(negedge clk);
            rgb = value;
            href = active;
            de = active;
            tx = $random(seed);
        end
    endtask

    task new_frame;
        input integer number;
        begin
            @(negedge clk);
            vs = 0;
            href = 0;
            de = 0;
            cfg[39 : 32] = number == 0 ? 52 : $random(seed);
            cfg[47 : 40] = number == 0 ? 128 : $random(seed);
            cfg[6] = number % 3 != 1;
            repeat(24) cycle(0, 0);
            vs = 1;
            repeat(24) cycle(0, 0);
        end
    endtask

    initial begin
        repeat(8) @(negedge clk);
        rst_n = 1;
        new_frame(0);
        // Gray pixels cover every possible horizontal Y pair and both
        // directions, including all eight/forty-code ramp boundaries.
        for(left_value = 0; left_value < 256; left_value = left_value + 1)begin
            for(right_value = 0; right_value < 256; right_value = right_value + 1)begin
                cycle(0, 0);
                cycle({3{left_value[7 : 0]}}, 1);
                cycle({3{right_value[7 : 0]}}, 1);
            end
        end
        // Independent RGB values, runtime floors, disabled sky protection,
        // row breaks, invalid enable/href combinations and reset recovery.
        for(frame = 1; frame <= 256; frame = frame + 1)begin
            new_frame(frame);
            for(n = 0; n < 4096; n = n + 1)begin
                cycle($random(seed), n % 32 >= 4);
                if(n % 29 == 0)
                    de = 0;
                if(n % 37 == 0)
                    href = 0;
            end
            if(frame % 31 == 0)begin
                rst_n = 0;
                repeat(8) cycle(0, 0);
                rst_n = 1;
                repeat(24) cycle(0, 0);
            end
        end
        repeat(24) cycle(0, 0);
        if(checks < 1200000 || original_checks < 1200000)
            $fatal(1, "Insufficient equivalence coverage");
        $display("PASS SKY_CARRY: %0d current-oracle cycles; %0d original-oracle cycles; all 65536 adjacent Y pairs; 256 runtime frames; blanking/reset; same six clocks", checks, original_checks);
        test_passed = 1;
        $finish;
    end

endmodule
