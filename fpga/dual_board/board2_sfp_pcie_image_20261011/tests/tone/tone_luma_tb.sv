`timescale 1ns/1ps

module tone_luma_tb;

    reg                     clk                         =   0;
    reg                     rst_n                       =   0;
    reg     [159 : 0]       packet                      =   0;
    reg     [7 : 0]         input_y                     =   0;
    wire    [7 : 0]         runtime_y;
    wire    [7 : 0]         static_y;
    wire    [7 : 0]         bypass_y;
    reg     [7 : 0]         expected_runtime[0 : 1];
    reg     [7 : 0]         expected_static[0 : 1];
    reg     [7 : 0]         expected_bypass[0 : 1];
    integer                 age                         =   0;
    integer                 checks                      =   0;
    integer                 y;
    integer                 low;
    integer                 high;
    integer                 i;
    integer                 representative_y[0 : 7];
    reg                     test_passed                 =   0;

    always #5 clk = ~clk;

    legacy_tone_luma#(
        .RUNTIME_CONTROL                (1)
    )runtime_dut(
        .cfg_data                       (packet),
        .clk                            (clk),
        .rst_n                          (rst_n),
        .pre_y                          (input_y),
        .post_y                         (runtime_y)
    );

    legacy_tone_luma#(
        .RUNTIME_CONTROL                (0),
        .LIGHTDOWN_ENABLE               (1),
        .LIGHTDOWN_THRESHOLD            (192),
        .DARKUP_ENABLE                  (1),
        .DARKUP_THRESHOLD               (128)
    )static_dut(
        .cfg_data                       (packet),
        .clk                            (clk),
        .rst_n                          (rst_n),
        .pre_y                          (input_y),
        .post_y                         (static_y)
    );

    legacy_tone_luma bypass_dut(
        .cfg_data                       (packet),
        .clk                            (clk),
        .rst_n                          (rst_n),
        .pre_y                          (input_y),
        .post_y                         (bypass_y)
    );

    function automatic [7 : 0] reference_y;
        input integer value;
        input integer light_enable;
        input integer light_threshold;
        input integer dark_enable;
        input integer dark_threshold;
        integer lift;
        integer reduce;
        integer result_value;
        begin
            lift = dark_enable && value <= dark_threshold ? (dark_threshold - value) / 8 : 0;
            reduce = light_enable && value >= light_threshold ? (value - light_threshold) / 8 : 0;
            result_value = value + lift - reduce;
            reference_y = result_value < 0 ? 0 : result_value > 255 ? 255 : result_value;
        end
    endfunction

    always@(posedge clk)begin
        if(!rst_n)begin
            age = 0;
            expected_runtime[0] = 0;
            expected_runtime[1] = 0;
            expected_static[0] = 0;
            expected_static[1] = 0;
            expected_bypass[0] = 0;
            expected_bypass[1] = 0;
        end
        else begin
            expected_runtime[1] = expected_runtime[0];
            expected_static[1] = expected_static[0];
            expected_bypass[1] = expected_bypass[0];
            expected_runtime[0] = reference_y(input_y, packet[11], packet[95 : 88],
                                             packet[12], packet[87 : 80]);
            expected_static[0] = reference_y(input_y, 1, 192, 1, 128);
            expected_bypass[0] = input_y;
            age = age + 1;
            #1;
            if(age >= 2)begin
                if(runtime_y !== expected_runtime[1] || static_y !== expected_static[1] ||
                   bypass_y !== expected_bypass[1])
                    $fatal(1, "Tone latency/data mismatch: runtime=%0d/%0d static=%0d/%0d bypass=%0d/%0d",
                           runtime_y, expected_runtime[1], static_y, expected_static[1],
                           bypass_y, expected_bypass[1]);
                checks = checks + 1;
            end
        end
    end

    task automatic drive;
        input integer value;
        input integer light_enable;
        input integer light_threshold;
        input integer dark_enable;
        input integer dark_threshold;
        begin
            @(negedge clk);
            input_y = value;
            packet = 0;
            packet[11] = light_enable;
            packet[12] = dark_enable;
            packet[95 : 88] = light_threshold;
            packet[87 : 80] = dark_threshold;
        end
    endtask

    initial begin
        representative_y[0] = 0;
        representative_y[1] = 1;
        representative_y[2] = 31;
        representative_y[3] = 64;
        representative_y[4] = 127;
        representative_y[5] = 192;
        representative_y[6] = 254;
        representative_y[7] = 255;
        repeat(4) @(negedge clk);
        rst_n = 1;

        // Every Y/threshold pair for each independent operation, with controls
        // changing every cycle to expose threshold/control pipeline mistakes.
        for(y = 0; y < 256; y = y + 1)
            for(low = 0; low < 256; low = low + 1)begin
                drive(y, 0, 255 - low, 1, low);
                drive(y, 1, low, 0, 255 - low);
                drive(y, 0, low, 0, 255 - low);
            end

        // All threshold combinations at black, white and interior boundaries.
        for(i = 0; i < 8; i = i + 1)
            for(low = 0; low < 256; low = low + 1)
                for(high = 0; high < 256; high = high + 1)
                    drive(representative_y[i], 1, high, 1, low);

        repeat(4) drive(0, 0, 0, 0, 0);
        #2;
        if(checks < 720896)
            $fatal(1, "Insufficient exhaustive tone checks: %0d", checks);
        $display("PASS tone: %0d cycles, exhaustive independent Y/threshold pairs, 8*256*256 dual curves", checks);
        test_passed = 1;
        $finish;
    end

endmodule
