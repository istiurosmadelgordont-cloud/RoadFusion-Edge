`timescale 1ns/1ps

module tone_pipeline_tb;

    localparam  WIDTH                           =   32;
    localparam  HEIGHT                          =   32;
    localparam  H_BLANK                         =   64;
    localparam  LINE_CLOCKS                     =   WIDTH + H_BLANK;
    localparam  RUNTIME_CLOCKS                  =   LINE_CLOCKS + 48;
    localparam  STATIC_CLOCKS                   =   30;

    reg                     clk                         =   0;
    reg                     rst_n                       =   0;
    reg                     input_vs                    =   1;
    reg                     input_de                    =   0;
    reg     [23 : 0]        input_rgb                   =   0;
    reg     [159 : 0]       packet                      =   0;
    wire    [3 : 0]         input_config;
    wire    [23 : 0]        runtime_rgb;
    wire    [23 : 0]        static_rgb;
    wire                    runtime_vs;
    wire                    runtime_href;
    wire                    runtime_de;
    wire                    static_vs;
    wire                    static_href;
    wire                    static_de;
    reg     [23 : 0]        runtime_expected[0 : RUNTIME_CLOCKS - 1];
    reg     [23 : 0]        static_expected[0 : STATIC_CLOCKS - 1];
    reg     [2 : 0]         runtime_sync[0 : RUNTIME_CLOCKS - 1];
    reg     [2 : 0]         static_sync[0 : STATIC_CLOCKS - 1];
    reg     [23 : 0]        original_history[0 : RUNTIME_CLOCKS - 1];
    integer                 age                         =   0;
    integer                 checks                      =   0;
    integer                 changed                     =   0;
    integer                 frame;
    integer                 x;
    integer                 y;
    integer                 i;
    integer                 value;
    reg                     test_passed                 =   0;

    always #5 clk = ~clk;

    assign input_config = {3'd0, packet[3]};

    rgb_enhancement_pipeline#(
        .RUNTIME_CONTROL                (1),
        .PIC_WIDTH                      (WIDTH),
        .PIC_HEIGHT                     (HEIGHT),
        .H_BLANK                        (H_BLANK),
        .GAMMA_CAPABLE                  (0),
        .CLAHE_CAPABLE                  (0)
    )runtime_dut(
        .cfg_data                       (packet),
        .clk                            (clk),
        .rst_n                          (rst_n),
        .pre_frame_vsync                (input_vs),
        .pre_frame_href                 (input_de),
        .pre_frame_clken                (input_de),
        .pre_img                        (input_rgb),
        .pre_config                     (input_config),
        .post_frame_vsync               (runtime_vs),
        .post_frame_href                (runtime_href),
        .post_frame_clken               (runtime_de),
        .post_img                       (runtime_rgb),
        .post_config                    ()
    );

    rgb_enhancement_pipeline#(
        .RUNTIME_CONTROL                (0),
        .PIC_WIDTH                      (WIDTH),
        .PIC_HEIGHT                     (HEIGHT),
        .H_BLANK                        (H_BLANK),
        .GAMMA_CAPABLE                  (0),
        .CLAHE_CAPABLE                  (0),
        .HIGHLIGHT_CAPABLE              (0),
        .LIGHTDOWN_ENABLE               (1),
        .LIGHTDOWN_THRESHOLD            (192),
        .DARKUP_ENABLE                  (1),
        .DARKUP_THRESHOLD               (128)
    )static_dut(
        .cfg_data                       (packet),
        .clk                            (clk),
        .rst_n                          (rst_n),
        .pre_frame_vsync                (input_vs),
        .pre_frame_href                 (input_de),
        .pre_frame_clken                (input_de),
        .pre_img                        (input_rgb),
        .pre_config                     (4'd0),
        .post_frame_vsync               (static_vs),
        .post_frame_href                (static_href),
        .post_frame_clken               (static_de),
        .post_img                       (static_rgb),
        .post_config                    ()
    );

    function automatic integer clamp;
        input integer value_in;
        begin
            clamp = value_in < 0 ? 0 : value_in > 255 ? 255 : value_in;
        end
    endfunction

    function automatic [23 : 0] reference_rgb;
        input [23 : 0] original;
        input integer light_enable;
        input integer light_threshold;
        input integer dark_enable;
        input integer dark_threshold;
        input integer highlight_enable;
        input integer knee;
        input integer slope;
        integer r;
        integer g;
        integer b;
        integer source_y;
        integer tone_y;
        integer cb;
        integer cr;
        integer weight;
        integer protected_y;
        begin
            if(!light_enable && !dark_enable && !highlight_enable)begin
                reference_rgb = original;
            end
            else begin
                r = original[23 : 16];
                g = original[15 : 8];
                b = original[7 : 0];
                source_y = (77 * r + 150 * g + 29 * b) >> 8;
                cb = ((-43 * r - 85 * g + 128 * b + 32768) >> 8) - 128;
                cr = ((128 * r - 107 * g - 21 * b + 32768) >> 8) - 128;
                tone_y = source_y;
                if(dark_enable && source_y <= dark_threshold)
                    tone_y = tone_y + (dark_threshold - source_y) / 8;
                if(light_enable && source_y >= light_threshold)
                    tone_y = tone_y - (source_y - light_threshold) / 8;
                tone_y = clamp(tone_y);
                if(highlight_enable)begin
                    weight = source_y <= 160 ? 0 : source_y >= 224 ? 256 : (source_y - 160) * 4;
                    protected_y = (source_y * weight + tone_y * (256 - weight) + 128) >> 8;
                    if(protected_y > knee)
                        protected_y = knee + (((protected_y - knee) * slope + 128) >> 8);
                    tone_y = clamp(protected_y);
                end
                r = clamp((tone_y * 1024 + cr * 1436) >>> 10);
                g = clamp((tone_y * 1024 - cr * 731 - cb * 352) >>> 10);
                b = clamp((tone_y * 1024 + cb * 1815) >>> 10);
                reference_rgb = {r[7 : 0], g[7 : 0], b[7 : 0]};
            end
        end
    endfunction

    always@(posedge clk)begin
        if(!rst_n)begin
            age = 0;
            for(i = 0; i < RUNTIME_CLOCKS; i = i + 1)begin
                runtime_expected[i] = 0;
                runtime_sync[i] = 0;
                original_history[i] = 0;
            end
            for(i = 0; i < STATIC_CLOCKS; i = i + 1)begin
                static_expected[i] = 0;
                static_sync[i] = 0;
            end
        end
        else begin
            for(i = RUNTIME_CLOCKS - 1; i > 0; i = i - 1)begin
                runtime_expected[i] = runtime_expected[i - 1];
                runtime_sync[i] = runtime_sync[i - 1];
                original_history[i] = original_history[i - 1];
            end
            for(i = STATIC_CLOCKS - 1; i > 0; i = i - 1)begin
                static_expected[i] = static_expected[i - 1];
                static_sync[i] = static_sync[i - 1];
            end
            runtime_expected[0] = reference_rgb(input_rgb, packet[11], packet[95 : 88],
                                                packet[12], packet[87 : 80], packet[3],
                                                packet[143 : 136], packet[152 : 144]);
            runtime_sync[0] = {input_vs, input_de, input_de};
            original_history[0] = input_rgb;
            static_expected[0] = reference_rgb(input_rgb, 1, 192, 1, 128, 0, 0, 0);
            static_sync[0] = {input_vs, input_de, input_de};
            age = age + 1;
            #1;
            if(age >= RUNTIME_CLOCKS)begin
                if({runtime_vs, runtime_href, runtime_de} !== runtime_sync[RUNTIME_CLOCKS - 1])
                    $fatal(1, "Runtime sync wrong; expected %0d clocks", RUNTIME_CLOCKS);
                if(runtime_de)begin
                    if(runtime_rgb !== runtime_expected[RUNTIME_CLOCKS - 1])
                        $fatal(1, "Runtime RGB/tone/reference/config misaligned: got=%h expected=%h", runtime_rgb,
                               runtime_expected[RUNTIME_CLOCKS - 1]);
                    if(runtime_rgb !== original_history[RUNTIME_CLOCKS - 1])
                        changed = changed + 1;
                    checks = checks + 1;
                end
            end
            if(age >= STATIC_CLOCKS)begin
                if({static_vs, static_href, static_de} !== static_sync[STATIC_CLOCKS - 1])
                    $fatal(1, "Static sync wrong; expected %0d clocks", STATIC_CLOCKS);
                if(static_de && static_rgb !== static_expected[STATIC_CLOCKS - 1])
                    $fatal(1, "Static RGB/tone wrong: got=%h expected=%h", static_rgb,
                           static_expected[STATIC_CLOCKS - 1]);
            end
        end
    end

    task automatic blank;
        input integer clocks;
        repeat(clocks)begin
            @(negedge clk);
            input_de = 0;
            input_rgb = 0;
        end
    endtask

    initial begin
        repeat(4) @(negedge clk);
        rst_n = 1;
        blank(RUNTIME_CLOCKS + 16);
        for(frame = 0; frame < 7; frame = frame + 1)begin
            @(negedge clk);
            input_de = 0;
            input_vs = 0;
            packet = 0;
            packet[11] = frame == 2 || frame == 3 || frame == 4 || frame == 5;
            packet[12] = frame == 1 || frame == 3 || frame == 4 || frame == 5;
            packet[95 : 88] = frame == 5 ? 0 : 192;
            packet[87 : 80] = frame == 5 ? 255 : 128;
            packet[3] = frame == 4;
            packet[143 : 136] = 192;
            packet[152 : 144] = 128;
            blank(256);
            @(negedge clk);
            input_vs = 1;
            blank(16);
            for(y = 0; y < HEIGHT; y = y + 1)begin
                for(x = 0; x < WIDTH; x = x + 1)begin
                    @(negedge clk);
                    input_de = 1;
                    value = (x * 7 + y * 13) & 255;
                    if(y[1 : 0] == 0)
                        input_rgb = {value[7 : 0], value[7 : 0], value[7 : 0]};
                    else if(y[1 : 0] == 1)
                        input_rgb = {value[7 : 0], 8'd0, 8'd255};
                    else if(y[1 : 0] == 2)
                        input_rgb = {8'd255, value[7 : 0], 8'd0};
                    else
                        input_rgb = {8'd0, 8'd255, value[7 : 0]};
                end
                blank(H_BLANK);
            end
        end
        @(negedge clk);
        input_vs = 0;
        blank(RUNTIME_CLOCKS + 64);
        #2;
        if(checks != 7 * WIDTH * HEIGHT || changed < 2000)
            $fatal(1, "Insufficient pipeline checks/effect: pixels=%0d changed=%0d", checks, changed);
        $display("PASS tone pipeline: pixels=%0d changed=%0d static_latency=%0d runtime_latency=%0d", checks,
                 changed, STATIC_CLOCKS, RUNTIME_CLOCKS);
        test_passed = 1;
        $finish;
    end

endmodule
