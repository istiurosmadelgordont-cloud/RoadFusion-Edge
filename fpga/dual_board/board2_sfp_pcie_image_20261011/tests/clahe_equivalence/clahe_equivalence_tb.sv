`timescale 1ns/1ps

module clahe_equivalence_tb;

    localparam  WIDTH                           =   16;
    localparam  HEIGHT                          =   16;
    localparam  FRAMES                          =   48;
    reg                     clk                         =   0;
    reg                     rst_n                       =   0;
    reg                     flush                       =   0;
    reg                     vs                          =   1;
    reg                     de                          =   0;
    reg     [7 : 0]         y                           =   0;
    reg     [159 : 0]       cfg                         =   0;
    reg     [31 : 0]        cdf                         =   0;
    wire    [2 : 0]         result_sync[0 : 1];
    wire    [2 : 0]         reference_sync[0 : 1];
    wire    [7 : 0]         result_y[0 : 1];
    wire    [7 : 0]         reference_y[0 : 1];
    reg     [2 : 0]         sync_history[0 : 12];
    reg     [7 : 0]         source_history[0 : 12];
    integer                 frame;
    integer                 row;
    integer                 column;
    integer                 i;
    integer                 mode;
    integer                 checks                      =   0;
    integer                 pixels                      =   0;
    integer                 changed                     =   0;
    reg                     test_passed                 =   0;

    always #5 clk = ~clk;

    genvar g;
    generate
        for(g = 0; g < 2; g = g + 1)begin : g_case
            clahe_luma#(
                .RUNTIME_CONTROL                (1),
                .PIC_WIDTH                      (WIDTH),
                .PIC_HEIGHT                     (HEIGHT),
                .DONOR_MODE                     (g)
            )dut(
                .cfg_data                       (cfg),
                .clk                            (clk),
                .rst_n                          (rst_n),
                .stats_flush                    (flush),
                .pre_frame_vsync                (vs),
                .pre_frame_href                 (de),
                .pre_frame_clken                (de),
                .pre_y                          (y),
                .post_frame_vsync               (result_sync[g][2]),
                .post_frame_href                (result_sync[g][1]),
                .post_frame_clken               (result_sync[g][0]),
                .post_y                         (result_y[g])
            );

            clahe_luma_reference#(
                .RUNTIME_CONTROL                (1),
                .PIC_WIDTH                      (WIDTH),
                .PIC_HEIGHT                     (HEIGHT),
                .DONOR_MODE                     (g)
            )reference(
                .cfg_data                       (cfg),
                .clk                            (clk),
                .rst_n                          (rst_n),
                .stats_flush                    (flush),
                .pre_frame_vsync                (vs),
                .pre_frame_href                 (de),
                .pre_frame_clken                (de),
                .pre_y                          (y),
                .post_frame_vsync               (reference_sync[g][2]),
                .post_frame_href                (reference_sync[g][1]),
                .post_frame_clken               (reference_sync[g][0]),
                .post_y                         (reference_y[g])
            );

            // Override only the synchronous map read outputs. This drives
            // all interpolation and blend stages with independent corner
            // values, including maps not yet available during startup.
            initial begin
                force dut.cdf_ul = cdf[31 : 24];
                force dut.cdf_ur = cdf[23 : 16];
                force dut.cdf_ll = cdf[15 : 8];
                force dut.cdf_lr = cdf[7 : 0];
                force reference.cdf_ul = cdf[31 : 24];
                force reference.cdf_ur = cdf[23 : 16];
                force reference.cdf_ll = cdf[15 : 8];
                force reference.cdf_lr = cdf[7 : 0];
            end
        end
    endgenerate

    always@(posedge clk)begin
        if(!rst_n)begin
            for(i = 0; i < 13; i = i + 1)begin
                sync_history[i] = 0;
                source_history[i] = 0;
            end
        end
        else begin
            for(i = 12; i > 0; i = i - 1)begin
                sync_history[i] = sync_history[i - 1];
                source_history[i] = source_history[i - 1];
            end
            sync_history[0] = {vs, de, de};
            source_history[0] = y;
            #1;
            for(mode = 0; mode < 2; mode = mode + 1)begin
                if(result_sync[mode] !== sync_history[12])
                    $fatal(1, "CLAHE thirteen-clock sync mismatch");
                if(result_sync[mode] !== reference_sync[mode] || result_y[mode] !== reference_y[mode])
                    $fatal(1, "CLAHE output differs from pre-preserve RTL mode=%0d cycle=%0d", mode, checks);
                if(result_sync[mode][0])begin
                    pixels = pixels + 1;
                    if(result_y[mode] != source_history[12])
                        changed = changed + 1;
                end
                checks = checks + 1;
            end
        end
    end

    task cycle;
        input [7 : 0] value;
        input active;
        begin
            @(negedge clk);
            de = active;
            y = value;
            cdf = $random;
        end
    endtask

    initial begin
        repeat(4) @(negedge clk);
        rst_n = 1;
        repeat(20) cycle(0, 0);
        for(frame = 0; frame < FRAMES; frame = frame + 1)begin
            @(negedge clk);
            cfg = 160'd0;
            cfg[7] = frame[0];
            cfg[8] = frame[1];
            cfg[111 : 96] = 384;
            cfg[114 : 112] = frame % 5;
            cfg[121 : 115] = (frame * 19) % 65;
            cfg[135 : 128] = frame * 13;
            flush = frame % 11 == 0;
            vs = 0;
            cycle(0, 0);
            flush = 0;
            repeat(20) cycle(0, 0);
            for(row = 0; row < HEIGHT; row = row + 1)begin
                for(column = 0; column < WIDTH; column = column + 1)
                    cycle(frame < 16 ? row * 16 + column : $random, 1);
                repeat(32) cycle(0, 0);
            end
            vs = 1;
            repeat(3072) cycle(0, 0);
            if(frame == 23)begin
                rst_n = 0;
                repeat(4) cycle(0, 0);
                rst_n = 1;
            end
        end
        repeat(30) cycle(0, 0);
        if(pixels != 2 * FRAMES * WIDTH * HEIGHT || changed < 1000)
            $fatal(1, "Insufficient CLAHE equivalence coverage pixels=%0d changed=%0d", pixels, changed);
        $display("PASS CLAHE equivalence: %0d cycle comparisons; %0d pixels; %0d changed; runtime frame settings; reset; donor 0/1; 13 clocks", checks, pixels, changed);
        test_passed = 1;
        $finish;
    end

endmodule
