`timescale 1ns/1ps

`define ENH dut.u_enhancement

module image_serial_tb;

    localparam  WIDTH                           =   16;
    localparam  HEIGHT                          =   16;
    localparam  H_BLANK                         =   32;
    localparam  LINE_CLOCKS                     =   WIDTH + H_BLANK;
    localparam  ENHANCE_CLOCKS                  =   LINE_CLOCKS + 48;
    localparam  TOTAL_CLOCKS                    =   2 * LINE_CLOCKS + 96;
    localparam  FRAMES                          =   12;
    localparam  V_BLANK                         =   3072;

    reg                     clk                         =   0;
    reg                     rst_n                       =   0;
    reg                     vs                          =   1;
    reg                     de                          =   0;
    reg     [23 : 0]        pixel                       =   0;
    reg     [159 : 0]       requested                   =   0;
    reg                     apply                       =   0;
    wire    [159 : 0]       active;
    wire                    pending;
    wire                    ack;
    wire    [2 : 0]         out_sync;
    wire    [23 : 0]        out_rgb;
    reg     [2 : 0]         sync_history[0 : TOTAL_CLOCKS - 1];
    reg     [23 : 0]        pixel_history[0 : TOTAL_CLOCKS - 1];
    reg     [159 : 0]       packet_history[0 : TOTAL_CLOCKS - 1];
    reg     [23 : 0]        dcp_history[0 : ENHANCE_CLOCKS - 1];
    reg     [7 : 0]         tone_expected[0 : 1];
    reg     [15 : 0]        tone_chroma[0 : 1];
    reg     [159 : 0]       old_packet;
    reg     [159 : 0]       expected_packet;
    reg     [23 : 0]        model_pixel;
    integer                 frame;
    integer                 row;
    integer                 column;
    integer                 i;
    integer                 age                         =   0;
    integer                 sync_checks                 =   0;
    integer                 tone_stage_checks           =   0;
    integer                 off_pixels                  =   0;
    integer                 dcp_pixels                  =   0;
    integer                 tone_pixels                 =   0;
    integer                 combined_pixels             =   0;
    integer                 pending_frame_checks        =   0;
    integer                 tone_changed_pixels         =   0;
    reg                     test_passed                 =   0;

    always #5 clk = ~clk;

    image_processing_top#(
        .RUNTIME_CONTROL                (1),
        .PIC_WIDTH                      (WIDTH),
        .PIC_HEIGHT                     (HEIGHT),
        .H_BLANK                        (H_BLANK),
        .CLAHE_DONOR_MODE                (0)
    )dut(
        .cfg_data                       (requested),
        .cfg_apply                      (apply),
        .cfg_active                     (active),
        .cfg_pending                    (pending),
        .cfg_ack                        (ack),
        .clk                            (clk),
        .rst_n                          (rst_n),
        .pre_frame_vsync                (vs),
        .pre_frame_href                 (de),
        .pre_frame_clken                (de),
        .pre_img                        (pixel),
        .dcp_en                         (1'b0),
        .gamma_en                       (1'b0),
        .clahe_en                       (1'b0),
        .highlight_en                   (1'b0),
        .post_frame_vsync               (out_sync[2]),
        .post_frame_href                (out_sync[1]),
        .post_frame_clken               (out_sync[0]),
        .post_img                       (out_rgb),
        .active_config                  (),
        .highlight_supported            ()
    );

    function [159 : 0] mode_packet;
        input integer which;
        reg [159 : 0] value;
        begin
            value                       =   160'h0080c030008200100000410000a08034000001c0;
            value[87 : 80]              =   120;
            value[95 : 88]              =   140;
            case(which)
                0, 7, 11: value[31 : 0] =   32'h000001c0;
                1, 8:     value[31 : 0] =   32'h00000181;
                2:        value[31 : 0] =   32'h000011c0;
                3:        value[31 : 0] =   32'h000009c0;
                4, 9:begin
                    value[31 : 0]       =   32'h000019c0;
                    value[87 : 80]      =   which == 4 ? 160 : 96;
                    value[95 : 88]      =   which == 4 ? 96 : 176;
                end
                5:        value[31 : 0] =   32'h000019c6;
                6, 10:    value[31 : 0] =   32'h000019ff;
            endcase
            mode_packet                 =   value;
        end
    endfunction

    function [7 : 0] clamp8;
        input integer value;
        begin
            clamp8                      =   value < 0 ? 0 : value > 255 ? 255 : value;
        end
    endfunction

    function [7 : 0] tone_model;
        input [7 : 0] y;
        input [159 : 0] cfg;
        integer lift;
        integer reduce;
        begin
            lift                        =   cfg[12] && y <= cfg[87 : 80] ?
                                                (cfg[87 : 80] - y) / 8 : 0;
            reduce                      =   cfg[11] && y >= cfg[95 : 88] ?
                                                (y - cfg[95 : 88]) / 8 : 0;
            tone_model                  =   clamp8(y + lift - reduce);
        end
    endfunction

    function [23 : 0] rgb_tone_model;
        input [23 : 0] rgb;
        input [159 : 0] cfg;
        integer r;
        integer g;
        integer b;
        integer y;
        integer cb;
        integer cr;
        integer yr;
        begin
            r                           =   rgb[23 : 16];
            g                           =   rgb[15 : 8];
            b                           =   rgb[7 : 0];
            y                           =   (77 * r + 150 * g + 29 * b) >> 8;
            cb                          =   (128 * b - 43 * r - 85 * g + 32768) >> 8;
            cr                          =   (128 * r - 107 * g - 21 * b + 32768) >> 8;
            yr                          =   tone_model(y[7 : 0], cfg);
            rgb_tone_model              =   {clamp8((yr * 1024 + (cr - 128) * 1436) >>> 10),
                                                clamp8((yr * 1024 - (cr - 128) * 731 - (cb - 128) * 352) >>> 10),
                                                clamp8((yr * 1024 + (cb - 128) * 1815) >>> 10)};
        end
    endfunction

    always@(posedge clk)begin
        if(!rst_n)begin
            age                         =   0;
            for(i = 0; i < TOTAL_CLOCKS; i = i + 1)begin
                sync_history[i]         =   0;
                pixel_history[i]        =   0;
                packet_history[i]       =   0;
            end
            for(i = 0; i < ENHANCE_CLOCKS; i = i + 1)
                dcp_history[i]          =   0;
            tone_expected[0]            =   0;
            tone_expected[1]            =   0;
            tone_chroma[0]              =   0;
            tone_chroma[1]              =   0;
        end
        else begin
            age                         =   age + 1;
            for(i = TOTAL_CLOCKS - 1; i > 0; i = i - 1)begin
                sync_history[i]         =   sync_history[i - 1];
                pixel_history[i]        =   pixel_history[i - 1];
                packet_history[i]       =   packet_history[i - 1];
            end
            sync_history[0]             =   {vs, de, de};
            pixel_history[0]            =   pixel;
            packet_history[0]           =   dut.frame_runtime;
            for(i = ENHANCE_CLOCKS - 1; i > 0; i = i - 1)
                dcp_history[i]          =   dcp_history[i - 1];
            dcp_history[0]              =   dut.dcp_rgb;
            tone_expected[1]            =   tone_expected[0];
            tone_expected[0]            =   tone_model(`ENH.clahe_y, `ENH.clahe_runtime);
            tone_chroma[1]              =   tone_chroma[0];
            tone_chroma[0]              =   {`ENH.clahe_cb, `ENH.clahe_cr};
            #1;
            if(age >= 2)begin
                if(`ENH.tone_y !== tone_expected[1] ||
                   {`ENH.tone_cb, `ENH.tone_cr} !== tone_chroma[1])
                    $fatal(1, "Tone arithmetic/chroma two-clock alignment mismatch");
                tone_stage_checks       =   tone_stage_checks + 1;
            end
            if(age >= TOTAL_CLOCKS)begin
                sync_checks             =   sync_checks + 1;
                if(out_sync !== sync_history[TOTAL_CLOCKS - 1])
                    $fatal(1, "Total VS/DE latency mismatch: expected %0d", TOTAL_CLOCKS);
                if(out_sync[0])begin
                    expected_packet     =   packet_history[TOTAL_CLOCKS - 1];
                    if($isunknown(out_rgb))
                        $fatal(1, "Unknown value in a valid output pixel");
                    if(`ENH.output_runtime !== expected_packet)
                        $fatal(1, "Runtime packet does not belong to output pixel");
                    if((expected_packet[31 : 0] & 32'h181f) == 0)begin
                        off_pixels      =   off_pixels + 1;
                        if(out_rgb !== pixel_history[TOTAL_CLOCKS - 1])
                            $fatal(1, "All-off exact RGB bypass mismatch");
                    end
                    else if((expected_packet[31 : 0] & 32'h181f) == 1)begin
                        dcp_pixels      =   dcp_pixels + 1;
                        if(out_rgb !== dcp_history[ENHANCE_CLOCKS - 1])
                            $fatal(1, "DCP-only enhancement bypass mismatch");
                    end
                    else if((expected_packet[31 : 0] & 32'h1f) == 0)begin
                        tone_pixels     =   tone_pixels + 1;
                        model_pixel     =   rgb_tone_model(pixel_history[TOTAL_CLOCKS - 1], expected_packet);
                        if(out_rgb !== model_pixel)
                            $fatal(1, "Full RGB -> Y tone -> RGB oracle mismatch: got %06h expected %06h", out_rgb, model_pixel);
                        if(out_rgb != pixel_history[TOTAL_CLOCKS - 1])
                            tone_changed_pixels = tone_changed_pixels + 1;
                    end
                    else
                        combined_pixels =   combined_pixels + 1;
                end
            end
        end
    end

    initial begin
        repeat(8) @(negedge clk);
        rst_n                           =   1;
        repeat(TOTAL_CLOCKS + 8) @(negedge clk);
        requested                       =   mode_packet(0);
        requested[10 : 9]              =   2'b11;
        apply                           =   !apply;
        repeat(12) @(negedge clk);
        for(frame = 0; frame < FRAMES; frame = frame + 1)begin
            vs                          =   0;
            repeat(8) @(negedge clk);
            if(active !== mode_packet(frame) || ack !== apply || pending)
                $fatal(1, "Frame boundary configuration commit failed at frame %0d", frame);
            vs                          =   1;
            old_packet                  =   active;
            for(row = 0; row < HEIGHT; row = row + 1)begin
                for(column = 0; column < WIDTH; column = column + 1)begin
                    de                  =   1;
                    pixel               =   {8'((column * 13 + row * 7) % 256),
                                                8'((column * 7 + row * 29) % 256),
                                                8'((column * 19 + row * 11) % 256)};
                    if(row == 3 && column == 8 && frame < FRAMES - 1)begin
                        requested       =   mode_packet(frame + 1);
                        requested[10 : 9] = 2'b11;
                        apply           =   !apply;
                    end
                    @(negedge clk);
                    if(active !== old_packet)
                        $fatal(1, "Configuration changed in the middle of a frame");
                end
                de                      =   0;
                pixel                   =   0;
                repeat(H_BLANK) @(negedge clk);
            end
            repeat(V_BLANK) @(negedge clk);
            if(frame < FRAMES - 1)begin
                if(!pending || active !== old_packet || ack === apply)
                    $fatal(1, "Pending packet did not wait for next frame");
                pending_frame_checks    =   pending_frame_checks + 1;
            end
        end
        repeat(TOTAL_CLOCKS + 16) @(negedge clk);
        if(off_pixels != 3 * WIDTH * HEIGHT || dcp_pixels != 2 * WIDTH * HEIGHT ||
           tone_pixels != 4 * WIDTH * HEIGHT || combined_pixels != 3 * WIDTH * HEIGHT ||
           tone_changed_pixels == 0 || pending_frame_checks != FRAMES - 1)
            $fatal(1, "Incomplete serial-path coverage");
        $display("IMAGE_SERIAL_PASS latency=%0d sync_checks=%0d off=%0d dcp=%0d tone=%0d combined=%0d tone_stage_checks=%0d pending_frames=%0d",
                 TOTAL_CLOCKS, sync_checks, off_pixels, dcp_pixels, tone_pixels, combined_pixels,
                 tone_stage_checks, pending_frame_checks);
        test_passed                     =   1;
        $finish;
    end

endmodule
