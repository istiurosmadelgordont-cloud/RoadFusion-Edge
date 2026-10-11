`timescale 1ns / 1ps

module sfp_register_tb;

parameter integer               HEIGHT = 4;
parameter integer               TEST_ABORT = 1;
localparam integer              WIDTH = 1920;
localparam integer              LINE_CLOCKS = 2200;
localparam integer              PIXELS = WIDTH * HEIGHT;
localparam real                 SOURCE_LINE_NS = LINE_CLOCKS * 1000.0 / 148.5;

reg                             pix_clk = 0;
reg                             rx_clk = 0;
reg                             rst_n = 0;
reg                             grs_n = 0;
reg             [3 : 0]         rx_charisk = 0;
reg             [31 : 0]        rx_data = 32'h55a109bc;
wire                            new_vs;
wire                            new_de;
wire            [15 : 0]        new_data;
wire                            old_vs;
wire                            old_de;
wire            [15 : 0]        old_data;
reg                             old_vs_delayed = 0;
reg                             old_valid_delayed = 0;
reg             [15 : 0]        old_data_delayed = 0;
reg                             new_previous_de = 0;
reg                             old_previous_de = 0;
reg             [23 : 0]        entry_rgb = 0;
reg                             entry_vs = 1;
reg                             entry_de = 0;
reg             [23 : 0]        entry_expected_rgb = 0;
reg                             entry_expected_vs = 1;
reg                             entry_expected_de = 0;
integer                         frame_id = 0;
integer                         expected_pixels = 0;
integer                         checked_new_pixels = 0;
integer                         checked_old_pixels = 0;
integer                         new_line_length = 0;
integer                         old_line_length = 0;
integer                         new_lines = 0;
integer                         old_lines = 0;
integer                         pixel_cycle = 0;
integer                         first_read_cycle = -1;
integer                         first_old_valid_cycle = -1;
integer                         first_new_valid_cycle = -1;
integer                         new_line_start = -1;
integer                         previous_new_line_start = -1;
integer                         decode_checks = 0;
integer                         entry_checks = 0;
integer                         total_checked_pixels = 0;
integer                         prefill_checks = 0;
integer                         row;
integer                         word_index;
integer                         timeout;
integer                         normal_frame;
reg                             expect_complete = 0;
reg                             checking_payload = 0;
reg                             test_passed = 0;
real                            frame_start_time;
real                            row_start_time;

always #3.365384615 pix_clk = ~pix_clk;
always #4.000 rx_clk = ~rx_clk;

GTP_GRS GRS_INST (
    .GRS_N                      (grs_n)
);

sfp_decode#(
    .RASTER_OUTPUT              (1),
    .PIC_WIDTH                  (WIDTH),
    .PIC_HEIGHT                 (HEIGHT),
    .H_BLANK                    (LINE_CLOCKS - WIDTH)
) dut (
    .clk_in                     (pix_clk),
    .rst_n                      (rst_n),
    .o_vs                       (new_vs),
    .data_valid_out             (new_de),
    .data_out                   (new_data),
    .rx_clk                     (rx_clk),
    .rx_rst_n                   (rst_n),
    .rx_charisk_align           (rx_charisk),
    .rx_data_align              (rx_data)
);

sfp_decode_reference#(
    .RASTER_OUTPUT              (1),
    .PIC_WIDTH                  (WIDTH),
    .PIC_HEIGHT                 (HEIGHT),
    .H_BLANK                    (LINE_CLOCKS - WIDTH)
) reference (
    .clk_in                     (pix_clk),
    .rst_n                      (rst_n),
    .o_vs                       (old_vs),
    .data_valid_out             (old_de),
    .data_out                   (old_data),
    .rx_clk                     (rx_clk),
    .rx_rst_n                   (rst_n),
    .rx_charisk_align           (rx_charisk),
    .rx_data_align              (rx_data)
);

function [15 : 0] pixel_value;
    input integer               frame;
    input integer               index;
    begin
        if(index == 0)begin
            pixel_value = 16'ha55a ^ frame;
        end
        else if(index == PIXELS - 1)begin
            pixel_value = 16'h7d3c ^ frame;
        end
        else begin
            pixel_value = ((index * 13) ^ (index >> 6) ^ (frame * 1777)) ^ 16'h3b17;
        end
    end
endfunction

function [23 : 0] expand_rgb565;
    input       [15 : 0]        pixel;
    begin
        expand_rgb565 = {pixel[15 : 11], pixel[15 : 13], pixel[10 : 5], pixel[10 : 9],
                         pixel[4 : 0], pixel[4 : 2]};
    end
endfunction

task check;
    input bit                   condition;
    input string                message;
    begin
        if(!condition)begin
            $fatal(1, "%s at %t", message, $time);
        end
    end
endtask

task send_word;
    input       [31 : 0]        value;
    input       [3 : 0]         charisk;
    begin
        @(negedge rx_clk);
        rx_data    = value;
        rx_charisk = charisk;
    end
endtask

task idle_words;
    input integer               count;
    begin
        repeat(count) send_word(32'h55a109bc, 0);
    end
endtask

task frame_marker;
    begin
        send_word(32'h55a101bc, 1);
        send_word(32'h55a102bc, 1);
        idle_words(200);
        check(!new_vs && !old_vs, "Frame reset did not release after marker");
    end
endtask

task send_rows;
    input integer               frame;
    input integer               row_count;
    integer                     send_row;
    integer                     send_word_index;
    real                        first_line;
    real                        target_line;
    begin
        first_line = $realtime + 200.0;
        for(send_row = 0; send_row < row_count; send_row = send_row + 1)begin
            target_line = first_line + send_row * SOURCE_LINE_NS;
            if(target_line > $realtime)begin
                #(target_line - $realtime);
            end
            send_word(32'h55a105bc, 1);
            send_word(32'h55a106bc, 1);
            for(send_word_index = 0; send_word_index < WIDTH / 2; send_word_index = send_word_index + 1)begin
                send_word({pixel_value(frame, send_row * WIDTH + send_word_index * 2 + 1),
                           pixel_value(frame, send_row * WIDTH + send_word_index * 2)}, 0);
            end
            send_word(32'h55a107bc, 1);
            send_word(32'h55a108bc, 1);
            idle_words(8);
        end
    end
endtask

// Extracted logical behavior of the unchanged top RGB/VS/DE entry stage.
// Sampling below also verifies the extra decoder clock precedes this stage.
always@(posedge pix_clk or negedge rst_n)begin
    if(!rst_n)begin
        entry_rgb <= 0;
        entry_vs  <= 1;
        entry_de  <= 0;
    end
    else begin
        entry_rgb <= expand_rgb565(new_data);
        entry_vs  <= ~new_vs;
        entry_de  <= new_de;
    end
end

always@(posedge pix_clk)begin
    pixel_cycle = pixel_cycle + 1;
    if(rst_n)begin
        check(new_vs === old_vs_delayed, "Decoder VS did not gain exactly one clock");
        check(dut.frame_reset === old_vs, "Internal frame-reset timing changed");
        check(dut.fifo_rd_en === reference.fifo_rd_en, "Raster read-enable/prefill timing changed");
        check((new_de && !new_vs) === old_valid_delayed,
            "Effective decoder DE did not gain exactly one clock");
        if(checking_payload && first_read_cycle < 0 && dut.fifo_rd_en && !dut.fifo_empty)begin
            first_read_cycle = pixel_cycle;
        end
        if(new_de && !new_vs)begin
            check(new_data === old_data_delayed, "Registered DRM RGB565 differs from delayed original");
            if(checking_payload)begin
                if(first_new_valid_cycle < 0)begin
                    first_new_valid_cycle = pixel_cycle;
                    check(first_new_valid_cycle - first_read_cycle == 2,
                        "Consumer-sampled new FIFO read-to-post latency is not two clocks");
                end
                check(new_data === pixel_value(frame_id, checked_new_pixels),
                    $sformatf("New RGB565 half-word order or pixel error at frame=%0d index=%0d: %h",
                        frame_id, checked_new_pixels, new_data));
                checked_new_pixels = checked_new_pixels + 1;
                total_checked_pixels = total_checked_pixels + 1;
            end
            if(!new_previous_de)begin
                if(previous_new_line_start >= 0 && expect_complete)begin
                    check(pixel_cycle - previous_new_line_start == LINE_CLOCKS,
                        "New raster line-start period differs from 2200 clocks");
                end
                previous_new_line_start = pixel_cycle;
            end
            new_line_length = new_line_length + 1;
        end
        else if(new_previous_de)begin
            if(expect_complete)begin
                check(new_line_length == WIDTH, "New raster DE line length differs from 1920 pixels");
            end
            new_lines       = new_lines + 1;
            new_line_length = 0;
        end
        if(old_de && !old_vs)begin
            if(checking_payload)begin
                if(first_old_valid_cycle < 0)begin
                    first_old_valid_cycle = pixel_cycle;
                    check(first_old_valid_cycle - first_read_cycle == 1,
                        "Consumer-sampled old FIFO read-to-post latency is not one clock");
                end
                check(old_data === pixel_value(frame_id, checked_old_pixels),
                    "Oracle pixel data disagrees with transmitted frame");
                checked_old_pixels = checked_old_pixels + 1;
            end
            old_line_length = old_line_length + 1;
        end
        else if(old_previous_de)begin
            if(expect_complete)begin
                check(old_line_length == WIDTH, "Oracle raster DE line length differs from 1920");
            end
            old_lines       = old_lines + 1;
            old_line_length = 0;
        end
        if(!old_vs && !new_vs && !dut.frame_reset)begin
            check(dut.rd_water_level === reference.rd_water_level,
                "Vendor FIFO read-water-level count changed during video/blanking");
            check(dut.fifo_empty === reference.fifo_empty,
                "Vendor FIFO empty flag changed during video/blanking");
        end
        if(dut.g_raster_output.u_raster_read_ctrl.frame_armed &&
           !dut.g_raster_output.u_raster_read_ctrl.running && !dut.frame_reset &&
           dut.rd_water_level >= WIDTH + 1536)begin
            prefill_checks = prefill_checks + 1;
        end
        check(!new_vs || !new_de, "New decoder emitted a valid pixel during VS reset");
        check(entry_vs === entry_expected_vs && entry_de === entry_expected_de,
            "Top entry VS/DE is not exactly one clock after decoder");
        if(entry_de)begin
            check(entry_rgb === entry_expected_rgb,
                "Top entry RGB565 expansion/alignment differs from decoder");
        end
        decode_checks = decode_checks + 1;
        entry_checks  = entry_checks + 1;
    end
    old_vs_delayed        = old_vs;
    old_valid_delayed     = old_de && !old_vs;
    old_data_delayed      = old_data;
    new_previous_de       = new_de && !new_vs;
    old_previous_de       = old_de && !old_vs;
    entry_expected_rgb    = expand_rgb565(new_data);
    entry_expected_vs     = ~new_vs;
    entry_expected_de     = new_de;
end

initial begin
    #100;
    grs_n = 1;
    repeat(10) @(negedge pix_clk);
    rst_n = 1;
    repeat(10) @(negedge pix_clk);
    // Two complete frames prove reset/prefill rearming and first/last pixels.
    for(normal_frame = 0; normal_frame < 2; normal_frame = normal_frame + 1)begin
        frame_marker();
        frame_id               = normal_frame;
        checked_new_pixels     = 0;
        checked_old_pixels     = 0;
        new_lines              = 0;
        old_lines              = 0;
        previous_new_line_start = -1;
        first_read_cycle        = -1;
        first_old_valid_cycle   = -1;
        first_new_valid_cycle   = -1;
        expect_complete        = 1;
        checking_payload       = 1;
        send_rows(frame_id, HEIGHT);
        timeout = 0;
        while(checked_new_pixels < PIXELS && timeout < LINE_CLOCKS * 5)begin
            @(negedge pix_clk);
            timeout = timeout + 1;
        end
        repeat(LINE_CLOCKS) @(negedge pix_clk);
        check(checked_new_pixels == PIXELS && checked_old_pixels == PIXELS,
            "Complete frame lost or duplicated first/last/intermediate pixels");
        check(new_lines == HEIGHT && old_lines == HEIGHT, "Complete frame line count changed");
        check(!dut.g_raster_output.frame_error && !reference.g_raster_output.frame_error,
            "Vendor FIFO raster unexpectedly aborted a complete frame");
        check(dut.rd_water_level == 0 && reference.rd_water_level == 0,
            "Frame payload remains in vendor FIFO after last pixel");
        checking_payload = 0;
        expect_complete  = 0;
        $display("PASS SFP registered FIFO frame=%0d pixels=%0d lines=%0d first=%h last=%h",
            frame_id, checked_new_pixels, new_lines, pixel_value(frame_id, 0), pixel_value(frame_id, PIXELS - 1));
        $display("PASS consumer-sampled FIFO read to decoder post-valid: old=%0d new=%0d clocks",
            first_old_valid_cycle - first_read_cycle, first_new_valid_cycle - first_read_cycle);
    end

    if(TEST_ABORT)begin
        // The next frame marker deliberately interrupts an active line after
        // two rows are received. Both implementations must discard the tail.
        frame_marker();
        frame_id               = 9;
        checked_new_pixels     = 0;
        checked_old_pixels     = 0;
        previous_new_line_start = -1;
        checking_payload       = 1;
        send_rows(frame_id, 2);
        wait(checked_new_pixels >= 128);
        checking_payload = 0;
        frame_marker();
        repeat(32) @(negedge pix_clk);
        check(!new_de && !old_de, "Frame marker left stale valid pixels");
        check(dut.g_raster_output.u_raster_read_ctrl.frame_armed &&
              reference.g_raster_output.u_raster_read_ctrl.frame_armed,
            "Frame marker did not rearm raster reader");
        $display("PASS SFP marker during active line: both readers flush and rearm");
    end
    check(prefill_checks >= 2, "1536-pixel prefill startup was not exercised");
    $display("SFP REGISTER TEST PASSED frames=2 height=%0d pixels=%0d sync=%0d entry=%0d extra_decode_clock=1 core=4496",
        HEIGHT, total_checked_pixels, decode_checks, entry_checks);
    test_passed = 1;
    $finish;
end

initial begin
    #40000000;
    $fatal(1, "SFP register test timeout");
end

endmodule
