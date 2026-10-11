`timescale 1ns / 1ps

module ddr_read_alignment_tb;

parameter integer               FULL_HD = 0;

localparam integer              WIDTH = 1920;
localparam integer              HEIGHT = FULL_HD ? 1080 : 2;
localparam integer              PIXELS = WIDTH * HEIGHT;
localparam integer              V_TOTAL = FULL_HD ? 1125 : 6;
localparam integer              V_BP = FULL_HD ? 36 : 2;
localparam integer              V_SYNC = FULL_HD ? 5 : 1;
localparam integer              VS_TO_DE = V_BP * 2200 + 192;

reg                             pix_clk = 0;
reg                             ddr_clk = 0;
reg                             rst_n = 0;
reg                             sync_rst_n = 0;
reg                             grs_n = 0;
reg                             write_en = 0;
reg             [255 : 0]       write_data = 0;
wire            [15 : 0]        video_data_out;
wire                            vs_o;
wire                            hs_o;
wire                            de_o;
wire            [8 : 0]         write_water_level;
reg                             ddr_stream_vs = 0;
reg                             ddr_stream_hs = 0;
reg                             ddr_stream_de = 0;
reg             [23 : 0]        image_stage_rgb = 0;
reg                             image_stage_vs = 0;
reg                             image_stage_hs = 0;
reg                             image_stage_de = 0;
reg             [1 : 0]         vs_history = 0;
reg             [1 : 0]         hs_history = 0;
reg             [1 : 0]         de_history = 0;
reg             [1 : 0]         sync_history_ready = 0;
integer                         pixel_cycle = 0;
integer                         read_pixels = 0;
integer                         checked_pixels = 0;
integer                         line_pixels = 0;
integer                         complete_lines = 0;
integer                         first_read_cycle = -1;
integer                         first_entry_cycle = -1;
integer                         last_read_cycle = -1;
integer                         last_entry_cycle = -1;
integer                         first_line_start = -1;
integer                         previous_line_start = -1;
integer                         sync_checks = 0;
integer                         vs_fall_cycle = -1;
integer                         written_blocks = 0;
integer                         block;
integer                         lane;
reg                             previous_entry_de = 0;
reg                             previous_vs = 0;
reg                             test_passed = 0;

always #3.365384615 pix_clk = ~pix_clk;
always #2.500 ddr_clk = ~ddr_clk;

GTP_GRS GRS_INST (
    .GRS_N                      (grs_n)
);

// The PDS uses source/read_ddr_fifo (256 -> 16, OUTPUT_REG=0), not the
// unrelated legacy ipcore/read_ddr_fifo (256 -> 32) directory.
rw_fifo_ctrl dut_fifo_ctrl (
    .rstn                       (rst_n),
    .ddr_clk                    (ddr_clk),
    .wfifo_wr_clk               (pix_clk),
    .wfifo_wr_en                (1'b0),
    .wfifo_wr_data16_in          (16'd0),
    .wfifo_rd_req               (1'b0),
    .wfifo_rd_water_level       (),
    .wfifo_rd_data256_out       (),
    .rfifo_rd_clk               (pix_clk),
    .rfifo_rd_en                (de_o),
    .rfifo_rd_data16_out        (video_data_out),
    .rfifo_wr_req               (write_en),
    .rfifo_wr_water_level       (write_water_level),
    .rfifo_wr_data256_in        (write_data),
    .vs_in                      (1'b0),
    .vs_out                     (vs_o)
);

sync_generator#(
    .V_TOTAL                    (V_TOTAL),
    .V_FP                       (FULL_HD ? 4 : 1),
    .V_BP                       (V_BP),
    .V_SYNC                     (V_SYNC),
    .V_ACT                      (HEIGHT),
    .H_TOTAL                    (2200),
    .H_FP                       (88),
    .H_BP                       (148),
    .H_SYNC                     (44),
    .H_ACT                      (WIDTH),
    .HV_OFFSET                  (0)
) user_sync_gen (
    .clk                        (pix_clk),
    .rstn                       (sync_rst_n),
    .vs_out                     (vs_o),
    .hs_out                     (hs_o),
    .de_out                     (de_o),
    .de_re                      (),
    .x_act                      (),
    .y_act                      ()
);

function [15 : 0] pixel_value;
    input integer               index;
    begin
        if(index == 0)begin
            pixel_value = 16'ha55a;
        end
        else if(index == PIXELS - 1)begin
            pixel_value = 16'h7d3c;
        end
        else begin
            pixel_value = (index * 13) ^ (index >> 6) ^ 16'h3b17;
        end
    end
endfunction

function [23 : 0] expand_rgb565;
    input       [15 : 0]        pixel;
    begin
        expand_rgb565 = {pixel[15 : 11], 3'b0, pixel[10 : 5], 2'b0, pixel[4 : 0], 3'b0};
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

// Proposed top wiring uses one control register to match the real FIFO read
// stage, then captures RGB and all controls together in the input register.
always@(posedge pix_clk or negedge rst_n)begin
    if(!rst_n)begin
        ddr_stream_vs   <= 0;
        ddr_stream_hs   <= 0;
        ddr_stream_de   <= 0;
        image_stage_rgb <= 0;
        image_stage_vs  <= 0;
        image_stage_hs  <= 0;
        image_stage_de  <= 0;
    end
    else begin
        ddr_stream_vs   <= vs_o;
        ddr_stream_hs   <= hs_o;
        ddr_stream_de   <= de_o;
        image_stage_rgb <= expand_rgb565(video_data_out);
        image_stage_vs  <= ~ddr_stream_vs;
        image_stage_hs  <= ddr_stream_hs;
        image_stage_de  <= ddr_stream_de;
    end
end

always@(posedge pix_clk)begin
    pixel_cycle = pixel_cycle + 1;
    if(rst_n)begin
        if(&sync_history_ready)begin
            check(image_stage_vs === ~vs_history[1] && image_stage_hs === hs_history[1] &&
                  image_stage_de === de_history[1], "Entry VS/HS/DE is not aligned at two clock stages");
            sync_checks = sync_checks + 1;
        end
        sync_history_ready = {sync_history_ready[0], 1'b1};
        if(previous_vs && !vs_o)begin
            vs_fall_cycle = pixel_cycle;
        end
        if(de_o)begin
            check(!dut_fifo_ctrl.rd_empty, "Actual DDR read FIFO was empty at active read");
            if(read_pixels == 0)begin
                first_read_cycle = pixel_cycle;
                check(pixel_cycle - vs_fall_cycle == VS_TO_DE,
                    "Actual VS falling edge to first DE differs from the sync generator porch");
            end
            if(read_pixels == PIXELS - 1)begin
                last_read_cycle = pixel_cycle;
            end
            read_pixels = read_pixels + 1;
        end
        if(image_stage_de)begin
            if(checked_pixels == 0)begin
                first_entry_cycle = pixel_cycle;
            end
            if(checked_pixels == PIXELS - 1)begin
                last_entry_cycle = pixel_cycle;
            end
            check(image_stage_rgb === expand_rgb565(pixel_value(checked_pixels)),
                $sformatf("DDR read FIFO/entry lost RGB565 pixel %0d: rgb=%h expected=%h",
                    checked_pixels, image_stage_rgb, expand_rgb565(pixel_value(checked_pixels))));
            if(!previous_entry_de)begin
                if(previous_line_start >= 0)begin
                    check(pixel_cycle - previous_line_start == 2200, "DDR restored row period is not 2200");
                end
                previous_line_start = pixel_cycle;
            end
            checked_pixels = checked_pixels + 1;
            line_pixels    = line_pixels + 1;
        end
        else if(previous_entry_de)begin
            check(line_pixels == WIDTH, "Restored entry row is not 1920 active pixels");
            complete_lines = complete_lines + 1;
            line_pixels    = 0;
        end
    end
    else begin
        sync_history_ready = 0;
    end
    vs_history       = {vs_history[0], vs_o};
    hs_history       = {hs_history[0], hs_o};
    de_history       = {de_history[0], de_o};
    previous_entry_de = image_stage_de;
    previous_vs       = vs_o;
end

always@(posedge ddr_clk)begin
    if(rst_n && write_en)begin
        check(!dut_fifo_ctrl.rd_wr_full, "DDR source overflowed the actual read FIFO");
        written_blocks = written_blocks + 1;
    end
end

initial begin
    #100;
    grs_n = 1;
    repeat(10) @(negedge pix_clk);
    rst_n = 1;
    repeat(10) @(negedge pix_clk);
    sync_rst_n = 1;
    wait(dut_fifo_ctrl.r_rd_rst);
    wait(!dut_fifo_ctrl.r_rd_rst);
    repeat(8) @(negedge ddr_clk);
    for(block = 0; block < PIXELS / 16; block = block + 1)begin
        @(negedge ddr_clk);
        write_en = 0;
        while(write_water_level >= 9'd240)begin
            @(negedge ddr_clk);
        end
        for(lane = 0; lane < 16; lane = lane + 1)begin
            write_data[lane * 16 +: 16] = pixel_value(block * 16 + lane);
        end
        write_en = 1;
    end
    @(negedge ddr_clk);
    write_en = 0;
    wait(checked_pixels == PIXELS);
    repeat(10) @(negedge pix_clk);
    check(read_pixels == PIXELS && complete_lines == HEIGHT, "DDR restored frame first/last or row count differs");
    check(written_blocks == PIXELS / 16, "DDR restored source block count differs");
    check(first_entry_cycle - first_read_cycle == 2 && last_entry_cycle - last_read_cycle == 2,
        "DDR FIFO read to consumer-sampled input stage is not exactly two clocks");
    check(dut_fifo_ctrl.rd_empty, "DDR FIFO has extra pixels after complete test frame");
    $display("DDR READ ALIGNMENT TEST PASSED pixels=%0d lines=%0d sync=%0d first=a55a last=7d3c read_to_entry=2 stream_control=1 entry=1 vs_fall_to_first_de=%0d blocks=%0d",
        checked_pixels, complete_lines, sync_checks, VS_TO_DE, written_blocks);
    test_passed = 1;
    $finish;
end

initial begin
    #(FULL_HD ? 20000000 : 100000);
    $fatal(1, "DDR read alignment test timeout");
end

endmodule
