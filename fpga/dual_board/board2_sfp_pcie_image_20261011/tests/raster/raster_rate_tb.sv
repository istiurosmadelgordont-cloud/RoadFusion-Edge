`timescale 1ns/1fs

module raster_rate_case#(
    parameter integer CASE_ID                   =   0,
    parameter integer PREFILL                   =   1536,
    parameter real    OUTPUT_MHZ                =   148.571428571,
    parameter real    SOURCE_MHZ                =   148.5,
    parameter real    WR_PHASE_NS               =   0.0,
    parameter integer LEVEL_DELAY               =   4,
    parameter integer EXPECT_ABORT              =   0
)(
    output reg              done                        =   0
);

    localparam integer WIDTH                    =   1920;
    localparam integer HEIGHT                   =   1080;
    localparam integer LINE                     =   2200;
    localparam integer CAPACITY                 =   4096;
    localparam real    SOURCE_LINE_NS           =   LINE * 1000.0 / SOURCE_MHZ;

    reg                     read_clk                    =   0;
    reg                     write_clk                   =   0;
    reg                     rst_n                       =   0;
    reg                     frame_reset                 =   0;
    wire                    fifo_read;
    wire                    frame_error;
    wire                    fifo_empty;
    wire    [12 : 0]        fifo_level;
    reg     [12 : 0]        level_pipeline[0 : LEVEL_DELAY - 1];
    integer                 written_words               =   0;
    integer                 read_pixels                 =   0;
    integer                 occupancy;
    integer                 peak_occupancy              =   0;
    integer                 smallest_row_inventory      =   4096;
    integer                 abort_row                   =   -1;
    integer                 overflow_words              =   0;
    integer                 complete_lines              =   0;
    integer                 line_reads                  =   0;
    integer                 completed_frames            =   0;
    integer                 checked_frames              =   0;
    integer                 i;
    integer                 frame;
    integer                 row;
    integer                 word_index;
    reg                     previous_read               =   0;
    reg                     error_seen                  =   0;
    real                    first_line_start;
    real                    next_row_start;

    always #(500.0 / OUTPUT_MHZ) read_clk = ~read_clk;

    initial begin
        #(WR_PHASE_NS);
        forever #4 write_clk = ~write_clk;
    end

    assign fifo_empty = 2 * written_words - read_pixels <= 0;
    assign fifo_level = level_pipeline[LEVEL_DELAY - 1];

    sfp_raster_read_ctrl#(
        .PIC_WIDTH                      (WIDTH),
        .PIC_HEIGHT                     (HEIGHT),
        .H_BLANK                        (LINE - WIDTH),
        .PREFILL_PIXELS                 (PREFILL)
    )dut(
        .clk                            (read_clk),
        .rst_n                          (rst_n),
        .frame_reset                    (frame_reset),
        .fifo_level                     (fifo_level),
        .fifo_empty                     (fifo_empty),
        .fifo_read                      (fifo_read),
        .frame_error                    (frame_error)
    );

    // The delayed read-side level is sampled every output clock; empty uses
    // current occupancy as requested. This deliberately exposes stale levels.
    always@(posedge read_clk)begin
        occupancy = 2 * written_words - read_pixels;
        if(!rst_n || frame_reset)begin
            read_pixels = 0;
            previous_read = 0;
            line_reads = 0;
            complete_lines = 0;
            error_seen = 0;
            for(i = 0; i < LEVEL_DELAY; i = i + 1)
                level_pipeline[i] <= 0;
        end
        else begin
            level_pipeline[0] <= occupancy;
            for(i = 1; i < LEVEL_DELAY; i = i + 1)
                level_pipeline[i] <= level_pipeline[i - 1];
            if(dut.running && dut.column_count == LINE - 1 && dut.row_count < HEIGHT - 1)begin
                if(fifo_level < smallest_row_inventory)
                    smallest_row_inventory = fifo_level;
            end
            if(fifo_read)begin
                if(occupancy <= 0)
                    $fatal(1, "Case %0d: read empty FIFO", CASE_ID);
                read_pixels = read_pixels + 1;
                line_reads = line_reads + 1;
            end
            else if(previous_read)begin
                if(line_reads != WIDTH && !EXPECT_ABORT)
                    $fatal(1, "Case %0d: line has %0d rather than 1920 pixels", CASE_ID, line_reads);
                if(line_reads == WIDTH)
                    complete_lines = complete_lines + 1;
                line_reads = 0;
            end
            previous_read = fifo_read;
            #0.001;
            if(frame_error && !error_seen)begin
                error_seen = 1;
                if(abort_row == -1)
                    abort_row = dut.row_count;
            end
            if(frame_error && !EXPECT_ABORT)
                $fatal(1, "Case %0d: frame_error at row %0d, fifo_level=%0d", CASE_ID,
                       dut.row_count, fifo_level);
        end
    end

    initial begin
        repeat(8) @(negedge read_clk);
        rst_n = 1;
        for(frame = 0; frame < 2; frame = frame + 1)begin
            @(negedge read_clk);
            frame_reset = 1;
            written_words = 0;
            repeat(128) @(negedge read_clk);
            frame_reset = 0;
            // The optical sender emits a complete 960-word burst at 125 MHz
            // once per source HDMI line, not one continuous active raster.
            first_line_start = $realtime + 6.0 * SOURCE_LINE_NS + WR_PHASE_NS;
            for(row = 0; row < HEIGHT; row = row + 1)begin
                next_row_start = first_line_start + row * SOURCE_LINE_NS;
                if(next_row_start > $realtime)
                    #(next_row_start - $realtime);
                for(word_index = 0; word_index < WIDTH / 2; word_index = word_index + 1)begin
                    @(posedge write_clk);
                    occupancy = 2 * written_words - read_pixels;
                    if(occupancy + 2 > CAPACITY)begin
                        overflow_words = overflow_words + 1;
                        if(!EXPECT_ABORT)
                            $fatal(1, "Case %0d: FIFO overflow at row %0d word %0d occupancy=%0d",
                                   CASE_ID, row, word_index, occupancy);
                    end
                    else begin
                        written_words = written_words + 1;
                        occupancy = 2 * written_words - read_pixels;
                        if(occupancy > peak_occupancy)
                            peak_occupancy = occupancy;
                    end
                end
            end
            #(4.0 * SOURCE_LINE_NS);
            if(EXPECT_ABORT)begin
                if(!frame_error || read_pixels >= WIDTH * HEIGHT)
                    $fatal(1, "Case %0d: 64-prefill did not expose expected frame abort", CASE_ID);
            end
            else begin
                if(frame_error || read_pixels != WIDTH * HEIGHT || complete_lines != HEIGHT)
                    $fatal(1, "Case %0d: incomplete frame: reads=%0d lines=%0d error=%0d", CASE_ID,
                           read_pixels, complete_lines, frame_error);
                if(2 * written_words != read_pixels)
                    $fatal(1, "Case %0d: frame payload remains in FIFO", CASE_ID);
                completed_frames = completed_frames + 1;
            end
            checked_frames = checked_frames + 1;
        end
        $display("PASS raster case=%0d source=%.6f output=%.6f prefill=%0d delay=%0d phase=%.1f frames=%0d full=%0d peak=%0d min_row_level=%0d abort_row=%0d",
                 CASE_ID, SOURCE_MHZ, OUTPUT_MHZ, PREFILL, LEVEL_DELAY, WR_PHASE_NS,
                 checked_frames, completed_frames, peak_occupancy, smallest_row_inventory, abort_row);
        done = 1;
    end

endmodule

module raster_rate_tb;

    wire [7 : 0] done;
    reg          test_passed = 0;

    raster_rate_case#(.CASE_ID(0), .PREFILL(64), .EXPECT_ABORT(1)) old_nominal(done[0]);
    raster_rate_case#(.CASE_ID(1), .WR_PHASE_NS(0.7)) nominal(done[1]);
    raster_rate_case#(.CASE_ID(2), .OUTPUT_MHZ(148.5884), .WR_PHASE_NS(3.3)) sta_corner(done[2]);
    raster_rate_case#(.CASE_ID(3), .OUTPUT_MHZ(148.571428571 * 1.0001),
                     .SOURCE_MHZ(148.5 * 0.9999), .WR_PHASE_NS(1.4)) fast_rx_slow_source(done[3]);
    raster_rate_case#(.CASE_ID(4), .OUTPUT_MHZ(148.571428571 * 0.9999),
                     .SOURCE_MHZ(148.5 * 1.0001), .WR_PHASE_NS(6.2), .LEVEL_DELAY(3)) slow_rx_fast_source(done[4]);
    raster_rate_case#(.CASE_ID(5), .OUTPUT_MHZ(148.5884 * 1.0001),
                     .SOURCE_MHZ(148.5 * 0.9999), .WR_PHASE_NS(7.1)) worst_rate_corner(done[5]);
    raster_rate_case#(.CASE_ID(6), .OUTPUT_MHZ(148.5884 * 0.9999),
                     .SOURCE_MHZ(148.5 * 1.0001), .WR_PHASE_NS(4.9), .LEVEL_DELAY(3)) opposite_corner(done[6]);
    raster_rate_case#(.CASE_ID(7), .PREFILL(64), .OUTPUT_MHZ(148.5884),
                     .WR_PHASE_NS(5.8), .EXPECT_ABORT(1)) old_sta_corner(done[7]);

    initial begin
        wait(&done);
        $display("PASS raster margin: 8 cases x 2 full1080p source frames, 6 passing new-prefill corners, 2 old-prefill abort cases");
        test_passed = 1;
        $finish;
    end

    initial begin
        #50000000;
        $fatal(1, "Raster test timeout");
    end

endmodule
