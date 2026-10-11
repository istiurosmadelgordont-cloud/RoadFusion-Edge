`timescale 1ns/1ps
module raster_read_tb;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0;
    reg frame = 0;
    reg [12:0] level = 0;
    reg empty = 0;
    wire rd, error;
    reg big_frame = 0;
    wire big_rd, big_error;
    integer cycle = 0;
    integer total = 0;
    integer run_length = 0;
    integer rows = 0;
    integer last_start = -1;
    integer big_total = 0;
    integer big_run = 0;
    integer big_rows = 0;
    integer big_start = -1;
    reg prior = 0;
    reg big_prior = 0;
    reg check_small = 0;
    reg small_done = 0;
    sfp_raster_read_ctrl #(.PIC_WIDTH(16), .PIC_HEIGHT(4), .H_BLANK(8), .PREFILL_PIXELS(4)) dut(
        .clk(clk), .rst_n(rst_n), .frame_reset(frame), .fifo_level(level),
        .fifo_empty(empty), .fifo_read(rd), .frame_error(error));
    sfp_raster_read_ctrl #(.PIC_WIDTH(1920), .PIC_HEIGHT(1080), .H_BLANK(280)) big(
        .clk(clk), .rst_n(rst_n), .frame_reset(big_frame), .fifo_level(13'd2048),
        .fifo_empty(1'b0), .fifo_read(big_rd), .frame_error(big_error));
    always @(posedge clk) begin
        cycle = cycle + 1;
        if(check_small) begin
            if(rd) begin
                total = total + 1;
                run_length = run_length + 1;
                if(!prior) begin
                    if(last_start >= 0 && cycle-last_start != 24) $fatal(1,"small line period");
                    last_start = cycle;
                end
            end
            if(prior && !rd) begin
                if(run_length != 16) $fatal(1,"small line length %d",run_length);
                rows = rows + 1;
                run_length = 0;
            end
            prior = rd;
        end
        if(big_rd) begin
            big_total = big_total + 1;
            big_run = big_run + 1;
            if(!big_prior) begin
                if(big_start >= 0 && cycle-big_start != 2200) $fatal(1,"1080p line period");
                big_start = cycle;
            end
        end
        if(big_prior && !big_rd) begin
            if(big_run != 1920) $fatal(1,"1080p line length");
            big_run = 0;
            big_rows = big_rows + 1;
        end
        big_prior = big_rd;
        if(big_error) $fatal(1,"unexpected 1080p error");
    end
    task marker;
        begin
            @(negedge clk); frame=1;
            repeat(3) @(negedge clk);
            frame=0;
        end
    endtask
    initial begin
        repeat(4) @(negedge clk);
        rst_n=1; level=64;
        repeat(8) @(negedge clk);
        if(rd || big_rd) $fatal(1,"read before first frame");
        big_frame=1;
        marker;
        big_frame=0; check_small=1;
        repeat(130) @(negedge clk);
        if(total!=64 || rows!=4 || error || rd) $fatal(1,"small frame count");
        check_small=0;
        // A missing next line must stop; later data cannot restart a damaged frame.
        level=20; marker;
        wait(rd); @(negedge clk); level=0;
        repeat(30) @(negedge clk);
        if(!error || rd) $fatal(1,"missing line not rejected");
        level=64;
        repeat(30) @(negedge clk);
        if(rd) $fatal(1,"bad frame restarted without VS");
        marker;
        wait(rd);
        if(error) $fatal(1,"frame recovery failed");
        // Unexpected FIFO empty during active pixels must also stop immediately.
        @(negedge clk); empty=1;
        repeat(2) @(negedge clk);
        if(rd || !error) $fatal(1,"empty FIFO not rejected");
        empty=0;
        marker;
        wait(rd);
        if(error) $fatal(1,"second recovery failed");
        small_done=1;
    end
    initial begin
        wait(big_rows==1080 && small_done);
        repeat(2300) @(negedge clk);
        if(big_total!=2073600 || big_rd) $fatal(1,"1080p extra/missing reads %d",big_total);
        $display("PASS: 1920x1080 exact pixel count, 2200-clock lines, frame gating, underrun and recovery");
        $finish;
    end
    initial begin
        #30000000;
        $fatal(1,"timeout");
    end
endmodule
