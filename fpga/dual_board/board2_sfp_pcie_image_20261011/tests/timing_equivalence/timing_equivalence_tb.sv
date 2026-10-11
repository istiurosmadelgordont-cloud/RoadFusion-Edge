`timescale 1ns/1ps
module timing_equivalence_tb;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0;
    reg vs = 1;
    reg hs = 0;
    reg en = 0;
    reg [23:0] rgb = 0;
    reg [159:0] cfg = 0;
    reg [7:0] tx = 0;
    wire [7:0] floor_new;
    wire [7:0] floor_old;
    wire [8:0] weight_new;
    wire [8:0] weight_old;
    wire [7:0] tx_new;
    wire [7:0] tx_old;
    reg [7:0] y = 0;
    reg [7:0] cb = 0;
    reg [7:0] cr = 0;
    wire [23:0] rgb_new;
    wire [23:0] rgb_old;
    wire [2:0] sync_new;
    wire [2:0] sync_old;
    integer cycles = 0;
    integer checked = 0;
    integer seed = 32'h12abc654;
    integer n;
    reg test_passed = 0;

    sky_protect #(.RUNTIME_CONTROL(1)) dut_sky(
        .clk(clk), .rst_n(rst_n), .pre_frame_vsync(vs), .pre_frame_href(hs),
        .pre_frame_clken(en), .pre_img(rgb), .cfg_data(cfg), .aligned_tx_img(tx),
        .post_tx_floor(floor_new), .post_weight(weight_new), .post_tx_img(tx_new)
    );
    sky_reference #(.RUNTIME_CONTROL(1)) ref_sky(
        .clk(clk), .rst_n(rst_n), .pre_frame_vsync(vs), .pre_frame_href(hs),
        .pre_frame_clken(en), .pre_img(rgb), .cfg_data(cfg), .aligned_tx_img(tx),
        .post_tx_floor(floor_old), .post_weight(weight_old), .post_tx_img(tx_old)
    );
    YCbCr2RGB dut_color(
        .i_sys_clk(clk), .i_rst_n(rst_n), .i_vs(vs), .i_hs(hs), .i_convert_en(en),
        .i_y_data(y), .i_cb_data(cb), .i_cr_data(cr), .o_red(rgb_new[23:16]),
        .o_green(rgb_new[15:8]), .o_blue(rgb_new[7:0]),
        .o_vs(sync_new[2]), .o_hs(sync_new[1]), .o_convert_en(sync_new[0])
    );
    YCbCr2RGB_reference ref_color(
        .i_sys_clk(clk), .i_rst_n(rst_n), .i_vs(vs), .i_hs(hs), .i_convert_en(en),
        .i_y_data(y), .i_cb_data(cb), .i_cr_data(cr), .o_red(rgb_old[23:16]),
        .o_green(rgb_old[15:8]), .o_blue(rgb_old[7:0]),
        .o_vs(sync_old[2]), .o_hs(sync_old[1]), .o_convert_en(sync_old[0])
    );
    always @(posedge clk) begin
        #1;
        cycles = cycles + 1;
        if(rst_n && cycles > 16)begin
            if({floor_new,weight_new,tx_new} !== {floor_old,weight_old,tx_old})
                $fatal(1,"sky mismatch cycle %0d floor %0d/%0d weight %0d/%0d",cycles,floor_new,floor_old,weight_new,weight_old);
            if({rgb_new,sync_new} !== {rgb_old,sync_old})
                $fatal(1,"color mismatch cycle %0d RGB %h/%h",cycles,rgb_new,rgb_old);
            checked = checked + 1;
        end
    end
    initial begin
        repeat(8) @(negedge clk);
        rst_n = 1;
        for(n=0;n<16777216;n=n+1)begin
            @(negedge clk);
            {y,cb,cr} = n[23:0];
            rgb = $random(seed);
            tx = $random(seed);
            // Random RGB features; real frame boundaries with blanking.
            if(n%4096==0)begin
                vs = 0;
                hs = 0;
                en = 0;
                cfg[39:32] = $random(seed);
                cfg[47:40] = $random(seed);
                cfg[6] = (n/4096)%2;
            end
            else begin
                vs = (n%4096)>=12;
                hs = (n%256)>=20;
                en = hs && ((n%31)!=0);
            end
        end
        vs = 0;
        hs = 0;
        en = 0;
        repeat(16) @(negedge clk);
        test_passed = 1;
        $display("PASS same-latency sky/config and exhaustive 256^3 YCbCr arithmetic: %0d cycles",checked);
        $finish;
    end
endmodule
