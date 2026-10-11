`timescale 1ns/1ps

module post_ddr_integration_tb;

    localparam WIDTH                        =   32;
    localparam HEIGHT                       =   16;
    localparam H_BLANK                      =   64;
    localparam LINE_CLOCKS                  =   WIDTH + H_BLANK;
    localparam CORE_CLOCKS                  =   2 * LINE_CLOCKS + 96;
    localparam FRAME_CLOCKS                 =   LINE_CLOCKS * 86;
    reg                     clk             =   0;
    reg                     pcie_clk        =   0;
    reg                     rst_n           =   0;
    reg                     ddr_ready       =   0;
    reg                     have_frame      =   0;
    reg                     pcie_ready      =   1;
    reg     [15 : 0]        source_rgb      =   0;
    reg                     source_vs       =   0;
    reg                     source_de       =   0;
    reg     [15 : 0]        read_response   =   0;
    reg     [159 : 0]       requested       =   160'h0080c040020301900000410000f38034000001c0;
    reg                     apply           =   0;
    reg                     raw_vs_d        =   0;
    reg                     raw_hs_d        =   0;
    reg                     raw_de_d        =   0;
    reg     [23 : 0]        old_rgb         =   0;
    reg     [2 : 0]         old_sync        =   0;
    reg     [23 : 0]        rgb_history[0 : CORE_CLOCKS];
    reg     [2 : 0]         sync_history[0 : CORE_CLOCKS];
    reg     [31 : 0]        mode_history[0 : CORE_CLOCKS];
    reg     [23 : 0]        captured_core_rgb;
    reg     [2 : 0]         captured_core_sync;
    reg     [159 : 0]       prior_active;
    reg                     prior_entry_vs;
    reg                     prior_ready;
    reg                     prior_read_vs   =   0;
    reg                     prior_de_out    =   0;
    integer                 i;
    integer                 source_count    =   0;
    integer                 read_count      =   0;
    integer                 age             =   0;
    integer                 sync_checks     =   0;
    integer                 input_checks    =   0;
    integer                 raw_write_checks =  0;
    integer                 bypass_pixels   =   0;
    integer                 processed_pixels = 0;
    integer                 changed_pixels  =   0;
    integer                 dma_checks      =   0;
    integer                 first_pixels    =   0;
    integer                 last_pixels     =   0;
    integer                 line_pixels     =   0;
    integer                 config_changes  =   0;
    integer                 reset_wait_checks = 0;
    reg                     test_passed     =   0;
    wire    [23 : 0]        hdmi_rgb;
    wire    [2 : 0]         hdmi_sync;

    always #5 clk = ~clk;
    always #4 pcie_clk = ~pcie_clk;

    hdmi_ddr_pcie#(
        .VIDEO_LENGTH                   (WIDTH),
        .VIDEO_HIGTH                    (HEIGHT),
        .IMAGE_H_BLANK                  (H_BLANK)
    )dut(
        .ref_clk                        (clk),
        .rst_board                      (rst_n),
        .ddr_pll_lock                   (),
        .ddr_init_done                  (),
        .mem_rst_n                      (),
        .mem_ck                         (),
        .mem_ck_n                       (),
        .mem_cke                        (),
        .mem_cs_n                       (),
        .mem_ras_n                      (),
        .mem_cas_n                      (),
        .mem_we_n                       (),
        .mem_odt                        (),
        .mem_a                          (),
        .mem_ba                         (),
        .mem_dqs                        (),
        .mem_dqs_n                      (),
        .mem_dq                         (),
        .mem_dm                         (),
        .hdmi_rst                       (),
        .iic_tx_scl                     (),
        .iic_tx_sda                     (),
        .iic_scl                        (),
        .iic_sda                        (),
        .hdmi_int_led                   (),
        .fram_done                      (),
        .pix_clk_in                     (clk),
        .vs_in                          (1'b0),
        .hs_in                          (1'b0),
        .de_in                          (1'b0),
        .r_in                           (8'd0),
        .g_in                           (8'd0),
        .b_in                           (8'd0),
        .ref_clk_n                      (pcie_clk),
        .ref_clk_p                      (pcie_clk),
        .rxn                            (1'b0),
        .rxp                            (1'b0),
        .txn                            (),
        .txp                            (),
        .pcie_perst_n                   (pcie_ready),
        .smlh_link_up                    (),
        .rdlh_link_up                    (),
        .pix_clk_out                    (),
        .vs_out                         (hdmi_sync[2]),
        .hs_out                         (hdmi_sync[1]),
        .de_out                         (hdmi_sync[0]),
        .r_out                          (hdmi_rgb[23 : 16]),
        .g_out                          (hdmi_rgb[15 : 8]),
        .b_out                          (hdmi_rgb[7 : 0]),
        .i_p_refckn_0                   (1'b0),
        .i_p_refckp_0                   (1'b0),
        .i_p_l2rxn                      (1'b0),
        .i_p_l2rxp                      (1'b0),
        .i_p_l3rxn                      (1'b0),
        .i_p_l3rxp                      (1'b0),
        .o_p_l2txn                      (),
        .o_p_l2txp                      (),
        .o_p_l3txn                      (),
        .o_p_l3txp                      (),
        .tx_disable                     ()
    );

    defparam dut.user_sync_gen.H_TOTAL = LINE_CLOCKS;
    defparam dut.user_sync_gen.H_SYNC = 8;
    defparam dut.user_sync_gen.H_BP = 12;
    defparam dut.user_sync_gen.H_FP = 44;
    defparam dut.user_sync_gen.H_ACT = WIDTH;
    defparam dut.user_sync_gen.V_TOTAL = 86;
    defparam dut.user_sync_gen.V_SYNC = 2;
    defparam dut.user_sync_gen.V_BP = 34;
    defparam dut.user_sync_gen.V_FP = 34;
    defparam dut.user_sync_gen.V_ACT = HEIGHT;

    function [23 : 0] expand565;
        input [15 : 0] rgb;
        begin
            expand565 = {rgb[15 : 11], 3'b0, rgb[10 : 5], 2'b0, rgb[4 : 0], 3'b0};
        end
    endfunction

    function [15 : 0] pattern565;
        input integer index;
        reg [4 : 0] r;
        reg [5 : 0] g;
        reg [4 : 0] b;
        begin
            r = 4 + (index * 7) % 25;
            g = 8 + (index * 11) % 52;
            b = 3 + (index * 13) % 27;
            pattern565 = {r, g, b};
        end
    endfunction

    // One-clock synchronous read response. The separate vendor test proves this latency
    // using the actual active 256-to-16 FIFO and its GTP_DRM9K simulation model.
    always@(posedge clk)begin
        if(!rst_n || !ddr_ready || !have_frame)begin
            read_response               <=  0;
            read_count                  <=  0;
            prior_read_vs               <=  0;
        end
        else begin
            prior_read_vs               <=  dut.vs_o;
            if(dut.vs_o && !prior_read_vs)
                read_count              <=  0;
            else if(dut.de_o)begin
                read_response           <=  pattern565(read_count);
                read_count              <=  read_count + 1;
            end
        end
    end

    // Independent, unchanged old DDR-to-HDMI restore sequence: RGB FF and sync two FFs.
    always@(posedge clk)begin
        raw_vs_d                        <=  dut.vs_o;
        raw_hs_d                        <=  dut.hs_o;
        raw_de_d                        <=  dut.de_o;
        old_rgb                         <=  expand565(read_response);
        old_sync                        <=  {raw_vs_d, raw_hs_d, raw_de_d};
    end

    // The SFP write stream intentionally differs from the read image.
    always@(negedge clk)begin
        source_count                    =   source_count + 1;
        source_rgb                      =   16'hf800 ^ source_count;
        source_de                       =   source_count % 7 != 0;
        source_vs                       =   source_count % 400 < 4;
    end

    always@(posedge clk)begin
        captured_core_rgb               =   dut.image_output_rgb;
        captured_core_sync              =   {~dut.image_output_vs, dut.image_output_hs, dut.image_output_de};
        prior_active                    =   dut.image_cfg_active;
        prior_entry_vs                  =   dut.image_stage_vs;
        prior_ready                     =   dut.image_frame_ready;
        if(!dut.image_rst_n)begin
            age                         =   0;
            prior_de_out                =   0;
            line_pixels                 =   0;
            for(i = 0; i <= CORE_CLOCKS; i = i + 1)begin
                rgb_history[i]          <=  0;
                sync_history[i]         <=  0;
                mode_history[i]         <=  0;
            end
        end
        else begin
            rgb_history[0]              <=  old_rgb;
            sync_history[0]             <=  old_sync;
            mode_history[0]             <=  dut.image_cfg_active[31 : 0];
            for(i = 1; i <= CORE_CLOCKS; i = i + 1)begin
                rgb_history[i]          <=  rgb_history[i - 1];
                sync_history[i]         <=  sync_history[i - 1];
                mode_history[i]         <=  mode_history[i - 1];
            end
            age                         =   age + 1;
        end
        #2;
        if(dut.ddr3_ctrl_top.video_data_in !== source_rgb ||
            dut.ddr3_ctrl_top.video_de_in !== source_de ||
            dut.ddr3_ctrl_top.video_vs_in !== source_vs ||
            dut.ddr3_ctrl_top.vs_in !== source_vs)
            $fatal(1, "DDR write stream passed through image processing");
        if(dut.ddr3_ctrl_top.video_rd_en !== dut.de_o)
            $fatal(1, "DDR read enable was delayed with consumer sync");
        raw_write_checks                =   raw_write_checks + 1;

        if(dut.u_pcie_dam_ctrl.hdmi_data_in !==
            {hdmi_rgb[23 : 19], hdmi_rgb[15 : 10], hdmi_rgb[7 : 3]} ||
            dut.u_pcie_dam_ctrl.vs_in !== hdmi_sync[2] ||
            dut.u_pcie_dam_ctrl.de_in !== hdmi_sync[0])
            $fatal(1, "HDMI and DMA do not share processed RGB/DE/VS");
        dma_checks                      =   dma_checks + 1;

        if(!dut.image_reset_async_n)begin
            if(dut.image_rst_n !== 0 || hdmi_sync[0] !== 0)
                $fatal(1, "Image output was not reset immediately");
        end
        else if(!prior_ready && dut.image_frame_ready)begin
            if(!old_sync[2])
                $fatal(1, "Image ready released outside restored DDR VS blanking");
            reset_wait_checks           =   reset_wait_checks + 1;
        end

        if(dut.image_rst_n && age > 2)begin
            if(dut.image_stage_rgb !== old_rgb ||
                {~dut.image_stage_vs, dut.image_stage_hs, dut.image_stage_de} !== old_sync)
                $fatal(1, "DDR restored RGB/control entry mismatch");
            input_checks                =   input_checks + 1;
        end

        if(dut.image_rst_n && age > CORE_CLOCKS + 3)begin
            if(hdmi_sync !== sync_history[CORE_CLOCKS])
                $fatal(1, "Output sync delay mismatch expected=%b got=%b age=%0d",
                    sync_history[CORE_CLOCKS], hdmi_sync, age);
            if(hdmi_rgb !== captured_core_rgb || hdmi_sync !== captured_core_sync)
                $fatal(1, "Shared output register differs from core output");
            sync_checks                 =   sync_checks + 1;
            if(hdmi_sync[0])begin
                if(!prior_de_out)begin
                    first_pixels        =   first_pixels + 1;
                    line_pixels         =   0;
                end
                line_pixels             =   line_pixels + 1;
                if((mode_history[CORE_CLOCKS] & 32'h0000181f) == 0)begin
                    if(hdmi_rgb !== rgb_history[CORE_CLOCKS])
                        $fatal(1, "All-off output changed old restored RGB expected=%h got=%h",
                            rgb_history[CORE_CLOCKS], hdmi_rgb);
                    bypass_pixels       =   bypass_pixels + 1;
                end
                else begin
                    processed_pixels    =   processed_pixels + 1;
                    if(hdmi_rgb !== rgb_history[CORE_CLOCKS])
                        changed_pixels  =   changed_pixels + 1;
                end
            end
            else if(prior_de_out)begin
                if(line_pixels != WIDTH)
                    $fatal(1, "Line did not retain first/last pixels: %0d", line_pixels);
                last_pixels             =   last_pixels + 1;
            end
            prior_de_out                =   hdmi_sync[0];
        end
        if(dut.image_rst_n && prior_active !== dut.image_cfg_active)begin
            if(prior_entry_vs !== 0)
                $fatal(1, "Configuration changed away from restored input frame boundary");
            config_changes              =   config_changes + 1;
        end
    end

    task set_mode;
        input [31 : 0] mode;
        begin
            @(negedge clk);
            requested[31 : 0]           =   mode;
            apply                       =   ~apply;
        end
    endtask

    initial begin
        force dut.pix_clk_out = clk;
        force dut.iic_clk = clk;
        force dut.pll_init_done = 1'b1;
        force dut.ddr_init_done = ddr_ready;
        force dut.fram_done = have_frame;
        force dut.ddr_ip_rst_n = rst_n;
        force dut.s_pclk_div2_rstn = pcie_ready;
        force dut.pclk_div2 = pcie_clk;
        force dut.post_rgb = source_rgb;
        force dut.post_frame_de = source_de;
        force dut.post_frame_vsync = source_vs;
        force dut.video_data_out = read_response;
        force dut.image_cfg_data = requested;
        force dut.image_cfg_apply = apply;
        repeat(12) @(negedge clk);
        rst_n                           =   1;
        repeat(12) @(negedge clk);
        ddr_ready                       =   1;
        repeat(12) @(negedge clk);
        if(dut.image_frame_ready)
            $fatal(1, "SFP marker released image before DDR frame exists");
        have_frame                      =   1;
        repeat(3 * FRAME_CLOCKS) @(negedge clk);
        wait(dut.de_o);
        repeat(11) @(negedge clk);
        set_mode(32'h000001c2);
        repeat(2 * FRAME_CLOCKS) @(negedge clk);
        set_mode(32'h000001c3);
        repeat(2 * FRAME_CLOCKS) @(negedge clk);
        wait(dut.de_o);
        repeat(11) @(negedge clk);
        pcie_ready                      =   0;
        repeat(8) @(negedge clk);
        pcie_ready                      =   1;
        if(dut.image_frame_ready)
            $fatal(1, "PCIe reset did not wait for next restored frame");
        repeat(2 * FRAME_CLOCKS) @(negedge clk);
        set_mode(32'h000001c0);
        repeat(2 * FRAME_CLOCKS) @(negedge clk);
        wait(dut.de_o);
        repeat(11) @(negedge clk);
        have_frame                      =   0;
        repeat(8) @(negedge clk);
        have_frame                      =   1;
        repeat(2 * FRAME_CLOCKS) @(negedge clk);
        if(bypass_pixels < 3 * WIDTH * HEIGHT || processed_pixels < WIDTH * HEIGHT ||
            changed_pixels < WIDTH * HEIGHT || config_changes < 2 || reset_wait_checks < 3 ||
            first_pixels < 100 || first_pixels != last_pixels)
            $fatal(1, "Insufficient integration coverage off=%0d processed=%0d changed=%0d config=%0d reset=%0d lines=%0d/%0d",
                bypass_pixels, processed_pixels, changed_pixels, config_changes, reset_wait_checks,
                first_pixels, last_pixels);
        test_passed                     =   1;
        $display("PASS post-DDR integration input=%0d sync=%0d rawwrites=%0d off=%0d processed=%0d changed=%0d DMA=%0d firstlast=%0d/%0d configs=%0d resets=%0d",
            input_checks, sync_checks, raw_write_checks, bypass_pixels, processed_pixels, changed_pixels,
            dma_checks, first_pixels, last_pixels, config_changes, reset_wait_checks);
        $finish;
    end

    initial begin
        #2000000;
        $fatal(1, "Integration timeout");
    end

endmodule
