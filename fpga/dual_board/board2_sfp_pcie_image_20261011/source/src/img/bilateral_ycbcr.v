// Optional pre-enhancement 3x3 bilateral Y denoise. Chroma and raw-Y reference
// belong to the same centre pixel; frame-edge samples retain their original Y.
module bilateral_ycbcr#(
    parameter   RUNTIME_CONTROL                  =   0,
    parameter   ENABLE                           =   0,
                SIGMA                            =   32,
                MIX                              =   256,
    parameter   PIC_WIDTH                        =   1920,
                PIC_HEIGHT                       =   1080,
                H_BLANK                          =   280
)(
    input   [159 : 0]       cfg_data                         ,
    input                   clk                              ,
                            rst_n                            ,
    input                   pre_vs                           ,
                            pre_href                         ,
                            pre_de                           ,
    input   [3 : 0]         pre_config                       ,
    input   [7 : 0]         pre_y                            ,
                            pre_cb                           ,
                            pre_cr                           ,
    output                  post_vs                          ,
                            post_href                        ,
                            post_de                          ,
    output  [3 : 0]         post_config                      ,
    output  [7 : 0]         post_y                           ,
                            post_cb                          ,
                            post_cr                          ,
                            post_original_y
);

generate
    if(RUNTIME_CONTROL ||(ENABLE && MIX != 0))begin : g_filter
        wire    [7 : 0]         filtered_y                       ;
        wire                    filter_vs                        ,
                                filter_de                        ;
        wire    [30 : 0]        aligned                          ;
        wire    [16 : 0]        mixed_y                          ;
        bilateral_filter_proc#(
            .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),
            .IMG_HDISP                      (PIC_WIDTH                       ),
            .IMG_VDISP                      (PIC_HEIGHT                      ),
            .DELAY_NUM                      (H_BLANK                         ),
            .GUIDE_ENABLE                   (0                               ),
            .RANGE_SIGMA                    (SIGMA                           )
        )u_filter(
            .cfg_data                       (cfg_data                        ),
            .clk                            (clk                             ),
            .rst_n                          (rst_n                           ),
            .per_img_vsync                  (pre_vs                          ),
            .per_img_href                   (pre_de                          ),
            .per_img_gray                   (pre_y                           ),
            .per_img_guide                  (8'd0                            ),
            .post_img_vsync                 (filter_vs                       ),
            .post_img_href                  (filter_de                       ),
            .post_img_gray                  (filtered_y                      )
        );

        video_line_delay#(
            .DATA_WIDTH                     (31                              ),
            .LINE_CLOCKS                    (PIC_WIDTH + H_BLANK             ),
            .EXTRA_CLOCKS                   (18                              )
        )u_alignment(
            .clk                            (clk                             ),
            .rst_n                          (rst_n                           ),
            .pre_data                       ({pre_config, pre_vs, pre_href, pre_de, pre_y, pre_cb, pre_cr}),
            .post_data                      (aligned                         )
        );

        assign  {post_config, post_vs, post_href, post_de, post_original_y, post_cb, post_cr} =   aligned;
        wire    [159 : 0]       aligned_config                   ;
        wire    [8 : 0]         mix_value                        ;
        frame_config_delay#(
            .DATA_WIDTH                     (160                             ),
            .DELAY_CLOCKS                   (PIC_WIDTH + H_BLANK + 18        )
        )u_mix_config(
            .clk                            (clk                             ),
            .rst_n                          (rst_n                           ),
            .pre_vs                         (pre_vs                          ),
            .pre_config                     (RUNTIME_CONTROL ? cfg_data : 160'd0),
            .post_config                    (aligned_config                  )
        );

        assign  mix_value                        =   RUNTIME_CONTROL ? (aligned_config[4] ?
                aligned_config[72 : 64] : 9'd0) : MIX;

        assign  mixed_y                          =   filtered_y * mix_value + post_original_y * (9'd256 - mix_value) +
                17'd128;

        assign  post_y                           =   mixed_y[15 : 8];
`ifndef SYNTHESIS

        always@(posedge clk)
            if(rst_n && filter_de !== post_de)
                $fatal(1, "Y denoise data/metadata DE mismatch");
`endif

    end
    else begin : g_bypass

        assign  {post_config, post_vs, post_href, post_de, post_y, post_cb, post_cr} =   {pre_config, pre_vs, pre_href,
                pre_de, pre_y, pre_cb, pre_cr};

        assign  post_original_y                  =   pre_y;
    end
endgenerate

`ifndef SYNTHESIS

initial
    if(MIX < 0 || MIX > 256)
        $error("Denoise MIX must be 0..256");
`endif

endmodule
