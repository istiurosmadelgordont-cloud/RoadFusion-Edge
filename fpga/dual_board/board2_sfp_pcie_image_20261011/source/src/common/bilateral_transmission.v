// Optional ordinary or original-Y-guided joint bilateral filtering of protected transmission.
// RGB, frame A and the diagnostic sky floor belong to the same centre pixel as filtered t.
// With filtering enabled, sky protection is a soft target; TX_MIN remains a hard floor.
module bilateral_transmission#(
    parameter   RUNTIME_CONTROL                  =   0,
    parameter   BILATERAL_ENABLE                 =   0,
    parameter   BILATERAL_GUIDE_ENABLE           =   1,
    parameter   TX_MIN                           =   52,
    parameter   PIC_WIDTH                        =   1920,
    parameter   PIC_HEIGHT                       =   1080,
    parameter   H_BLANK                          =   280
)(
    input   [159 : 0]       cfg_data                         ,
    input                   clk                              ,
    input                   rst_n                            ,
    input                   pre_frame_vsync                  ,
    input                   pre_frame_href                   ,
    input                   pre_frame_clken                  ,
    input   [23 : 0]        pre_img                          ,
    input   [7 : 0]         pre_tx_img                       ,
    input   [23 : 0]        pre_A                            ,
    input   [7 : 0]         pre_tx_floor                     ,
    output                  post_frame_vsync                 ,
    output                  post_frame_href                  ,
    output                  post_frame_clken                 ,
    output  [23 : 0]        post_img                         ,
    output  [7 : 0]         post_tx_img                      ,
    output  [23 : 0]        post_A                           ,
    output  [7 : 0]         post_tx_floor                    ,
    output  [7 : 0]         post_raw_tx_img
);

generate
    if(RUNTIME_CONTROL || BILATERAL_ENABLE)begin : g_bilateral
        wire    [7 : 0]         filtered_tx_img                  ;
        wire                    filtered_frame_vsync             ;
        wire                    filtered_frame_href              ;
        wire    [66 : 0]        delayed_pixel                    ;
        reg     [15 : 0]        guide_y_sum                      ;
        wire    [7 : 0]         guide_y                          ;
        reg     [7 : 0]         filter_input_tx                  ;
        reg                     filter_input_vs                  ;
        reg                     filter_input_de                  ;

        // Isolate the RGB guide sum from the line-buffer write port.
        // Protected t and sync enter both matrices on the same registered clock.
        always@(posedge clk or negedge rst_n)begin
            if(!rst_n)begin
                guide_y_sum                     <=  0;
                filter_input_tx                 <=  0;
                filter_input_vs                 <=  0;
                filter_input_de                 <=  0;
            end
            else begin
                guide_y_sum                     <=  pre_img[23 : 16] * 16'd77 +
                                                    pre_img[15 : 8] * 16'd150 + pre_img[7 : 0] * 16'd29;
                filter_input_tx                 <=  pre_tx_img < pre_tx_floor ? pre_tx_floor : pre_tx_img;
                filter_input_vs                 <=  pre_frame_vsync;
                filter_input_de                 <=  pre_frame_clken;
            end
        end

        assign  guide_y                          =   guide_y_sum[15 : 8];
        bilateral_filter_proc#(
            .GUIDE_ENABLE                   (BILATERAL_GUIDE_ENABLE          ),
            .IMG_HDISP                      (PIC_WIDTH                       ),
            .IMG_VDISP                      (PIC_HEIGHT                      ),
            .DELAY_NUM                      (H_BLANK                         )
        )u_bilateral_filter(
            .cfg_data                       (160'd0                          ),
            .clk                            (clk                             ),
            .rst_n                          (rst_n                           ),
            .per_img_vsync                  (filter_input_vs                 ),
            .per_img_href                   (filter_input_de                 ),
            .per_img_gray                   (filter_input_tx                 ),
            .per_img_guide                  (guide_y                         ),
            .post_img_vsync                 (filtered_frame_vsync            ),
            .post_img_href                  (filtered_frame_href             ),
            .post_img_gray                  (filtered_tx_img                 )
        );

        // Matrix centre delay = one complete line + 2 clocks.
        // Filter arithmetic = 16 clocks, with one input guide register. Preserve the original VS pulse timing.
        video_line_delay#(
            .DATA_WIDTH                     (67                              ),
            .LINE_CLOCKS                    (PIC_WIDTH + H_BLANK             ),
            .EXTRA_CLOCKS                   (19                              )
        )u_pixel_alignment(
            .clk                            (clk                             ),
            .rst_n                          (rst_n                           ),
            .pre_data                       ({pre_frame_vsync, pre_frame_href, pre_frame_clken, pre_img, pre_A,
                    pre_tx_floor, pre_tx_img}),
            .post_data                      (delayed_pixel                   )
        );

        assign  {post_frame_vsync, post_frame_href, post_frame_clken, post_img, post_A, post_tx_floor,
                post_raw_tx_img} =   delayed_pixel;
        wire    [159 : 0]       aligned_config                   ;
        wire    [7 : 0]         selected_tx                      ;
        wire                    filter_enabled                   ;
        wire    [7 : 0]         base_floor                       ;
        wire    [7 : 0]         clamp_floor                      ;
        frame_config_delay#(
            .DATA_WIDTH                     (160                             ),
            .DELAY_CLOCKS                   (PIC_WIDTH + H_BLANK + 19        )
        )u_filter_config(
            .clk                            (clk                             ),
            .rst_n                          (rst_n                           ),
            .pre_vs                         (pre_frame_vsync                 ),
            .pre_config                     (RUNTIME_CONTROL ? cfg_data : 160'd0),
            .post_config                    (aligned_config                  )
        );

        assign  filter_enabled                   =   RUNTIME_CONTROL ? aligned_config[5] : 1'b1;
        assign  base_floor                       =   RUNTIME_CONTROL ? aligned_config[39 : 32] : TX_MIN;
        assign  selected_tx                      =   filter_enabled ? filtered_tx_img : post_raw_tx_img;
        assign  clamp_floor                      =   filter_enabled ? base_floor : post_tx_floor;

        // Reapplying the centre sky floor would restore unsmoothed boundary changes.
        // Disabled filtering retains the exact original raw-t/sky-floor result.
        assign  post_tx_img                      =   selected_tx < clamp_floor ? clamp_floor : selected_tx;
    end
    else begin : g_bypass

        assign  post_frame_vsync                 =   pre_frame_vsync;
        assign  post_frame_href                  =   pre_frame_href;
        assign  post_frame_clken                 =   pre_frame_clken;
        assign  post_img                         =   pre_img;
        assign  post_A                           =   pre_A;
        assign  post_tx_floor                    =   pre_tx_floor;
        assign  post_raw_tx_img                  =   pre_tx_img;
        assign  post_tx_img                      =   pre_tx_img < pre_tx_floor ? pre_tx_floor : pre_tx_img;
    end
endgenerate

endmodule
