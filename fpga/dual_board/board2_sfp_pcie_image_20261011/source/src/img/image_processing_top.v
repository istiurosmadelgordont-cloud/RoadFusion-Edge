// RGB888 -> optional DCP -> serial Gamma(Y)/CLAHE(Y)/highlight(Y) -> RGB888.
// Mode inputs are synchronous to clk and become effective at a frame boundary.
module image_processing_top#(
    parameter   RUNTIME_CONTROL                  =   1,
    parameter   PIC_WIDTH                        =   1920,
    parameter   PIC_HEIGHT                       =   1080,
    parameter   H_BLANK                          =   280,
    parameter   DENOISE_ENABLE                   =   0,
    parameter   DENOISE_SIGMA                    =   32,
    parameter   DENOISE_MIX                      =   256,
    parameter   DCP_CAPABLE                      =   1,
    parameter   GAMMA_CAPABLE                    =   1,
    parameter   CLAHE_CAPABLE                    =   1,
    parameter   OMEGA                            =   243,
    parameter   TX_MIN                           =   52,
    parameter   SKY_PROTECT_ENABLE               =   1,
    parameter   SKY_TX_MIN                       =   128,
    parameter   BILATERAL_ENABLE                 =   0,
    parameter   BILATERAL_GUIDE_ENABLE           =   1,
    parameter   CLAHE_CLIP_LIMIT                 =   400,
    parameter   CLAHE_STRENGTH                   =   3,
    parameter   CLAHE_ANTI_HALO_ENABLE           =   1,
    parameter   CLAHE_SHADOW_ENABLE              =   1,
    parameter   CLAHE_SHADOW_GAIN                =   64,
    parameter   CLAHE_BOOST_LIMIT                =   64,
    parameter   CLAHE_DONOR_MODE                 =   1,
    parameter   LIGHTDOWN_ENABLE                 =   0,
    parameter   LIGHTDOWN_THRESHOLD              =   0,
    parameter   DARKUP_ENABLE                    =   0,
    parameter   DARKUP_THRESHOLD                 =   0,
    parameter   HIGHLIGHT_CAPABLE                =   1,
    parameter   HIGHLIGHT_KNEE                   =   192,
    parameter   HIGHLIGHT_SLOPE                  =   128
)(
    input   [159 : 0]       cfg_data                         ,
    input                   cfg_apply                        ,
    output  [159 : 0]       cfg_active                       ,
    output                  cfg_pending                      ,
    output                  cfg_ack                          ,
    input                   clk                              ,
    input                   rst_n                            ,
    input                   pre_frame_vsync                  ,
    input                   pre_frame_href                   ,
    input                   pre_frame_clken                  ,
    input   [23 : 0]        pre_img                          ,
    input                   dcp_en                           ,
    input                   gamma_en                         ,
    input                   clahe_en                         ,
    input                   highlight_en                     ,
    output                  post_frame_vsync                 ,
    output                  post_frame_href                  ,
    output                  post_frame_clken                 ,
    output  [23 : 0]        post_img                         ,
    output  [3 : 0]         active_config                    ,
    output                  highlight_supported
);

localparam  LINE_CLOCKS                      =   PIC_WIDTH + H_BLANK;
localparam  DCP_CLOCKS                       =   (RUNTIME_CONTROL || BILATERAL_ENABLE) ? LINE_CLOCKS + 48 : 29;
reg                     previous_vs                      ,
                        config_seen                      ;
reg     [3 : 0]         frame_config                     ;
wire                    frame_start                      ;
wire    [3 : 0]         requested_config                 ,
                        input_config                     ;
wire    [30 : 0]        delayed_input                    ;
wire    [3 : 0]         dcp_config                       ;
wire                    dcp_vs                           ,
                        dcp_href                         ,
                        dcp_de                           ;
wire    [23 : 0]        dcp_rgb                          ,
                        calculated_dcp_rgb               ;
wire                    calculated_dcp_vs                ,
                        calculated_dcp_href              ,
                        calculated_dcp_de                ;
wire    [7 : 0]         calculated_tx                    ,
                        dcp_tx                           ;
wire    [3 : 0]         output_config                    ;
wire    [159 : 0]       frame_runtime                    ;
wire    [159 : 0]       dcp_runtime                      ;
localparam [31 : 0]        RESET_MODE                       =   (DENOISE_ENABLE != 0 ?
        32'd16 : 32'd0) |(BILATERAL_ENABLE != 0 ? 32'd32 : 32'd0) |(SKY_PROTECT_ENABLE != 0 ?
        32'd64 : 32'd0) |(LIGHTDOWN_ENABLE != 0 ? 32'd2048 : 32'd0) |
        (DARKUP_ENABLE != 0 ? 32'd4096 : 32'd0) |(CLAHE_ANTI_HALO_ENABLE != 0 ? 32'd128 : 32'd0) |(CLAHE_SHADOW_ENABLE != 0 ? 32'd256 : 32'd0);
localparam [31 : 0]        RESET_DCP                        =   (OMEGA << 16) |(SKY_TX_MIN << 8) | TX_MIN;
localparam [31 : 0]        RESET_DENOISE                    =   (LIGHTDOWN_THRESHOLD << 24) |(DARKUP_THRESHOLD << 16) |
        (DENOISE_SIGMA << 9) | DENOISE_MIX;
localparam [31 : 0]        RESET_CLAHE                      =   (CLAHE_SHADOW_GAIN << 19) |(CLAHE_STRENGTH << 16) |
        CLAHE_CLIP_LIMIT;
localparam [31 : 0]        RESET_HIGH                       =   (HIGHLIGHT_SLOPE << 16) |(HIGHLIGHT_KNEE << 8) |
        CLAHE_BOOST_LIMIT;
localparam [159 : 0]       RESET_CONFIG                     =   {RESET_HIGH, RESET_CLAHE, RESET_DENOISE, RESET_DCP,
        RESET_MODE};

generate
    if(RUNTIME_CONTROL)begin : g_runtime_control
        image_runtime_control#(
            .RESET_CONFIG                   (RESET_CONFIG                    )
        )u_control(
            .clk                            (clk                             ),
            .rst_n                          (rst_n                           ),
            .frame_start                    (frame_start                     ),
            .requested_config               (cfg_data                        ),
            .apply_toggle                   (cfg_apply                       ),
            .frame_config                   (frame_runtime                   ),
            .active_config                  (cfg_active                      ),
            .pending                        (cfg_pending                     ),
            .ack_toggle                     (cfg_ack                         )
        );

    end
    else begin : g_static_control

        assign  frame_runtime                    =   0;
        assign  cfg_active                       =   RESET_CONFIG;
        assign  cfg_pending                      =   0;
        assign  cfg_ack                          =   0;
    end
endgenerate

frame_config_delay#(
    .DATA_WIDTH                     (160                             ),
    .DELAY_CLOCKS                   (DCP_CLOCKS                      )
)u_dcp_runtime(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (pre_frame_vsync                 ),
    .pre_config                     (frame_runtime                   ),
    .post_config                    (dcp_runtime                     )
);

assign  frame_start                      =   previous_vs && !pre_frame_vsync;
assign  requested_config                 =   RUNTIME_CONTROL ?
                                             {frame_runtime[0] && DCP_CAPABLE, frame_runtime[1] && GAMMA_CAPABLE,
        frame_runtime[2] && CLAHE_CAPABLE, frame_runtime[3] && HIGHLIGHT_CAPABLE} : {dcp_en && DCP_CAPABLE, gamma_en &&
        GAMMA_CAPABLE, clahe_en && CLAHE_CAPABLE && CLAHE_STRENGTH != 0, highlight_en && HIGHLIGHT_CAPABLE};

assign  input_config                     =   frame_start || !config_seen ? requested_config : frame_config;
assign  active_config                    =   frame_config;
assign  highlight_supported              =   HIGHLIGHT_CAPABLE != 0;

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        previous_vs                      <=  1;
        config_seen                      <=  0;
        frame_config                     <=  0;
    end
    else begin
        previous_vs                      <=  pre_frame_vsync;
        config_seen                      <=  1;
        if(frame_start || !config_seen)
            frame_config                     <=  requested_config;
    end
end

video_stream_delay#(
    .DATA_WIDTH                     (31                              ),
    .DELAY_CLOCKS                   (DCP_CLOCKS                      ),
    .LINE_CLOCKS                    (LINE_CLOCKS                     )
)u_dcp_identity(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_data                       ({input_config, pre_frame_vsync, pre_frame_href, pre_frame_clken, pre_img}),
    .post_data                      (delayed_input                   )
);

assign  {dcp_config, dcp_vs, dcp_href, dcp_de} =   delayed_input[30 : 24];

generate
    if(DCP_CAPABLE)begin : g_dcp
        dcp_rgb#(
            .OMEGA                          (OMEGA                           ),
            .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),
            .PIC_WIDTH                      (PIC_WIDTH                       ),
            .PIC_HEIGHT                     (PIC_HEIGHT                      ),
            .H_BLANK                        (H_BLANK                         ),
            .TX_MIN                         (TX_MIN                          ),
            .SKY_PROTECT_ENABLE             (SKY_PROTECT_ENABLE              ),
            .SKY_TX_MIN                     (SKY_TX_MIN                      ),
            .BILATERAL_ENABLE               (BILATERAL_ENABLE                ),
            .BILATERAL_GUIDE_ENABLE         (BILATERAL_GUIDE_ENABLE          )
        )u_dcp_rgb(
            .cfg_data                       (frame_runtime                   ),
            .clk                            (clk                             ),
            .rst_n                          (rst_n                           ),
            .pre_frame_vsync                (pre_frame_vsync                 ),
            .pre_frame_href                 (pre_frame_href                  ),
            .pre_frame_clken                (pre_frame_clken                 ),
            .pre_img                        (pre_img                         ),
            .post_frame_vsync               (calculated_dcp_vs               ),
            .post_frame_href                (calculated_dcp_href             ),
            .post_frame_clken               (calculated_dcp_de               ),
            .post_img                       (calculated_dcp_rgb              ),
            .post_tx                        (calculated_tx                   )
        );

    end
    else begin : g_no_dcp

        assign  {calculated_dcp_vs, calculated_dcp_href, calculated_dcp_de} =   delayed_input[26 : 24];
        assign  calculated_dcp_rgb               =   delayed_input[23 : 0];
        assign  calculated_tx                    =   8'd255;
    end
endgenerate

assign  dcp_rgb                          =   dcp_config[3] ? calculated_dcp_rgb : delayed_input[23 : 0];
assign  dcp_tx                           =   dcp_config[3] ? calculated_tx : 8'd255;
rgb_enhancement_pipeline#(
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),
    .H_BLANK                        (H_BLANK                         ),
    .DENOISE_ENABLE                 (DENOISE_ENABLE                  ),
    .DENOISE_SIGMA                  (DENOISE_SIGMA                   ),
    .DENOISE_MIX                    (DENOISE_MIX                     ),
    .PIC_WIDTH                      (PIC_WIDTH                       ),
    .PIC_HEIGHT                     (PIC_HEIGHT                      ),
    .GAMMA_CAPABLE                  (GAMMA_CAPABLE                   ),
    .CLAHE_CAPABLE                  (CLAHE_CAPABLE                   ),
    .CLAHE_CLIP_LIMIT               (CLAHE_CLIP_LIMIT                ),
    .CLAHE_STRENGTH                 (CLAHE_STRENGTH                  ),
    .CLAHE_ANTI_HALO_ENABLE         (CLAHE_ANTI_HALO_ENABLE          ),
    .CLAHE_SHADOW_ENABLE            (CLAHE_SHADOW_ENABLE             ),
    .CLAHE_SHADOW_GAIN              (CLAHE_SHADOW_GAIN               ),
    .CLAHE_DONOR_MODE               (CLAHE_DONOR_MODE                ),
    .LIGHTDOWN_ENABLE               (LIGHTDOWN_ENABLE                ),
    .LIGHTDOWN_THRESHOLD            (LIGHTDOWN_THRESHOLD             ),
    .DARKUP_ENABLE                  (DARKUP_ENABLE                   ),
    .DARKUP_THRESHOLD               (DARKUP_THRESHOLD                ),
    .HIGHLIGHT_CAPABLE              (HIGHLIGHT_CAPABLE               ),
    .HIGHLIGHT_KNEE                 (HIGHLIGHT_KNEE                  ),
    .HIGHLIGHT_SLOPE                (HIGHLIGHT_SLOPE                 ),
    .CLAHE_BOOST_LIMIT              (CLAHE_BOOST_LIMIT               )
)u_enhancement(
    .cfg_data                       (dcp_runtime                     ),
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_frame_vsync                (dcp_vs                          ),
    .pre_frame_href                 (dcp_href                        ),
    .pre_frame_clken                (dcp_de                          ),
    .pre_img                        (dcp_rgb                         ),
    .pre_config                     (dcp_config                      ),
    .post_frame_vsync               (post_frame_vsync                ),
    .post_frame_href                (post_frame_href                 ),
    .post_frame_clken               (post_frame_clken                ),
    .post_img                       (post_img                        ),
    .post_config                    (output_config                   )
);

endmodule
