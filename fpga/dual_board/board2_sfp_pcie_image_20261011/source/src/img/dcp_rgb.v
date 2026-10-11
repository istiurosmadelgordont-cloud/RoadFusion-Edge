// DCP only: raw/normalised dark channels, frame A, t, sky protection/filter and RGB recovery.
// This block contains no Gamma, CLAHE or colour-space enhancement conversions.
module dcp_rgb#(
    parameter   RUNTIME_CONTROL                  =   0,
    parameter   PIC_WIDTH                        =   1920,
    parameter   PIC_HEIGHT                       =   1080,
    parameter   H_BLANK                          =   280,
    parameter   OMEGA                            =   243,
    parameter   TX_MIN                           =   52,
    parameter   SKY_PROTECT_ENABLE               =   1,
    parameter   SKY_TX_MIN                       =   128,
    parameter   BILATERAL_ENABLE                 =   0,
    parameter   BILATERAL_GUIDE_ENABLE           =   1
)(
    input   [159 : 0]       cfg_data                         ,
    input                   clk                              ,
                            rst_n                            ,
    input                   pre_frame_vsync                  ,
                            pre_frame_href                   ,
                            pre_frame_clken                  ,
    input   [23 : 0]        pre_img                          ,
    output                  post_frame_vsync                 ,
                            post_frame_href                  ,
                            post_frame_clken                 ,
    output  [23 : 0]        post_img                         ,
    output  [7 : 0]         post_tx
);

wire                    dark_channel_frame_vsync         ;
wire                    dark_channel_frame_href          ;
wire                    dark_channel_frame_clken         ;
wire    [7 : 0]         dark_channel_img                 ;
wire                    src_cal_A_frame_vsync            ;
wire                    src_cal_A_frame_href             ;
wire                    src_cal_A_frame_clken            ;
wire    [7 : 0]         src_cal_A_result                 ;
wire    [23 : 0]        src_cal_A_rgb /*synthesis PAP_MARK_DEBUG="true"*/                   ;
wire    [23 : 0]        raw_cal_A_rgb /*synthesis PAP_MARK_DEBUG="true"*/                   ;
wire                    raw_A_estimate_valid /*synthesis PAP_MARK_DEBUG="true"*/ ;
wire    [23 : 0]        smoothed_A_rgb /*synthesis PAP_MARK_DEBUG="true"*/        ;
wire                    a_stabilize_initialized /*synthesis PAP_MARK_DEBUG="true"*/ ;
wire                    a_stabilize_updated /*synthesis PAP_MARK_DEBUG="true"*/ ;
localparam  NORMALIZE_CLOCKS                =   10;
localparam  ALIGN_CLOCKS                    =   NORMALIZE_CLOCKS + 7;
localparam  RECOVERY_CLOCKS                 =   12;
wire    [23 : 0]        normalized_rgb                   ;
reg     [15 : 0]        normalize_numerator_r            ;
reg     [15 : 0]        normalize_numerator_g            ;
reg     [15 : 0]        normalize_numerator_b            ;
reg     [23 : 0]        normalize_A                      ;
wire    [2 : 0]         normalize_sync                   ;
wire                    normalized_frame_vsync           ;
wire                    normalized_frame_href            ;
wire                    normalized_frame_clken           ;
wire    [7 : 0]         normalized_dark_img              ;
wire                    src_cal_A_valid       /*synthesis PAP_MARK_DEBUG="true"*/           ;
wire                    src_tx_frame_vsync               ;
wire                    src_tx_frame_href                ;
wire                    src_tx_frame_clken               ;
wire    [7 : 0]         src_tx_img                       ;
wire                    cal_src_frame_vsync              ;
wire                    cal_src_frame_href               ;
wire                    cal_src_frame_clken              ;
wire    [23 : 0]        cal_img                          ;
wire                    cal_tx_frame_vsync               ;
wire                    cal_tx_frame_href                ;
wire                    cal_tx_frame_clken               ;
wire    [7 : 0]         cal_tx_img                       ;
wire    [23 : 0]        cal_A                            ;
dark_channel#(
    .PIC_WIDTH                      (PIC_WIDTH                       )
)u_dark_channel(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    //Image data prepred to be processd
    .pre_frame_vsync                (pre_frame_vsync                 ),
    .pre_frame_href                 (pre_frame_href                  ),
    .pre_frame_clken                (pre_frame_clken                 ),
    .pre_img                        (pre_img                         ),
    //Image data has been processd
    .post_frame_vsync               (dark_channel_frame_vsync        ),
    .post_frame_href                (dark_channel_frame_href         ),
    .post_frame_clken               (dark_channel_frame_clken        ),
    .post_img                       (dark_channel_img                )
);

calculate_A u_calculate_A(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    //Image data prepred to be processd
    .pre_img                        (pre_img                         ),
    .dark_channel_frame_vsync       (dark_channel_frame_vsync        ),
    .dark_channel_frame_href        (dark_channel_frame_href         ),
    .dark_channel_frame_clken       (dark_channel_frame_clken        ),
    .dark_channel_img               (dark_channel_img                ),
    //Image data has been processd
    .post_frame_vsync               (src_cal_A_frame_vsync           ),
    .post_frame_href                (src_cal_A_frame_href            ),
    .post_frame_clken               (src_cal_A_frame_clken           ),
    .post_result                    (src_cal_A_result                ),
    .post_result_rgb                (raw_cal_A_rgb                   ),
    .post_done                      (src_cal_A_valid                 ),
    .post_estimate_valid            (raw_A_estimate_valid            )
);

atmosphere_temporal u_atmosphere_temporal(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .raw_A_rgb                      (raw_cal_A_rgb                   ),
    .raw_valid                      (raw_A_estimate_valid            ),
    .smoothed_A_rgb                 (smoothed_A_rgb                  ),
    .initialized                    (a_stabilize_initialized         ),
    .update_pulse                   (a_stabilize_updated             )
);

// Use the temporally smoothed atmospheric light for every pixel.
// Normalisation and RGB recovery sample the same A estimate.
assign  src_cal_A_rgb                    =   smoothed_A_rgb;

// Normalize every RGB component by the A sampled with that input pixel.
// One numerator clock plus nine divider clocks precede the five-clock dark channel.
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        normalize_numerator_r            <=  0;
        normalize_numerator_g            <=  0;
        normalize_numerator_b            <=  0;
        normalize_A                      <=  0;
    end
    else begin
        normalize_numerator_r            <=  {pre_img[23 : 16], 8'd0} - {8'd0, pre_img[23 : 16]};
        normalize_numerator_g            <=  {pre_img[15 : 8], 8'd0} - {8'd0, pre_img[15 : 8]};
        normalize_numerator_b            <=  {pre_img[7 : 0], 8'd0} - {8'd0, pre_img[7 : 0]};
        normalize_A                      <=  src_cal_A_rgb;
    end
end

pipelined_divide_u8 u_normalize_red(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .numerator                      (normalize_numerator_r           ),
    .denominator                    (normalize_A[23 : 16]            ),
    .quotient                       (normalized_rgb[23 : 16]         )
);

pipelined_divide_u8 u_normalize_green(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .numerator                      (normalize_numerator_g           ),
    .denominator                    (normalize_A[15 : 8]             ),
    .quotient                       (normalized_rgb[15 : 8]          )
);

pipelined_divide_u8 u_normalize_blue(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .numerator                      (normalize_numerator_b           ),
    .denominator                    (normalize_A[7 : 0]              ),
    .quotient                       (normalized_rgb[7 : 0]           )
);

video_stream_delay#(
    .DATA_WIDTH                     (3                               ),
    .DELAY_CLOCKS                   (NORMALIZE_CLOCKS                )
)u_normalize_sync(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_data                       ({pre_frame_vsync, pre_frame_href, pre_frame_clken}),
    .post_data                      (normalize_sync                  )
);

dark_channel#(
    .PIC_WIDTH                      (PIC_WIDTH                       )
)u_normalized_dark_channel(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_frame_vsync                (normalize_sync[2]               ),
    .pre_frame_href                 (normalize_sync[1]               ),
    .pre_frame_clken                (normalize_sync[0]               ),
    .pre_img                        (normalized_rgb                  ),
    .post_frame_vsync               (normalized_frame_vsync          ),
    .post_frame_href                (normalized_frame_href           ),
    .post_frame_clken               (normalized_frame_clken          ),
    .post_img                       (normalized_dark_img             )
);

wire    [159 : 0]       tx_config                        ;
wire    [159 : 0]       aligned_config                   ;
frame_config_delay#(
    .DATA_WIDTH                     (160                             ),
    .DELAY_CLOCKS                   (NORMALIZE_CLOCKS + 5            )
)u_tx_config(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (pre_frame_vsync                 ),
    .pre_config                     (RUNTIME_CONTROL ? cfg_data : 160'd0),
    .post_config                    (tx_config                       )
);

frame_config_delay#(
    .DATA_WIDTH                     (160                             ),
    .DELAY_CLOCKS                   (ALIGN_CLOCKS                    )
)u_aligned_config(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (pre_frame_vsync                 ),
    .pre_config                     (RUNTIME_CONTROL ? cfg_data : 160'd0),
    .post_config                    (aligned_config                  )
);

tx_get#(
    .NORMALIZED_INPUT               (1                               ),
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),
    .modification_value             (OMEGA                           )
)u_tx_get(
    .cfg_data                       (tx_config                       ),
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    //Image data prepred to be processd
    .pre_frame_vsync                (normalized_frame_vsync          ),
    .pre_frame_href                 (normalized_frame_href           ),
    .pre_frame_clken                (normalized_frame_clken          ),
    .pre_img                        (normalized_dark_img             ),
    .A_value                        (8'd255                          ),
    //Image data has been processd
    .post_frame_vsync               (src_tx_frame_vsync              ),
    .post_frame_href                (src_tx_frame_href               ),
    .post_frame_clken               (src_tx_frame_clken              ),
    .post_img                       (src_tx_img                      )
);

time_alignment#(
    .src_delay                      (ALIGN_CLOCKS                    )
)u_time_alignment(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    //Image data prepred to be processd
    .pre_src_frame_vsync            (pre_frame_vsync                 ),
    .pre_src_frame_href             (pre_frame_href                  ),
    .pre_src_frame_clken            (pre_frame_clken                 ),
    .pre_img                        (pre_img                         ),
    .pre_tx_frame_vsync             (src_tx_frame_vsync              ),
    .pre_tx_frame_href              (src_tx_frame_href               ),
    .pre_tx_frame_clken             (src_tx_frame_clken              ),
    .pre_tx_img                     (src_tx_img                      ),
    .pre_A                          (src_cal_A_rgb                   ),
    //Image data has been processd
    .post_src_frame_vsync           (cal_src_frame_vsync             ),
    .post_src_frame_href            (cal_src_frame_href              ),
    .post_src_frame_clken           (cal_src_frame_clken             ),
    .post_img                       (cal_img                         ),
    .post_tx_frame_vsync            (cal_tx_frame_vsync              ),
    .post_tx_frame_href             (cal_tx_frame_href               ),
    .post_tx_frame_clken            (cal_tx_frame_clken              ),
    .post_tx_img                    (cal_tx_img                      ),
    .post_A                         (cal_A                           )
);

// Sky protection follows the original pixel; delay its six-clock result to match normalized t.
// Sky protection is included before the optional transmission filter.
wire    [7 : 0]         recovery_tx_img                  /*synthesis PAP_MARK_DEBUG="true"*/ ;
wire                    recovery_frame_vsync             /*synthesis PAP_MARK_DEBUG="true"*/ ;
wire                    recovery_frame_href              ;
wire                    recovery_frame_clken             /*synthesis PAP_MARK_DEBUG="true"*/ ;
wire    [23 : 0]        recovery_img                     /*synthesis PAP_MARK_DEBUG="true"*/ ;
wire    [23 : 0]        recovery_A                       /*synthesis PAP_MARK_DEBUG="true"*/ ;
wire    [7 : 0]         recovery_tx_floor                /*synthesis PAP_MARK_DEBUG="true"*/ ;
wire    [7 : 0]         recovery_raw_tx_img              /*synthesis PAP_MARK_DEBUG="true"*/ ;
wire    [7 : 0]         sky_tx_floor                     ;
wire    [7 : 0]         sky_tx_floor_six                 ;
wire    [8 : 0]         sky_weight                       ;

generate
    if(RUNTIME_CONTROL || SKY_PROTECT_ENABLE)begin : g_sky_protect
        sky_protect#(
            .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),
            .TX_MIN                         (TX_MIN                          ),
            .SKY_TX_MIN                     (SKY_TX_MIN                      )
        )u_sky_protect(
            .cfg_data                       (cfg_data                        ),
            .clk                            (clk                             ),
            .rst_n                          (rst_n                           ),
            .pre_frame_vsync                (pre_frame_vsync                 ),
            .pre_frame_href                 (pre_frame_href                  ),
            .pre_frame_clken                (pre_frame_clken                 ),
            .pre_img                        (pre_img                         ),
            .aligned_tx_img                 (cal_tx_img                      ),
            .post_tx_img                    (                                ),
            .post_tx_floor                  (sky_tx_floor_six                ),
            .post_weight                    (sky_weight                      )
        );

    end
    else begin : g_no_sky_protect

        assign  sky_tx_floor_six                 =   TX_MIN;
        assign  sky_weight                       =   9'd0;
    end
endgenerate

video_stream_delay#(
    .DATA_WIDTH                     (8                               ),
    .DELAY_CLOCKS                   (ALIGN_CLOCKS - 6                )
)u_sky_floor_alignment(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_data                       (sky_tx_floor_six                ),
    .post_data                      (sky_tx_floor                    )
);

// Filter protected t; TX_MIN remains a hard floor, sky protection a soft target.
bilateral_transmission#(
    .TX_MIN                         (TX_MIN                          ),
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),
    .BILATERAL_ENABLE               (BILATERAL_ENABLE                ),
    .BILATERAL_GUIDE_ENABLE         (BILATERAL_GUIDE_ENABLE          ),
    .PIC_WIDTH                      (PIC_WIDTH                       ),
    .PIC_HEIGHT                     (PIC_HEIGHT                      ),
    .H_BLANK                        (H_BLANK                         )
)u_bilateral_transmission(
    .cfg_data                       (aligned_config                  ),
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_frame_vsync                (cal_src_frame_vsync             ),
    .pre_frame_href                 (cal_src_frame_href              ),
    .pre_frame_clken                (cal_src_frame_clken             ),
    .pre_img                        (cal_img                         ),
    .pre_tx_img                     (cal_tx_img                      ),
    .pre_A                          (cal_A                           ),
    .pre_tx_floor                   (sky_tx_floor                    ),
    .post_frame_vsync               (recovery_frame_vsync            ),
    .post_frame_href                (recovery_frame_href             ),
    .post_frame_clken               (recovery_frame_clken            ),
    .post_img                       (recovery_img                    ),
    .post_tx_img                    (recovery_tx_img                 ),
    .post_A                         (recovery_A                      ),
    .post_tx_floor                  (recovery_tx_floor               ),
    .post_raw_tx_img                (recovery_raw_tx_img             )
);

// Recover RGB for the downstream enhancement subsystem.
wire                    dehaze_frame_vsync               /*synthesis PAP_MARK_DEBUG="true"*/ ;
wire                    dehaze_frame_href                ;
wire                    dehaze_frame_clken               /*synthesis PAP_MARK_DEBUG="true"*/ ;
wire    [23 : 0]        dehaze_img                       ;
haze_removal_cal#(
    .tx_min                         (RUNTIME_CONTROL ? 1 : TX_MIN    )
)u_haze_removal_cal(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_src_frame_vsync            (recovery_frame_vsync            ),
    .pre_src_frame_href             (recovery_frame_href             ),
    .pre_src_frame_clken            (recovery_frame_clken            ),
    .pre_img                        (recovery_img                    ),
    .pre_tx_frame_vsync             (recovery_frame_vsync            ),
    .pre_tx_frame_href              (recovery_frame_href             ),
    .pre_tx_frame_clken             (recovery_frame_clken            ),
    .pre_tx_img                     (recovery_tx_img                 ),
    .pre_A                          (recovery_A                      ),
    .post_frame_vsync               (dehaze_frame_vsync              ),
    .post_frame_href                (dehaze_frame_href               ),
    .post_frame_clken               (dehaze_frame_clken              ),
    .post_img                       (dehaze_img                      )
);

assign  post_frame_vsync                 =   dehaze_frame_vsync;
assign  post_frame_href                  =   dehaze_frame_href;
assign  post_frame_clken                 =   dehaze_frame_clken;
assign  post_img                         =   dehaze_img;
video_stream_delay#(
    .DATA_WIDTH                     (8                               ),
    .DELAY_CLOCKS                   (RECOVERY_CLOCKS                 )
)u_output_transmission(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_data                       (recovery_tx_img                 ),
    .post_data                      (post_tx                         )
);

endmodule
