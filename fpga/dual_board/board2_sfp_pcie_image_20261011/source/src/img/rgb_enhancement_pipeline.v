// One colour conversion pair around a serial Y-only enhancement pipeline.
// config[3:0] = {DCP, Gamma, CLAHE, highlight}; already frame-latched.
// Legacy threshold-based dark lift/highlight reduction follows CLAHE.
module rgb_enhancement_pipeline#(
    parameter   RUNTIME_CONTROL                  =   0,
    parameter   PIC_WIDTH                        =   1920,
    parameter   PIC_HEIGHT                       =   1080,
    parameter   H_BLANK                          =   280,
    parameter   DENOISE_ENABLE                   =   0,
    parameter   DENOISE_SIGMA                    =   32,
    parameter   DENOISE_MIX                      =   256,
    parameter   GAMMA_CAPABLE                    =   1,
    parameter   CLAHE_CAPABLE                    =   1,
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
    input                   clk                              ,
    input                   rst_n                            ,
    input                   pre_frame_vsync                  ,
    input                   pre_frame_href                   ,
    input                   pre_frame_clken                  ,
    input   [23 : 0]        pre_img                          ,
    input   [3 : 0]         pre_config                       ,
    output                  post_frame_vsync                 ,
    output                  post_frame_href                  ,
    output                  post_frame_clken                 ,
    output  [23 : 0]        post_img                         ,
    output  [3 : 0]         post_config
);

localparam  CLAHE_CLOCKS                    =   13;
localparam  TONE_CLOCKS                     =   2;
localparam  HIGHLIGHT_CLOCKS                =   7;
localparam  ENHANCE_FIXED_CLOCKS            =   3 + 1 + CLAHE_CLOCKS + TONE_CLOCKS + HIGHLIGHT_CLOCKS + 3;
localparam  DENOISE_CLOCKS                   =   RUNTIME_CONTROL ||(DENOISE_ENABLE && DENOISE_MIX != 0) ?
        PIC_WIDTH + H_BLANK + 19 : 1;
wire                    noise_vs                         ,
                        noise_href                       ,
                        noise_de                         ;
wire    [3 : 0]         noise_config                     ;
wire    [7 : 0]         noise_y                          ,
                        noise_cb                         ,
                        noise_cr                         ,
                        noise_original_y                 ;
wire                    rgb_vs                           ,
                        rgb_href                         ,
                        rgb_de                           ;
wire    [7 : 0]         rgb_y                            ,
                        rgb_cb                           ,
                        rgb_cr                           ;
wire    [3 : 0]         rgb_config                       ;
wire    [30 : 0]        gamma_identity                   ;
wire                    gamma_vs                         ,
                        gamma_href                       ,
                        gamma_de                         ;
wire    [3 : 0]         gamma_config                     ;
wire    [7 : 0]         gamma_y                          ,
                        gamma_cb                         ,
                        gamma_cr                         ;
wire    [7 : 0]         gamma_lut_y                      ;
wire    [30 : 0]        clahe_identity                   ;
wire                    clahe_vs                         ,
                        clahe_href                       ,
                        clahe_de                         ;
wire    [3 : 0]         clahe_config                     ;
wire    [7 : 0]         clahe_y                          ,
                        clahe_cb                         ,
                        clahe_cr                         ;
wire    [7 : 0]         clahe_calculated_y               ;
wire    [30 : 0]        tone_identity                    ;
wire                    tone_vs                          ;
wire                    tone_href                        ;
wire                    tone_de                          ;
wire    [3 : 0]         tone_config                      ;
wire    [7 : 0]         tone_y                           ;
wire    [7 : 0]         tone_cb                          ;
wire    [7 : 0]         tone_cr                          ;
wire    [30 : 0]        highlight_identity               ;
wire                    highlight_vs                     ,
                        highlight_href                   ,
                        highlight_de                     ;
wire    [3 : 0]         highlight_config                 ;
wire    [7 : 0]         highlight_y                      ,
                        highlight_cb                     ,
                        highlight_cr                     ;
wire                    converted_vs                     ,
                        converted_href                   ,
                        converted_de                     ;
wire    [23 : 0]        converted_img                    ;
wire    [26 : 0]        original_rgb                     ;
wire    [159 : 0]       rgb_runtime                      ;
wire    [159 : 0]       noise_runtime                    ;
wire    [159 : 0]       gamma_runtime                    ;
wire    [159 : 0]       clahe_runtime                    ;
wire    [159 : 0]       tone_runtime                     ;
wire    [159 : 0]       output_runtime                   ;
wire    [3 : 0]         denoised_config                  ;
wire                    denoised_vs                      ;
wire                    denoised_href                    ;
wire                    denoised_de                      ;
wire    [7 : 0]         denoised_y                       ;
wire    [7 : 0]         denoised_cb                      ;
wire    [7 : 0]         denoised_cr                      ;
wire    [7 : 0]         denoised_original_y              ;
wire    [38 : 0]        noise_result                     ;

frame_config_delay#(
    .DATA_WIDTH                     (160                             ),
    .DELAY_CLOCKS                   (3                               )
)u_rgb_runtime(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (pre_frame_vsync                 ),
    .pre_config                     (RUNTIME_CONTROL ? cfg_data : 160'd0),
    .post_config                    (rgb_runtime                     )
);

frame_config_delay#(
    .DATA_WIDTH                     (160                             ),
    .DELAY_CLOCKS                   (DENOISE_CLOCKS                  )
)u_noise_runtime(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (rgb_vs                          ),
    .pre_config                     (rgb_runtime                     ),
    .post_config                    (noise_runtime                   )
);

frame_config_delay#(
    .DATA_WIDTH                     (160                             ),
    .DELAY_CLOCKS                   (1                               )
)u_gamma_runtime(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (noise_vs                        ),
    .pre_config                     (noise_runtime                   ),
    .post_config                    (gamma_runtime                   )
);

frame_config_delay#(
    .DATA_WIDTH                     (160                             ),
    .DELAY_CLOCKS                   (CLAHE_CLOCKS                    )
)u_clahe_runtime(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (gamma_vs                        ),
    .pre_config                     (gamma_runtime                   ),
    .post_config                    (clahe_runtime                   )
);

frame_config_delay#(
    .DATA_WIDTH                     (160                             ),
    .DELAY_CLOCKS                   (TONE_CLOCKS                     )
)u_tone_runtime(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (clahe_vs                        ),
    .pre_config                     (clahe_runtime                   ),
    .post_config                    (tone_runtime                    )
);

frame_config_delay#(
    .DATA_WIDTH                     (160                             ),
    .DELAY_CLOCKS                   (HIGHLIGHT_CLOCKS + 3            )
)u_output_runtime(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (tone_vs                         ),
    .pre_config                     (tone_runtime                    ),
    .post_config                    (output_runtime                  )
);

VIP_RGB888_YCbCr444 u_rgb_to_ycbcr(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_frame_vsync                (pre_frame_vsync                 ),
    .pre_frame_href                 (pre_frame_href                  ),
    .pre_frame_clken                (pre_frame_clken                 ),
    .pre_img_red                    (pre_img[23 : 16]                ),
    .pre_img_green                  (pre_img[15 : 8]                 ),
    .pre_img_blue                   (pre_img[7 : 0]                  ),
    .post_frame_vsync               (rgb_vs                          ),
    .post_frame_href                (rgb_href                        ),
    .post_frame_clken               (rgb_de                          ),
    .post_img_Y                     (rgb_y                           ),
    .post_img_Cb                    (rgb_cb                          ),
    .post_img_Cr                    (rgb_cr                          )
);

video_stream_delay#(
    .DATA_WIDTH                     (4                               ),
    .DELAY_CLOCKS                   (3                               )
)u_rgb_config(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_data                       (pre_config                      ),
    .post_data                      (rgb_config                      )
);

// Denoise before both Gamma and CLAHE so their gain and CDF do not amplify raw noise.
bilateral_ycbcr#(
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),
    .ENABLE                         (DENOISE_ENABLE                  ),
    .SIGMA                          (DENOISE_SIGMA                   ),
    .MIX                            (DENOISE_MIX                     ),
    .PIC_WIDTH                      (PIC_WIDTH                       ),
    .PIC_HEIGHT                     (PIC_HEIGHT                      ),
    .H_BLANK                        (H_BLANK                         )
)u_denoise(
    .cfg_data                       (rgb_runtime                     ),
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (rgb_vs                          ),
    .pre_href                       (rgb_href                        ),
    .pre_de                         (rgb_de                          ),
    .pre_config                     (rgb_config                      ),
    .pre_y                          (rgb_y                           ),
    .pre_cb                         (rgb_cb                          ),
    .pre_cr                         (rgb_cr                          ),
    .post_vs                        (denoised_vs                     ),
    .post_href                      (denoised_href                   ),
    .post_de                        (denoised_de                     ),
    .post_config                    (denoised_config                 ),
    .post_y                         (denoised_y                      ),
    .post_cb                        (denoised_cb                     ),
    .post_cr                        (denoised_cr                     ),
    .post_original_y                (denoised_original_y             )
);

// Register the Y blend before the Gamma ROM input. Cb/Cr, mode bits and
// the pre-enhancement highlight reference take exactly the same extra clock.
video_stream_delay#(
    .DATA_WIDTH                     (39                              ),
    .DELAY_CLOCKS                   (1                               )
)u_denoise_result(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_data                       ({denoised_config, denoised_vs, denoised_href, denoised_de,
                                      denoised_y, denoised_cb, denoised_cr, denoised_original_y}),
    .post_data                      (noise_result                     )
);

assign  {noise_config, noise_vs, noise_href, noise_de, noise_y, noise_cb, noise_cr,
         noise_original_y} =   noise_result;

// Gamma has one clock even when disabled; Cb/Cr and mode bits take that clock.
video_stream_delay#(
    .DATA_WIDTH                     (31                              ),
    .DELAY_CLOCKS                   (1                               )
)u_gamma_identity(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_data                       ({noise_config, noise_vs, noise_href, noise_de, noise_y, noise_cb, noise_cr}),
    .post_data                      (gamma_identity                  )
);

assign  {gamma_config, gamma_vs, gamma_href, gamma_de, gamma_cb, gamma_cr} =   {gamma_identity[30 : 24],
        gamma_identity[15 : 0]};

generate
    if(GAMMA_CAPABLE)begin : g_gamma
        gamma_lookuptable u_gamma_lut(
            .video_clk                      (clk                             ),
            .rst_n                          (rst_n                           ),
            .video_data                     (noise_y                         ),
            .video_de                       (noise_de                        ),
            .gamma_de                       (                                ),
            .gamma_data                     (gamma_lut_y                     )
        );

    end
    else begin : g_no_gamma

        assign  gamma_lut_y                      =   gamma_identity[23 : 16];
    end
endgenerate

assign  gamma_y                          =   gamma_config[2] && GAMMA_CAPABLE ? gamma_lut_y : gamma_identity[23 : 16];
// Statistics must describe the Y actually entering CLAHE. A DCP/Gamma mode
// change invalidates the old CDF only; pixel controls retain their fixed latency.
reg     [1 : 0]         statistics_mode                  ;
reg                     statistics_seen                  ;
wire                    statistics_change                ;
reg                     statistics_flush                 ;
reg     [159 : 0]       statistics_runtime               ;
// Include DCP, t-filter, denoise, Gamma and clip-limit changes. Pixel-only
// CLAHE blending and highlights do not change the histogram population.
localparam [159 : 0]       STATISTICS_MASK                  =   160'h000000000000ffff00007fff00ffffff00000073;

// Register the change event before distributing synchronous statistics clear.
assign  statistics_change                =   !statistics_seen ||
                                             statistics_mode != gamma_config[3 : 2] ||
                                             (RUNTIME_CONTROL &&
        ((statistics_runtime ^ gamma_runtime) & STATISTICS_MASK) != 0);

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        statistics_mode                  <=  0;
        statistics_runtime               <=  0;
        statistics_seen                  <=  0;
        statistics_flush                 <=  0;
    end
    else begin
        statistics_mode                  <=  gamma_config[3 : 2];
        statistics_runtime               <=  gamma_runtime;
        statistics_seen                  <=  1;
        statistics_flush                 <=  statistics_change;
    end
end

video_stream_delay#(
    .DATA_WIDTH                     (31                              ),
    .DELAY_CLOCKS                   (CLAHE_CLOCKS                    )
)u_clahe_identity(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_data                       ({gamma_config, gamma_vs, gamma_href, gamma_de, gamma_y, gamma_cb, gamma_cr}),
    .post_data                      (clahe_identity                  )
);

assign  {clahe_config, clahe_vs, clahe_href, clahe_de, clahe_cb, clahe_cr} =   {clahe_identity[30 : 24],
        clahe_identity[15 : 0]};

generate
    if(CLAHE_CAPABLE &&(RUNTIME_CONTROL || CLAHE_STRENGTH != 0))begin : g_clahe
        clahe_luma#(
            .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),
            .PIC_WIDTH                      (PIC_WIDTH                       ),
            .PIC_HEIGHT                     (PIC_HEIGHT                      ),
            .CLIP_LIMIT                     (CLAHE_CLIP_LIMIT                ),
            .STRENGTH                       (CLAHE_STRENGTH                  ),
            .ANTI_HALO_ENABLE               (CLAHE_ANTI_HALO_ENABLE          ),
            .SHADOW_ENABLE                  (CLAHE_SHADOW_ENABLE             ),
            .SHADOW_GAIN                    (CLAHE_SHADOW_GAIN               ),
            .DONOR_MODE                     (CLAHE_DONOR_MODE                ),
            .BOOST_LIMIT                    (CLAHE_BOOST_LIMIT               )
        )u_clahe_luma(
            .cfg_data                       (gamma_runtime                   ),
            .clk                            (clk                             ),
            .rst_n                          (rst_n                           ),
            .stats_flush                    (statistics_flush                ),
            .pre_frame_vsync                (gamma_vs                        ),
            .pre_frame_href                 (gamma_href                      ),
            .pre_frame_clken                (gamma_de                        ),
            .pre_y                          (gamma_y                         ),
            .post_frame_vsync               (                                ),
            .post_frame_href                (                                ),
            .post_frame_clken               (                                ),
            .post_y                         (clahe_calculated_y              )
        );

    end
    else begin : g_no_clahe

        assign  clahe_calculated_y               =   clahe_identity[23 : 16];
    end
endgenerate

assign  clahe_y                          =   clahe_config[1] &&
                                             CLAHE_CAPABLE &&
                                             (RUNTIME_CONTROL || CLAHE_STRENGTH != 0) ?
                                             clahe_calculated_y : clahe_identity[23 : 16];
// Reuse the legacy video_enhance Y curve without its extra conversion pair.
// Two clocks are retained when both controls are disabled.
legacy_tone_luma#(
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),
    .LIGHTDOWN_ENABLE               (LIGHTDOWN_ENABLE                ),
    .LIGHTDOWN_THRESHOLD            (LIGHTDOWN_THRESHOLD             ),
    .DARKUP_ENABLE                  (DARKUP_ENABLE                   ),
    .DARKUP_THRESHOLD               (DARKUP_THRESHOLD                )
)u_legacy_tone(
    .cfg_data                       (clahe_runtime                   ),
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_y                          (clahe_y                         ),
    .post_y                         (tone_y                          )
);

video_stream_delay#(
    .DATA_WIDTH                     (31                              ),
    .DELAY_CLOCKS                   (TONE_CLOCKS                     )
)u_tone_identity(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_data                       ({clahe_config, clahe_vs, clahe_href, clahe_de, clahe_y, clahe_cb, clahe_cr}),
    .post_data                      (tone_identity                   )
);

assign  {tone_config, tone_vs, tone_href, tone_de, tone_cb, tone_cr} =   {tone_identity[30 : 24],
        tone_identity[15 : 0]};

// Highlight reference is the Y before Gamma/CLAHE/tone, aligned to the final Y.
video_stream_delay#(
    .DATA_WIDTH                     (31                              ),
    .DELAY_CLOCKS                   (HIGHLIGHT_CLOCKS                )
)u_highlight_reserved(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_data                       ({tone_config, tone_vs, tone_href, tone_de, tone_y, tone_cb, tone_cr}),
    .post_data                      (highlight_identity              )
);

wire    [7 : 0]         original_y_aligned               ,
                        protected_y                      ;
video_stream_delay#(
    .DATA_WIDTH                     (8                               ),
    .DELAY_CLOCKS                   (1 + CLAHE_CLOCKS + TONE_CLOCKS  )
)u_highlight_reference(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_data                       (noise_original_y                ),
    .post_data                      (original_y_aligned              )
);

generate
    if(HIGHLIGHT_CAPABLE)begin : g_highlight
        highlight_luma#(
            .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),
            .KNEE                           (HIGHLIGHT_KNEE                  ),
            .SLOPE                          (HIGHLIGHT_SLOPE                 )
        )u_highlight(
            .cfg_data                       (tone_runtime                    ),
            .clk                            (clk                             ),
            .rst_n                          (rst_n                           ),
            .original_y                     (original_y_aligned              ),
            .enhanced_y                     (tone_y                          ),
            .post_y                         (protected_y                     )
        );

    end
    else begin : g_no_highlight

        assign  protected_y                      =   highlight_identity[23 : 16];
    end
endgenerate

assign  {highlight_config, highlight_vs, highlight_href, highlight_de, highlight_cb,
        highlight_cr} =   {highlight_identity[30 : 24], highlight_identity[15 : 0]};

assign  highlight_y                      =   highlight_config[0] &&
                                             HIGHLIGHT_CAPABLE ?
                                             protected_y : highlight_identity[23 : 16];
YCbCr2RGB u_ycbcr_to_rgb(
    .i_sys_clk                      (clk                             ),
    .i_rst_n                        (rst_n                           ),
    .i_vs                           (highlight_vs                    ),
    .i_hs                           (highlight_href                  ),
    .i_convert_en                   (highlight_de                    ),
    .i_y_data                       (highlight_y                     ),
    .i_cb_data                      (highlight_cb                    ),
    .i_cr_data                      (highlight_cr                    ),
    .o_vs                           (converted_vs                    ),
    .o_hs                           (converted_href                  ),
    .o_convert_en                   (converted_de                    ),
    .o_red                          (converted_img[23 : 16]          ),
    .o_green                        (converted_img[15 : 8]           ),
    .o_blue                         (converted_img[7 : 0]            )
);

video_stream_delay#(
    .DATA_WIDTH                     (4                               ),
    .DELAY_CLOCKS                   (3                               )
)u_output_config(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_data                       (highlight_config                ),
    .post_data                      (post_config                     )
);

video_stream_delay#(
    .DATA_WIDTH                     (27                              ),
    .DELAY_CLOCKS                   (ENHANCE_FIXED_CLOCKS + DENOISE_CLOCKS),
    .LINE_CLOCKS                    (PIC_WIDTH + H_BLANK             )
)u_exact_rgb_bypass(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_data                       ({pre_frame_vsync, pre_frame_href, pre_frame_clken, pre_img}),
    .post_data                      (original_rgb                    )
);

// Runtime modes retain ENHANCE_FIXED_CLOCKS+DENOISE_CLOCKS clocks. With all enhancement disabled, preserve
// exact RGB rather than introducing a colour-conversion round-trip error.

assign  {post_frame_vsync, post_frame_href, post_frame_clken} =   original_rgb[26 : 24];
assign  post_img                         =   (RUNTIME_CONTROL ? (output_runtime[4] &&
        output_runtime[72 : 64] != 0) : (DENOISE_ENABLE && DENOISE_MIX != 0)) ||
                                             | post_config[2 : 0] ||
                                             (RUNTIME_CONTROL ? (output_runtime[11] || output_runtime[12]) :
                                             (LIGHTDOWN_ENABLE || DARKUP_ENABLE)) ?
                                             converted_img : original_rgb[23 : 0];

endmodule
