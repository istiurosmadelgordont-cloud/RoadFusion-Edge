// One pixel per clock. CLAHE statistics from frame N are used in frame N+1.
// Tile histogram, clipping, CDF storage and adaptive blend follow the donor.
module clahe_luma_reference#(
    parameter   RUNTIME_CONTROL                  =   0,
    parameter   PIC_WIDTH                        =   1920,
    parameter   PIC_HEIGHT                       =   1080,
    parameter   CLIP_LIMIT                       =   384,
    parameter   STRENGTH                         =   3,
    parameter   ANTI_HALO_ENABLE                 =   1,
    parameter   SHADOW_ENABLE                    =   1,
    parameter   SHADOW_GAIN                      =   48,
    parameter   BOOST_LIMIT                      =   48,
    parameter   DONOR_MODE                       =   0
)(
    input   [159 : 0]       cfg_data                         ,
    input                   clk                              ,
    input                   rst_n                            ,
    input                   stats_flush                      ,
    input                   pre_frame_vsync                  ,
    input                   pre_frame_href                   ,
    input                   pre_frame_clken                  ,
    input   [7 : 0]         pre_y                            ,
    output                  post_frame_vsync                 ,
    output                  post_frame_href                  ,
    output                  post_frame_clken                 ,
    output  [7 : 0]         post_y
);

wire    [159 : 0]       blend_config                     ;
wire    [159 : 0]       finish_config                    ;
wire    [2 : 0]         strength_value                   ;
wire                    anti_halo_value                  ;
wire                    shadow_value                     ;
wire    [6 : 0]         shadow_gain_value                ;
wire                    finish_anti_halo                 ;
wire                    finish_shadow                    ;
wire    [7 : 0]         boost_value                      ;
frame_config_delay#(
    .DATA_WIDTH                     (160                             ),
    .DELAY_CLOCKS                   (7                               )
)u_blend_config(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (pre_frame_vsync                 ),
    .pre_config                     (RUNTIME_CONTROL ? cfg_data : 160'd0),
    .post_config                    (blend_config                    )
);

frame_config_delay#(
    .DATA_WIDTH                     (160                             ),
    .DELAY_CLOCKS                   (11                              )
)u_finish_config(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (pre_frame_vsync                 ),
    .pre_config                     (RUNTIME_CONTROL ? cfg_data : 160'd0),
    .post_config                    (finish_config                   )
);

assign  strength_value                   =   RUNTIME_CONTROL ? blend_config[114 : 112] : STRENGTH;
assign  anti_halo_value                  =   RUNTIME_CONTROL ? blend_config[7] : ANTI_HALO_ENABLE;
assign  shadow_value                     =   RUNTIME_CONTROL ? blend_config[8] : SHADOW_ENABLE;
assign  shadow_gain_value                =   RUNTIME_CONTROL ? blend_config[121 : 115] : SHADOW_GAIN;
assign  finish_anti_halo                 =   RUNTIME_CONTROL ? finish_config[7] : ANTI_HALO_ENABLE;
assign  finish_shadow                    =   RUNTIME_CONTROL ? finish_config[8] : SHADOW_ENABLE;
assign  boost_value                      =   RUNTIME_CONTROL ? finish_config[135 : 128] : BOOST_LIMIT;
// Clear statistics synchronously after an upstream mode change.
// The thirteen-clock pixel pipeline retains only the real reset.
// Legacy callers tie stats_flush to zero.
localparam  TILE_WIDTH                       =   PIC_WIDTH / 8;
localparam  TILE_HEIGHT                      =   PIC_HEIGHT / 8;
localparam  TILE_PIXELS                      =   TILE_WIDTH * TILE_HEIGHT;
reg     [10 : 0]        pixel_x                          ,
                        pixel_y                          ;
reg     [2 : 0]         tile_row                         ;
reg                     previous_vs                      ;
reg                     frame_seen                       ,
                        frame_accept                     ;
wire                    frame_reset                      ;
wire                    histogram_ready                  ,
                        histogram_de                     ,
                        histogram_row_done               ;
wire    [2 : 0]         read_col                         ;
wire    [7 : 0]         read_bin                         ;
wire    [15 : 0]        read_count                       ;
wire                    clear_start                      ,
                        frame_cdf_done                   ;
wire                    cdf_write_enable                 ;
wire    [5 : 0]         cdf_write_tile                   ;
wire    [7 : 0]         cdf_write_bin                    ,
                        cdf_write_data                   ;
wire    [5 : 0]         tile_ul                          ,
                        tile_ur                          ,
                        tile_ll                          ,
                        tile_lr                          ;
wire    [8 : 0]         weight_x                         ,
                        weight_y                         ;
wire    [7 : 0]         cdf_ul                           ,
                        cdf_ur                           ,
                        cdf_ll                           ,
                        cdf_lr                           ;
wire    [2 : 0]         histogram_col                    ;
reg     [7 : 0]         cdf_sample_y                     ;

assign  frame_reset                      =   previous_vs && !pre_frame_vsync;
assign  histogram_col                    =   pixel_x / TILE_WIDTH;
assign  histogram_de                     =   pre_frame_clken && histogram_ready &&(frame_accept || !frame_seen);

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        pixel_x                          <=  0;
        pixel_y                          <=  0;
        tile_row                         <=  0;
        previous_vs                      <=  0;
        frame_seen                       <=  0;
        frame_accept                     <=  0;
    end
    else begin
        previous_vs                      <=  pre_frame_vsync;
        if(stats_flush || frame_reset)begin
            pixel_x                          <=  0;
            pixel_y                          <=  0;
            tile_row                         <=  0;
            frame_seen                       <=  0;
            frame_accept                     <=  0;
        end
        else begin
            if(histogram_row_done)
                tile_row                         <=  tile_row + 1'b1;
            if(pre_frame_clken)begin
                if(!frame_seen)begin
                    frame_seen                       <=  1;
                    frame_accept                     <=  histogram_ready;
                end
                if(pixel_x == PIC_WIDTH - 1)begin
                    pixel_x                          <=  0;
                    pixel_y                          <=  pixel_y == PIC_HEIGHT - 1 ? 0 : pixel_y + 1'b1;
                end
                else
                    pixel_x                          <=  pixel_x + 1'b1;
            end
        end
    end
end

clahe_histogram#(
    .PIC_WIDTH                      (PIC_WIDTH                       ),
    .TILE_HEIGHT                    (TILE_HEIGHT                     )
)u_histogram(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .clear                          (stats_flush                     ),
    .frame_reset                    (frame_reset                     ),
    .pre_de                         (histogram_de                    ),
    .pre_y                          (pre_y                           ),
    .pixel_x                        (pixel_x                         ),
    .tile_col                       (histogram_col                   ),
    .row_done                       (histogram_row_done              ),
    .read_col                       (read_col                        ),
    .read_bin                       (read_bin                        ),
    .read_count                     (read_count                      ),
    .clear_start                    (clear_start                     ),
    .init_done                      (histogram_ready                 )
);

clahe_clipper#(
    .TILE_PIXELS                    (TILE_PIXELS                     ),
    .DONOR_MODE                     (DONOR_MODE                      )
)u_clipper(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .clear                          (stats_flush                     ),
    .hist_data                      (read_count                      ),
    .rd_bin                         (read_bin                        ),
    .hist_row_done                  (histogram_row_done              ),
    .clip_limit                     (RUNTIME_CONTROL ? cfg_data[111 : 96] : CLIP_LIMIT[15 : 0]),
    .tile_row_in                    (tile_row                        ),
    .rd_col                         (read_col                        ),
    .cdf_wr_en                      (cdf_write_enable                ),
    .cdf_wr_tile                    (cdf_write_tile                  ),
    .cdf_wr_bin                     (cdf_write_bin                   ),
    .cdf_wr_data                    (cdf_write_data                  ),
    .hist_clear_start               (clear_start                     ),
    .frame_cdf_done                 (frame_cdf_done                  )
);

clahe_tiles_registered#(
    .PIC_WIDTH                      (PIC_WIDTH                       ),
    .PIC_HEIGHT                     (PIC_HEIGHT                      ),
    .DONOR_MODE                     (DONOR_MODE                      )
)u_tiles(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pixel_x                        (pixel_x                         ),
    .pixel_y                        (pixel_y                         ),
    .tile_ul                        (tile_ul                         ),
    .tile_ur                        (tile_ur                         ),
    .tile_ll                        (tile_ll                         ),
    .tile_lr                        (tile_lr                         ),
    .weight_x                       (weight_x                        ),
    .weight_y                       (weight_y                        )
);

clahe_cdf_store u_cdf_store(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .clear                          (stats_flush                     ),
    .stall                          (1'b0                            ),
    .frame_cdf_done                 (frame_cdf_done                  ),
    .wr_en                          (cdf_write_enable                ),
    .wr_tile                        (cdf_write_tile                  ),
    .wr_bin                         (cdf_write_bin                   ),
    .wr_data                        (cdf_write_data                  ),
    .tile_ul                        (tile_ul                         ),
    .tile_ur                        (tile_ur                         ),
    .tile_ll                        (tile_ll                         ),
    .tile_lr                        (tile_lr                         ),
    .pixel_y                        (cdf_sample_y                    ),
    .cdf_ul                         (cdf_ul                          ),
    .cdf_ur                         (cdf_ur                          ),
    .cdf_ll                         (cdf_ll                          ),
    .cdf_lr                         (cdf_lr                          )
);

// Stage 0: weights and the synchronous CDF lookup refer to the same pixel.
reg     [8 : 0]         wx_r                             ,
                        wy_r                             ;
reg     [8 : 0]         wx_aligned                       ,
                        wy_aligned                       ;
reg     [7 : 0]         cdf_ul_r                         ,
                        cdf_ur_r                         ,
                        cdf_ll_r                         ,
                        cdf_lr_r                         ;
// Preserve a fabric register before the horizontal DSP inputs. Two clocks
// separate the synchronous CDF RAM output from the multiply/post-add input.
reg     [7 : 0]         horizontal_left_top              /* synthesis syn_preserve = 1 */;
reg     [7 : 0]         horizontal_left_bottom           /* synthesis syn_preserve = 1 */;
reg signed [8 : 0]         horizontal_difference_top       /* synthesis syn_preserve = 1 */;
reg signed [8 : 0]         horizontal_difference_bottom    /* synthesis syn_preserve = 1 */;
reg     [8 : 0]         wx_horizontal                    ,
                        wy_prepared                      ;
reg     [8 : 0]         wy_vertical                      ;
reg     [16 : 0]        top_vertical                     ;
reg     [8 : 0]         wy_horizontal                    ;
reg     [16 : 0]        horizontal_top                   ,
                        horizontal_bottom                ;
reg     [24 : 0]        top_scaled                       ;
reg signed [27 : 0]        vertical_product                 ;
reg signed [17 : 0]        vertical_difference              ;
wire signed [28 : 0]        sum                              ;
reg     [7 : 0]         mapped_y                         ;
reg     [7 : 0]         original_y[0 : 7]                ;
reg     [7 : 0]         magnitude_original               ;
reg     [7 : 0]         blend_magnitude_r                /* synthesis syn_preserve = 1 */;
reg     [6 : 0]         blend_alpha_r                    /* synthesis syn_preserve = 1 */;
reg     [6 : 0]         magnitude_distance               ;
reg                     magnitude_negative               ;
reg     [7 : 0]         blend_original                   ;
reg     [6 : 0]         blend_distance                   ;
reg     [14 : 0]        blend_product                    ;
reg                     blend_negative                   ;
wire    [7 : 0]         blend_magnitude                  ;
wire    [6 : 0]         current_alpha                    ;
reg     [6 : 0]         regular_alpha_r                  ,
                        positive_alpha_r                 ;
reg     [7 : 0]         rounded_source                   ;
reg     [8 : 0]         rounded_amount                   ;
reg                     rounded_negative                 ;
reg     [6 : 0]         rounded_distance                 ;
reg     [7 : 0]         limited_source                   ;
reg     [8 : 0]         limited_amount                   ;
reg                     limited_negative                 ;
wire    [15 : 0]        rounded_product                  ;
reg     [7 : 0]         blended_y                        ;
reg     [12 : 0]        vs_pipe                          ,
                        href_pipe                        ,
                        de_pipe                          ;
wire    [6 : 0]         original_distance                ;
// Factor the exact four-corner sum into three multiplications. Keep every
// fractional bit until the final rounding, so the previous map is unchanged.

assign  sum                              =   $signed({4'b0, top_scaled}) + $signed({vertical_product[27],
        vertical_product}) + (DONOR_MODE ? 29'sd0 : 29'sd32768);

assign  original_distance                =   original_y[7] < 128 ? original_y[7] : 255 - original_y[7];

function [16 : 0] horizontal_sum;
    input   [7 : 0]         left_y                           ;
    input signed [8 : 0]      difference                       ;
    input   [8 : 0]         weight                           ;
    reg signed [18 : 0]        product                          ;
    reg signed [19 : 0]        result                           ;
    begin
        product                          =  difference * $signed({1'b0, weight});
        result                           =  $signed({4'b0, left_y, 8'b0}) + product;
        horizontal_sum                   =  result[16 : 0];
    end
endfunction

// Resolve the runtime gain and shadow ramp one clock before the multiplier.
// These functions retain the original alpha for every source and setting.
function [6 : 0] regular_alpha_for;
    input   [7 : 0]         source_y                         ;
    input   [2 : 0]         strength                         ;
    input                   anti_halo                        ;
    reg     [6 : 0]         distance                         ,
                            strength_limit                   ;
    begin
        distance                         =  source_y < 128 ? source_y : 255 - source_y;
        strength_limit                   =  {strength, 4'b0};
        regular_alpha_for                =  anti_halo && distance < strength_limit ?
                distance : strength_limit;
    end
endfunction

function [6 : 0] positive_alpha_for;
    input   [7 : 0]         source_y                         ;
    input   [6 : 0]         regular                          ;
    input   [2 : 0]         strength                         ;
    input   [6 : 0]         shadow_gain                      ;
    input                   anti_halo                        ,
                            shadow                           ;
    reg     [9 : 0]         shadow_ramp                      ;
    reg     [6 : 0]         shadow_alpha                     ,
                            strength_limit                   ;
    begin
        shadow_ramp                      =  {source_y, 2'b0};
        shadow_alpha                     =  shadow_ramp < {3'b0, shadow_gain} ?
                shadow_ramp[6 : 0] : shadow_gain;
        strength_limit                   =  {strength, 4'b0};
        positive_alpha_for               =  regular;
        if(shadow && anti_halo && source_y < 128 && shadow_alpha > regular)
            positive_alpha_for               =  shadow_alpha < strength_limit ? shadow_alpha : strength_limit;
    end
endfunction

assign  blend_magnitude                  =   mapped_y < original_y[7] ?
        original_y[7] - mapped_y : mapped_y - original_y[7];

assign  current_alpha                    =   mapped_y > original_y[7] ? positive_alpha_r : regular_alpha_r;
// The multiply uses the precomputed alpha. Signed rounding, limiting and
// saturation remain on independent clocks with matching pixel/config data.

function [7 : 0] finish_blend;
    input   [7 : 0]         source_y                         ;
    input   [14 : 0]        magnitude_product                ;
    input                   negative                         ;
    input   [6 : 0]         distance                         ;
    reg     [15 : 0]        rounded_product                  ;
    reg     [8 : 0]         amount                           ;
    begin
        rounded_product                  =  {1'b0, magnitude_product} + (DONOR_MODE && negative ? 16'd31 : 16'd32);
        amount                           =  rounded_product[14 : 6];
        if(finish_anti_halo)begin
            if(finish_shadow && !negative && source_y < 128)begin
                if(amount > boost_value)
                    amount                           =  boost_value;
            end
            else if(amount > {2'b0, distance})
                amount                           =  {2'b0, distance};
        end
        if(negative)
            finish_blend                     =  amount > {1'b0, source_y} ? 8'd0 : source_y - amount;
        else
            finish_blend                     =  amount >(9'd255 - {1'b0, source_y}) ? 8'd255 : source_y + amount;
    end
endfunction

// Keep rounding, gain limiting and output saturation on separate clocks.
assign  rounded_product                  =   {1'b0, blend_product} +
                                             (DONOR_MODE && blend_negative ? 16'd31 : 16'd32);

function [8 : 0] limit_blend_amount;
    input   [7 : 0]         source_y                         ;
    input   [8 : 0]         amount                           ;
    input                   negative                         ;
    input   [6 : 0]         distance                         ;
    begin
        limit_blend_amount               =  amount;
        if(finish_anti_halo)begin
            if(finish_shadow && !negative && source_y < 128)begin
                if(amount > {1'b0, boost_value})
                    limit_blend_amount               =  {1'b0, boost_value};
            end
            else if(amount > {2'b0, distance})
                limit_blend_amount               =  {2'b0, distance};
        end
    end
endfunction

function [7 : 0] saturate_blend;
    input   [7 : 0]         source_y                         ;
    input   [8 : 0]         amount                           ;
    input                   negative                         ;
    reg     [9 : 0]         result                           ;
    begin
        if(negative)begin
            result                           =  {2'b0, source_y} - {1'b0, amount};
            saturate_blend                   =  result[9] ? 8'd0 : result[7 : 0];
        end
        else begin
            result                           =  {2'b0, source_y} + {1'b0, amount};
            saturate_blend                   =  |result[9 : 8] ? 8'd255 : result[7 : 0];
        end
    end
endfunction

// Combinational counterpart used by the exhaustive blend reference test.

function [7 : 0] blend_y;
    input   [7 : 0]         source_y                         ,
                            target_y                         ;
    input   [6 : 0]         alpha                            ,
                            distance                         ;
    reg     [7 : 0]         magnitude                        ;
    reg     [14 : 0]        magnitude_product                ;
    reg                     negative                         ;
    begin
        negative                         =  target_y < source_y;
        magnitude                        =  negative ? source_y - target_y : target_y - source_y;
        magnitude_product                =  magnitude * alpha;
        blend_y                          =  finish_blend(source_y, magnitude_product, negative, distance);
    end
endfunction

always@(posedge clk or negedge rst_n)begin : pixel_pipeline
    integer                 i                                ;
    if(!rst_n)begin
        wx_r                             <=  0;
        wy_r                             <=  0;
        wx_aligned                       <=  0;
        wy_aligned                       <=  0;
        cdf_ul_r                         <=  0;
        cdf_ur_r                         <=  0;
        cdf_ll_r                         <=  0;
        cdf_lr_r                         <=  0;
        cdf_sample_y                     <=  0;
        horizontal_left_top              <=  0;
        horizontal_left_bottom           <=  0;
        horizontal_difference_top        <=  0;
        horizontal_difference_bottom     <=  0;
        wx_horizontal                    <=  0;
        wy_prepared                      <=  0;
        regular_alpha_r                  <=  0;
        positive_alpha_r                 <=  0;
        wy_vertical                      <=  0;
        top_vertical                     <=  0;
        vertical_difference              <=  0;
        wy_horizontal                    <=  0;
        horizontal_top                   <=  0;
        horizontal_bottom                <=  0;
        top_scaled                       <=  0;
        vertical_product                 <=  0;
        mapped_y                         <=  0;
        magnitude_original               <=  0;
        blend_magnitude_r                <=  0;
        blend_alpha_r                    <=  0;
        magnitude_distance               <=  0;
        magnitude_negative               <=  0;
        blend_original                   <=  0;
        blend_product                    <=  0;
        blend_negative                   <=  0;
        blend_distance                   <=  0;
        rounded_source                   <=  0;
        rounded_amount                   <=  0;
        rounded_negative                 <=  0;
        rounded_distance                 <=  0;
        limited_source                   <=  0;
        limited_amount                   <=  0;
        limited_negative                 <=  0;
        blended_y                        <=  0;
        vs_pipe                          <=  0;
        href_pipe                        <=  0;
        de_pipe                          <=  0;
        for(i = 0; i < 8; i = i + 1)
            original_y[i]                    <=  0;
    end
    else begin
        cdf_sample_y                     <=  pre_y;
        wx_r                             <=  weight_x;
        wy_r                             <=  weight_y;
        // Stage 2: isolate the RAM output/bank selection from the DSP input.
        wx_aligned                       <=  wx_r;
        wy_aligned                       <=  wy_r;
        cdf_ul_r                         <=  cdf_ul;
        cdf_ur_r                         <=  cdf_ur;
        cdf_ll_r                         <=  cdf_ll;
        cdf_lr_r                         <=  cdf_lr;
        // Stage 3: prepare the horizontal difference, left value and weight.
        horizontal_left_top              <=  cdf_ul_r;
        horizontal_left_bottom           <=  cdf_ll_r;
        horizontal_difference_top        <=  $signed({1'b0, cdf_ur_r}) - $signed({1'b0, cdf_ul_r});
        horizontal_difference_bottom     <=  $signed({1'b0, cdf_lr_r}) - $signed({1'b0, cdf_ll_r});
        wx_horizontal                    <=  wx_aligned;
        wy_prepared                      <=  wy_aligned;
        // Stage 4: horizontal interpolation, exact Q8 values.
        wy_horizontal                    <=  wy_prepared;
        horizontal_top                   <=  horizontal_sum(horizontal_left_top, horizontal_difference_top,
                wx_horizontal);
        horizontal_bottom                <=  horizontal_sum(horizontal_left_bottom, horizontal_difference_bottom,
                wx_horizontal);
        // Stage 5: register the vertical difference and matching top/weight.
        // Stage 6: multiply the registered difference, with matching top.
        vertical_difference              <=  $signed({1'b0, horizontal_bottom}) -
                $signed({1'b0, horizontal_top});
        top_vertical                     <=  horizontal_top;
        wy_vertical                      <=  wy_horizontal;
        top_scaled                       <=  {top_vertical, 8'b0};
        vertical_product                 <=  vertical_difference * $signed({1'b0, wy_vertical});
        mapped_y                         <=  sum[23 : 16];
        original_y[0]                    <=  pre_y;
        for(i = 1; i < 8; i = i + 1)
            original_y[i]                    <=  original_y[i - 1];
        // Stage 7: the original source and runtime config precede blend by one clock.
        regular_alpha_r                  <=  regular_alpha_for(original_y[6], strength_value, anti_halo_value);
        positive_alpha_r                 <=  positive_alpha_for(original_y[6],
                regular_alpha_for(original_y[6], strength_value, anti_halo_value),
                strength_value, shadow_gain_value, anti_halo_value, shadow_value);
        // Stage 8: isolate magnitude/sign selection from the blend DSP input.
        magnitude_original               <=  original_y[7];
        blend_magnitude_r                <=  blend_magnitude;
        blend_alpha_r                    <=  current_alpha;
        magnitude_distance               <=  original_distance;
        magnitude_negative               <=  mapped_y < original_y[7];
        // Stage 9: multiply only registered magnitude and alpha values.
        blend_original                   <=  magnitude_original;
        blend_distance                   <=  magnitude_distance;
        blend_product                    <=  blend_magnitude_r * blend_alpha_r;
        blend_negative                   <=  magnitude_negative;
        // Stage 10: exact signed rounding, with the original donor convention.
        rounded_source                   <=  blend_original;
        rounded_amount                   <=  rounded_product[14 : 6];
        rounded_negative                 <=  blend_negative;
        rounded_distance                 <=  blend_distance;
        // Stage 11: anti-halo and shadow gain limit.
        limited_source                   <=  rounded_source;
        limited_amount                   <=  limit_blend_amount(rounded_source, rounded_amount,
                rounded_negative, rounded_distance);
        limited_negative                 <=  rounded_negative;
        // Stage 12: add/subtract and saturate to the eight-bit output.
        blended_y                        <=  saturate_blend(limited_source, limited_amount, limited_negative);
        vs_pipe                          <=  {vs_pipe[11 : 0], pre_frame_vsync};
        href_pipe                        <=  {href_pipe[11 : 0], pre_frame_href};
        de_pipe                          <=  {de_pipe[11 : 0], pre_frame_clken};
    end
end

assign  post_frame_vsync                 =   vs_pipe[12];
assign  post_frame_href                  =   href_pipe[12];
assign  post_frame_clken                 =   de_pipe[12];
assign  post_y                           =   de_pipe[12] ? blended_y : 8'd0;
`ifndef SYNTHESIS

initial begin
    if(PIC_WIDTH < 16 || PIC_HEIGHT < 16 || PIC_WIDTH % 8 != 0 || PIC_HEIGHT % 8 != 0 || TILE_PIXELS > 65535 ||
            CLIP_LIMIT < 1 || CLIP_LIMIT > 65535 || STRENGTH < 0 || STRENGTH > 4 ||(SHADOW_ENABLE != 0 &&
            SHADOW_ENABLE != 1) || SHADOW_GAIN < 0 || SHADOW_GAIN > 64 || BOOST_LIMIT < 0 || BOOST_LIMIT > 255)
        $error("Invalid CLAHE dimensions, clip limit or strength");
end

`endif

endmodule
