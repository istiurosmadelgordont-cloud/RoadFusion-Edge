// Soft protection for bright, pale regions with a gentle horizontal-gradient penalty.
// This is a streaming heuristic, not semantic sky segmentation.
// The mask follows the source pixel by six clocks, matching time_alignment.
module sky_protect#(
    parameter   RUNTIME_CONTROL                  =   0,
    parameter   TX_MIN                           =   52,
    parameter   SKY_TX_MIN                       =   128
)(
    input   [159 : 0]       cfg_data                         ,
    input                   clk                              ,
    input                   rst_n                            ,
    input                   pre_frame_vsync                  ,
    input                   pre_frame_href                   ,
    input                   pre_frame_clken                  ,
    input   [23 : 0]        pre_img                          ,
    input   [7 : 0]         aligned_tx_img                   ,
    output  [7 : 0]         post_tx_img                      ,
    output  [7 : 0]         post_tx_floor                    ,
    output  [8 : 0]         post_weight
);

wire    [159 : 0]       floor_config                     ;
wire    [7 : 0]         BASE_FLOOR                       ;
wire    [7 : 0]         SKY_FLOOR                        ;
wire    [7 : 0]         FLOOR_STEP                       ;
frame_config_delay#(
    .DATA_WIDTH                     (160                             ),
    .DELAY_CLOCKS                   (2                               )
)u_floor_config(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (pre_frame_vsync                 ),
    .pre_config                     (RUNTIME_CONTROL ? cfg_data : 160'd0),
    .post_config                    (floor_config                    )
);

assign  BASE_FLOOR                       =   RUNTIME_CONTROL ? floor_config[39 : 32] : TX_MIN;
assign  SKY_FLOOR                        =   RUNTIME_CONTROL ?
                                             (floor_config[6] ?
        floor_config[47 : 40] : BASE_FLOOR) : (SKY_TX_MIN > TX_MIN ? SKY_TX_MIN : TX_MIN);

assign  FLOOR_STEP                       =   SKY_FLOOR > BASE_FLOOR ? SKY_FLOOR - BASE_FLOOR : 8'd0;
wire    [7 : 0]         red                              =   pre_img[23 : 16];
wire    [7 : 0]         green                            =   pre_img[15 : 8];
wire    [7 : 0]         blue                             =   pre_img[7 : 0];
wire    [7 : 0]         rg_max                           =   red > green ? red : green;
wire    [7 : 0]         rg_min                           =   red < green ? red : green;
wire    [7 : 0]         rgb_max                          =   rg_max > blue ? rg_max : blue;
wire    [7 : 0]         rgb_min                          =   rg_min < blue ? rg_min : blue;
wire                    pixel_valid                      =   pre_frame_vsync && pre_frame_href && pre_frame_clken;
reg     [15 : 0]        weighted_red_s1                  ;
reg     [15 : 0]        weighted_green_s1                ;
reg     [15 : 0]        weighted_blue_s1                 ;
reg     [7 : 0]         color_difference_s1              ;
reg                     product_valid_s1                 ;
wire    [15 : 0]        y_partial                        ;
wire    [15 : 0]        y_carry                          ;
wire    [15 : 0]        y_sum                            ;
wire    [7 : 0]         y_value                          =   y_sum[15 : 8];
reg     [7 : 0]         left_y                           ;
reg                     left_valid                       ;
// Retain these diagnostic feature names; features now occupy stage 2.
reg     [7 : 0]         y_s1                             ;
reg     [7 : 0]         chroma_s1                        ;
reg     [7 : 0]         gradient_s1                      ;
reg                     valid_s1                         ;

// Compress three products before the final luminance adder. The exact
// weighted sum stays below 65536; its Q8 luminance is unchanged.
assign  y_partial                       =   weighted_red_s1 ^ weighted_green_s1 ^ weighted_blue_s1;
assign  y_carry                         =   ((weighted_red_s1 & weighted_green_s1) |
                                             (weighted_red_s1 & weighted_blue_s1) |
                                             (weighted_green_s1 & weighted_blue_s1)) << 1;
assign  y_sum                           =   y_partial + y_carry;

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        weighted_red_s1                 <=  0;
        weighted_green_s1               <=  0;
        weighted_blue_s1                <=  0;
        color_difference_s1             <=  0;
        product_valid_s1                <=  0;
        left_y                          <=  0;
        left_valid                      <=  0;
        y_s1                            <=  0;
        chroma_s1                       <=  0;
        gradient_s1                     <=  0;
        valid_s1                        <=  0;
    end
    else begin
        // Stage 1: independent products. No gradient follows a multiplier.
        weighted_red_s1                 <=  red * 16'd77;
        weighted_green_s1               <=  green * 16'd150;
        weighted_blue_s1                <=  blue * 16'd29;
        color_difference_s1             <=  rgb_max - rgb_min;
        product_valid_s1                <=  pixel_valid;
        // Stage 2: luminance, colour difference and left-neighbour gradient.
        valid_s1                        <=  product_valid_s1;
        if(product_valid_s1)begin
            y_s1                        <=  y_value;
            chroma_s1                   <=  color_difference_s1;
            if(!left_valid)
                gradient_s1             <=  0;
            else if(y_value >= left_y)
                gradient_s1             <=  y_value - left_y;
            else
                gradient_s1             <=  left_y - y_value;
            left_y                      <=  y_value;
            left_valid                  <=  1;
        end
        else begin
            y_s1                        <=  0;
            chroma_s1                   <=  0;
            gradient_s1                 <=  0;
            left_valid                  <=  0;
        end
    end
end

reg     [8 : 0]         brightness_weight                ;
reg     [8 : 0]         chroma_weight                    ;
reg     [8 : 0]         flatness_weight                  ;

always@(*)begin
    if(y_s1 <= 8'd160)
        brightness_weight               =   0;
    else if(y_s1 >= 8'd224)
        brightness_weight               =   256;
    else
        brightness_weight               =   (9'd0 + y_s1 - 9'd160) << 2;
    if(chroma_s1 <= 8'd16)
        chroma_weight                   =   256;
    else if(chroma_s1 >= 8'd80)
        chroma_weight                   =   0;
    else
        chroma_weight                   =   (9'd80 - chroma_s1) << 2;
    if(gradient_s1 <= 8'd8)
        flatness_weight                 =   256;
    else if(gradient_s1 >= 8'd40)
        flatness_weight                 =   128;
    else
        flatness_weight                 =   9'd256 - ((gradient_s1 - 9'd8) << 2);
end

reg     [8 : 0]         brightness_s3                    ;
reg     [8 : 0]         chroma_s3                        ;
reg     [8 : 0]         flatness_s3                      ;
reg                     valid_s3                         ;
reg     [7 : 0]         base_s3                          ;
reg     [7 : 0]         step_s3                          ;
reg     [8 : 0]         selected_weight                  ;
reg     [8 : 0]         weight_s4                        ;
reg     [7 : 0]         base_s4                          ;
reg     [7 : 0]         step_s4                          ;
reg     [16 : 0]        product_s5                       ;
reg     [8 : 0]         weight_s5                        ;
reg     [7 : 0]         base_s5                          ;
reg     [7 : 0]         floor_s6                         ;
reg     [8 : 0]         weight_s6                        ;
wire    [8 : 0]         rounded_upper                    =   product_s5[16 : 8] + {8'd0, product_s5[7]};

always@(*)begin
    selected_weight                     =   brightness_s3;
    if(chroma_s3 < selected_weight)
        selected_weight                 =   chroma_s3;
    if(flatness_s3 < selected_weight)
        selected_weight                 =   flatness_s3;
    if(!valid_s3)
        selected_weight                 =   0;
end

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        brightness_s3                   <=  0;
        chroma_s3                       <=  0;
        flatness_s3                     <=  0;
        valid_s3                        <=  0;
        base_s3                         <=  TX_MIN;
        step_s3                         <=  0;
        weight_s4                       <=  0;
        base_s4                         <=  TX_MIN;
        step_s4                         <=  0;
        product_s5                      <=  0;
        weight_s5                       <=  0;
        base_s5                         <=  TX_MIN;
        floor_s6                        <=  TX_MIN;
        weight_s6                       <=  0;
    end
    else begin
        // Stage 3: three ramps and paired parameters (two-clock cfg delay).
        brightness_s3                   <=  brightness_weight;
        chroma_s3                       <=  chroma_weight;
        flatness_s3                     <=  flatness_weight;
        valid_s3                        <=  valid_s1;
        base_s3                         <=  BASE_FLOOR;
        step_s3                         <=  FLOOR_STEP;
        // Stage 4: minimum; stage 5: interpolation product.
        weight_s4                       <=  selected_weight;
        base_s4                         <=  base_s3;
        step_s4                         <=  step_s3;
        product_s5                      <=  step_s4 * weight_s4;
        weight_s5                       <=  weight_s4;
        base_s5                         <=  base_s4;
        // Stage 6: exact round(P/256) = P[16:8] + P[7], then base.
        floor_s6                        <=  base_s5 + rounded_upper;
        weight_s6                       <=  weight_s5;
    end
end

assign  post_tx_floor                    =   floor_s6;
assign  post_weight                      =   weight_s6;
assign  post_tx_img                      =   aligned_tx_img > post_tx_floor ? aligned_tx_img : post_tx_floor;

endmodule
