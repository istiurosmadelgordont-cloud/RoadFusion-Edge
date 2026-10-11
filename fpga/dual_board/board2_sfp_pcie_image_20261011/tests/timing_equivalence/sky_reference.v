// Soft protection for bright, pale regions with a gentle horizontal-gradient penalty.
// This is a streaming heuristic, not semantic sky segmentation.
// The mask follows the source pixel by six clocks, matching time_alignment.
module sky_reference#(
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
    .DELAY_CLOCKS                   (1                               )
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
wire    [15 : 0]        y_sum                            =   red * 16'd77 + green * 16'd150 + blue * 16'd29;
wire    [7 : 0]         y_value                          =   y_sum[15 : 8];
wire    [7 : 0]         rg_max                           =   red > green ? red : green;
wire    [7 : 0]         rg_min                           =   red < green ? red : green;
wire    [7 : 0]         rgb_max                          =   rg_max > blue ? rg_max : blue;
wire    [7 : 0]         rgb_min                          =   rg_min < blue ? rg_min : blue;
wire                    pixel_valid                      =   pre_frame_vsync && pre_frame_href && pre_frame_clken;
reg     [7 : 0]         left_y                           ;
reg                     left_valid                       ;
reg     [7 : 0]         y_s1                             ;
reg     [7 : 0]         chroma_s1                        ;
reg     [7 : 0]         gradient_s1                      ;
reg                     valid_s1                         ;
// Stage 1. Blanking clears the left neighbour, so rows never join each other.

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        left_y                           <=  8'd0;
        left_valid                       <=  1'b0;
        y_s1                             <=  8'd0;
        chroma_s1                        <=  8'd0;
        gradient_s1                      <=  8'd0;
        valid_s1                         <=  1'b0;
    end
    else begin
        valid_s1                         <=  pixel_valid;
        if(pixel_valid)begin
            y_s1                             <=  y_value;
            chroma_s1                        <=  rgb_max - rgb_min;
            if(!left_valid)
                gradient_s1                      <=  8'd0;
            else if(y_value >= left_y)
                gradient_s1                      <=  y_value - left_y;
            else
                gradient_s1                      <=  left_y - y_value;
            left_y                           <=  y_value;
            left_valid                       <=  1'b1;
        end
        else begin
            y_s1                             <=  8'd0;
            chroma_s1                        <=  8'd0;
            gradient_s1                      <=  8'd0;
            left_valid                       <=  1'b0;
        end
    end
end

reg     [8 : 0]         brightness_weight                ;
reg     [8 : 0]         chroma_weight                    ;
reg     [8 : 0]         flatness_weight                  ;
// All weights range from 0 to 256. Power-of-two ramps need no division.

always@(*)begin
    if(y_s1 <= 8'd160)
        brightness_weight                =  9'd0;
    else if(y_s1 >= 8'd224)
        brightness_weight                =  9'd256;
    else
        brightness_weight                =  (9'd0 + y_s1 - 9'd160) << 2;
    if(chroma_s1 <= 8'd16)
        chroma_weight                    =  9'd256;
    else if(chroma_s1 >= 8'd80)
        chroma_weight                    =  9'd0;
    else
        chroma_weight                    =  (9'd80 - chroma_s1) << 2;
    // A one-code gradient change now changes this weight by at most four.
    // Strong edges retain half protection instead of cancelling it abruptly.
    if(gradient_s1 <= 8'd8)
        flatness_weight                  =  9'd256;
    else if(gradient_s1 >= 8'd40)
        flatness_weight                  =  9'd128;
    else
        flatness_weight                  =  9'd256 - ((gradient_s1 - 9'd8) << 2);
end

reg     [8 : 0]         brightness_s2                    ;
reg     [8 : 0]         chroma_s2                        ;
reg     [8 : 0]         flatness_s2                      ;
reg                     valid_s2                         ;
reg     [7 : 0]         base_s2                          ;
reg     [7 : 0]         step_s2                          ;
reg     [8 : 0]         weight_s2                        ;
reg     [8 : 0]         weight_s3                        ;
reg     [7 : 0]         base_s3                          ;
reg     [7 : 0]         step_s3                          ;
reg     [16 : 0]        product_s4                       ;
reg     [8 : 0]         weight_s4                        ;
reg     [7 : 0]         base_s4                          ;
reg     [16 : 0]        rounded_s5                       ;
reg     [8 : 0]         weight_s5                        ;
reg     [7 : 0]         base_s5                          ;
reg     [7 : 0]         floor_s6                         ;
reg     [8 : 0]         weight_s6                        ;

always@(*)begin
    weight_s2                       =   brightness_s2;
    if(chroma_s2 < weight_s2)
        weight_s2                       =   chroma_s2;
    if(flatness_s2 < weight_s2)
        weight_s2                       =   flatness_s2;
    if(!valid_s2)
        weight_s2                       =   9'd0;
end

// Separate ramps, minimum, multiplication, rounding and floor addition.
// BASE/STEP are sampled with stage-1 features, preserving runtime alignment.
// The total remains six clocks, so legacy and DCP alignment stay compatible.

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        brightness_s2                   <=  0;
        chroma_s2                       <=  0;
        flatness_s2                     <=  0;
        valid_s2                        <=  0;
        base_s2                         <=  TX_MIN;
        step_s2                         <=  0;
        weight_s3                       <=  0;
        base_s3                         <=  TX_MIN;
        step_s3                         <=  0;
        product_s4                      <=  0;
        weight_s4                       <=  0;
        base_s4                         <=  TX_MIN;
        rounded_s5                      <=  17'd128;
        weight_s5                       <=  0;
        base_s5                         <=  TX_MIN;
        floor_s6                        <=  TX_MIN;
        weight_s6                       <=  0;
    end
    else begin
        // Stage 2: three independent ramps and their paired frame parameters.
        brightness_s2                   <=  brightness_weight;
        chroma_s2                       <=  chroma_weight;
        flatness_s2                     <=  flatness_weight;
        valid_s2                        <=  valid_s1;
        base_s2                         <=  BASE_FLOOR;
        step_s2                         <=  FLOOR_STEP;
        // Stage 3: minimum weight, with invalid pixels masked to zero.
        weight_s3                       <=  weight_s2;
        base_s3                         <=  base_s2;
        step_s3                         <=  step_s2;
        // Stage 4: interpolation product only.
        product_s4                      <=  step_s3 * weight_s3;
        weight_s4                       <=  weight_s3;
        base_s4                         <=  base_s3;
        // Stage 5: round once before division by 256.
        rounded_s5                      <=  product_s4 + 17'd128;
        weight_s5                       <=  weight_s4;
        base_s5                         <=  base_s4;
        // Stage 6: add the base floor, with no multiplier on this path.
        floor_s6                        <=  base_s5 + rounded_s5[16 : 8];
        weight_s6                       <=  weight_s5;
    end
end

assign  post_tx_floor                    =   floor_s6;
assign  post_weight                      =   weight_s6;
assign  post_tx_img                      =   aligned_tx_img > post_tx_floor ? aligned_tx_img : post_tx_floor;

endmodule
