// Reference-based highlight protection and a soft knee.
// Seven clocks, one pixel per clock; the original integer mapping is unchanged.
module highlight_luma#(
    parameter   RUNTIME_CONTROL                  =   0,
    parameter   KNEE                             =   192,
    parameter   SLOPE                            =   128
)(
    input   [159 : 0]       cfg_data                         ,
    input                   clk                              ,
    input                   rst_n                            ,
    input   [7 : 0]         original_y                       ,
    input   [7 : 0]         enhanced_y                       ,
    output reg [7 : 0]         post_y
);

wire    [7 : 0]         knee_value                       ;
wire    [8 : 0]         slope_value                      ;
wire    [8 : 0]         input_weight                     ;
reg     [8 : 0]         weight_s1                        ;
reg signed [8 : 0]         difference_s1                    ;
reg     [7 : 0]         processed_s1                     ;
reg signed [18 : 0]        product_s2                       ;
reg     [15 : 0]        processed_scaled_s2              ;
reg signed [18 : 0]        sum_s3                           ;
wire signed [19 : 0]       rounded_sum                      ;
reg     [7 : 0]         blended_s4                       ;
reg     [7 : 0]         blended_s5                       ;
reg     [7 : 0]         blended_s6                       ;
reg     [7 : 0]         difference_s5                    ;
reg                     compress_s5                      ;
reg                     compress_s6                      ;
reg     [17 : 0]        knee_product_s6                  ;
reg     [7 : 0]         knee_pipe[0 : 5]                 ;
reg     [8 : 0]         slope_pipe[0 : 4]                ;
wire    [10 : 0]        compressed                       ;
integer                 i                                ;

assign  knee_value                      =   RUNTIME_CONTROL ? cfg_data[143 : 136] : KNEE;
assign  slope_value                     =   RUNTIME_CONTROL ? cfg_data[152 : 144] : SLOPE;
assign  input_weight                    =   original_y <= 160 ? 9'd0 : original_y >= 224 ?
                                            9'd256 : ({1'b0, original_y} - 9'd160) << 2;
assign  rounded_sum                     =   $signed({sum_s3[18], sum_s3}) + 20'sd128;
// Extract the rounding bit before the final adder; no 18-bit carry chain is
// followed by a second full-width adder in the last stage.
assign  compressed                      =   {3'd0, knee_pipe[5]} + {1'b0, knee_product_s6[17 : 8]} +
                                            {10'd0, knee_product_s6[7]};

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        weight_s1                       <=  0;
        difference_s1                   <=  0;
        processed_s1                    <=  0;
        product_s2                      <=  0;
        processed_scaled_s2             <=  0;
        sum_s3                          <=  0;
        blended_s4                      <=  0;
        blended_s5                      <=  0;
        blended_s6                      <=  0;
        difference_s5                   <=  0;
        compress_s5                     <=  0;
        compress_s6                     <=  0;
        knee_product_s6                 <=  0;
        post_y                          <=  0;
        for(i = 0; i < 6; i = i + 1)
            knee_pipe[i]                    <=  0;
        for(i = 0; i < 5; i = i + 1)
            slope_pipe[i]                   <=  0;
    end
    else begin
        // Stage 1: source-dependent weight and signed blend difference.
        weight_s1                       <=  input_weight;
        difference_s1                   <=  $signed({1'b0, original_y}) - $signed({1'b0, enhanced_y});
        processed_s1                    <=  enhanced_y;
        knee_pipe[0]                    <=  knee_value;
        slope_pipe[0]                   <=  slope_value;
        for(i = 1; i < 6; i = i + 1)
            knee_pipe[i]                    <=  knee_pipe[i - 1];
        for(i = 1; i < 5; i = i + 1)
            slope_pipe[i]                   <=  slope_pipe[i - 1];
        // Stage 2: processed*256 + (source-processed)*weight is identical
        // to the original two-product weighted sum, with one multiplier.
        product_s2                      <=  difference_s1 * $signed({1'b0, weight_s1});
        processed_scaled_s2             <=  {processed_s1, 8'd0};
        // Stages 3/4: blend sum, then Q8 rounding.
        sum_s3                          <=  $signed({3'd0, processed_scaled_s2}) + product_s2;
        blended_s4                      <=  rounded_sum[15 : 8];
        // Stage 5: soft-knee difference and a matching identity sample.
        compress_s5                     <=  blended_s4 > knee_pipe[3];
        difference_s5                   <=  blended_s4 > knee_pipe[3] ? blended_s4 - knee_pipe[3] : 8'd0;
        blended_s5                      <=  blended_s4;
        // Stage 6: soft-knee product.
        knee_product_s6                 <=  difference_s5 * slope_pipe[4];
        compress_s6                     <=  compress_s5;
        blended_s6                      <=  blended_s5;
        // Stage 7: rounded compressed output and saturation.
        if(compress_s6)
            post_y                          <=  |compressed[10 : 8] ? 8'd255 : compressed[7 : 0];
        else
            post_y                          <=  blended_s6;
    end
end

`ifndef SYNTHESIS
initial begin
    if(KNEE < 0 || KNEE > 255 || SLOPE < 0 || SLOPE > 256)
        $error("Invalid highlight knee/slope");
end
`endif

endmodule
