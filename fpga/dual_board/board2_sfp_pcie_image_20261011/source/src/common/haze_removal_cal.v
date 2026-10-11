// Exact ((I - A) * 256 + A * t) / t, saturated to RGB888.
// Input register, two numerator stages and nine divider clocks: twelve total.
module haze_removal_cal#(
    parameter   tx_min                           =   8'd26
)(
    input                   clk                              ,
    input                   rst_n                            ,
    //处理前数据
    input                   pre_src_frame_vsync              ,
    input                   pre_src_frame_href               ,
    input                   pre_src_frame_clken              ,
    input   [23 : 0]        pre_img                          ,
    input                   pre_tx_frame_vsync               ,
    input                   pre_tx_frame_href                ,
    input                   pre_tx_frame_clken               ,
    input   [7 : 0]         pre_tx_img                       ,
    input   [23 : 0]        pre_A                            ,
    output                  post_frame_vsync                 ,
    output                  post_frame_href                  ,
    output                  post_frame_clken                 ,
    output  [23 : 0]        post_img
);

wire    [7 : 0]         tx_value                         ;
reg     [7 : 0]         tx_value_d0                      ;
reg     [23 : 0]        pixel_d0                         ;
reg     [23 : 0]        atmosphere_d0                    ;
reg signed [8 : 0]         difference_r                     ;
reg signed [8 : 0]         difference_g                     ;
reg signed [8 : 0]         difference_b                     ;
reg     [15 : 0]        atmosphere_product_r             ;
reg     [15 : 0]        atmosphere_product_g             ;
reg     [15 : 0]        atmosphere_product_b             ;
reg     [7 : 0]         tx_value_d1                      ;
reg     [7 : 0]         tx_value_d2                      ;
reg signed [16 : 0]        value_tem_r                      ;
reg signed [16 : 0]        value_tem_g                      ;
reg signed [16 : 0]        value_tem_b                      ;
wire    [7 : 0]         recovered_r                      ;
wire    [7 : 0]         recovered_g                      ;
wire    [7 : 0]         recovered_b                      ;
reg     [11 : 0]        vs_pipe                          ;
reg     [11 : 0]        href_pipe                        ;
reg     [11 : 0]        de_pipe                          ;

// A zero floor still uses one as the divisor; normal callers supply t >= 1.
assign  tx_value                        =   pre_tx_img < tx_min ? tx_min :
                                            (pre_tx_img == 0 ? 8'd1 : pre_tx_img);

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        tx_value_d0                     <=  0;
        pixel_d0                        <=  0;
        atmosphere_d0                   <=  0;
        difference_r                    <=  0;
        difference_g                    <=  0;
        difference_b                    <=  0;
        atmosphere_product_r            <=  0;
        atmosphere_product_g            <=  0;
        atmosphere_product_b            <=  0;
        tx_value_d1                     <=  0;
        tx_value_d2                     <=  0;
        value_tem_r                     <=  0;
        value_tem_g                     <=  0;
        value_tem_b                     <=  0;
        vs_pipe                         <=  0;
        href_pipe                       <=  0;
        de_pipe                         <=  0;
    end
    else begin
        // Stage 1: floor t before the multiplier and align its RGB/A sample.
        tx_value_d0                     <=  tx_value;
        pixel_d0                        <=  pre_img;
        atmosphere_d0                   <=  pre_A;
        // Stage 2: signed difference and 8x8 atmospheric-light product.
        difference_r                    <=  $signed({1'b0, pixel_d0[23 : 16]}) - $signed({1'b0, atmosphere_d0[23 : 16]});
        difference_g                    <=  $signed({1'b0, pixel_d0[15 : 8]}) - $signed({1'b0, atmosphere_d0[15 : 8]});
        difference_b                    <=  $signed({1'b0, pixel_d0[7 : 0]}) - $signed({1'b0, atmosphere_d0[7 : 0]});
        atmosphere_product_r            <=  atmosphere_d0[23 : 16] * tx_value_d0;
        atmosphere_product_g            <=  atmosphere_d0[15 : 8] * tx_value_d0;
        atmosphere_product_b            <=  atmosphere_d0[7 : 0] * tx_value_d0;
        tx_value_d1                     <=  tx_value_d0;
        // Stage 3: numerator. Its complete range fits signed 17 bits.
        value_tem_r                     <=  $signed({difference_r, 8'd0}) +
                                            $signed({1'b0, atmosphere_product_r});
        value_tem_g                     <=  $signed({difference_g, 8'd0}) +
                                            $signed({1'b0, atmosphere_product_g});
        value_tem_b                     <=  $signed({difference_b, 8'd0}) +
                                            $signed({1'b0, atmosphere_product_b});
        tx_value_d2                     <=  tx_value_d1;
        vs_pipe                         <=  {vs_pipe[10 : 0], pre_tx_frame_vsync};
        href_pipe                       <=  {href_pipe[10 : 0], pre_tx_frame_href};
        de_pipe                         <=  {de_pipe[10 : 0], pre_tx_frame_clken};
    end
end

// Negative numerators recover to black. Positive values are divided exactly
// and clipped before the divider computes its eight result bits.
pipelined_divide_u8 u_divide_red(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .numerator                      (value_tem_r[16] ? 16'd0 : value_tem_r[15 : 0]),
    .denominator                    (tx_value_d2                     ),
    .quotient                       (recovered_r                     )
);

pipelined_divide_u8 u_divide_green(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .numerator                      (value_tem_g[16] ? 16'd0 : value_tem_g[15 : 0]),
    .denominator                    (tx_value_d2                     ),
    .quotient                       (recovered_g                     )
);

pipelined_divide_u8 u_divide_blue(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .numerator                      (value_tem_b[16] ? 16'd0 : value_tem_b[15 : 0]),
    .denominator                    (tx_value_d2                     ),
    .quotient                       (recovered_b                     )
);

assign  post_frame_vsync                 =   vs_pipe[11];
assign  post_frame_href                  =   href_pipe[11];
assign  post_frame_clken                 =   de_pipe[11];
assign  post_img                         =   {recovered_r, recovered_g, recovered_b};

endmodule
