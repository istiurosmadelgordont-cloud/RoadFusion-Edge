module tx_get#(
    parameter   RUNTIME_CONTROL                  =   0,
    parameter   NORMALIZED_INPUT                 =   0,
    parameter   modification_value               =   8'd243
)(
    input   [159 : 0]       cfg_data                         ,
    input                   clk                              ,
    input                   rst_n                            ,
    input                   pre_frame_vsync                  ,
    input                   pre_frame_href                   ,
    input                   pre_frame_clken                  ,
    input   [7 : 0]         pre_img                          ,
    input   [7 : 0]         A_value                          ,
    output                  post_frame_vsync                 ,
    output                  post_frame_href                  ,
    output                  post_frame_clken                 ,
    output  [7 : 0]         post_img
);

reg     [15 : 0]        modify_A                         ;
reg     [7 : 0]         A_value_d1                       ;
reg                     pre_frame_vsync_d1               ;
reg                     pre_frame_href_d1                ;
reg                     pre_frame_clken_d1               ;
//---------------------------------------------

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        modify_A                         <=  0;
        A_value_d1                       <=  0;
        pre_frame_vsync_d1               <=  0;
        pre_frame_href_d1                <=  0;
        pre_frame_clken_d1               <=  0;
    end
    else begin
        modify_A                         <=  pre_img * (RUNTIME_CONTROL ? cfg_data[55 : 48] : modification_value);
        A_value_d1                       <=  A_value;
        pre_frame_vsync_d1               <=  pre_frame_vsync;
        pre_frame_href_d1                <=  pre_frame_href;
        pre_frame_clken_d1               <=  pre_frame_clken;
    end
end

// Normalized DCP uses A = 255 and an 8-bit omega, so n <= 255 * 255.
// Let h = n[15:8], l = n[7:0]. Then floor(n / 255) = h + (h + l >= 255).
// The folded sum never reaches 510 in this range; its correction is one bit.
// Thus t = 255 - h - carry, with no wide division/addition chain.
generate
    if(NORMALIZED_INPUT)begin : g_normalized
        wire    [8 : 0]         folded_product                   ;
        wire                    division_carry                   ;
        reg     [7 : 0]         result_d2                        ;
        reg                     frame_vsync_d2                   ;
        reg                     frame_href_d2                    ;
        reg                     frame_clken_d2                   ;

        assign  folded_product                  =   {1'b0, modify_A[15 : 8]} + {1'b0, modify_A[7 : 0]};
        assign  division_carry                  =   folded_product[8] || (&folded_product[7 : 0]);

        always@(posedge clk or negedge rst_n)begin
            if(!rst_n)begin
                result_d2                       <=  8'd255;
                frame_vsync_d2                  <=  0;
                frame_href_d2                   <=  0;
                frame_clken_d2                  <=  0;
            end
            else begin
                result_d2                       <=  (~modify_A[15 : 8]) - division_carry;
                frame_vsync_d2                  <=  pre_frame_vsync_d1;
                frame_href_d2                   <=  pre_frame_href_d1;
                frame_clken_d2                  <=  pre_frame_clken_d1;
            end
        end

        assign  post_img                         =   result_d2;
        assign  post_frame_vsync                 =   frame_vsync_d2;
        assign  post_frame_href                  =   frame_href_d2;
        assign  post_frame_clken                 =   frame_clken_d2;
    end
    else begin : g_dynamic_atmosphere
        wire    [15 : 0]        normalized_dark                  ;

        assign  normalized_dark                 =   A_value_d1 == 0 ? 16'd0 : modify_A / A_value_d1;
        assign  post_img                        =   A_value_d1 == 0 ?
                                                    8'd255 : normalized_dark >= 16'd255 ?
                                                    8'd0 : 16'd255 - normalized_dark;
        assign  post_frame_vsync                =   pre_frame_vsync_d1;
        assign  post_frame_href                 =   pre_frame_href_d1;
        assign  post_frame_clken                =   pre_frame_clken_d1;
    end
endgenerate

endmodule
