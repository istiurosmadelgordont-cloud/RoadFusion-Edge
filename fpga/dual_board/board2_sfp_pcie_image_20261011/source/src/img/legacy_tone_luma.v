// Original video_enhance threshold curve on Y, without another conversion pair.
// Stage 1 compares/subtracts thresholds; stage 2 adds/subtracts with saturation.
// Both controls disabled still retain two clocks of latency.
module legacy_tone_luma#(
    parameter   RUNTIME_CONTROL                  =   0,
    parameter   LIGHTDOWN_ENABLE                 =   0,
    parameter   LIGHTDOWN_THRESHOLD              =   0,
    parameter   DARKUP_ENABLE                    =   0,
    parameter   DARKUP_THRESHOLD                 =   0
)(
    input   [159 : 0]       cfg_data                         ,
    input                   clk                              ,
    input                   rst_n                            ,
    input   [7 : 0]         pre_y                            ,
    output reg [7 : 0]      post_y
);

wire                    lightdown_enable                 ;
wire                    darkup_enable                    ;
wire    [7 : 0]         lightdown_threshold              ;
wire    [7 : 0]         darkup_threshold                 ;
wire    [7 : 0]         lift_difference                  ;
wire    [7 : 0]         reduce_difference                ;
reg     [7 : 0]         y_s1                             ;
reg     [4 : 0]         lift_s1                          ;
reg     [4 : 0]         reduce_s1                        ;
wire signed [9 : 0]     result_y                         ;

assign  lightdown_enable                 =   RUNTIME_CONTROL ? cfg_data[11] : LIGHTDOWN_ENABLE;
assign  darkup_enable                    =   RUNTIME_CONTROL ? cfg_data[12] : DARKUP_ENABLE;
assign  lightdown_threshold              =   RUNTIME_CONTROL ? cfg_data[95 : 88] : LIGHTDOWN_THRESHOLD;
assign  darkup_threshold                 =   RUNTIME_CONTROL ? cfg_data[87 : 80] : DARKUP_THRESHOLD;
assign  lift_difference                  =   darkup_enable && pre_y <= darkup_threshold ?
                                            darkup_threshold - pre_y : 8'd0;
assign  reduce_difference                =   lightdown_enable && pre_y >= lightdown_threshold ?
                                            pre_y - lightdown_threshold : 8'd0;
assign  result_y                         =   $signed({2'd0, y_s1}) + $signed({5'd0, lift_s1}) -
                                            $signed({5'd0, reduce_s1});

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        y_s1                            <=  0;
        lift_s1                         <=  0;
        reduce_s1                       <=  0;
        post_y                          <=  0;
    end
    else begin
        y_s1                            <=  pre_y;
        lift_s1                         <=  lift_difference[7 : 3];
        reduce_s1                       <=  reduce_difference[7 : 3];

        if(result_y < 0)
            post_y                          <=  8'd0;
        else if(result_y > 255)
            post_y                          <=  8'd255;
        else
            post_y                          <=  result_y[7 : 0];
    end
end

`ifndef SYNTHESIS

initial begin
    if(LIGHTDOWN_THRESHOLD < 0 || LIGHTDOWN_THRESHOLD > 255 ||
       DARKUP_THRESHOLD < 0 || DARKUP_THRESHOLD > 255)
        $error("Legacy tone thresholds must be 0..255");
end

`endif

endmodule
