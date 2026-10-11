// Exact unsigned division with an 8-bit saturated result.
// One sample per clock; nine clocks from input to quotient.
// Stage 0 handles saturation/zero; stages 1..8 resolve one quotient bit each.
module pipelined_divide_u8(
    input                   clk                              ,
    input                   rst_n                            ,
    input   [15 : 0]        numerator                        ,
    input   [7 : 0]         denominator                      ,
    output  [7 : 0]         quotient
);

wire    [15 : 0]        saturation_limit                 ;
reg     [15 : 0]        remainder_pipe[0 : 8]            ;
reg     [7 : 0]         denominator_pipe[0 : 8]          ;
reg     [7 : 0]         quotient_pipe[0 : 8]             ;
reg     [8 : 0]         saturated_pipe                   ;

assign  saturation_limit                =   {denominator, 8'd0} - {8'd0, denominator};
assign  quotient                        =   saturated_pipe[8] ? 8'd255 : quotient_pipe[8];

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        remainder_pipe[0]                <=  0;
        denominator_pipe[0]              <=  0;
        quotient_pipe[0]                 <=  0;
        saturated_pipe[0]                <=  0;
    end
    else begin
        denominator_pipe[0]              <=  denominator;
        quotient_pipe[0]                 <=  0;
        saturated_pipe[0]                <=  denominator != 0 && numerator >= saturation_limit;
        remainder_pipe[0]                <=  denominator == 0 || numerator >= saturation_limit ?
                                              16'd0 : numerator;
    end
end

genvar                  stage                            ;
generate
    for(stage = 1; stage < 9; stage = stage + 1)begin : g_divide
        wire    [15 : 0]        shifted_denominator              ;

        assign  shifted_denominator             =   {8'd0, denominator_pipe[stage - 1]} << (8 - stage);

        always@(posedge clk or negedge rst_n)begin
            if(!rst_n)begin
                remainder_pipe[stage]            <=  0;
                denominator_pipe[stage]          <=  0;
                quotient_pipe[stage]             <=  0;
                saturated_pipe[stage]            <=  0;
            end
            else begin
                denominator_pipe[stage]          <=  denominator_pipe[stage - 1];
                saturated_pipe[stage]            <=  saturated_pipe[stage - 1];
                if(denominator_pipe[stage - 1] != 0 &&
                   remainder_pipe[stage - 1] >= shifted_denominator)begin
                    remainder_pipe[stage]            <=  remainder_pipe[stage - 1] - shifted_denominator;
                    quotient_pipe[stage]             <=  quotient_pipe[stage - 1] | (8'd1 << (8 - stage));
                end
                else begin
                    remainder_pipe[stage]            <=  remainder_pipe[stage - 1];
                    quotient_pipe[stage]             <=  quotient_pipe[stage - 1];
                end
            end
        end
    end
endgenerate

endmodule
