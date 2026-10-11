// Full-range BT.601 YCbCr to RGB888, Q10 constants, three clock pipeline.
module YCbCr2RGB(
        input               i_sys_clk           ,
        input               i_rst_n             ,
        input               i_vs                ,
        input               i_hs                ,
        input               i_convert_en        ,
        input   [7  : 0]    i_y_data            ,
        input   [7  : 0]    i_cr_data           ,
        input   [7  : 0]    i_cb_data           ,
        output              o_vs                ,
        output              o_hs                ,
        output              o_convert_en        ,
        output reg [7 : 0]  o_red               ,
        output reg [7 : 0]  o_green             ,
        output reg [7 : 0]  o_blue
);

    reg         [2  : 0]    vs_pipe             ;
    reg         [2  : 0]    hs_pipe             ;
    reg         [2  : 0]    en_pipe             ;
    reg signed  [20 : 0]    y_scaled            ;
    reg signed  [20 : 0]    r_cr                ;
    reg signed  [20 : 0]    g_cr                ;
    reg signed  [20 : 0]    g_cb                ;
    reg signed  [20 : 0]    b_cb                ;
    reg signed  [20 : 0]    r_sum               ;
    reg signed  [20 : 0]    g_sum               ;
    reg signed  [20 : 0]    b_sum               ;

    // Carry-save compression leaves one final carry-propagating green sum.
    wire        [20 : 0]    green_partial       ;
    wire        [20 : 0]    green_carry         ;

    assign green_partial = y_scaled ^ g_cr ^ g_cb;
    assign green_carry   = ((y_scaled & g_cr) | (y_scaled & g_cb) | (g_cr & g_cb)) << 1;

    wire signed [10 : 0]    cr_centered         ;
    wire signed [10 : 0]    cb_centered         ;

    assign cr_centered = $signed({3'b0, i_cr_data}) - 11'sd128;
    assign cb_centered = $signed({3'b0, i_cb_data}) - 11'sd128;

    //---------------------------------------------
    // Signed Q10 result to saturated RGB component.
    function [7 : 0] clamp_q10;
        input signed [20 : 0] value;
        reg   signed [20 : 0] pixel;

        begin
            pixel = value >>> 10;

            if (pixel < 0) begin
                clamp_q10 = 0;
            end
            else if (pixel > 255) begin
                clamp_q10 = 255;
            end
            else begin
                clamp_q10 = pixel[7 : 0];
            end
        end
    endfunction

    //---------------------------------------------
    // Advance through blanking to drain the last pixels of each line.
    always@(posedge i_sys_clk or negedge i_rst_n)begin
        if(!i_rst_n)begin
            vs_pipe     <= 0;
            hs_pipe     <= 0;
            en_pipe     <= 0;
            y_scaled    <= 0;
            r_cr        <= 0;
            g_cr        <= 0;
            g_cb        <= 0;
            b_cb        <= 0;
            r_sum       <= 0;
            g_sum       <= 0;
            b_sum       <= 0;
            o_red       <= 0;
            o_green     <= 0;
            o_blue      <= 0;
        end
        else begin
            vs_pipe     <= {vs_pipe[1 : 0], i_vs};
            hs_pipe     <= {hs_pipe[1 : 0], i_hs};
            en_pipe     <= {en_pipe[1 : 0], i_convert_en};

            // Stage 1: scale Y and multiply centered chroma (negative green coefficients).
            y_scaled    <= $signed({3'b0, i_y_data, 10'b0});
            r_cr        <= cr_centered * 21'sd1436;
            g_cr        <= cr_centered * -21'sd731;
            g_cb        <= cb_centered * -21'sd352;
            b_cb        <= cb_centered * 21'sd1815;

            // Stage 2: combine Y and chroma products.
            r_sum       <= y_scaled + r_cr;
            g_sum       <= $signed(green_partial) + $signed(green_carry);
            b_sum       <= y_scaled + b_cb;

            // Stage 3: clamp the final RGB values.
            o_red       <= clamp_q10(r_sum);
            o_green     <= clamp_q10(g_sum);
            o_blue      <= clamp_q10(b_sum);
        end
    end

    assign o_vs         = vs_pipe[2];
    assign o_hs         = hs_pipe[2];
    assign o_convert_en = en_pipe[2];

endmodule
