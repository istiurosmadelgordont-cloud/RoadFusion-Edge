// Frame-rate atmospheric-light stabilization. No pixel stream is delayed here.
// raw_valid must identify a completed frame with a usable A candidate.
// Successive raw_valid pulses must be at least six pixel clocks apart.
module atmosphere_temporal(
    input                   clk                     ,
    input                   rst_n                   ,
    input   [23 : 0]        raw_A_rgb               ,
    input                   raw_valid               ,

    output  reg [23 : 0]    smoothed_A_rgb          ,
    output  reg             initialized             ,
    output  reg             update_pulse
);

    localparam  [2 : 0]     STATE_IDLE          = 3'd0   ;
    localparam  [2 : 0]     STATE_DIFFERENCE    = 3'd1   ;
    localparam  [2 : 0]     STATE_SCENE         = 3'd2   ;
    localparam  [2 : 0]     STATE_STEP          = 3'd3   ;
    localparam  [2 : 0]     STATE_APPLY         = 3'd4   ;
    localparam  [2 : 0]     STATE_PUBLISH       = 3'd5   ;

    reg         [2 : 0]     state                   ;
    reg         [23 : 0]    sampled_A_rgb           ;
    reg         [15 : 0]    average_q8[0 : 2]       ;
    reg         [15 : 0]    next_average_q8[0 : 2]  ;
    reg         [15 : 0]    magnitude_q8[0 : 2]     ;
    reg         [10 : 0]    step_q8[0 : 2]          ;
    reg                     negative_step[0 : 2]    ;
    reg         [8 : 0]     scene_magnitude[0 : 2]  ;
    reg                     scene_direction[0 : 2]  ;
    reg                     move_channel[0 : 2]     ;
    reg                     fast_channel[0 : 2]     ;
    reg                     large_direction[0 : 2]  ;
    reg         [1 : 0]     large_count[0 : 2]      ;

    wire        [8 : 0]     byte_magnitude[0 : 2]   ;
    wire                    direction_up[0 : 2]     ;
    wire        [15 : 0]    raw_fixed_q8[0 : 2]     ;
    wire signed [17 : 0]    applied_q8[0 : 2]       ;
    wire        [23 : 0]    next_A_rgb              ;

    integer channel;
    genvar  g;

    // Round EMA magnitudes symmetrically while retaining Q8 fractional state.
    // Ordinary frames use 1/8 with a two-code limit. Three consistent large
    // estimates enable 1/4 with a four-code limit; one outlier cannot jump A.
    function [10 : 0] calculate_step;
        input   [15 : 0]    magnitude;
        input               enable_move;
        input               fast;
        reg     [13 : 0]    rounded_step;
        reg     [10 : 0]    step;
        begin
            // These expressions equal (magnitude + 2)/4 and (+4)/8.
            // Add after the shift to avoid a full-width carry chain.
            rounded_step = fast ? magnitude[15 : 2] + {13'd0, magnitude[1]} :
                                  {1'b0, magnitude[15 : 3]} + {13'd0, magnitude[2]};

            if(!enable_move)begin
                step = 11'd0;
            end
            else if(fast)begin
                if(rounded_step > 14'd1024)begin
                    step = 11'd1024;
                end
                else begin
                    step = rounded_step[10 : 0];
                end
            end
            else begin
                if(rounded_step > 14'd512)begin
                    step = 11'd512;
                end
                else begin
                    step = rounded_step[10 : 0];
                end
            end

            calculate_step = step;
        end
    endfunction

    // Separate byte comparison, Q8 step, accumulation and publication stages
    // keep the frame-rate feedback arithmetic off a long combinational path.
    generate
        for(g = 0; g < 3; g = g + 1)begin : g_channel
            wire    [15 : 0]    rounded_average;

            assign direction_up[g]        = sampled_A_rgb[g * 8 +: 8] >= smoothed_A_rgb[g * 8 +: 8];
            assign raw_fixed_q8[g]        = {sampled_A_rgb[g * 8 +: 8], 8'd0};
            assign byte_magnitude[g]      = direction_up[g] ?
                                            {1'b0, sampled_A_rgb[g * 8 +: 8]} -
                                            {1'b0, smoothed_A_rgb[g * 8 +: 8]} :
                                            {1'b0, smoothed_A_rgb[g * 8 +: 8]} -
                                            {1'b0, sampled_A_rgb[g * 8 +: 8]};
            // Select a direct add/subtract. Do not negate the step and then add.
            assign applied_q8[g]          = negative_step[g] ?
                                            $signed({2'b00, average_q8[g]}) - $signed({7'd0, step_q8[g]}) :
                                            $signed({2'b00, average_q8[g]}) + $signed({7'd0, step_q8[g]});
            assign rounded_average        = next_average_q8[g] + 16'd128;
            assign next_A_rgb[g * 8 +: 8]  = rounded_average[15 : 8];
        end
    endgenerate

    always@(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state               <= STATE_IDLE;
            sampled_A_rgb       <= {3{8'd230}};
            smoothed_A_rgb      <= {3{8'd230}};
            initialized         <= 1'b0;
            update_pulse        <= 1'b0;

            for(channel = 0; channel < 3; channel = channel + 1)begin
                average_q8[channel]       <= {8'd230, 8'd0};
                next_average_q8[channel]  <= {8'd230, 8'd0};
                magnitude_q8[channel]     <= 16'd0;
                step_q8[channel]          <= 11'd0;
                negative_step[channel]    <= 1'b0;
                scene_magnitude[channel]  <= 9'd0;
                scene_direction[channel]  <= 1'b0;
                move_channel[channel]     <= 1'b0;
                fast_channel[channel]     <= 1'b0;
                large_direction[channel]  <= 1'b0;
                large_count[channel]      <= 2'd0;
            end
        end
        else begin
            update_pulse <= 1'b0;

            case(state)
                STATE_IDLE : begin
                    if(raw_valid)begin
                        if(!initialized)begin
                            // The first real estimate avoids a slow startup from 230.
                            smoothed_A_rgb <= raw_A_rgb;
                            initialized    <= 1'b1;
                            update_pulse   <= 1'b1;

                            for(channel = 0; channel < 3; channel = channel + 1)begin
                                average_q8[channel]      <= {raw_A_rgb[channel * 8 +: 8], 8'd0};
                                next_average_q8[channel] <= {raw_A_rgb[channel * 8 +: 8], 8'd0};
                                large_count[channel]     <= 2'd0;
                            end
                        end
                        else begin
                            sampled_A_rgb <= raw_A_rgb;
                            state         <= STATE_DIFFERENCE;
                        end
                    end
                end

                STATE_DIFFERENCE : begin
                    for(channel = 0; channel < 3; channel = channel + 1)begin
                        // Both subtractions are parallel paths; no subtract/negate cascade.
                        if(raw_fixed_q8[channel] >= average_q8[channel])begin
                            magnitude_q8[channel]  <= raw_fixed_q8[channel] - average_q8[channel];
                            negative_step[channel] <= 1'b0;
                        end
                        else begin
                            magnitude_q8[channel]  <= average_q8[channel] - raw_fixed_q8[channel];
                            negative_step[channel] <= 1'b1;
                        end
                        // Register local byte features before scene-history feedback.
                        scene_magnitude[channel] <= byte_magnitude[channel];
                        scene_direction[channel] <= direction_up[channel];
                    end

                    state <= STATE_SCENE;
                end

                STATE_SCENE : begin
                    for(channel = 0; channel < 3; channel = channel + 1)begin
                        // A <= two-code fluctuation holds both the byte and Q8 state.
                        move_channel[channel]  <= scene_magnitude[channel] > 9'd2;
                        fast_channel[channel]  <= 1'b0;

                        if(scene_magnitude[channel] >= 9'd32)begin
                            large_direction[channel] <= scene_direction[channel];

                            if(large_count[channel] != 2'd0 &&
                               large_direction[channel] == scene_direction[channel])begin
                                if(large_count[channel] < 2'd3)begin
                                    large_count[channel] <= large_count[channel] + 2'd1;
                                end
                                fast_channel[channel] <= large_count[channel] >= 2'd2;
                            end
                            else begin
                                large_count[channel] <= 2'd1;
                            end
                        end
                        else begin
                            large_count[channel] <= 2'd0;
                        end
                    end

                    state <= STATE_STEP;
                end

                STATE_STEP : begin
                    for(channel = 0; channel < 3; channel = channel + 1)begin
                        step_q8[channel] <= calculate_step(magnitude_q8[channel],
                                                           move_channel[channel],
                                                           fast_channel[channel]);
                    end

                    state <= STATE_APPLY;
                end

                STATE_APPLY : begin
                    for(channel = 0; channel < 3; channel = channel + 1)begin
                        if(applied_q8[channel] < 18'sd0)begin
                            next_average_q8[channel] <= 16'd0;
                        end
                        else if(applied_q8[channel] > 18'sd65280)begin
                            next_average_q8[channel] <= 16'd65280;
                        end
                        else begin
                            next_average_q8[channel] <= applied_q8[channel][15 : 0];
                        end
                    end

                    state <= STATE_PUBLISH;
                end

                STATE_PUBLISH : begin
                    // RGB components commit together while the video is in blanking.
                    smoothed_A_rgb <= next_A_rgb;
                    update_pulse   <= next_A_rgb != smoothed_A_rgb;

                    for(channel = 0; channel < 3; channel = channel + 1)begin
                        average_q8[channel] <= next_average_q8[channel];
                    end

                    state <= STATE_IDLE;
                end

                default : begin
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

endmodule
