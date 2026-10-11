// Select by maximum dark value, then maximum RGB component.
// Keep all three components of that same pixel as atmospheric light.
// RGB is delayed by the same five clocks as the causal 3x3 dark-channel path.
// Two more stages prepare RGB maxima before the running-candidate comparison.
module calculate_A#(
        parameter src_delay = 5
)(
        input               clk                             ,
        input               rst_n                           ,
        input   [23 : 0]    pre_img                         ,

        input               dark_channel_frame_vsync        ,
        input               dark_channel_frame_href         ,
        input               dark_channel_frame_clken        ,
        input   [7  : 0]    dark_channel_img                ,

        output              post_frame_vsync                ,
        output              post_frame_href                 ,
        output              post_frame_clken                ,
        output  [7  : 0]    post_result                     ,
        output  [23 : 0]    post_result_rgb                 ,
        output              post_done                       ,
        output              post_estimate_valid
);

    reg     [23 : 0]    pre_img_d[src_delay - 1 : 0]    ;
    reg                 dark_channel_frame_vsync_d0     ;
    reg                 dark_channel_frame_href_d0      ;
    reg                 dark_channel_frame_clken_d0     ;
    reg     [7  : 0]    max_dark_channel_img          /*synthesis PAP_MARK_DEBUG="true"*/  ;
    reg     [7  : 0]    A_value                         ;
    reg     [7  : 0]    A_value_out                     ;
    reg     [23 : 0]    A_rgb /*synthesis PAP_MARK_DEBUG="true"*/                          ;
    reg     [23 : 0]    A_rgb_out /*synthesis PAP_MARK_DEBUG="true"*/                      ;
    reg                 have_candidate                  ;
    reg                 A_value_valid                   ;
    reg                 A_estimate_valid                ;
    reg     [23 : 0]    candidate_rgb_s1                ;
    reg     [23 : 0]    candidate_rgb_s2                ;
    reg     [7 : 0]     max_rg_s1                       ;
    reg     [7 : 0]     candidate_max_s2                ;
    reg     [7 : 0]     candidate_dark_s1               ;
    reg     [7 : 0]     candidate_dark_s2               ;
    reg                 candidate_vsync_s1              ;
    reg                 candidate_vsync_s2              ;
    reg                 candidate_href_s1               ;
    reg                 candidate_href_s2               ;
    reg                 candidate_clken_s1              ;
    reg                 candidate_clken_s2              ;

    wire    [23 : 0]    post_img                        ;
    wire    [7  : 0]    max_rg                          ;
    wire                neg_dark_channel_frame_vsync     ;

    integer i;

    assign post_img                     = pre_img_d[src_delay - 1];
    assign max_rg                       = post_img[23 : 16] > post_img[15 : 8] ?
                                          post_img[23 : 16] : post_img[15 : 8];
    assign neg_dark_channel_frame_vsync  = dark_channel_frame_vsync_d0 && !candidate_vsync_s2;

    //---------------------------------------------
    // Delay RGB to align with dark-channel data.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < src_delay; i = i + 1) begin
                pre_img_d[i] <= 0;
            end

            dark_channel_frame_vsync_d0  <= 0;
            dark_channel_frame_href_d0   <= 0;
            dark_channel_frame_clken_d0  <= 0;
        end
        else begin
            pre_img_d[0] <= pre_img;
            for (i = 1; i < src_delay; i = i + 1) begin
                pre_img_d[i] <= pre_img_d[i - 1];
            end

            dark_channel_frame_vsync_d0  <= candidate_vsync_s2;
            dark_channel_frame_href_d0   <= candidate_href_s2;
            dark_channel_frame_clken_d0  <= candidate_clken_s2;
        end
    end

    // Prepare maxima without pipelining the running-maximum feedback itself.
    // Every dark/RGB/control sample takes the same two preparation stages.
    always@(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            candidate_rgb_s1            <= 0;
            candidate_rgb_s2            <= 0;
            max_rg_s1                   <= 0;
            candidate_max_s2            <= 0;
            candidate_dark_s1           <= 0;
            candidate_dark_s2           <= 0;
            candidate_vsync_s1          <= 0;
            candidate_vsync_s2          <= 0;
            candidate_href_s1           <= 0;
            candidate_href_s2           <= 0;
            candidate_clken_s1          <= 0;
            candidate_clken_s2          <= 0;
        end
        else begin
            max_rg_s1                   <= max_rg;
            candidate_rgb_s1            <= post_img;
            candidate_dark_s1           <= dark_channel_img;
            candidate_vsync_s1          <= dark_channel_frame_vsync;
            candidate_href_s1           <= dark_channel_frame_href;
            candidate_clken_s1          <= dark_channel_frame_clken;
            candidate_max_s2            <= max_rg_s1 > candidate_rgb_s1[7 : 0] ?
                                           max_rg_s1 : candidate_rgb_s1[7 : 0];
            candidate_rgb_s2            <= candidate_rgb_s1;
            candidate_dark_s2           <= candidate_dark_s1;
            candidate_vsync_s2          <= candidate_vsync_s1;
            candidate_href_s2           <= candidate_href_s1;
            candidate_clken_s2          <= candidate_clken_s1;
        end
    end

    //---------------------------------------------
    // Accumulate paired dark/RGB candidates; publish A at frame end.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            max_dark_channel_img    <= 0;
            A_value                 <= 0;
            have_candidate          <= 0;
            A_value_out             <= 8'd230;
            A_rgb                   <= 0;
            A_rgb_out               <= {3{8'd230}};
            A_value_valid           <= 0;
            A_estimate_valid        <= 0;
        end
        else begin
            A_value_valid       <= 0;
            A_estimate_valid    <= 0;

            if (neg_dark_channel_frame_vsync) begin
                // Nonblocking assignment publishes the completed frame before clearing.
                // Empty frames retain the last usable A; an all-black valid frame yields 0.
                if (have_candidate) begin
                    A_value_out <= A_value;
                    A_rgb_out   <= A_rgb;
                    A_estimate_valid <= 1;
                end

                A_value_valid           <= 1;
                max_dark_channel_img    <= 0;
                A_value                 <= 0;
                A_rgb                   <= 0;
                have_candidate          <= 0;
            end
            else if (candidate_vsync_s2 && candidate_href_s2 && candidate_clken_s2) begin
                if (!have_candidate || candidate_dark_s2 > max_dark_channel_img) begin
                    max_dark_channel_img    <= candidate_dark_s2;
                    A_value                 <= candidate_max_s2;
                    A_rgb                   <= candidate_rgb_s2;
                    have_candidate          <= 1;
                end
                else if (candidate_dark_s2 == max_dark_channel_img && candidate_max_s2 > A_value) begin
                    A_value <= candidate_max_s2;
                    A_rgb   <= candidate_rgb_s2;
                end
            end
        end
    end

    assign post_frame_vsync  = dark_channel_frame_vsync_d0;
    assign post_frame_href   = dark_channel_frame_href_d0;
    assign post_frame_clken  = dark_channel_frame_clken_d0;
    assign post_result       = A_value_out;
    assign post_result_rgb   = A_rgb_out;
    assign post_done            = A_value_valid;
    // Unlike post_done, empty frames do not initialise the temporal estimator.
    assign post_estimate_valid  = A_estimate_valid;

endmodule
