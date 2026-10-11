// Delay a frame-constant control packet without using video-sized RAM.
// Consecutive frame starts must be more than DELAY_CLOCKS cycles apart.
module frame_config_delay#(
    parameter   DATA_WIDTH                       =   160,
    parameter   DELAY_CLOCKS                     =   1
)(
    input                   clk                              ,
    input                   rst_n                            ,
    input                   pre_vs                           ,
    input   [DATA_WIDTH - 1 : 0] pre_config                       ,
    output  [DATA_WIDTH - 1 : 0] post_config
);

generate
    if(DELAY_CLOCKS == 0)begin : g_direct

        assign  post_config                      =   pre_config;
    end
    else begin : g_delay
        localparam  COUNT_WIDTH                      =   DELAY_CLOCKS < 2 ? 1 : $clog2(DELAY_CLOCKS + 1);
        reg                     previous_vs                      ;
        reg                     seen                             ;
        reg                     busy                             ;
        reg     [COUNT_WIDTH - 1 : 0] remaining                        ;
        reg     [DATA_WIDTH - 1 : 0] held_config                      ;
        reg     [DATA_WIDTH - 1 : 0] output_config                    ;
        wire                    start_frame                      ;

        assign  start_frame                      =   previous_vs && !pre_vs;
        assign  post_config                      =   output_config;

        always@(posedge clk or negedge rst_n)begin
            if(!rst_n)begin
                previous_vs                      <=  1'b1;
                seen                             <=  1'b0;
                busy                             <=  1'b0;
                remaining                        <=  0;
                held_config                      <=  0;
                output_config                    <=  0;
            end
            else begin
                previous_vs                      <=  pre_vs;
                seen                             <=  1'b1;
                if(start_frame)begin
                    held_config                      <=  pre_config;
                    remaining                        <=  DELAY_CLOCKS - 1;
                    busy                             <=  DELAY_CLOCKS != 1;
                    if(DELAY_CLOCKS == 1)
                        output_config                    <=  pre_config;
                end
                else if(busy)begin
                    remaining                        <=  remaining - 1'b1;
                    if(remaining == 1)begin
                        output_config                    <=  held_config;
                        busy                             <=  1'b0;
                    end
                end
            end
        end

`ifndef SYNTHESIS

        always@(posedge clk)
            if(rst_n && seen && start_frame && busy)
                $fatal(1, "Frame control packets overlap; check video VS period");
`endif

    end
endgenerate

endmodule
