// Delay packed data and its control bits on every clock, including blanking.
module video_stream_delay#(
    parameter   DATA_WIDTH                       =   27,
    parameter   DELAY_CLOCKS                     =   8,
    parameter   LINE_CLOCKS                      =   2200
)(
    input                   clk                              ,
    input                   rst_n                            ,
    input   [DATA_WIDTH - 1 : 0]pre_data                         ,
    output  [DATA_WIDTH - 1 : 0]post_data
);

generate
    if(DELAY_CLOCKS == 0)begin : g_direct

        assign  post_data                        =   pre_data;
    end
    else if(DELAY_CLOCKS > LINE_CLOCKS && DELAY_CLOCKS < 2 * LINE_CLOCKS)begin : g_line
        video_line_delay#(
            .DATA_WIDTH                     (DATA_WIDTH                      ),
            .LINE_CLOCKS                    (LINE_CLOCKS                     ),
            .EXTRA_CLOCKS                   (DELAY_CLOCKS - LINE_CLOCKS      )
        )u_line_delay(
            .clk                            (clk                             ),
            .rst_n                          (rst_n                           ),
            .pre_data                       (pre_data                        ),
            .post_data                      (post_data                       )
        );

    end
    else begin : g_register
        reg     [DATA_WIDTH - 1 : 0]data_delay[0 : DELAY_CLOCKS - 1] ;
        integer                 i                                ;

        always@(posedge clk or negedge rst_n)begin
            if(!rst_n)begin
                for(i = 0; i < DELAY_CLOCKS; i = i + 1)
                    data_delay[i]                    <=  0;
            end
            else begin
                data_delay[0]                    <=  pre_data;
                for(i = 1; i < DELAY_CLOCKS; i = i + 1)
                    data_delay[i]                    <=  data_delay[i - 1];
            end
        end

        assign  post_data                        =   data_delay[DELAY_CLOCKS - 1];
    end
endgenerate

endmodule
