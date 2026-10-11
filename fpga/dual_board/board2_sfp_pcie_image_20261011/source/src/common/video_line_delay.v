// One raster line plus a short register pipeline, including all blanking clocks.
// The RAM has no bulk reset. A fill flag suppresses data until it is initialised.
module video_line_delay#(
    parameter   DATA_WIDTH                       =   67,
    parameter   LINE_CLOCKS                      =   2200,
    parameter   EXTRA_CLOCKS                     =   16
)(
    input                   clk                              ,
    input                   rst_n                            ,
    input   [DATA_WIDTH - 1 : 0]pre_data                         ,
    output  [DATA_WIDTH - 1 : 0]post_data
);

localparam  ADDRESS_WIDTH                    =   $clog2(LINE_CLOCKS);
reg     [DATA_WIDTH - 1 : 0]line_ram[0 : LINE_CLOCKS - 1]    ;
reg     [ADDRESS_WIDTH - 1 : 0]write_address                    ;
wire    [ADDRESS_WIDTH - 1 : 0]read_address                     ;
reg     [ADDRESS_WIDTH - 1 : 0]fill_count                       ;
reg     [DATA_WIDTH - 1 : 0]ram_data                         ;
reg                     ram_valid                        ;
wire    [DATA_WIDTH - 1 : 0]line_data                        ;
// Reading the next slot avoids a simultaneous read/write to the same address.
// The synchronous read register makes the line delay exactly LINE_CLOCKS.

assign  read_address                     =   write_address == LINE_CLOCKS - 1 ? 0 : write_address + 1'b1;
assign  line_data                        =   ram_valid ? ram_data : {DATA_WIDTH {1'b0}};

always@(posedge clk)begin
    if(rst_n)begin
        line_ram[write_address]          <=  pre_data;
        ram_data                         <=  line_ram[read_address];
    end
end

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        write_address                    <=  0;
        fill_count                       <=  0;
        ram_valid                        <=  0;
    end
    else begin
        write_address                    <=  read_address;
        ram_valid                        <=  fill_count == LINE_CLOCKS - 1;
        if(fill_count < LINE_CLOCKS - 1)
            fill_count                       <=  fill_count + 1'b1;
    end
end

reg     [DATA_WIDTH - 1 : 0]data_delay[0 : EXTRA_CLOCKS - 1] ;
integer                 i                                ;

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        for(i = 0; i < EXTRA_CLOCKS; i = i + 1)
            data_delay[i]                    <=  0;
    end
    else begin
        data_delay[0]                    <=  line_data;
        for(i = 1; i < EXTRA_CLOCKS; i = i + 1)
            data_delay[i]                    <=  data_delay[i - 1];
    end
end

assign  post_data                        =   data_delay[EXTRA_CLOCKS - 1];

endmodule
