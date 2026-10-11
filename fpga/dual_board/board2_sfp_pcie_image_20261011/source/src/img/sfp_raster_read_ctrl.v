// Read the existing SFP RX FIFO as a fixed raster without adding image RAM.
// A short input line aborts reads until the next frame marker.
module sfp_raster_read_ctrl#(
    parameter   PIC_WIDTH                       =   1920,
    parameter   PIC_HEIGHT                      =   1080,
    parameter   H_BLANK                         =   280,
    parameter   PREFILL_PIXELS                  =   64
)(
    input                   clk                         ,
    input                   rst_n                       ,
    input                   frame_reset                 ,
    input   [12 : 0]        fifo_level                  ,
    input                   fifo_empty                  ,
    output                  fifo_read                   ,
    output  reg             frame_error
);

localparam  LINE_CLOCKS                      =   PIC_WIDTH + H_BLANK;
reg     [15 : 0]        column_count                ;
reg     [15 : 0]        row_count                   ;
reg                     frame_armed                 ;
reg                     running                     ;

assign  fifo_read                        =   running && !frame_reset && !frame_error &&
                                            column_count < PIC_WIDTH && !fifo_empty;

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        column_count                    <=  0;
        row_count                       <=  0;
        frame_armed                     <=  0;
        running                         <=  0;
        frame_error                     <=  0;
    end
    else if(frame_reset)begin
        column_count                    <=  0;
        row_count                       <=  0;
        frame_armed                     <=  1;
        running                         <=  0;
        frame_error                     <=  0;
    end
    else if(frame_armed && !running && !frame_error)begin
        if(fifo_level >= PIC_WIDTH + PREFILL_PIXELS)begin
            frame_armed                 <=  0;
            running                     <=  1;
        end
    end
    else if(running)begin
        if(column_count < PIC_WIDTH && fifo_empty)begin
            running                     <=  0;
            frame_error                 <=  1;
        end
        else if(column_count == LINE_CLOCKS - 1)begin
            column_count                <=  0;
            if(row_count == PIC_HEIGHT - 1)begin
                running                 <=  0;
            end
            else if(fifo_level < PIC_WIDTH)begin
                running                 <=  0;
                frame_error             <=  1;
            end
            else begin
                row_count               <=  row_count + 1'b1;
            end
        end
        else begin
            column_count                <=  column_count + 1'b1;
        end
    end
end

endmodule
