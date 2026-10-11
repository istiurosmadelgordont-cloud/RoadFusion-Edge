// Circular line delay memory; availability is handled by the matrix module.
module fifo_ram#(
        parameter DATA_WIDTH = 8       ,
        parameter DATA_DEPTH = 1920
)(
        input                               clk         ,
        input                               rst_n       ,
        input                               sclr        ,
        input                               wr_en       ,
        input       [DATA_WIDTH - 1 : 0]    wr_data     ,
        output                              wr_full     ,
        input                               rd_en       ,
        output reg  [DATA_WIDTH - 1 : 0]    rd_data     ,
        output                              rd_empty
);

    localparam ADDRESS_WIDTH = DATA_DEPTH > 1 ? $clog2(DATA_DEPTH) : 1;

    (* ram_style = "block" *) reg [DATA_WIDTH - 1 : 0] fifo_buffer[0 : DATA_DEPTH - 1];
    reg [ADDRESS_WIDTH - 1 : 0] wr_pointer;
    reg [ADDRESS_WIDTH - 1 : 0] rd_pointer;

    //---------------------------------------------
    // Keep RAM free of reset logic for block-RAM inference.
    always @(posedge clk) begin
        if (rst_n && !sclr && wr_en) begin
            fifo_buffer[wr_pointer] <= wr_data;
        end
    end

    //---------------------------------------------
    // Reset addressing and the registered read port.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_pointer  <= 0;
            rd_pointer  <= 0;
            rd_data     <= 0;
        end
        else if (sclr) begin
            wr_pointer  <= 0;
            rd_pointer  <= 0;
            rd_data     <= 0;
        end
        else begin
            if (wr_en) begin
                wr_pointer <= wr_pointer == DATA_DEPTH - 1 ? 0 : wr_pointer + 1'b1;
            end

            if (rd_en) begin
                rd_pointer <= rd_pointer == DATA_DEPTH - 1 ? 0 : rd_pointer + 1'b1;
                rd_data    <= fifo_buffer[rd_pointer];
            end
            else begin
                rd_data <= 0;
            end
        end
    end

    assign wr_full  = 1'b0;
    assign rd_empty = 1'b0;

endmodule
