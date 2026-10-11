`timescale 1ns / 1ps

// Data-bearing model of the configured 32768 x 16 -> 4096 x 128 FIFO.
// Flags use ideal pointer visibility; asynchronous reset and registered read
// data are modelled, without the vendor's internal pointer synchronizers.
module hdmi_pcie_fifo (
    input                       wr_clk,
    input                       wr_rst,
    input                       wr_en,
    input       [15 : 0]        wr_data,
    output                      wr_full,
    output      [15 : 0]        wr_water_level,
    output                      almost_full,
    input                       rd_clk,
    input                       rd_rst,
    input                       rd_en,
    output reg  [127 : 0]       rd_data,
    output                      rd_empty,
    output      [12 : 0]        rd_water_level,
    output                      almost_empty
);

reg             [15 : 0]        memory [0 : 32767];
integer                         wr_pointer = 0;
integer                         rd_pointer = 0;
integer                         write_count = 0;
integer                         read_count = 0;
integer                         available;
integer                         lane;

always@(*)begin
    if(wr_rst || rd_rst || wr_pointer < rd_pointer)begin
        available = 0;
    end
    else begin
        available = wr_pointer - rd_pointer;
    end
end

assign wr_full          = available >= 32768;
assign wr_water_level   = available;
assign almost_full      = available >= 32760;
assign rd_empty         = available < 8;
assign rd_water_level   = available / 8;
assign almost_empty     = available < 16;

always@(posedge wr_clk or posedge wr_rst)begin
    if(wr_rst)begin
        wr_pointer <= 0;
    end
    else if(wr_en)begin
        if(wr_full)begin
            $fatal(1, "FIFO write overflow");
        end
        memory[wr_pointer % 32768] <= wr_data;
        wr_pointer                <= wr_pointer + 1;
        write_count               <= write_count + 1;
    end
end

always@(posedge rd_clk or posedge rd_rst)begin
    if(rd_rst)begin
        rd_pointer <= 0;
        rd_data    <= 0;
    end
    else if(rd_en)begin
        if(rd_empty)begin
            $fatal(1, "FIFO read underflow");
        end
        for(lane = 0; lane < 8; lane = lane + 1)begin
            rd_data[lane * 16 +: 16] <= memory[(rd_pointer + lane) % 32768];
        end
        rd_pointer <= rd_pointer + 8;
        read_count <= read_count + 8;
    end
end

endmodule
