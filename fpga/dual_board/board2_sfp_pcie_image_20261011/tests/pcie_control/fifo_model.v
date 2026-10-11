`timescale 1ns/1ps
// Controller test double, NOT a timing model of the vendor asynchronous FIFO.
// A registered read response has the same latency expected by this controller.
module hdmi_pcie_fifo(
    input wr_clk,wr_rst,wr_en,input [15:0] wr_data,
    output wr_full,output [12:0] wr_water_level,output almost_full,
    input rd_clk,rd_rst,rd_en,output reg [127:0] rd_data,
    output rd_empty,output [12:0] rd_water_level,output almost_empty
);
    reg supply=1;
    reg inject_full=0;
    reg [31:0] read_count=0;
    assign wr_full=inject_full;
    assign wr_water_level=0;
    assign almost_full=inject_full;
    assign rd_empty=!supply;
    assign rd_water_level=supply ? 13'd32 : 13'd0;
    assign almost_empty=!supply;
    always @(posedge rd_clk) begin
        if(rd_rst) begin rd_data<=0;read_count<=0;end
        else if(rd_en) begin
            if(!supply) $fatal(1,"FIFO read while empty");
            rd_data<={4{read_count}};
            read_count<=read_count+1;
        end
    end
endmodule
