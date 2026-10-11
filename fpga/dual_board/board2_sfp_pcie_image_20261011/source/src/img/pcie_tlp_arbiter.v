// Both sources emit complete PCIe MWr packets on the posted slave2 interface.
// A selected source owns the interface from first VALID through accepted TLAST.
module pcie_tlp_arbiter(
    input                   clk                              ,
    input                   rst_n                            ,
    input                   dma_tvalid                       ,
    input   [127 : 0]       dma_tdata                        ,
    input                   dma_tlast                        ,
    input                   dma_tuser                        ,
    output                  dma_tready                       ,
    input                   ack_tvalid                       ,
    input   [127 : 0]       ack_tdata                        ,
    input                   ack_tlast                        ,
    input                   ack_tuser                        ,
    output                  ack_tready                       ,
    output                  tx_tvalid                        ,
    output  [127 : 0]       tx_tdata                         ,
    output                  tx_tlast                         ,
    output                  tx_tuser                         ,
    input                   tx_tready
);

reg                     packet_locked                    ;
reg                     packet_owner                     ;
reg                     prefer_ack                       ;
wire                    selected_owner                   ;

assign  selected_owner                  =   packet_locked ? packet_owner :
                                           ack_tvalid && (!dma_tvalid || prefer_ack);
assign  tx_tvalid                       =   rst_n && (selected_owner ? ack_tvalid : dma_tvalid);
assign  tx_tdata                        =   selected_owner ? ack_tdata : dma_tdata;
assign  tx_tlast                        =   selected_owner ? ack_tlast : dma_tlast;
assign  tx_tuser                        =   selected_owner ? ack_tuser : dma_tuser;
assign  dma_tready                      =   rst_n && !selected_owner && tx_tready;
assign  ack_tready                      =   rst_n && selected_owner && tx_tready;

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        packet_locked                   <=  0;
        packet_owner                    <=  0;
        prefer_ack                      <=  1;
    end
    else begin
        if(tx_tvalid)begin
            if(tx_tready && tx_tlast)begin
                packet_locked           <=  0;
                prefer_ack              <=  !selected_owner;
            end
            else begin
                packet_locked           <=  1;
                packet_owner            <=  selected_owner;
            end
        end
    end
end

endmodule
