// Four parity banks: each holds 16 tiles and two complete frame pages.
// Adjacent UL/UR/LL/LR tiles use separate banks. Clamped neighbours reuse
// the same address and read value, so every tile is stored only once.
module clahe_cdf_store(
    input                   clk                              ,
    input                   rst_n                            ,
    input                   clear                            ,
    input                   stall                            ,
    input                   frame_cdf_done                   ,
    input                   wr_en                            ,
    input   [5 : 0]         wr_tile                          ,
    input   [7 : 0]         wr_bin                           ,
    input   [7 : 0]         wr_data                          ,
    input   [5 : 0]         tile_ul                          ,
    input   [5 : 0]         tile_ur                          ,
    input   [5 : 0]         tile_ll                          ,
    input   [5 : 0]         tile_lr                          ,
    input   [7 : 0]         pixel_y                          ,
    output  [7 : 0]         cdf_ul                           ,
    output  [7 : 0]         cdf_ur                           ,
    output  [7 : 0]         cdf_ll                           ,
    output  [7 : 0]         cdf_lr
);

(* ram_style = "block" *)
reg     [7 : 0]         bank_00[0 : 8191]                ;
(* ram_style = "block" *)
reg     [7 : 0]         bank_01[0 : 8191]                ;
(* ram_style = "block" *)
reg     [7 : 0]         bank_10[0 : 8191]                ;
(* ram_style = "block" *)
reg     [7 : 0]         bank_11[0 : 8191]                ;
reg                     write_page                       ;
reg                     cdf_ready                        ;
reg                     read_valid                       ;
reg     [7 : 0]         original_y                       ;
reg     [1 : 0]         select_ul                        ,
                        select_ur                        ,
                        select_ll                        ,
                        select_lr                        ;
reg     [7 : 0]         data_00                          ,
                        data_01                          ,
                        data_10                          ,
                        data_11                          ;
reg     [12 : 0]        address_00                       ,
                        address_01                       ,
                        address_10                       ,
                        address_11                       ;
wire    [1 : 0]         write_bank                       ;
wire    [12 : 0]        write_address                    ;
wire    [12 : 0]        address_ul                       ,
                        address_ur                       ,
                        address_ll                       ,
                        address_lr                       ;
wire                    read_page                        ;

function [1 : 0] tile_bank;
    input   [5 : 0]         tile                             ;
    begin
        tile_bank                        =  {tile[3], tile[0]};
    end
endfunction

function [12 : 0] tile_address;
    input                   page                             ;
    input   [5 : 0]         tile                             ;
    input   [7 : 0]         value                            ;
    begin
        tile_address                     =  {page, tile[5 : 4], tile[2 : 1], value};
    end
endfunction

assign  read_page                        =   !write_page;
assign  write_bank                       =   tile_bank(wr_tile);
assign  write_address                    =   tile_address(write_page, wr_tile, wr_bin);
assign  address_ul                       =   tile_address(read_page, tile_ul, pixel_y);
assign  address_ur                       =   tile_address(read_page, tile_ur, pixel_y);
assign  address_ll                       =   tile_address(read_page, tile_ll, pixel_y);
assign  address_lr                       =   tile_address(read_page, tile_lr, pixel_y);
// Repeated edge requests always have identical addresses.

always@(*)begin
    address_00                       =  0;
    address_01                       =  0;
    address_10                       =  0;
    address_11                       =  0;
    case(tile_bank(tile_ul))
        0:
            address_00                       =  address_ul;
        1:
            address_01                       =  address_ul;
        2:
            address_10                       =  address_ul;
        3:
            address_11                       =  address_ul;
    endcase
    case(tile_bank(tile_ur))
        0:
            address_00                       =  address_ur;
        1:
            address_01                       =  address_ur;
        2:
            address_10                       =  address_ur;
        3:
            address_11                       =  address_ur;
    endcase
    case(tile_bank(tile_ll))
        0:
            address_00                       =  address_ll;
        1:
            address_01                       =  address_ll;
        2:
            address_10                       =  address_ll;
        3:
            address_11                       =  address_ll;
    endcase
    case(tile_bank(tile_lr))
        0:
            address_00                       =  address_lr;
        1:
            address_01                       =  address_lr;
        2:
            address_10                       =  address_lr;
        3:
            address_11                       =  address_lr;
    endcase
end

// A coincident final-bin write stays on the page being completed because
// write_page changes only after that clock edge.

always@(posedge clk)begin
    if(!rst_n || clear)begin
        write_page                       <=  0;
        cdf_ready                        <=  0;
    end
    else if(frame_cdf_done)begin
        write_page                       <=  !write_page;
        cdf_ready                        <=  1;
    end
end

// One write and one synchronous read per bank, on different frame pages.
// No RAM reset loops, table replication, or blanking-time copy is needed.

always@(posedge clk)begin
    if(rst_n && !clear && wr_en && write_bank == 0)
        bank_00[write_address]           <=  wr_data;
    if(rst_n && !clear && !stall && cdf_ready)
        data_00                          <=  bank_00[address_00];
end

always@(posedge clk)begin
    if(rst_n && !clear && wr_en && write_bank == 1)
        bank_01[write_address]           <=  wr_data;
    if(rst_n && !clear && !stall && cdf_ready)
        data_01                          <=  bank_01[address_01];
end

always@(posedge clk)begin
    if(rst_n && !clear && wr_en && write_bank == 2)
        bank_10[write_address]           <=  wr_data;
    if(rst_n && !clear && !stall && cdf_ready)
        data_10                          <=  bank_10[address_10];
end

always@(posedge clk)begin
    if(rst_n && !clear && wr_en && write_bank == 3)
        bank_11[write_address]           <=  wr_data;
    if(rst_n && !clear && !stall && cdf_ready)
        data_11                          <=  bank_11[address_11];
end

// Selectors, validity and identity bypass follow the one-clock RAM lookup.

always@(posedge clk)begin
    if(!rst_n || clear)begin
        read_valid                       <=  0;
        original_y                       <=  0;
        select_ul                        <=  0;
        select_ur                        <=  0;
        select_ll                        <=  0;
        select_lr                        <=  0;
    end
    else if(!stall)begin
        read_valid                       <=  cdf_ready;
        original_y                       <=  pixel_y;
        select_ul                        <=  tile_bank(tile_ul);
        select_ur                        <=  tile_bank(tile_ur);
        select_ll                        <=  tile_bank(tile_ll);
        select_lr                        <=  tile_bank(tile_lr);
    end
end

function [7 : 0] select_data;
    input   [1 : 0]         select                           ;
    input   [7 : 0]         d00                              ,
                            d01                              ,
                            d10                              ,
                            d11                              ;
    begin
        case(select)
            0:
                select_data                      =  d00;
            1:
                select_data                      =  d01;
            2:
                select_data                      =  d10;
            3:
                select_data                      =  d11;
        endcase
    end
endfunction

assign  cdf_ul                           =   read_valid ?
                                             select_data(select_ul, data_00, data_01, data_10, data_11) : original_y;

assign  cdf_ur                           =   read_valid ?
                                             select_data(select_ur, data_00, data_01, data_10, data_11) : original_y;

assign  cdf_ll                           =   read_valid ?
                                             select_data(select_ll, data_00, data_01, data_10, data_11) : original_y;

assign  cdf_lr                           =   read_valid ?
                                             select_data(select_lr, data_00, data_01, data_10, data_11) : original_y;
`ifndef SYNTHESIS
// Supported read neighbourhoods are adjacent or clamped 2x2 tiles.

always@(posedge clk)begin
    if(rst_n && !clear && !stall && cdf_ready)begin
        if((tile_bank(tile_ul) == tile_bank(tile_ur) && address_ul != address_ur) ||
                (tile_bank(tile_ul) == tile_bank(tile_ll) && address_ul != address_ll) ||
                (tile_bank(tile_ul) == tile_bank(tile_lr) && address_ul != address_lr) ||
                (tile_bank(tile_ur) == tile_bank(tile_ll) && address_ur != address_ll) ||
                (tile_bank(tile_ur) == tile_bank(tile_lr) && address_ur != address_lr) ||
                (tile_bank(tile_ll) == tile_bank(tile_lr) && address_ll != address_lr))
            $error("CLAHE CDF bank conflict: non-adjacent tile requests");
    end
end

`endif

endmodule
