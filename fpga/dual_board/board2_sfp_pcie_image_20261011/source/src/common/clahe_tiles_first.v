`timescale 1ns/1ps
// Selects the four neighboring 8x8 CLAHE tiles and calculates the
// horizontal/vertical distance from the upper-left tile center.
module clahe_tiles_first(
    input wire [10 : 0]        x                                ,
    input wire [10 : 0]        y                                ,
    output wire [5 : 0]         tile_ul                          ,
    output wire [5 : 0]         tile_ur                          ,
    output wire [5 : 0]         tile_ll                          ,
    output wire [5 : 0]         tile_lr                          ,
    output wire [10 : 0]        x_intra                          ,
    output wire [10 : 0]        y_intra
);

wire                    at_left_edge                     =   (x < 11'd120);
wire                    at_right_edge                    =   (x >= 11'd1800);
wire    [2 : 0]         col_left                         =   at_left_edge ? 3'd0 : at_right_edge ?
        3'd7 : (x < 11'd360) ? 3'd0 : (x < 11'd600) ? 3'd1 : (x < 11'd840) ? 3'd2 : (x < 11'd1080) ?
        3'd3 : (x < 11'd1320) ? 3'd4 : (x < 11'd1560) ? 3'd5 : 3'd6;
wire    [2 : 0]         col_right                        =   (at_left_edge || at_right_edge) ?
        col_left : (col_left + 3'd1);

assign  x_intra                          =   (at_left_edge || at_right_edge) ?
                                             11'd0 : (x < 11'd360) ?
                                             (x - 11'd120) : (x < 11'd600) ?
                                             (x - 11'd360) : (x < 11'd840) ?
                                             (x - 11'd600) : (x < 11'd1080) ?
                                             (x - 11'd840) : (x < 11'd1320) ?
                                             (x - 11'd1080) : (x < 11'd1560) ?
                                             (x - 11'd1320) : (x - 11'd1560);
wire                    at_top_edge                      =   (y < 11'd67);
wire                    at_bot_edge                      =   (y >= 11'd1012);
wire    [2 : 0]         row_top                          =   at_top_edge ? 3'd0 : at_bot_edge ? 3'd7 : (y < 11'd202) ?
        3'd0 : (y < 11'd337) ? 3'd1 : (y < 11'd472) ? 3'd2 : (y < 11'd607) ? 3'd3 : (y < 11'd742) ?
        3'd4 : (y < 11'd877) ? 3'd5 : 3'd6;
wire    [2 : 0]         row_bot                          =   (at_top_edge || at_bot_edge) ? row_top : (row_top + 3'd1);

assign  y_intra                          =   (at_top_edge || at_bot_edge) ?
                                             11'd0 : (y < 11'd202) ?
                                             (y - 11'd67) : (y < 11'd337) ?
                                             (y - 11'd202) : (y < 11'd472) ?
                                             (y - 11'd337) : (y < 11'd607) ?
                                             (y - 11'd472) : (y < 11'd742) ?
                                             (y - 11'd607) : (y < 11'd877) ?
                                             (y - 11'd742) : (y - 11'd877);

assign  tile_ul                          =   {row_top, col_left};
assign  tile_ur                          =   {row_top, col_right};
assign  tile_ll                          =   {row_bot, col_left};
assign  tile_lr                          =   {row_bot, col_right};

endmodule
