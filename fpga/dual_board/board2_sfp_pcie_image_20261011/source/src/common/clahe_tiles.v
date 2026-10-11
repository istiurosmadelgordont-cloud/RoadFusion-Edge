// DONOR_MODE reproduces first-version 1080p centres and integer weights.
// The alternate path uses exact half-pixel centres and rounded Q8 weights.
module clahe_tiles#(
    parameter   PIC_WIDTH                        =   1920,
    parameter   PIC_HEIGHT                       =   1080,
    parameter   DONOR_MODE                       =   0
)(
    input   [10 : 0]        pixel_x                          ,
    input   [10 : 0]        pixel_y                          ,
    output  [5 : 0]         tile_ul                          ,
    output  [5 : 0]         tile_ur                          ,
    output  [5 : 0]         tile_ll                          ,
    output  [5 : 0]         tile_lr                          ,
    output  [8 : 0]         weight_x                         ,
    output  [8 : 0]         weight_y
);

generate
    if(DONOR_MODE && PIC_WIDTH == 1920 && PIC_HEIGHT == 1080)begin : g_first
        wire    [10 : 0]        x_intra                          ,
                                y_intra                          ;
        wire    [19 : 0]        x_product                        ,
                                y_product                        ;
        clahe_tiles_first u_first(
            .x                              (pixel_x                         ),
            .y                              (pixel_y                         ),
            .tile_ul                        (tile_ul                         ),
            .tile_ur                        (tile_ur                         ),
            .tile_ll                        (tile_ll                         ),
            .tile_lr                        (tile_lr                         ),
            .x_intra                        (x_intra                         ),
            .y_intra                        (y_intra                         )
        );

        assign  x_product                        =   x_intra * 20'd273;
        assign  y_product                        =   y_intra * 20'd243;
        assign  weight_x                         =   {1'b0, x_product[15 : 8]};
        assign  weight_y                         =   y_product[15 : 7];
    end
    else begin : g_exact
        localparam  TILE_WIDTH                       =   PIC_WIDTH / 8;
        localparam  TILE_HEIGHT                      =   PIC_HEIGHT / 8;
        reg     [8 : 0]         x_weights[0 : 2 * TILE_WIDTH - 1] /* synthesis syn_romstyle = "lut_rom" */;
        reg     [8 : 0]         y_weights[0 : 2 * TILE_HEIGHT - 1] /* synthesis syn_romstyle = "lut_rom" */;
        reg     [2 : 0]         left_col                         ,
                                right_col                        ,
                                top_row                          ,
                                bottom_row                       ;
        reg     [8 : 0]         offset_x                         ,
                                offset_y                         ;
        wire    [11 : 0]        twice_x                          ,
                                twice_y                          ;
        integer                 init_x                           ,
                                init_y                           ;
        integer                 column                           ,
                                row                              ;
        // The divisions are evaluated at elaboration, not implemented as dividers.

        initial begin
            for(init_x = 0; init_x < 2 * TILE_WIDTH; init_x = init_x + 1)
                x_weights[init_x]                =  (init_x * 256 + TILE_WIDTH) / (2 * TILE_WIDTH);
            for(init_y = 0; init_y < 2 * TILE_HEIGHT; init_y = init_y + 1)
                y_weights[init_y]                =  (init_y * 256 + TILE_HEIGHT) / (2 * TILE_HEIGHT);
        end

        assign  twice_x                          =   {pixel_x, 1'b0};
        assign  twice_y                          =   {pixel_y, 1'b0};

        always@(*)begin
            left_col                         =  0;
            right_col                        =  0;
            offset_x                         =  0;
            if(twice_x >= 2 * PIC_WIDTH - TILE_WIDTH)begin
                left_col                         =  7;
                right_col                        =  7;
            end
            else if(twice_x >= TILE_WIDTH)begin
                for(column = 0; column < 7; column = column + 1)
                    if(twice_x >=(2 * column + 1) * TILE_WIDTH)
                        left_col                         =  column[2 : 0];
                right_col                        =  left_col + 1'b1;
                offset_x                         =  twice_x - TILE_WIDTH - left_col * (2 * TILE_WIDTH);
            end
            top_row                          =  0;
            bottom_row                       =  0;
            offset_y                         =  0;
            if(twice_y >= 2 * PIC_HEIGHT - TILE_HEIGHT)begin
                top_row                          =  7;
                bottom_row                       =  7;
            end
            else if(twice_y >= TILE_HEIGHT)begin
                for(row = 0; row < 7; row = row + 1)
                    if(twice_y >=(2 * row + 1) * TILE_HEIGHT)
                        top_row                          =  row[2 : 0];
                bottom_row                       =  top_row + 1'b1;
                offset_y                         =  twice_y - TILE_HEIGHT - top_row * (2 * TILE_HEIGHT);
            end
        end

        assign  tile_ul                          =   {top_row, left_col};
        assign  tile_ur                          =   {top_row, right_col};
        assign  tile_ll                          =   {bottom_row, left_col};
        assign  tile_lr                          =   {bottom_row, right_col};
        assign  weight_x                         =   x_weights[offset_x];
        assign  weight_y                         =   y_weights[offset_y];
    end
endgenerate

endmodule


// DONOR_MODE reproduces first-version 1080p centres and integer weights.
// The alternate path uses exact half-pixel centres and rounded Q8 weights.
module clahe_tiles_registered#(
    parameter   PIC_WIDTH                        =   1920,
    parameter   PIC_HEIGHT                       =   1080,
    parameter   DONOR_MODE                       =   0
)(
    input                   clk                              ,
    input                   rst_n                            ,
    input   [10 : 0]        pixel_x                          ,
    input   [10 : 0]        pixel_y                          ,
    output reg [5 : 0]         tile_ul                          ,
    output reg [5 : 0]         tile_ur                          ,
    output reg [5 : 0]         tile_ll                          ,
    output reg [5 : 0]         tile_lr                          ,
    output  [8 : 0]         weight_x                         ,
    output  [8 : 0]         weight_y
);
wire    [5 : 0]         raw_tile_ul                     ;
wire    [5 : 0]         raw_tile_ur                     ;
wire    [5 : 0]         raw_tile_ll                     ;
wire    [5 : 0]         raw_tile_lr                     ;


generate
    if(DONOR_MODE && PIC_WIDTH == 1920 && PIC_HEIGHT == 1080)begin : g_first
        wire    [10 : 0]        x_intra                          ,
                                y_intra                          ;
        reg     [7 : 0]         offset_x_r                       ,
                                offset_y_r                       ;
        // The donor X weight is eight bits; do not let constant-bit pruning
        // rename the ROM and discard its ADS LUT mapping attribute.
        reg     [7 : 0]         x_weights[0 : 255]               /* synthesis syn_romstyle = "lut_rom" */;
        reg     [8 : 0]         y_weights[0 : 255]               /* synthesis syn_romstyle = "lut_rom" */;
        integer                 index                            ;
        initial begin
            for(index = 0; index < 256; index = index + 1)begin
                x_weights[index]                 =  ((index * 273) >> 8) & 255;
                y_weights[index]                 =  ((index * 243) >> 7) & 511;
            end
        end
        always@(posedge clk or negedge rst_n)begin
            if(!rst_n)begin
                offset_x_r                       <=  0;
                offset_y_r                       <=  0;
            end
            else begin
                offset_x_r                       <=  x_intra[7 : 0];
                offset_y_r                       <=  y_intra[7 : 0];
            end
        end
        clahe_tiles_first u_first(
            .x                              (pixel_x                         ),
            .y                              (pixel_y                         ),
            .tile_ul                        (raw_tile_ul                         ),
            .tile_ur                        (raw_tile_ur                         ),
            .tile_ll                        (raw_tile_ll                         ),
            .tile_lr                        (raw_tile_lr                         ),
            .x_intra                        (x_intra                         ),
            .y_intra                        (y_intra                         )
        );

        assign  weight_x                         =   {1'b0, x_weights[offset_x_r]};
        assign  weight_y                         =   y_weights[offset_y_r];
    end
    else begin : g_exact
        localparam  TILE_WIDTH                       =   PIC_WIDTH / 8;
        localparam  TILE_HEIGHT                      =   PIC_HEIGHT / 8;
        reg     [8 : 0]         x_weights[0 : 2 * TILE_WIDTH - 1] /* synthesis syn_romstyle = "lut_rom" */;
        reg     [8 : 0]         y_weights[0 : 2 * TILE_HEIGHT - 1] /* synthesis syn_romstyle = "lut_rom" */;
        reg     [2 : 0]         left_col                         ,
                                right_col                        ,
                                top_row                          ,
                                bottom_row                       ;
        reg     [8 : 0]         offset_x                         ,
                                offset_y                         ;
        reg     [8 : 0]         offset_x_r                       ,
                                offset_y_r                       ;
        always@(posedge clk or negedge rst_n)begin
            if(!rst_n)begin
                offset_x_r                       <=  0;
                offset_y_r                       <=  0;
            end
            else begin
                offset_x_r                       <=  offset_x;
                offset_y_r                       <=  offset_y;
            end
        end
        wire    [11 : 0]        twice_x                          ,
                                twice_y                          ;
        integer                 init_x                           ,
                                init_y                           ;
        integer                 column                           ,
                                row                              ;
        // The divisions are evaluated at elaboration, not implemented as dividers.

        initial begin
            for(init_x = 0; init_x < 2 * TILE_WIDTH; init_x = init_x + 1)
                x_weights[init_x]                =  (init_x * 256 + TILE_WIDTH) / (2 * TILE_WIDTH);
            for(init_y = 0; init_y < 2 * TILE_HEIGHT; init_y = init_y + 1)
                y_weights[init_y]                =  (init_y * 256 + TILE_HEIGHT) / (2 * TILE_HEIGHT);
        end

        assign  twice_x                          =   {pixel_x, 1'b0};
        assign  twice_y                          =   {pixel_y, 1'b0};

        always@(*)begin
            left_col                         =  0;
            right_col                        =  0;
            offset_x                         =  0;
            if(twice_x >= 2 * PIC_WIDTH - TILE_WIDTH)begin
                left_col                         =  7;
                right_col                        =  7;
            end
            else if(twice_x >= TILE_WIDTH)begin
                for(column = 0; column < 7; column = column + 1)
                    if(twice_x >=(2 * column + 1) * TILE_WIDTH)
                        left_col                         =  column[2 : 0];
                right_col                        =  left_col + 1'b1;
                offset_x                         =  twice_x - TILE_WIDTH - left_col * (2 * TILE_WIDTH);
            end
            top_row                          =  0;
            bottom_row                       =  0;
            offset_y                         =  0;
            if(twice_y >= 2 * PIC_HEIGHT - TILE_HEIGHT)begin
                top_row                          =  7;
                bottom_row                       =  7;
            end
            else if(twice_y >= TILE_HEIGHT)begin
                for(row = 0; row < 7; row = row + 1)
                    if(twice_y >=(2 * row + 1) * TILE_HEIGHT)
                        top_row                          =  row[2 : 0];
                bottom_row                       =  top_row + 1'b1;
                offset_y                         =  twice_y - TILE_HEIGHT - top_row * (2 * TILE_HEIGHT);
            end
        end

        assign  raw_tile_ul                          =   {top_row, left_col};
        assign  raw_tile_ur                          =   {top_row, right_col};
        assign  raw_tile_ll                          =   {bottom_row, left_col};
        assign  raw_tile_lr                          =   {bottom_row, right_col};
        assign  weight_x                         =   x_weights[offset_x_r];
        assign  weight_y                         =   y_weights[offset_y_r];
    end
endgenerate


always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        tile_ul                          <=  0;
        tile_ur                          <=  0;
        tile_ll                          <=  0;
        tile_lr                          <=  0;
    end
    else begin
        tile_ul                          <=  raw_tile_ul;
        tile_ur                          <=  raw_tile_ur;
        tile_ll                          <=  raw_tile_ll;
        tile_lr                          <=  raw_tile_lr;
    end
end

endmodule
