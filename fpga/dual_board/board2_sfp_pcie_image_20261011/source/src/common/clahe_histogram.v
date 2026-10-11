// Two synchronous RAM pages keep histogram accumulation and CDF reads separate.
// Read, increment and writeback stages retain one pixel per clock. Forwarding
// covers both consecutive bins and a bin revisited before its RAM write is read.
module clahe_histogram#(
    parameter   PIC_WIDTH                        =   1920,
    parameter   TILE_HEIGHT                      =   135
)(
    input                   clk                              ,
    input                   rst_n                            ,
    input                   clear                            ,
    input                   frame_reset                      ,
    input                   pre_de                           ,
    input   [7 : 0]         pre_y                            ,
    input   [10 : 0]        pixel_x                          ,
    input   [2 : 0]         tile_col                         ,
    output reg              row_done                         ,
    input   [2 : 0]         read_col                         ,
    input   [7 : 0]         read_bin                         ,
    output  [15 : 0]        read_count                       ,
    input                   clear_start                      ,
    output                  init_done
);

(* ram_style = "block" *)
reg     [15 : 0]        histogram_0[0 : 2047]           ;
(* ram_style = "block" *)
reg     [15 : 0]        histogram_1[0 : 2047]           ;
reg     [15 : 0]        ram_data_0                       ;
reg     [15 : 0]        ram_data_1                       ;
reg                     init_active                      ;
reg     [10 : 0]        init_addr                        ;
reg                     write_page                       ;
reg                     read_page                        ;
reg     [10 : 0]        tile_line                        ;
reg                     sample_valid                     ;
reg                     sample_page                      ;
reg     [10 : 0]        sample_addr                      ;
reg                     sample_row_end                   ;
reg                     pending_valid                    ;
reg                     pending_page                     ;
reg     [10 : 0]        pending_addr                     ;
reg     [15 : 0]        pending_count                    ;
reg                     pending_row_end                  ;
reg                     written_valid                    ;
reg                     written_page                     ;
reg     [10 : 0]        written_addr                     ;
reg     [15 : 0]        written_count                    ;
reg                     clear_active                     ;
reg                     clear_page                       ;
reg     [10 : 0]        clear_addr                       ;
wire    [10 : 0]        write_addr                       ;
wire    [10 : 0]        cdf_addr                         ;
wire    [10 : 0]        ram_address_0                    ;
wire    [10 : 0]        ram_address_1                    ;
wire    [15 : 0]        stored_count                     ;
wire                    forward_pending                  ;
wire                    forward_written                  ;
wire                    writeback_valid                  ;

assign  write_addr                       =   {tile_col, pre_y};
assign  cdf_addr                         =   {read_col, read_bin};
assign  ram_address_0                    =   write_page ? cdf_addr : write_addr;
assign  ram_address_1                    =   write_page ? write_addr : cdf_addr;
assign  stored_count                     =   sample_page ? ram_data_1 : ram_data_0;
assign  read_count                       =   read_page ? ram_data_1 : ram_data_0;
assign  init_done                        =   !init_active;
assign  forward_pending                  =   pending_valid && pending_page == sample_page &&
                                             pending_addr == sample_addr;
assign  forward_written                  =   written_valid && written_page == sample_page &&
                                             written_addr == sample_addr;
assign  writeback_valid                  =   pending_valid && !init_active &&
                                             !(clear_active && clear_page == pending_page);

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        init_active                      <=  1;
        init_addr                        <=  0;
        write_page                       <=  0;
        read_page                        <=  1;
        tile_line                        <=  0;
        sample_valid                     <=  0;
        sample_page                      <=  0;
        sample_addr                      <=  0;
        sample_row_end                   <=  0;
        pending_valid                    <=  0;
        pending_page                     <=  0;
        pending_addr                     <=  0;
        pending_count                    <=  0;
        pending_row_end                  <=  0;
        written_valid                    <=  0;
        written_page                     <=  0;
        written_addr                     <=  0;
        written_count                    <=  0;
        row_done                         <=  0;
        clear_active                     <=  0;
        clear_page                       <=  0;
        clear_addr                       <=  0;
    end
    else if(clear || frame_reset)begin
        init_active                      <=  1;
        init_addr                        <=  0;
        write_page                       <=  0;
        read_page                        <=  1;
        tile_line                        <=  0;
        sample_valid                     <=  0;
        sample_row_end                   <=  0;
        pending_valid                    <=  0;
        pending_row_end                  <=  0;
        written_valid                    <=  0;
        row_done                         <=  0;
        clear_active                     <=  0;
    end
    else begin
        row_done                         <=  0;
        read_page                        <=  !write_page;
        sample_valid                     <=  pre_de && init_done;
        sample_row_end                   <=  0;
        pending_valid                    <=  sample_valid;
        pending_row_end                  <=  sample_row_end;
        written_valid                    <=  writeback_valid;

        if(init_active)begin
            if(init_addr == 2047)
                init_active                      <=  0;
            else
                init_addr                        <=  init_addr + 1'b1;
        end

        if(pre_de && init_done)begin
            sample_page                      <=  write_page;
            sample_addr                      <=  write_addr;
            if(pixel_x == PIC_WIDTH - 1)begin
                if(tile_line == TILE_HEIGHT - 1)begin
                    tile_line                        <=  0;
                    sample_row_end                   <=  1;
                end
                else
                    tile_line                        <=  tile_line + 1'b1;
            end
        end

        if(sample_valid)begin
            pending_page                     <=  sample_page;
            pending_addr                     <=  sample_addr;
            pending_count                    <=  (forward_pending ? pending_count :
                                                  forward_written ? written_count : stored_count) + 1'b1;
        end

        if(writeback_valid)begin
            written_page                     <=  pending_page;
            written_addr                     <=  pending_addr;
            written_count                    <=  pending_count;
            if(pending_row_end)begin
                // Publish only after the last increment reaches the RAM.
                // The video line blank contains the two pending pipeline clocks.
                write_page                       <=  !write_page;
                row_done                         <=  1;
            end
        end

        if(clear_start)begin
            clear_active                     <=  1;
            clear_page                       <=  !write_page;
            clear_addr                       <=  0;
        end
        else if(clear_active)begin
            if(clear_addr == 2047)
                clear_active                     <=  0;
            else
                clear_addr                       <=  clear_addr + 1'b1;
        end
    end
end

// One synchronous read and one write per page infer block RAM. Registered
// read-page selection follows the same address edge as both RAM outputs.
always@(posedge clk)begin
    if(!clear && !frame_reset)begin
        ram_data_0                       <=  histogram_0[ram_address_0];
        if(init_active)
            histogram_0[init_addr]          <=  0;
        else if(clear_active && !clear_page)
            histogram_0[clear_addr]         <=  0;
        else if(pending_valid && !pending_page)
            histogram_0[pending_addr]       <=  pending_count;
    end
end

always@(posedge clk)begin
    if(!clear && !frame_reset)begin
        ram_data_1                       <=  histogram_1[ram_address_1];
        if(init_active)
            histogram_1[init_addr]          <=  0;
        else if(clear_active && clear_page)
            histogram_1[clear_addr]         <=  0;
        else if(pending_valid && pending_page)
            histogram_1[pending_addr]       <=  pending_count;
    end
end

`ifndef SYNTHESIS

always@(posedge clk)begin
    if(rst_n && !clear && !frame_reset && pre_de &&
            (sample_row_end || pending_row_end))
        $fatal(1, "CLAHE histogram requires two blank clocks at each tile-row boundary");
end

`endif

endmodule
