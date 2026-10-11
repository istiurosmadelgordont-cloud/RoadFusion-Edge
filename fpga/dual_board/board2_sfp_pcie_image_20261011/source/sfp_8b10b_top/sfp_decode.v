
module sfp_decode#(
    parameter   RASTER_OUTPUT                   =   0,
    parameter   PIC_WIDTH                       =   1920,
    parameter   PIC_HEIGHT                      =   1080,
    parameter   H_BLANK                         =   280,
	parameter VS_POSE_DATA1	=	32'h55a101bc,
	parameter VS_POSE_DATA2	=	32'h55a102bc,		
	parameter DATA_START1   =	32'h55a105bc,
	parameter DATA_START2   =	32'h55a106bc,
	parameter DATA_END1     =	32'h55a107bc,
	parameter DATA_END2	    =	32'h55a108bc,
	parameter UNUSE_DATA   	=	32'h55a109bc

)	
(
	input			    clk_in /* synthesis syn_keep="1" */,
	input               rst_n,
	output  reg         o_vs/* synthesis syn_keep="1" */,           //场输出信号
	output  reg     	data_valid_out,   //数据输出有效信号
	output      [15:0]  data_out,         //数据输出信号
	
	input    		    rx_clk/* synthesis syn_keep="1" */,
	input               rx_rst_n,
	input       [3:0]   rx_charisk_align/* synthesis syn_keep="1" */, //rx端K码校正后接收信号
	input       [31:0]  rx_data_align/* synthesis syn_keep="1" */     //rx端校正后发送数据 
	
);
//reg define 
reg 		sfp_line_end_t;
reg 		sfp_line_end_t1;

reg [7:0]	data_end_en_d;
reg 		sfp_line_end;
reg [7:0]	cnt_vs;	
reg 		data_start_en/* synthesis syn_keep="1" */;	
reg 		data_end_en/* synthesis syn_keep="1" */;
reg 		fifo_wren;	
reg [31:0]	fifo_datain;
wire        fifo_rd_en;
reg         rx_frame_reset;
reg         frame_reset_sync0;
reg         frame_reset_sync1;
reg         frame_reset;
reg         fifo_read_valid;

reg [31:0]	rx_data_align_d0;
reg [3:0]	rx_charisk_align_d0;
reg [31:0]	rx_data_align_d1;
reg [3:0]	rx_charisk_align_d1;
reg [3:0]	data_start_en_d;

//wire define 	
wire [12:0]  rd_water_level;
wire         fifo_empty;
wire         fifo_almost_empty;
	
//*****************************************************
//**                    main code
//*****************************************************

//对输入的数据进行打拍
always@(posedge rx_clk or negedge rx_rst_n)begin
	if(!rx_rst_n)begin		
		rx_charisk_align_d0 <= 1'b0;
		rx_data_align_d0 <= 16'd0;	
		rx_charisk_align_d1 <= 1'b0;
		rx_data_align_d1 <= 16'd0;							
	end
	else begin		
		rx_charisk_align_d0 <= rx_charisk_align;
		rx_data_align_d0 <= rx_data_align;		
		rx_charisk_align_d1 <= rx_charisk_align_d0;
		rx_data_align_d1 <= rx_data_align_d0;					
	end
end	

//对数据接收开始和结束信号进行移位
always@(posedge rx_clk or negedge rx_rst_n)begin
	if(!rx_rst_n)begin	
		data_end_en_d <= 8'd0;
		data_start_en_d <= 4'd0;
	end						
	else begin
		data_end_en_d <= {data_end_en_d[6:0],data_end_en};
		data_start_en_d <= {data_start_en_d[2:0],data_start_en};			
	end			
end	

//产生一行数据接收结束信号
always@(posedge rx_clk or negedge rx_rst_n)begin
	if(!rx_rst_n)	
		sfp_line_end <= 1'd0;					
	else if(data_end_en_d >0)
		sfp_line_end <= 1'd1;
    else
		sfp_line_end <= 1'd0;	
end	

// RX 域检测场同步标志，并产生 RX FIFO 写侧复位。
always@(posedge rx_clk or negedge rx_rst_n)begin
	if(!rx_rst_n)begin		
		rx_frame_reset <= 1'b0;	
		cnt_vs <= 8'h0;		
	end	
	else if((rx_charisk_align == 4'd1) && (rx_data_align == VS_POSE_DATA2) 
		&& ((rx_data_align_d0 == VS_POSE_DATA1)))begin
		rx_frame_reset <= 1'b1;	
		cnt_vs <= 8'h0;
	end
	else if(rx_frame_reset && (cnt_vs >= 100))begin
			rx_frame_reset <= 1'b0;	
			cnt_vs <= cnt_vs;	
	end			
	else if(rx_frame_reset)begin
			rx_frame_reset <= rx_frame_reset;
			cnt_vs <= cnt_vs + 1;			
	end
	else begin
			rx_frame_reset <= 1'b0;
			cnt_vs <= cnt_vs;
	end
end		

// 将场同步/复位电平同步到 clk_in 域，保证 o_vs 与 data_out 同域。
always@(posedge clk_in or negedge rst_n)begin
	if(!rst_n)begin
		frame_reset_sync0 <= 1'b0;
		frame_reset_sync1 <= 1'b0;
        frame_reset <= 1'b0;
        o_vs        <= 1'b0;
	end
	else begin
		frame_reset_sync0 <= rx_frame_reset;
		frame_reset_sync1 <= frame_reset_sync0;
        frame_reset <= frame_reset_sync1;
        o_vs        <= frame_reset;
	end
end
	
//产生数据接收开始信号	
always@(posedge rx_clk or negedge rx_rst_n)begin
	if(!rx_rst_n)begin		
		data_start_en <= 1'b0;		
	end	
	else if((rx_charisk_align == 4'd1) && (rx_data_align == DATA_START2) 
		&& ((rx_data_align_d0 == DATA_START1)))begin
		data_start_en <= 1'b1;
	end		
	else begin
		data_start_en <= 1'b0;		
	end		
end	

//产生数据接收结束信号		
always@(posedge rx_clk or negedge rx_rst_n)begin
	if(!rx_rst_n)begin		
		data_end_en <= 1'b0;		
	end	
	else if((rx_charisk_align == 4'd1) && (rx_data_align == DATA_END2) 
		&& ((rx_data_align_d0 == DATA_END1)))begin
		data_end_en <= 1'b1;
	end		
	else begin
		data_end_en <= 1'b0;		
	end		
end	

//产生fifo写使能	
always@(posedge rx_clk or negedge rx_rst_n)begin
	if(!rx_rst_n)	
		fifo_wren <= 1'b0;		
	else if(data_start_en_d[1]) 
		fifo_wren <= 1'b1;	
	else if(data_end_en) 
		fifo_wren <= 1'b0;		
	else 
		fifo_wren <= fifo_wren;			
end	

//产生fifo写数据
always@(posedge rx_clk or negedge rx_rst_n)begin
	if(!rx_rst_n)	
		fifo_datain <= 32'b0;		
	else 
		fifo_datain <= rx_data_align_d1;					
end		
	
/*****************读数据*****************************/
	
//对一行数据接收结束信号进行时钟域切换
always@(posedge clk_in or negedge rst_n)begin
	if(!rst_n || frame_reset)begin	
		sfp_line_end_t <= 1'b0;	
		sfp_line_end_t1 <= 1'b0;			
    end		
	else begin
		sfp_line_end_t  <= sfp_line_end;
		sfp_line_end_t1 <= sfp_line_end_t;		
    end		
end	

//产生fifo读使能
generate
    if(RASTER_OUTPUT)begin : g_raster_output
        wire    frame_error /* synthesis PAP_MARK_DEBUG="1" */;

        sfp_raster_read_ctrl#(
            .PIC_WIDTH                      (PIC_WIDTH                       ),
            .PIC_HEIGHT                     (PIC_HEIGHT                      ),
            .H_BLANK                        (H_BLANK                         ),
            // 3456/4096 startup pixels: absorb read-clock drift within a full frame.
            .PREFILL_PIXELS                 (1536                            )
        )u_raster_read_ctrl(
            .clk                            (clk_in                          ),
            .rst_n                          (rst_n                           ),
            .frame_reset                    (frame_reset                     ),
            .fifo_level                     (rd_water_level                  ),
            .fifo_empty                     (fifo_empty                      ),
            .fifo_read                      (fifo_rd_en                      ),
            .frame_error                    (frame_error                     )
        );
    end
    else begin : g_legacy_output
        reg     legacy_fifo_rd_en;

        assign  fifo_rd_en = legacy_fifo_rd_en;
        always@(posedge clk_in or negedge rst_n)begin
        	if(!rst_n || frame_reset)
        		legacy_fifo_rd_en <= 1'b0;					
        	else begin
        		if(sfp_line_end_t1)	
        			legacy_fifo_rd_en <= 1'b1;	
                else if(rd_water_level <= 1)
        			legacy_fifo_rd_en <= 1'b0;	
                else
        			legacy_fifo_rd_en <= legacy_fifo_rd_en;			
            end		
        end
    end
endgenerate	

//产生数据输出有效信号
// The DRM read output register adds one clock. Delay DE by the same clock;
// internal raster/reset decisions retain their original frame_reset timing.
always@(posedge clk_in or negedge rst_n)begin
    if(!rst_n || frame_reset)begin
        fifo_read_valid <= 1'b0;
        data_valid_out  <= 1'b0;
    end
    else begin
        fifo_read_valid <= fifo_rd_en && !fifo_empty;
        data_valid_out  <= fifo_read_valid;
    end
end

wire  ful    /* synthesis syn_keep="1" */ ;
// The generated wrapper fixes OUTPUT_REG=0 as a localparam. Instantiate its
// existing IPML core with the same geometry/flags and enable the actual DRM
// output register. Keep the original named hierarchy for debug/constraints.
generate
    if(1)begin : u_sfp_rx_32x2048
        ipml_fifo_v1_9_sfp_rx_32x2048#(
            .c_SIM_DEVICE                   ("LOGOS"                         ),
            .c_WR_DEPTH_WIDTH               (11                              ),
            .c_WR_DATA_WIDTH                (32                              ),
            .c_RD_DEPTH_WIDTH               (12                              ),
            .c_RD_DATA_WIDTH                (16                              ),
            .c_OUTPUT_REG                   (1                               ),
            .c_RD_OCE_EN                    (0                               ),
            .c_RESET_TYPE                   ("ASYNC_RESET"                   ),
            .c_POWER_OPT                    (0                               ),
            .c_RD_CLK_OR_POL_INV            (0                               ),
            .c_WR_BYTE_EN                   (0                               ),
            .c_BE_WIDTH                     (1                               ),
            .c_FIFO_TYPE                    ("ASYN"                          ),
            .c_ALMOST_FULL_NUM              (2000                            ),
            .c_ALMOST_EMPTY_NUM             (4                               )
        )U_ipml_fifo_sfp_rx_32x2048(
            .wr_clk                         (rx_clk                          ),
            .wr_rst                         (rx_frame_reset                  ),
            .wr_en                          (fifo_wren && !ful               ),
            .wr_data                        (fifo_datain                     ),
            .wr_full                        (ful                             ),
            .wr_byte_en                     (1'b1                            ),
            .wr_water_level                 (                                ),
            .almost_full                    (                                ),
            .rd_clk                         (clk_in                          ),
            // Reset the data register with its matching delayed external VS.
            .rd_rst                         (o_vs                            ),
            .rd_en                          (fifo_rd_en && !fifo_empty       ),
            .rd_data                        (data_out                        ),
            .rd_oce                         (1'b1                            ),
            .rd_empty                       (fifo_empty                      ),
            .rd_water_level                 (rd_water_level                  ),
            .almost_empty                   (fifo_almost_empty               )
        );
    end
endgenerate

endmodule
