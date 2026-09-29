module hdmi_ddr_pcie #(
  parameter PCIE_ENABLE          = 1                   ,//PCIE模块例化，0不例化，1例化
  parameter PROJECT_MODE         = 1                    ,//PROJECT_MODE 0:仿真；>=1：上板子；2：使用pcie
  parameter VIDEO_LENGTH         = 1920                 ,
  parameter VIDEO_HIGTH          = 1080                 ,
  parameter PIXEL_WIDTH          = 16                 ,    
  parameter MEM_ROW_ADDR_WIDTH   = 15                 ,
  parameter MEM_COL_ADDR_WIDTH   = 10                 ,
  parameter MEM_BADDR_WIDTH      = 3                  ,
  parameter MEM_DQ_WIDTH         = 32                 ,
  parameter MEM_DM_WIDTH         = MEM_DQ_WIDTH/8     ,
  parameter MEM_DQS_WIDTH        = MEM_DQ_WIDTH/8     ,
  parameter M_AXI_BRUST_LEN      = 8                  ,
  parameter RW_ADDR_MIN          = 22'b0              ,
  parameter RW_ADDR_MAX          = VIDEO_LENGTH*VIDEO_HIGTH*PIXEL_WIDTH/MEM_DQ_WIDTH       ,//@540p  518400个地址   
  parameter CTRL_ADDR_WIDTH      = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH
)(
  input                                  ref_clk         ,
  input                                  rst_board       /* synthesis syn_keep="1" */,
  output                                 ddr_pll_lock        ,           
  output                                 ddr_init_done   ,
  //DDR 
  output                                 mem_rst_n       ,                       
  output                                 mem_ck          ,
  output                                 mem_ck_n        ,
  output                                 mem_cke         ,
  output                                 mem_cs_n        ,
  output                                 mem_ras_n       ,
  output                                 mem_cas_n       ,
  output                                 mem_we_n        ,  
  output                                 mem_odt         ,
  output     [MEM_ROW_ADDR_WIDTH-1:0]    mem_a           ,   
  output     [MEM_BADDR_WIDTH-1:0]       mem_ba          ,   
  inout      [MEM_DQS_WIDTH-1:0]         mem_dqs         ,
  inout      [MEM_DQS_WIDTH-1:0]         mem_dqs_n       ,
  inout      [MEM_DQ_WIDTH-1:0]          mem_dq          ,
  output     [MEM_DM_WIDTH-1:0]          mem_dm          ,
//MS72XX配置
  output wire                               hdmi_rst   ,
  output                                   iic_tx_scl        ,
  inout                                    iic_tx_sda        ,
  output                                   iic_scl            ,
  inout                                    iic_sda            ,
  output wire                              hdmi_int_led      ,
  output wire                              fram_done         ,
//HDMI IN
  input wire                               pix_clk_in      ,//HDMI输入时钟 1080p @148.5Mhz
  input wire                               vs_in           /* synthesis PAP_MARK_DEBUG="1" */,//帧同步
  input wire                               hs_in           ,//行同步
  input wire                               de_in           /* synthesis PAP_MARK_DEBUG="1" */,//数据有效信号
  input wire [7 : 0]                       r_in            /* synthesis PAP_MARK_DEBUG="1" */,
  input wire [7 : 0]                       g_in            /* synthesis PAP_MARK_DEBUG="1" */,
  input wire [7 : 0]                       b_in            /* synthesis PAP_MARK_DEBUG="1" */,
//pcie
	input  wire                          ref_clk_n         ,
	input  wire                          ref_clk_p         ,
	input  wire                          rxn               ,
	input  wire                          rxp               ,
	output wire                          txn               ,
	output wire                          txp               ,
	input  wire                          pcie_perst_n      ,
	output wire							 smlh_link_up	   ,
    output wire							 rdlh_link_up	   ,	
//HDMI_OUT
    output                               pix_clk_out                   ,//pixclk                           
    output     reg                       vs_out                    , 
    output     reg                       hs_out                    , 
    output     reg                       de_out                    ,
    output     reg[7:0]                  r_out                     , 
    output     reg[7:0]                  g_out                     , 
    output     reg[7:0]                  b_out         			   ,
//光口接口
	input               					 i_p_refckn_0,              //差分参考时钟
	input               					 i_p_refckp_0,
	input               					 i_p_l2rxn,                 //差分接收数据
	input               					 i_p_l2rxp,
	input               					 i_p_l3rxn,                   
	input               					 i_p_l3rxp,
	output              					 o_p_l2txn,                 //差分发送数据
	output              					 o_p_l2txp,
	output              					 o_p_l3txn,
	output              					 o_p_l3txp,
	output  	 [1 : 0]      				 tx_disable	             //tx端发送禁止使能

   );
parameter DQ_WIDTH = MEM_DQ_WIDTH; 
wire 									iic_clk;//10mhz
wire 									pll_init_done; 
reg   [15:0]							rstn_1ms       ;
wire                                   ddr_ip_clk      /* synthesis PAP_MARK_DEBUG="1" */;
wire                                   ddr_ip_rst_n    /* synthesis PAP_MARK_DEBUG="1" */;   
//写地址通道↓                                                  
wire [3 : 0]                           M_AXI_AWID     /* synthesis PAP_MARK_DEBUG="1" */;
wire [CTRL_ADDR_WIDTH-1 : 0]           M_AXI_AWADDR   /* synthesis PAP_MARK_DEBUG="1" */;
//wire [3 : 0]                           M_AXI_AWLEN    /* synthesis PAP_MARK_DEBUG="1" */;
wire                                   M_AXI_AWUSER   /* synthesis PAP_MARK_DEBUG="1" */;
wire                                   M_AXI_AWVALID   /* synthesis PAP_MARK_DEBUG="1" */;
wire                                   M_AXI_AWREADY   /* synthesis PAP_MARK_DEBUG="1" */;
//写数据通道↓                                                 
wire [DQ_WIDTH*8-1 : 0]                M_AXI_WDATA    /* synthesis PAP_MARK_DEBUG="1" */;
wire [DQ_WIDTH-1 : 0]                  M_AXI_WSTRB    /* synthesis PAP_MARK_DEBUG="1" */;
wire                                   M_AXI_WLAST    /* synthesis PAP_MARK_DEBUG="1" */;
wire [3 : 0]                           M_AXI_WUSER    /* synthesis PAP_MARK_DEBUG="1" */;
wire                                   M_AXI_WREADY   /* synthesis PAP_MARK_DEBUG="1" */;                                                
//读地址通道↓                                                 
wire [3 : 0]                           M_AXI_ARID     /* synthesis PAP_MARK_DEBUG="1" */;
wire                                   M_AXI_ARUSER   /* synthesis PAP_MARK_DEBUG="1" */;
wire [CTRL_ADDR_WIDTH-1 : 0]           M_AXI_ARADDR   /* synthesis PAP_MARK_DEBUG="1" */;
//wire [3 : 0]                           M_AXI_ARLEN    /* synthesis PAP_MARK_DEBUG="1" */;
wire                                   M_AXI_ARVALID   /* synthesis PAP_MARK_DEBUG="1" */;
wire                                   M_AXI_ARREADY   /* synthesis PAP_MARK_DEBUG="1" */;
//读数据通道↓                                                
wire  [3 : 0]                          M_AXI_RID      /* synthesis PAP_MARK_DEBUG="1" */;
wire  [DQ_WIDTH*8-1 : 0]               M_AXI_RDATA    /* synthesis PAP_MARK_DEBUG="1" */;
wire                                   M_AXI_RLAST    /* synthesis PAP_MARK_DEBUG="1" */;
wire                                   M_AXI_RVALID   /* synthesis PAP_MARK_DEBUG="1" */;
//debug控制线
wire  [1 : 0 ]                         init_read_clk_ctrl;
wire  [3 : 0 ]                         init_slip_step;
wire                                   force_read_clk_ctrl; 
wire[15 : 0] video_data_out/* synthesis PAP_MARK_DEBUG="1" */;
always @(posedge iic_clk)begin
	if(!pll_init_done)
	    rstn_1ms <= 16'd0;
	else if(!ddr_init_done) begin
  	    rstn_1ms <= 16'd0;  
    end
	else if(!rst_board) begin
  	    rstn_1ms <= 16'd0;  
    end
	else begin
		if(rstn_1ms == 16'h2710)
		    rstn_1ms <= rstn_1ms;
		else
		    rstn_1ms <= rstn_1ms + 1'b1;
	end
end
assign hdmi_rst = (rstn_1ms == 16'h2710);
pll pll_inst1 (
  .clkin1(ref_clk),        // input
  .pll_lock(pll_init_done),    // output
  .clkout0(iic_clk),      // output
  .clkout1(pix_clk_out)       // output
);
hdmi_ctrl user_hdmi_ctrl(
    .clk         (  iic_clk    ), //input       clk,
    .rst_n       (  hdmi_rst   ), //input       rstn, 
    .init_over   (  hdmi_int_led  ), //output      init_over,
    .iic_tx_scl  (  iic_tx_scl ), //output      iic_scl,
    .iic_tx_sda  (  iic_tx_sda ), //inout       iic_sda
    .iic_scl     (  iic_scl    ), //output      iic_scl,
    .iic_sda     (  iic_sda    )  //inout       iic_sda
);  
ipsxb_rst_sync_v1_1 u_core_clk_rst_sync(
    .clk                        (ref_clk        ),
    .rst_n                      (rst_board       ),
    .sig_async                  (1'b1),
    .sig_synced                 (ddr_ip_rst_n   )
);

wire  [15:0] post_rgb ;
// 当前 HSST IP 仅启用 Lane2 全双工；保留旧顶层 Lane3 端口但固定为无效。
assign o_p_l3txn = 1'b0;
assign o_p_l3txp = 1'b0;

wire        sfp_tx_user_clk;
wire        sfp_tx_rst_n;
wire [3:0]  sfp_gt_txcharisk;
wire [31:0] sfp_gt_txdata;
wire        sfp_rx_user_clk;
wire        sfp_rx_rst_n;
wire [3:0]  sfp_gt_rxcharisk;
wire [31:0] sfp_gt_rxdata;
wire        sfp_pll_rst;
wire        sfp_wtchdg_clr;


//光口传输顶层模块   
sfp_8b10b_top u_sfp_8b10b_top(
    .tx_disable                 (tx_disable         ),
	.tx0_user_clk                (sfp_tx_user_clk    ),
	.tx0_rst_n                   (sfp_tx_rst_n       ),
	.tx0_gt_txcharisk            (sfp_gt_txcharisk   ),
	.tx0_gt_txdata               (sfp_gt_txdata      ),
	.rx1_user_clk                (sfp_rx_user_clk    ),
	.rx1_rst_n                   (sfp_rx_rst_n       ),
	.rx1_gt_rxcharisk            (sfp_gt_rxcharisk   ),
	.rx1_gt_rxdata               (sfp_gt_rxdata      ),
	.i_pll_rst_0                 (sfp_pll_rst        ),
	.i_wtchdg_clr_0              (sfp_wtchdg_clr     ),
	//用户接口
	.clk_in     	   			(pix_clk_in         ),      //发送端输入时钟
	.rx_clk_in                  (pix_clk_out        ),      //接收解码使用本板 PLL 像素时钟
	.rst_n						(rst_board          ),      //输入复位
    .h_pixel                    (VIDEO_LENGTH       ),  //输入行像素
	.vs_in						(vs_in   ),      //输入场信号
	.data_valid_in 				(de_in   ),      //输入数据有效信号
	.data_in					({r_in[7:3],g_in[7:2],b_in[7:3]}            ),      //输入数据

	.o_vs						(post_frame_vsync   ),      //输出场信号
	.data_valid_out				(post_frame_de      ),	    //输出数据有效信号
	.data_out	   				(post_rgb           )       //输出数据
);  





ddr3_ctrl_top #(
    .VIDEO_LENGTH     (VIDEO_LENGTH)                    ,
    .VIDEO_HIGTH      (VIDEO_HIGTH)                     ,
    .PIXEL_WIDTH      (PIXEL_WIDTH  )                   ,
	.CTRL_ADDR_WIDTH  (CTRL_ADDR_WIDTH  )               ,
	.DQ_WIDTH	     (DQ_WIDTH  )                       ,
    .M_AXI_BRUST_LEN  (M_AXI_BRUST_LEN   )
)
ddr3_ctrl_top (
	.DDR_INIT_DONE           (ddr_init_done),
	.M_AXI_ACLK              (ddr_ip_clk   ),
	.M_AXI_ARESETN           (ddr_ip_rst_n  && ddr_init_done),
     .pix_clk_out             (pix_clk_out),//1080p 148.5m
      
	//写地址通道↓                                                              
	.M_AXI_AWID              (M_AXI_AWID   ),
	.M_AXI_AWADDR            (M_AXI_AWADDR ),
//	.M_AXI_AWLEN             (),
	.M_AXI_AWUSER            (M_AXI_AWUSER ),
	.M_AXI_AWVALID           (M_AXI_AWVALID),
	.M_AXI_AWREADY           (M_AXI_AWREADY),
	//写数据通道↓                                                              
	.M_AXI_WDATA             (M_AXI_WDATA),
	.M_AXI_WSTRB             (M_AXI_WSTRB),
	.M_AXI_WLAST             (M_AXI_WLAST),
	.M_AXI_WUSER             (M_AXI_WUSER),
	.M_AXI_WREADY            (M_AXI_WREADY),
                                                             
	//读地址通道↓                                                              
	.M_AXI_ARID              (M_AXI_ARID),
    .M_AXI_ARUSER            (M_AXI_ARUSER),
	.M_AXI_ARADDR            (M_AXI_ARADDR),
//	.M_AXI_ARLEN             (),
	.M_AXI_ARVALID           (M_AXI_ARVALID),
	.M_AXI_ARREADY           (M_AXI_ARREADY),
	//读数据通道↓                                                              
	.M_AXI_RID               (M_AXI_RID   ),
	.M_AXI_RDATA             (M_AXI_RDATA ),
	.M_AXI_RLAST             (M_AXI_RLAST ),
	.M_AXI_RVALID            (M_AXI_RVALID),
//video
    .vs_in                   (post_frame_vsync       ),
    .vs_out                  (vs_o      ),
//fifo0信号          
    .video_clk_in           (pix_clk_out),                                                                                                                  
    .video_de_in            (post_frame_de    ),
    .video_data_in          ({post_rgb[15:11],post_rgb[10:5],post_rgb[4:0]}  ),
    .video_rd_en            (de_o   ),
    .video_data_out         (video_data_out),
    .fram_done              (fram_done     ),
    .video_vs_in            (post_frame_vsync ),//用于抓取复位
	.wr_addr_min             (RW_ADDR_MIN),//写数据ddr最小地址0地址开始算，1920*1080*16 = 33177600 bits
    .wr_addr_max             (RW_ADDR_MAX) //写数据ddr最大地址，一个地址存32位 33177600/32 = 1036800 = 20'b1111_1101_0010_0000_0000
);

reg vs   ;
reg hs   ;
reg de   ;

     always@(posedge pix_clk_out) begin
        r_out<={video_data_out[15:11],3'b0};
        g_out<={video_data_out[10:5],2'b0};
        b_out<={video_data_out[4:0],3'b0}; 
        vs<=vs_o;
        hs<=hs_o;
        de<=de_o;
		vs_out<=vs;
        hs_out<=hs;
        de_out<=de;
     end 
sync_generator user_sync_gen(
    .clk       (pix_clk_out ),//1080p参考时钟为148.5mhz
    .rstn      (ddr_ip_rst_n && ddr_init_done && fram_done),
    .vs_out    (vs_o),
    .hs_out    (hs_o),
    .de_out    (de_o),
    .de_re     (),
    .x_act     (x_act),
    .y_act     (y_act)
);	
	//ddr IP例化
ddr_test  #
  (
   //***************************************************************************
   // The following parameters are Memory Feature
   //***************************************************************************
   .MEM_ROW_WIDTH          (MEM_ROW_ADDR_WIDTH),     //行宽度？
   .MEM_COLUMN_WIDTH       (MEM_COL_ADDR_WIDTH),     //列宽度？
   .MEM_BANK_WIDTH         (MEM_BADDR_WIDTH   ),     //bank宽度？
   .MEM_DQ_WIDTH           (MEM_DQ_WIDTH      ),     //数据宽度
   .MEM_DM_WIDTH           (MEM_DM_WIDTH      ),     
   .MEM_DQS_WIDTH          (MEM_DQS_WIDTH     ),     
   .CTRL_ADDR_WIDTH        (CTRL_ADDR_WIDTH   )     //地址宽度 = 行宽度+列宽度+bank地址？不太理解
  )
  I_ipsxb_ddr_top(
   .ref_clk                (ref_clk                ),
   .resetn                 (ddr_ip_rst_n           ),
   .ddr_init_done          (ddr_init_done          ),
   .ddrphy_clkin           (ddr_ip_clk             ),
   .pll_lock               (ddr_pll_lock               ), 
    //写地址
   .axi_awaddr             (M_AXI_AWADDR           ),//写地址
   .axi_awuser_ap          (M_AXI_AWUSER           ),//precharge？
   .axi_awuser_id          (M_AXI_AWID             ),
   .axi_awlen              (M_AXI_BRUST_LEN        ),//突发长度
   .axi_awready            (M_AXI_AWREADY          ),//out,从机awready
   .axi_awvalid            (M_AXI_AWVALID          ),//主机awvalid
    //写数据
   .axi_wdata              (M_AXI_WDATA            ),//位宽为DQ_WIDTH*8  迷惑 这里为什么是32*8？
   .axi_wstrb              (M_AXI_WSTRB            ),
   .axi_wready             (M_AXI_WREADY           ),//OUT 从机wready 这里没用valid 所以主机收到ready继续发送就行
   .axi_wusero_id          (M_AXI_WUSER            ),
   .axi_wusero_last        (M_AXI_WLAST            ),//out 从机写last
    //读地址                   
   .axi_araddr             (M_AXI_ARADDR           ),
   .axi_aruser_ap          (M_AXI_ARUSER           ),
   .axi_aruser_id          (M_AXI_ARID             ),
   .axi_arlen              (M_AXI_BRUST_LEN        ),
   .axi_arready            (M_AXI_ARREADY          ),
   .axi_arvalid            (M_AXI_ARVALID          ),
    //读数据
   .axi_rdata              (M_AXI_RDATA             ),
   .axi_rid                (M_AXI_RID            ),
   .axi_rlast              (M_AXI_RLAST            ),
   .axi_rvalid             (M_AXI_RVALID           ),

   .apb_clk                (1'b0                   ),
   .apb_rst_n              (1'b1                   ),
   .apb_sel                (1'b0                   ),
   .apb_enable             (1'b0                   ),
   .apb_addr               (8'b0                   ),
   .apb_write              (1'b0                   ),
   .apb_ready              (                       ),
   .apb_wdata              (16'b0                  ),
   .apb_rdata              (                       ),
   .apb_int                (                       ),
   .debug_data             (                       ),
   .debug_slice_state      (                       ),
   .debug_calib_ctrl       (                       ),
   .ck_dly_set_bin         (                       ),
   .force_ck_dly_en        (1'b0                   ),
   .force_ck_dly_set_bin   (8'h05                  ),
   .dll_step               (                       ),
   .dll_lock               (                       ),
   .init_read_clk_ctrl     (2'b0                   ),                                                       
   .init_slip_step         (4'b0                   ), 
   .force_read_clk_ctrl    (1'b0                   ),  
   .ddrphy_gate_update_en  (1'b0                   ),
   .update_com_val_err_flag(                       ),
   .rd_fake_stop           (1'b0                   ),
    //内存接口
   .mem_rst_n              (mem_rst_n              ),
   .mem_ck                 (mem_ck                 ),
   .mem_ck_n               (mem_ck_n               ),
   .mem_cke                (mem_cke                ),
   .mem_cs_n               (mem_cs_n               ),
   .mem_ras_n              (mem_ras_n              ),
   .mem_cas_n              (mem_cas_n              ),
   .mem_we_n               (mem_we_n               ),
   .mem_odt                (mem_odt                ),
   .mem_a                  (mem_a                  ),
   .mem_ba                 (mem_ba                 ),
   .mem_dqs                (mem_dqs                ),
   .mem_dqs_n              (mem_dqs_n              ),
   .mem_dq                 (mem_dq                 ),
   .mem_dm                 (mem_dm                 )
  );
wire         video_enhance_vs_out;
wire         video_enhance_hs_out;
wire         video_enhance_de_out;
wire [7 : 0] video_enhance_r_out;
wire [7 : 0] video_enhance_g_out;
wire [7 : 0] video_enhance_b_out;


  
  
video_enhance u_video_enhance(
.pix_clk(pix_clk_out),//input  wire            
.vs_in  (vs_out),//input  wire            
.hs_in  (),//input  wire            
.de_in  (de_out),//input  wire         zoom_de_out              
.r_in   (r_out),//input  wire [7 : 0] zoom_data_out[31 : 24]   
.g_in   (g_out),//input  wire [7 : 0] zoom_data_out[21 : 14]   
.b_in   (b_out),//input  wire [7 : 0] zoom_data_out[11 :  4]
   
.vs_out (video_enhance_vs_out  ),//output wire                               
.hs_out (video_enhance_hs_out  ),//output wire            
.de_out (video_enhance_de_out  ),//output wire            
.r_out  (video_enhance_r_out   ),//output wire [7 : 0]    
.g_out  (video_enhance_g_out   ),//output wire [7 : 0]    
.b_out  (video_enhance_b_out   ), //output wire [7 : 0]    
.video_enhance_lightdown_num (video_enhance_lightdown_num),//input wire [7 : 0]            
.video_enhance_lightdown_sw  (video_enhance_lightdown_sw ),//input wire                    
.video_enhance_darkup_num    (video_enhance_darkup_num   ),//input wire [7 : 0]            
.video_enhance_darkup_sw     (video_enhance_darkup_sw    )//input wire                            
   );  
  
  
  
  
wire [7  : 0]    video_enhance_lightdown_num;
wire             video_enhance_lightdown_sw ;
wire [7  : 0]    video_enhance_darkup_num   ;
wire             video_enhance_darkup_sw    ;

wire         axis_master_tvalid;
wire         axis_master_tready;
wire [127:0] axis_master_tdata/* synthesis PAP_MARK_DEBUG="1" */;
wire [3:0]   axis_master_tkeep;
wire         axis_master_tlast;
wire [7:0]   axis_master_tuser;

wire         axis_slave0_tready;
wire         axis_slave0_tvalid;
wire [127:0] axis_slave0_tdata;
wire         axis_slave0_tlast;
wire         axis_slave0_tuser;

wire         axis_slave1_tready;
wire         axis_slave1_tvalid;
wire [127:0] axis_slave1_tdata;
wire         axis_slave1_tlast;
wire         axis_slave1_tuser;

pcie_dma_ctrl u_pcie_dam_ctrl(
   .clk                (pclk_div2 ), //input wire   
   .pix_clk_out        (pix_clk_out),          
   // PCIE_FIX_20260912: DMA logic follows the PCIe core reset synchronized
   // into pclk_div2 instead of using the raw board reset.
   .rstn               (s_pclk_div2_rstn), //input              
    
   .axis_master_tvalid (axis_master_tvalid), //input wire                
   .axis_master_tready (axis_master_tready), //output wire               
   .axis_master_tdata  (axis_master_tdata), //input wire    [127:0]     
   .axis_master_tkeep  (axis_master_tkeep), //input wire    [3:0]       
   .axis_master_tlast  (axis_master_tlast), //input wire                
   .axis_master_tuser  (axis_master_tuser), //input wire    [7:0]       
 
   .ep_bus_num         (cfg_pbus_num), //input  [7 : 0]         
   .ep_dev_num         (cfg_pbus_dev_num), //input  [4 : 0] 
        
   .AXIS_S_TREADY      (axis_slave2_tready ), //input                  
   .AXIS_S_TVALID      (axis_slave2_tvalid ), //output                 
   .AXIS_S_TDATA       (axis_slave2_tdata  ), //output [127:0]         
   .AXIS_S_TLAST       (axis_slave2_tlast  ), //output                 
   .AXIS_S_TUSER       (axis_slave2_tuser  ), //output 
   .hdmi_data_in       ({video_enhance_r_out[7:3],video_enhance_g_out[7:2],video_enhance_b_out[7:3]}      ), // input 32bits
   .vs_in              (video_enhance_vs_out             ),                   
   .de_in              (video_enhance_de_out            ) ,
   .video_enhance_lightdown_num (video_enhance_lightdown_num),// output reg [7 : 0]        
   .video_enhance_lightdown_sw  (video_enhance_lightdown_sw ),// output reg                
   .video_enhance_darkup_num    (video_enhance_darkup_num   ),// output reg [7 : 0]        
   .video_enhance_darkup_sw     (video_enhance_darkup_sw    ) // output reg            
   );


//----------------------------------------------------------rst debounce ----------------------------------------------------------
//ASYNC RST  define IPSL_PCIE_SPEEDUP_SIM when simulation
hsst_rst_cross_sync_v1_0 #(
    `ifdef IPSL_PCIE_SPEEDUP_SIM
    .RST_CNTR_VALUE     (16'h10             )
    `else
    .RST_CNTR_VALUE     (16'hC000           )
    `endif
)
u_refclk_buttonrstn_debounce(
    .clk                (pcie_ref_clk            ),
    .rstn_in            (rst_board       ),
    .rstn_out           (sync_button_rst_n  )
);

hsst_rst_cross_sync_v1_0 #(
    `ifdef IPSL_PCIE_SPEEDUP_SIM
    .RST_CNTR_VALUE     (16'h10             )
    `else
    .RST_CNTR_VALUE     (16'hC000           )
    `endif
)
u_refclk_perstn_debounce(
    .clk                (pcie_ref_clk            ),
    .rstn_in            (pcie_perst_n            ),
    .rstn_out           (sync_perst_n       )
);

ipsl_pcie_sync_v1_0  u_ref_core_rstn_sync    (
    .clk                (pcie_ref_clk            ),
    .rst_n              (core_rst_n         ),
    .sig_async          (1'b1               ),
    .sig_synced         (ref_core_rst_n     )
);

ipsl_pcie_sync_v1_0  u_pclk_core_rstn_sync   (
    .clk                (pclk               ),
    .rst_n              (core_rst_n         ),
    .sig_async          (1'b1               ),
    .sig_synced         (s_pclk_rstn        )
);

ipsl_pcie_sync_v1_0  u_pclk_div2_core_rstn_sync   (
    .clk                (pclk_div2          ),
    .rst_n              (core_rst_n         ),
    .sig_async          (1'b1               ),
    .sig_synced         (s_pclk_div2_rstn   )
);
//axis slave 2 interface
wire            axis_slave2_tready      ;
wire            axis_slave2_tvalid      ;
wire    [127:0] axis_slave2_tdata       ;
wire            axis_slave2_tlast       ;
wire            axis_slave2_tuser       ;

wire    [7:0]   cfg_pbus_num            ;
wire    [4:0]   cfg_pbus_dev_num        ;
wire    [2:0]   cfg_max_rd_req_size     ;
wire    [2:0]   cfg_max_payload_size    ;
wire            cfg_rcb                 ;
//system signal
wire    [4:0]   smlh_ltssm_state       /* synthesis PAP_MARK_DEBUG="1" */;
    

assign axis_slave0_tvalid      = 'd0;
assign axis_slave0_tlast       = 'd0;
assign axis_slave0_tuser       = 'd0;
assign axis_slave0_tdata       = 'd0;
assign axis_slave1_tvalid      = 'd0;
assign axis_slave1_tlast       = 'd0;
assign axis_slave1_tuser       = 'd0;
assign axis_slave1_tdata       = 'd0;

pcie_test u_ipsl_pcie_wrap
(
    .button_rst_n               (sync_button_rst_n      ),
    .power_up_rst_n             (sync_perst_n           ),
    .perst_n                    (sync_perst_n           ),
    //clk and rst
    .free_clk                   (ref_clk               ),
    .pclk                       (pclk                   ),      //output
    .pclk_div2                  (pclk_div2              ),      //output
    .ref_clk                    (pcie_ref_clk                ),      //output
    .ref_clk_n                  (ref_clk_n              ),      //input
    .ref_clk_p                  (ref_clk_p              ),      //input
    .core_rst_n                 (core_rst_n             ),      //output
    
    //APB interface to  DBI cfg
//  .p_clk                      (ref_clk                ),      //input
    // PCIE_FIX_20260912: unused DBI/APB inputs must be driven, not left open.
    .p_sel                      (1'b0                   ),      //input
    .p_strb                     (4'b0000                ),      //input  [ 3:0]
    .p_addr                     (16'd0                  ),      //input  [15:0]
    .p_wdata                    (32'd0                  ),      //input  [31:0]
    .p_ce                       (1'b0                   ),      //input
    .p_we                       (1'b0                   ),      //input
    .p_rdy                      (                       ),      //output
    .p_rdata                    (                       ),      //output [31:0]
    
    //PHY diff signals
    .rxn                        (rxn                    ),      //input   max[3:0]
    .rxp                        (rxp                    ),      //input   max[3:0]
    .txn                        (txn                    ),      //output  max[3:0]
    .txp                        (txp                    ),      //output  max[3:0]
    
    .pcs_nearend_loop           ({1{1'b0}}              ),      //input
    .pma_nearend_ploop          ({1{1'b0}}              ),      //input
    .pma_nearend_sloop          ({1{1'b0}}              ),      //input
    .sfp_refckn                 (i_p_refckn_0            ),
    .sfp_refckp                 (i_p_refckp_0            ),
    .sfp_rxn                    (i_p_l2rxn               ),
    .sfp_rxp                    (i_p_l2rxp               ),
    .sfp_txn                    (o_p_l2txn               ),
    .sfp_txp                    (o_p_l2txp               ),
    .sfp_txd                    (sfp_gt_txdata            ),
    .sfp_txk                    (sfp_gt_txcharisk         ),
    .sfp_rxd                    (sfp_gt_rxdata            ),
    .sfp_rxk                    (sfp_gt_rxcharisk         ),
    .sfp_tx_clk                 (sfp_tx_user_clk          ),
    .sfp_rx_clk                 (sfp_rx_user_clk          ),
    .sfp_tx_done                (sfp_tx_rst_n             ),
    .sfp_rx_done                (sfp_rx_rst_n             ),
    .sfp_pll_rst                (sfp_pll_rst              ),
    .sfp_wtchdg_clr             (sfp_wtchdg_clr           ),
    
    //AXIS master interface
    .axis_master_tvalid         (axis_master_tvalid     ),      //output
    .axis_master_tready         (axis_master_tready     ),      //input
    .axis_master_tdata          (axis_master_tdata      ),      //output [127:0]
    .axis_master_tkeep          (axis_master_tkeep      ),      //output [3:0]
    .axis_master_tlast          (axis_master_tlast      ),      //output
    .axis_master_tuser          (axis_master_tuser      ),      //output [7:0]
    
    //axis slave 0 interface
    .axis_slave0_tready         (axis_slave0_tready     ),      //output
    .axis_slave0_tvalid         (axis_slave0_tvalid     ),      //input
    .axis_slave0_tdata          (axis_slave0_tdata      ),      //input  [127:0]
    .axis_slave0_tlast          (axis_slave0_tlast      ),      //input
    .axis_slave0_tuser          (axis_slave0_tuser      ),      //input
    
    //axis slave 1 interface
    .axis_slave1_tready         (axis_slave1_tready     ),      //output
    .axis_slave1_tvalid         (axis_slave1_tvalid     ),      //input
    .axis_slave1_tdata          (axis_slave1_tdata      ),      //input  [127:0]
    .axis_slave1_tlast          (axis_slave1_tlast      ),      //input
    .axis_slave1_tuser          (axis_slave1_tuser      ),      //input
    //axis slave 2 interface
    .axis_slave2_tready         (axis_slave2_tready     ),      //output
    .axis_slave2_tvalid         (axis_slave2_tvalid     ),      //input
    .axis_slave2_tdata          (axis_slave2_tdata      ),      //input  [127:0]
    .axis_slave2_tlast          (axis_slave2_tlast      ),      //input
    .axis_slave2_tuser          (axis_slave2_tuser      ),      //input
     
    .pm_xtlh_block_tlp          (                       ),      //output
    
    .cfg_send_cor_err_mux       (                       ),      //output
    .cfg_send_nf_err_mux        (                       ),      //output
    .cfg_send_f_err_mux         (                       ),      //output
    .cfg_sys_err_rc             (                       ),      //output
    .cfg_aer_rc_err_mux         (                       ),      //output
    //radm timeout
    .radm_cpl_timeout           (                       ),      //output
    
    //configuration signals
    .cfg_max_rd_req_size        (cfg_max_rd_req_size    ),      //output [2:0]
    .cfg_bus_master_en          (                       ),      //output
    .cfg_max_payload_size       (cfg_max_payload_size   ),      //output [2:0]
    .cfg_ext_tag_en             (                       ),      //output
    .cfg_rcb                    (cfg_rcb                ),      //output
    .cfg_mem_space_en           (                       ),      //output
    .cfg_pm_no_soft_rst         (                       ),      //output
    .cfg_crs_sw_vis_en          (                       ),      //output
    .cfg_no_snoop_en            (                       ),      //output
    .cfg_relax_order_en         (                       ),      //output
    .cfg_tph_req_en             (                       ),      //output [2-1:0]
    .cfg_pf_tph_st_mode         (                       ),      //output [3-1:0]
    .rbar_ctrl_update           (                       ),      //output
    .cfg_atomic_req_en          (                       ),      //output
    
    .cfg_pbus_num               (cfg_pbus_num           ),      //output [7:0]
    .cfg_pbus_dev_num           (cfg_pbus_dev_num       ),      //output [4:0]
    
    //debug signals
    .radm_idle                  (                       ),      //output
    .radm_q_not_empty           (                       ),      //output
    .radm_qoverflow             (                       ),      //output
    .diag_ctrl_bus              (2'b0                   ),      //input   [1:0]
    .cfg_link_auto_bw_mux       (                       ),      //output              merge cfg_link_auto_bw_msi and cfg_link_auto_bw_int
    .cfg_bw_mgt_mux             (                       ),      //output              merge cfg_bw_mgt_int and cfg_bw_mgt_msi
    .cfg_pme_mux                (                       ),      //output              merge cfg_pme_int and cfg_pme_msi
    .app_ras_des_sd_hold_ltssm  (1'b0                   ),      //input
    .app_ras_des_tba_ctrl       (2'b0                   ),      //input   [1:0]
    
    .dyn_debug_info_sel         (4'b0                   ),      //input   [3:0]
    .debug_info_mux             (                       ),      //output  [132:0]
    
    //system signal
    .smlh_link_up               (smlh_link_up           ),      //output
    .rdlh_link_up               (rdlh_link_up           ),      //output
    .smlh_ltssm_state           (smlh_ltssm_state       )       //output  [4:0]
);
endmodule
