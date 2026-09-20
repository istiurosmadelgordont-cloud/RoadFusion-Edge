// Created by IP Generator (Version 2022.1 build 99559)
// creat V3 on 2023.7.31
// #add eth interfacing loop


`timescale 1ps/1ps

`define DDR3
//仿真128*72 ->64*36
module test_ddr #(
  parameter PCIE_ENABLE          = 1                   ,//PCIE模块例化，0不例化，1例化
  parameter PROJECT_MODE         = 1                    ,//PROJECT_MODE 0:仿真；>=1：上板子；2：使用pcie
  parameter VIDEO_LENGTH         = 1920                 ,
  parameter VIDEO_HIGTH          = 1080                 ,
  parameter ZOOM_VIDEO_LENGTH    = 960                 ,
  parameter ZOOM_VIDEO_HIGTH     = 540                 ,
  parameter PIXEL_WIDTH          = 16                 ,    
  parameter MEM_ROW_ADDR_WIDTH   = 15                 ,
  parameter MEM_COL_ADDR_WIDTH   = 10                 ,
  parameter MEM_BADDR_WIDTH      = 3                  ,
  parameter MEM_DQ_WIDTH         = 32                 ,
  parameter MEM_DM_WIDTH         = MEM_DQ_WIDTH/8     ,
  parameter MEM_DQS_WIDTH        = MEM_DQ_WIDTH/8     ,
  parameter M_AXI_BRUST_LEN      = 8                  ,
  parameter RW_ADDR_MIN          = 20'b0              ,
  parameter RW_ADDR_MAX          = ZOOM_VIDEO_LENGTH*ZOOM_VIDEO_HIGTH*PIXEL_WIDTH/MEM_DQ_WIDTH       ,//@540p  518400个地址   
  parameter CTRL_ADDR_WIDTH      = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH			,
  parameter BOARD_MAC  			 =  48'ha0_b1_c2_d3_e4_e4		,   
  parameter BOARD_IP   			 =  {8'd192,8'd168,8'd1,8'd10}
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
  output wire                              fram0_done         ,
  output wire                              fram1_done         ,
  output wire                              fram2_done         ,
  output wire                              fram3_done         ,
//HDMI IN
  input wire                               pix_clk_in      ,//HDMI输入时钟 1080p @148.5Mhz
  input wire                               vs_in           /* synthesis PAP_MARK_DEBUG="1" */,//帧同步
  input wire                               hs_in           ,//行同步
  input wire                               de_in           /* synthesis PAP_MARK_DEBUG="1" */,//数据有效信号
  input wire [7 : 0]                       r_in            /* synthesis PAP_MARK_DEBUG="1" */,
  input wire [7 : 0]                       g_in            /* synthesis PAP_MARK_DEBUG="1" */,
  input wire [7 : 0]                       b_in            /* synthesis PAP_MARK_DEBUG="1" */,
//HDMI OUT
  output                                 pix_clk_out     ,
  output reg                             r_vs_out        /* synthesis PAP_MARK_DEBUG="1" */,  
  output reg                             r_hs_out        , 
  output reg                             r_de_out        /* synthesis PAP_MARK_DEBUG="1" */, 
  output reg  [7 : 0]                    r_r_out         /* synthesis PAP_MARK_DEBUG="1" */, 
  output reg  [7 : 0]                    r_g_out         /* synthesis PAP_MARK_DEBUG="1" */,
  output reg  [7 : 0]                    r_b_out         /* synthesis PAP_MARK_DEBUG="1" */,  
//coms1	
  inout                                cmos1_scl            ,//cmos1 i2c 
  inout                                cmos1_sda            ,//cmos1 i2c 
  input                                cmos1_vsync          /* synthesis PAP_MARK_DEBUG="1" */,//cmos1 vsync
  input                                cmos1_href           ,//cmos1 hsync refrence,data valid
  input                                cmos1_pclk           ,//cmos1 pxiel clock
  input   [7:0]                        cmos1_data           ,//cmos1 data
  output                               cmos1_reset          /* synthesis PAP_MARK_DEBUG="1" */, //cmos1 reset
  //coms2
  inout                                cmos2_scl            ,//cmos2 i2c 
  inout                                cmos2_sda            ,//cmos2 i2c 
  input                                cmos2_vsync          ,//cmos2 vsync
  input                                cmos2_href           ,//cmos2 hsync refrence,data valid
  input                                cmos2_pclk           ,//cmos2 pxiel clock
  input   [7:0]                        cmos2_data           ,//cmos2 data
  output                               cmos2_reset          ,
  
 // ethernet
  output wire                          eth_rst_n_0        , //以太网复位信号
  input  wire                          eth_rgmii_rxc_0  /* synthesis PAP_MARK_DEBUG="1" */   ,
  input  wire                          eth_rgmii_rx_ctl_0  /* synthesis PAP_MARK_DEBUG="1" */,
  input  wire [3:0]                    eth_rgmii_rxd_0    /* synthesis PAP_MARK_DEBUG="1" */ ,                   
  output wire                          eth_rgmii_txc_0   /* synthesis PAP_MARK_DEBUG="1" */ ,
  output wire                          eth_rgmii_tx_ctl_0 /* synthesis PAP_MARK_DEBUG="1" */,
  output wire [3:0]                    eth_rgmii_txd_0 /* synthesis PAP_MARK_DEBUG="1" */  ,
 
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

/******************************PARAMETER********************************************/
parameter DQ_WIDTH = MEM_DQ_WIDTH;
parameter DE_IN_WAIT  = 4'd0;
parameter DE_IN_CNT  = 4'd1;
parameter DE_IN_END  = 4'd2;

parameter DE_OUT_WAIT  = 4'd0;
parameter DE_OUT_CNT  = 4'd1;
parameter DE_OUT_END  = 4'd2;

parameter PIX_IN_WAIT = 3'd0;
parameter PIX_IN_CNT = 3'd1;
parameter PIX_IN_END = 3'd2;

parameter PIX_OUT_WAIT = 3'd0;
parameter PIX_OUT_CNT = 3'd1;
parameter PIX_OUT_END = 3'd2;
/******************************wire********************************************/      
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
//w_fifo
wire  [31 : 0]                        rgb_in/* synthesis PAP_MARK_DEBUG="1" */;
wire  [15 : 0]                        video0_data_out/* synthesis PAP_MARK_DEBUG="1" */;
wire  [15 : 0]                        video1_data_out/* synthesis PAP_MARK_DEBUG="1" */;
wire  [15 : 0]                        video2_data_out/* synthesis PAP_MARK_DEBUG="1" */;
wire  [15 : 0]                        video3_data_out/* synthesis PAP_MARK_DEBUG="1" */;



//wire                                    fram_done;
//iic
wire                                 iic_clk;//10mhz
wire                                 pll_init_done;
wire                                 [11 : 0] x_act /* synthesis PAP_MARK_DEBUG="1" */;
wire                                 [11 : 0] y_act /* synthesis PAP_MARK_DEBUG="1" */;
//wire                                 hdmi_rst;

wire                                vs_out/* synthesis PAP_MARK_DEBUG="1" */;
wire                                hs_out;
wire                                de_out/* synthesis PAP_MARK_DEBUG="1" */;

wire                                zoom_de_out/* synthesis PAP_MARK_DEBUG="1" */;
wire [31: 0]           				zoom_data_out;

wire                                clk_25M;
wire [1:0]                          cmos_init_done/* synthesis PAP_MARK_DEBUG="1" */;
wire [15:0]                         cmos1_d_16bit;
wire                                cmos1_href_16bit/* synthesis PAP_MARK_DEBUG="1" */;
wire                                cmos1_pclk_16bit;
wire[15:0]                          cmos2_d_16bit       /*synthesis PAP_MARK_DEBUG="1"*/;
wire                                cmos2_href_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
wire                                cmos2_pclk_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
/******************************reg********************************************/
reg [11 : 0]              de_in_cnt    /* synthesis PAP_MARK_DEBUG="1" */;
reg                       de_in_d0     /* synthesis PAP_MARK_DEBUG="1" */;
reg                       de_in_d1     /* synthesis PAP_MARK_DEBUG="1" */;
reg                       vs_in_d0     /* synthesis PAP_MARK_DEBUG="1" */;
reg                       vs_in_d1     /* synthesis PAP_MARK_DEBUG="1" */;
reg [3 : 0]               de_in_state  /* synthesis PAP_MARK_DEBUG="1" */;

reg                       zoom_vs_in_d0   ;
reg                       zoom_vs_in_d1   ;
reg                       zoom_de_in_d0   ;
reg                       zoom_de_in_d1   ;
reg [11 : 0]              zoom_de_in_cnt  /* synthesis PAP_MARK_DEBUG="1" */;
reg [3 : 0]               zoom_de_in_state;

reg [11 : 0]              de_out_cnt    /* synthesis PAP_MARK_DEBUG="1" */;
reg                       r_de_out_d0   /* synthesis PAP_MARK_DEBUG="1" */;//由于fifo需要预读出，所以多打一拍
reg                       r_vs_out_d0   /* synthesis PAP_MARK_DEBUG="1" */;
reg                       de_out_d0     /* synthesis PAP_MARK_DEBUG="1" */;
reg                       de_out_d1     /* synthesis PAP_MARK_DEBUG="1" */;
reg                       vs_out_d0     /* synthesis PAP_MARK_DEBUG="1" */;
reg                       vs_out_d1     /* syntqqhesis PAP_MARK_DEBUG="1" */;
reg [3 : 0]               de_out_state  /* synthesis PAP_MARK_DEBUG="1" */;
reg [11 : 0]              r_x_act_d0    /* synthesis PAP_MARK_DEBUG="1" */;
reg [11 : 0]              r_x_act       /* synthesis PAP_MARK_DEBUG="1" */;
reg                       video0_rd_en/* synthesis PAP_MARK_DEBUG="1" */;
reg                       video1_rd_en/* synthesis PAP_MARK_DEBUG="1" */;
reg                       video2_rd_en/* synthesis PAP_MARK_DEBUG="1" */;
reg                       video3_rd_en/* synthesis PAP_MARK_DEBUG="1" */;
reg                       video_pre_rd_flag/* synthesis PAP_MARK_DEBUG="1" */;
wire                      w_video_pre_rd_flag;
assign w_video_pre_rd_flag = video_pre_rd_flag;
reg                        v_sync_flag;
reg [15:0]                rstn_1ms       ;
 
reg [7:0]                   cmos1_d_d0          ;
reg                         cmos1_href_d0       ;
reg                         cmos1_vsync_d0      ;
reg [7:0]                   cmos2_d_d0          /*synthesis PAP_MARK_DEBUG="1"*/;
reg                         cmos2_href_d0       /*synthesis PAP_MARK_DEBUG="1"*/;
reg                         cmos2_vsync_d0      /*synthesis PAP_MARK_DEBUG="1"*/;

reg [2:0]                   out_state            /*synthesis PAP_MARK_DEBUG="1"*/;
assign eth_rst_n_0 = ddr_ip_rst_n;
/******************************assign********************************************/

assign rgb_in[31:24] = r_in;
assign rgb_in[23:22] = 2'd0;
assign rgb_in[21:14] = g_in;
assign rgb_in[13:12] = 2'd0;
assign rgb_in[11: 4] = b_in;
assign rgb_in[3 : 2] = 2'd0;
assign rgb_in[1 : 0] = 2'd0;
//assign rgb_in = de_in ? {r_in[7:3] , g_in[7:2] , b_in[7:3]} : 16'b0;
//临时
//assign de_out                 =       ddr_init_done? 1'b1 : 1'b0;
/******************************always********************************************/
/******************************instant********************************************/

//测量de个数
always @(posedge pix_clk_in) begin//抓下降沿
    if(!ddr_ip_rst_n) begin  
        vs_in_d0 <= 'd0;
        vs_in_d1 <= 'd0;
        de_in_d0 <= 'd0;
        de_in_d1 <= 'd0;
        de_in_cnt <= 'd0; 
        de_in_state <= 0;
    end
    else begin
       case(de_in_state) 
            DE_IN_WAIT:
            begin
                vs_in_d0 <= vs_in;
                vs_in_d1 <= vs_in_d0;
                if(!vs_in_d0 && vs_in_d1) begin
                    de_in_state <= DE_IN_CNT;//抓取vs_in下降沿，当抓到下降沿时开始计数
                end
            end
            DE_IN_CNT:
            begin
                de_in_d0 <= de_in;
                de_in_d1 <= de_in_d0;
                if(de_in_d0 && !de_in_d1) begin
                    de_in_cnt <= de_in_cnt + 1'd1;//抓取de上升沿，de上升时计数
                end
                else if(de_in_cnt == VIDEO_HIGTH) begin
                    de_in_state <= DE_IN_END;
                end
            end
            DE_IN_END:
            begin
                vs_in_d0 <= vs_in;
                vs_in_d1 <= vs_in_d0;
                if(vs_in_d0 && !vs_in_d1) begin
                    de_in_cnt <= 'd0;
                    de_in_state <= DE_IN_WAIT;//抓取vs_in上升沿，当抓到上升沿时计数归零
                end
            end
        endcase  
    end
end
//测量zoom de个数
always @(posedge pix_clk_in) begin//抓下降沿
    if(!ddr_ip_rst_n) begin  
        zoom_vs_in_d0    <= 'd0;
        zoom_vs_in_d1    <= 'd0;
        zoom_de_in_d0    <= 'd0;
        zoom_de_in_d1    <= 'd0;
        zoom_de_in_cnt   <= 'd0; 
        zoom_de_in_state <= 0;
    end
    else begin
       case(zoom_de_in_state) 
            DE_IN_WAIT:
            begin
                zoom_vs_in_d0 <= vs_in;
                zoom_vs_in_d1 <= zoom_vs_in_d0;
                if(!zoom_vs_in_d0 && zoom_vs_in_d1) begin
                    zoom_de_in_state <= DE_IN_CNT;//抓取vs_in下降沿，当抓到下降沿时开始计数
                end
            end
            DE_IN_CNT:
            begin
                zoom_de_in_d0 <= zoom_de_out;
                zoom_de_in_d1 <= zoom_de_in_d0;
                if(zoom_de_in_d0 && !zoom_de_in_d1) begin
                    zoom_de_in_cnt <= zoom_de_in_cnt + 1'd1;//抓取de上升沿，de上升时计数
                end
                else if(zoom_de_in_cnt == ZOOM_VIDEO_HIGTH) begin
                    zoom_de_in_state <= DE_IN_END;
                end
            end
            DE_IN_END:
            begin
                zoom_vs_in_d0 <= vs_in;
                zoom_vs_in_d1 <= zoom_vs_in_d0;
                if(vs_in_d0 && !vs_in_d1) begin
                    zoom_de_in_cnt <= 'd0;
                    zoom_de_in_state <= DE_IN_WAIT;//抓取vs_in上升沿，当抓到上升沿时计数归零
                end
            end
        endcase  
    end
end
//测量de_out个数
always @(posedge pix_clk_out) begin//抓下降沿
    if(!ddr_ip_rst_n) begin  
        vs_out_d0 <= 'd0;
        vs_out_d1 <= 'd0;
        de_out_d0 <= 'd0;
        de_out_d1 <= 'd0;
        de_out_cnt <= 'd0; 
        de_out_state <= 'd0;
    end
    else begin
       case(de_out_state) 
            DE_OUT_WAIT:
            begin
                vs_out_d0 <= vs_out;
                vs_out_d1 <= vs_out_d0;
                if(!vs_out_d0 && vs_out_d1) begin
                    de_out_state <= DE_OUT_CNT;//抓取vs_in下降沿，当抓到下降沿时开始计数
                end
            end
            DE_OUT_CNT:
            begin
                de_out_d0 <= de_out;
                de_out_d1 <= de_out_d0;
                if (de_out_d0 && !de_out_d1) begin
                    de_out_cnt <= de_out_cnt + 1'd1;//抓取de上升沿，de上升时计数
                end
                else if(de_out_cnt == VIDEO_HIGTH) begin
                    de_out_state <= DE_OUT_END;
                end
            end
            DE_OUT_END:
            begin
                vs_out_d0 <= vs_out;
                vs_out_d1 <= vs_out_d0;
                if(vs_out_d0 && !vs_out_d1) begin
                    de_out_cnt <= 'd0;
                    de_out_state <= DE_OUT_WAIT;//抓取vs_in上升沿，当抓到上升沿时计数归零
                end
            end
        endcase  
    end
end


//将传输后的信号进行输出
always @(posedge pix_clk_out) begin
    if(!ddr_ip_rst_n) begin
        r_vs_out <= 'd0;
        r_hs_out <= 'd0;
        r_de_out <= 'd0;
        r_r_out  <= 'd0;
        r_g_out  <= 'd0;
        r_b_out  <= 'd0;
        v_sync_flag <= 'd0;
        video0_rd_en <= 1'b0; 
        video1_rd_en <= 1'b0; 
        video2_rd_en <= 1'b0; 
        video3_rd_en <= 1'b0; 
        video_pre_rd_flag <= 1'b0;
        
        out_state <= 'd0;
    end 
    else if(ddr_init_done) begin 
        r_vs_out_d0 <= vs_out;
        r_vs_out    <= r_vs_out_d0;
        r_hs_out <= hs_out;
        r_de_out_d0 <= de_out;
        r_de_out <= r_de_out_d0;
        //r_de_out <= de_out;

        r_x_act_d0 <= x_act;//X轴坐标随着多打+一拍
        r_x_act <= r_x_act_d0;

       if(vs_out_d0 && !vs_out_d1) begin
            video_pre_rd_flag <= 'd0;
       end
       else if(!vs_out_d0 && vs_out_d1 && !video_pre_rd_flag && (fram0_done || fram1_done || fram2_done || fram3_done)) begin
            video0_rd_en      <= 'd1;
            video1_rd_en      <= 'd1;
            video2_rd_en      <= 'd1;
            video3_rd_en      <= 'd1;
            video_pre_rd_flag <= 'd1;
            out_state         <= 'd1;
       end
       else begin
           if( fram0_done && (r_x_act >= 0) && (r_x_act < ZOOM_VIDEO_LENGTH - 1) && (y_act < ZOOM_VIDEO_HIGTH ) && (y_act >= 0)) begin//左上角
                //test
                //test_out <=  video0_data_out;
                r_r_out  <=  {video0_data_out[15:11],3'b0};
                r_g_out  <=  {video0_data_out[10:5],2'b0};
                r_b_out  <=  {video0_data_out[4:0],3'b0};  
                video0_rd_en <= de_out; //预读出
                video1_rd_en <= 'd0; 
                video2_rd_en <= 'd0; 
                video3_rd_en <= 'd0; //'d0; 
                out_state    <= 'd2;
                if(r_x_act == ZOOM_VIDEO_LENGTH - 2) begin
                    video1_rd_en <= de_out;
                    video0_rd_en <= 'd0;
                end
            end
            if(fram1_done &&(r_x_act >= ZOOM_VIDEO_LENGTH - 1) && (r_x_act < VIDEO_LENGTH - 1) && (y_act < ZOOM_VIDEO_HIGTH )&& (y_act >= 0)) begin//实际上是r_x_act 0~63
                //r_r_out  <= video1_data_out[31:24] ; 
                //r_g_out  <='d0 ; 
                //r_b_out  <='d0 ;
                //test
                //test_out <=  video1_data_out; 
                r_r_out  <= {video1_data_out[15:11],3'b0};
                r_g_out  <= {video1_data_out[10:5],2'b0};
                r_b_out  <= {video1_data_out[4:0],3'b0};  
                //r_r_out  <= {video1_data_out[15 : 11],3'b0};
                //r_g_out  <= {video1_data_out[10 :  5],2'b0};
                //r_b_out  <= {video1_data_out[4  :  0],3'b0}; 
                //
                //r_r_out  <= 'hff;
                //r_g_out  <= 'd00;
                //r_b_out  <= 'd00;
                video0_rd_en <= 'd0; 
                video1_rd_en <= de_out; 
                video2_rd_en <= 'd0; 
                video3_rd_en <= 'd0; 
                out_state    <= 'd3;

            end  
            if( fram2_done &&(r_x_act >= 0) && (r_x_act < ZOOM_VIDEO_LENGTH - 1) && (y_act < VIDEO_HIGTH )&& (y_act >= ZOOM_VIDEO_HIGTH)) begin//实际上是r_x_act 0~63
                //r_r_out  <='d0 ; 
                //r_g_out  <= video2_data_out[21:14] ; 
                //r_b_out  <='d0 ; 
                //test
                //test_out <=  video2_data_out;
                r_r_out  <= {video2_data_out[15:11],3'b0};
                r_g_out  <= {video2_data_out[10:5],2'b0};
                r_b_out  <= {video2_data_out[4:0],3'b0};   
                video0_rd_en <= 'd0; 
                video1_rd_en <= 'd0; 
                video2_rd_en <= de_out; 
                video3_rd_en <= 'd0; 
                out_state    <= 'd4;
                if(r_x_act == ZOOM_VIDEO_LENGTH - 2) begin
                    video3_rd_en <= de_out;
                    video2_rd_en <= 'd0;
                end
            end    
            if(fram3_done &&(r_x_act >= ZOOM_VIDEO_LENGTH - 1) && (r_x_act < VIDEO_LENGTH - 1) && (y_act < VIDEO_HIGTH )&& (y_act >= ZOOM_VIDEO_HIGTH)) begin//实际上是r_x_act 0~63
                //r_r_out  <='d0 ; 
                //r_g_out  <='d0 ; 
                //r_b_out  <=video3_data_out[11:4] ;
                //test
                //test_out <=  video3_data_out;
                r_r_out  <= {video3_data_out[15:11],3'b0};
                r_g_out  <= {video3_data_out[10:5],2'b0};
                r_b_out  <= {video3_data_out[4:0],3'b0};       
                video0_rd_en <= 'd0; 
                video1_rd_en <= 'd0; 
                video2_rd_en <= 'd0; 
                video3_rd_en <= de_out; 
                
                out_state    <= 'd5;
            end 
        end         
    end
    else begin
        r_vs_out <= 'd0;
        r_hs_out <= 'd0;
        r_de_out <= 'd0;
        r_r_out  <= 8'hff ;
        r_g_out  <= 8'h00 ;
        r_b_out  <= 8'h00 ;
        video0_rd_en <= 1'b0; 
        video1_rd_en <= 1'b0; 
        video2_rd_en <= 1'b0; 
        video3_rd_en <= 1'b0; 
        out_state    <= 'd7; 
    end            
end

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
hdmi_ctrl user_hdmi_ctrl(
    .clk         (  iic_clk    ), //input       clk,
    .rst_n       (  hdmi_rst   ), //input       rstn, 
    .init_over   (  hdmi_int_led  ), //output      init_over,
    .iic_tx_scl  (  iic_tx_scl ), //output      iic_scl,
    .iic_tx_sda  (  iic_tx_sda ), //inout       iic_sda
    .iic_scl     (  iic_scl    ), //output      iic_scl,
    .iic_sda     (  iic_sda    )  //inout       iic_sda
);
//ov5640配置
//CMOS1 Camera 720P，缩放至540P，30帧
ov5640_reg_cfg_0	coms1_reg_config(
	.clk_25M                 (clk_25M            ),//input
	.camera_rstn             (cmos1_reset        ),//input
	.initial_en              (initial_en         ),//input		
	.i2c_sclk                (cmos1_scl          ),//output
	.i2c_sdat                (cmos1_sda          ),//inout
	.reg_conf_done           (cmos_init_done[0]  ),//output config_finished
	.reg_index               (                   ),//output reg [8:0]
	.clock_20k               (                   ) //output reg
);
//CMOS2 Camera 
ov5640_reg_config	coms2_reg_config(
    	.clk_25M                 (clk_25M            ),//input
    	.camera_rstn             (cmos2_reset        ),//input
    	.initial_en              (initial_en         ),//input		
    	.i2c_sclk                (cmos2_scl          ),//output
    	.i2c_sdat                (cmos2_sda          ),//inout
    	.reg_conf_done           (cmos_init_done[1]  ),//output config_finished
    	.reg_index               (                   ),//output reg [8:0]
    	.clock_20k               (                   ) //output reg
    );
ov5640_power_on_delay	power_on_delay_inst(
	.clk_50M                 (ref_clk        ),//input
	.reset_n                 (ddr_ip_rst_n           ),//input	
	.camera1_rstn            (cmos1_reset    ),//output
	.camera2_rstn            (cmos2_reset    ),//output	
	.camera_pwnd             (               ),//output
	.initial_en              (initial_en     ) //output		
);
//CMOS 8bit转16bit//////////
//CMOS1
always@(posedge cmos1_pclk)
    begin
        cmos1_d_d0        <= cmos1_data    ;
        cmos1_href_d0     <= cmos1_href    ;
        cmos1_vsync_d0    <= cmos1_vsync   ;
    end

cmos_8_16bit cmos1_8_16bit(
	.pclk           (cmos1_pclk       ),//input
	.rst_n          (cmos_init_done[0]),//input
	.pdata_i        (cmos1_d_d0       ),//input[7:0]
	.de_i           (cmos1_href_d0    ),//input
	.vs_i           (cmos1_vsync_d0    ),//input
	
	.pixel_clk      (cmos1_pclk_16bit ),//output
	.pdata_o        (cmos1_d_16bit    ),//output[15:0] rgb565 r在高5位
	.de_o           (cmos1_href_16bit ) //output
);
//CMOS2
always@(posedge cmos2_pclk)
    begin
        cmos2_d_d0        <= cmos2_data    ;
        cmos2_href_d0     <= cmos2_href    ;
        cmos2_vsync_d0    <= cmos2_vsync   ;
    end

cmos_8_16bit cmos2_8_16bit(
	.pclk           (cmos2_pclk       ),//input
	.rst_n          (cmos_init_done[1]),//input
	.pdata_i        (cmos2_d_d0       ),//input[7:0]
	.de_i           (cmos2_href_d0    ),//input
	.vs_i           (cmos2_vsync_d0    ),//input
	
	.pixel_clk      (cmos2_pclk_16bit ),//output
	.pdata_o        (cmos2_d_16bit    ),//output[15:0]
	.de_o           (cmos2_href_16bit ) //output
);

 

ipsxb_rst_sync_v1_1 u_core_clk_rst_sync(
    .clk                        (ref_clk        ),
    .rst_n                      (rst_board       ),
    .sig_async                  (1'b1),
    .sig_synced                 (ddr_ip_rst_n   )
);

//初始化顺序：DDR->AXI_M & FIFO ->IIC PLL -HDMI
axi_m_arbitration #(
    .VIDEO_LENGTH     (VIDEO_LENGTH)                    ,
    .VIDEO_HIGTH      (VIDEO_HIGTH)                     ,
    .ZOOM_VIDEO_LENGTH(ZOOM_VIDEO_LENGTH )              ,
    .ZOOM_VIDEO_HIGTH (ZOOM_VIDEO_HIGTH )               ,
    .PIXEL_WIDTH      (PIXEL_WIDTH  )                            ,
	.CTRL_ADDR_WIDTH  (CTRL_ADDR_WIDTH  )                            ,
	.DQ_WIDTH	     (DQ_WIDTH  )                            ,
    .M_AXI_BRUST_LEN  (M_AXI_BRUST_LEN   )
)
user_axi_m_arbitration (
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
    .vs_in                   (vs_in       ),
    .vs_out                  (vs_out      ),
//fifo0信号          
    .video0_clk_in           (pix_clk_in),                                                                                                                  
    .video0_de_in            (zoom_de_out    ),
    .video0_data_in          ({zoom_data_out[31 : 27],zoom_data_out[21 : 16],zoom_data_out[11 :  7]}    ),
    .video0_rd_en            (video0_rd_en   ),
    .video0_data_out         (video0_data_out),
    .fram0_done              (fram0_done     ),
    .video0_vs_in            (vs_in ),//用于抓取复位
//fifo1信号
    // .video1_clk_in           (pix_clk_in),                                                               
    // .video1_de_in            (post_frame_de  ),
    // .video1_data_in          (post_rgb       ),
    // .video1_rd_en            (video1_rd_en   ),
    // .video1_data_out         (video1_data_out),
    // .fram1_done              (fram1_done     ),
    // .video1_vs_in            (post_frame_vsync ),//用于抓取复位
    .video1_clk_in           (rgmii_clk_0    ),    
    .video1_de_in            (eth0_rx_de     ),
    .video1_data_in          ({eth0_rx_data[15:11],eth0_rx_data[10:5],eth0_rx_data[4:0]} ),
    .video1_rd_en            (video1_rd_en   ),
    .video1_data_out         (video1_data_out),
    .fram1_done              (fram1_done     ),
    .video1_vs_in            (eth0_rx_vs     ),//用于抓取复位
//fifo2信号，接CMOS1                                  
    .video2_clk_in           (cmos1_pclk_16bit),                       
    .video2_de_in            (cmos1_href_16bit ),
    .video2_data_in          (cmos1_d_16bit),//27
    .video2_rd_en            (video2_rd_en   ),
    .video2_data_out         (video2_data_out),
    .fram2_done              (fram2_done     ),
    .video2_vs_in            (cmos1_vsync_d0 ),//用于抓取复位
//fifo3信号                                        
    .video3_clk_in           (cmos2_pclk_16bit),                       
    .video3_de_in            (cmos2_href_16bit    ),
    .video3_data_in          (cmos2_d_16bit ),
    .video3_rd_en            (video3_rd_en   ),
    .video3_data_out         (video3_data_out),
    .fram3_done              (fram3_done     ),
    .video3_vs_in            (cmos2_vsync_d0 ),//用于抓取复位
    //其他
    .wr_addr_min             (RW_ADDR_MIN),//写数据ddr最小地址0地址开始算，1920*1080*16 = 33177600 bits
    .wr_addr_max             (RW_ADDR_MAX), //写数据ddr最大地址，一个地址存32位 33177600/32 = 1036800 = 20'b1111_1101_0010_0000_0000
    .y_act                   (y_act)        , 
    .x_act                   (x_act)  

);

sync_generator user_sync_gen(
    .clk       (pix_clk_out ),//1080p参考时钟为148.5mhz
    .rstn      (ddr_ip_rst_n && ddr_init_done),
    .vs_out    (vs_out),
    .hs_out    (hs_out),
    .de_out    (de_out),
    .de_re     (),
    .x_act     (x_act),
    .y_act     (y_act)
);

video_zoom hdmi_video_zoom(
    .clk                (pix_clk_in),
    .rstn               (ddr_ip_rst_n && ddr_init_done),
    .vs_in              (vs_in                        ) ,
    .hs_in              (hs_in                        ) ,
    .de_in              (de_in                        ) ,
    .video_data_in      (rgb_in                       ),
    .de_out             (zoom_de_out                  ),
    .video_data_out     (zoom_data_out                )
   );

wire  [15:0] post_rgb ;
//光口传输顶层模块   
sfp_8b10b_top u_sfp_8b10b_top(
    //光口接口
    .i_p_refckn_0               (i_p_refckn_0       ),
    .i_p_refckp_0               (i_p_refckp_0       ),
    .i_p_l2rxn                  (i_p_l2rxn          ),
    .i_p_l2rxp                  (i_p_l2rxp          ),
    .i_p_l3rxn                  (i_p_l3rxn          ),
    .i_p_l3rxp                  (i_p_l3rxp          ),
    .o_p_l2txn                  (o_p_l2txn          ),
    .o_p_l2txp                  (o_p_l2txp          ),
    .o_p_l3txn                  (o_p_l3txn          ),
    .o_p_l3txp                  (o_p_l3txp          ),
    .tx_disable                 (tx_disable         ),
	//用户接口
	.drp_clk    	    		(ref_clk            ),      //光口输入时钟  
	.clk_in     	   			(pix_clk_in         ),      //输入时钟
	.rst_n						(ddr_ip_rst_n && ddr_init_done         ),      //输入复位
    .h_pixel                    (ZOOM_VIDEO_LENGTH       ),  //输入行像素
	.vs_in						(vs_in   ),      //输入场信号
	.data_valid_in 				(zoom_de_out   ),      //输入数据有效信号
	.data_in					({zoom_data_out[31 : 27],zoom_data_out[21 : 16],zoom_data_out[11 :  7]}            ),      //输入数据

	.o_vs						(post_frame_vsync   ),      //输出场信号
	.data_valid_out				(post_frame_de      ),	    //输出数据有效信号
	.data_out	   				(post_rgb           )       //输出数据
);  


pll_cfg user_pll_cfg (
  .clkin1(ref_clk),        // input
  .pll_lock(pll_init_done),    // output
  .clkout0(iic_clk),      // output
  .clkout1(clk_25M)       // output
);

pll_video_out user_pll_video_out (
  .clkin1(ref_clk),        // input
  .pll_lock(pll_lock),    // output
  .clkout0(pix_clk_out)       // output
);


reg [11 : 0]              cmos1_de_in_cnt    /* synthesis PAP_MARK_DEBUG="1" */;
reg [11 : 0]              cmos1_h_cnt        /* synthesis PAP_MARK_DEBUG="1" */;

reg                       cmos1_de_in_d0     /* synthesis PAP_MARK_DEBUG="1" */;
reg                       cmos1_de_in_d1     /* synthesis PAP_MARK_DEBUG="1" */;
reg                       cmos1_vs_in_d0     /* synthesis PAP_MARK_DEBUG="1" */;
reg                       cmos1_vs_in_d1     /* synthesis PAP_MARK_DEBUG="1" */;
reg [3 : 0]               cmos1_de_in_state  /* synthesis PAP_MARK_DEBUG="1" */;

always @(posedge cmos1_pclk_16bit) begin//抓下降沿
    if(!ddr_ip_rst_n) begin  
        cmos1_vs_in_d0 <= 'd0;
        cmos1_vs_in_d1 <= 'd0;
        cmos1_de_in_d0 <= 'd0;
        cmos1_de_in_d1 <= 'd0;
        cmos1_de_in_cnt <= 'd0; 
        cmos1_de_in_state <= 'd0;
        cmos1_h_cnt     <= 'd0;
    end
    else begin
       case(cmos1_de_in_state) 
            DE_IN_WAIT:
            begin
                cmos1_vs_in_d0 <= cmos1_vsync;
                cmos1_vs_in_d1 <= cmos1_vs_in_d0;
                if(!cmos1_vs_in_d0 && cmos1_vs_in_d1) begin
                    cmos1_de_in_state <= DE_IN_CNT;//抓取vs_in下降沿，当抓到下降沿时开始计数
                end
            end
            DE_IN_CNT:
            begin
                cmos1_de_in_d0 <= cmos1_href_16bit;
                cmos1_de_in_d1 <= cmos1_de_in_d0;
                if(cmos1_de_in_d0 && !cmos1_de_in_d1) begin
                    cmos1_de_in_cnt <= cmos1_de_in_cnt + 1'd1;//抓取de上升沿，de上升时计数
                end
                if(cmos1_href_16bit) begin
                    cmos1_h_cnt <= cmos1_h_cnt +'d1;
                end
                else begin
                    cmos1_h_cnt <= 'd0;
                end
                cmos1_vs_in_d0 <= cmos1_vsync;
                cmos1_vs_in_d1 <= cmos1_vs_in_d0;
                if(cmos1_vs_in_d0 && !cmos1_vs_in_d1) begin
                    cmos1_de_in_cnt <= 'd0;
                    cmos1_de_in_state <= DE_IN_WAIT;//抓取vs_in上升沿，当抓到上升沿时计数归零
                end
            end
        endcase  
    end
end
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
///////////////////////////////////ethernet///////////////////////////////////////
wire [15:0] eth0_rx_data ;
wire eth0_rx_de ;
wire eth0_rx_vs ;





ethernet#(
  . BOARD_MAC   (BOARD_MAC) ,
  . BOARD_IP    (BOARD_IP )
  )u_ethernet(  
  .			      rst_board 				(rst_board 		)			    ,		
  .               eth_rgmii_rxc_0  			(eth_rgmii_rxc_0  	)		    ,
  .               eth_rgmii_rx_ctl_0 		(eth_rgmii_rx_ctl_0)			,
  .               eth_rgmii_rxd_0   		(eth_rgmii_rxd_0   )		    ,                   
  .               eth_rgmii_txc_0   		(eth_rgmii_txc_0   )		    ,
  .               eth_rgmii_tx_ctl_0		(eth_rgmii_tx_ctl_0)		    ,
  .               eth_rgmii_txd_0 			(eth_rgmii_txd_0 	)	        ,
  .               eth0_rx_data 				(eth0_rx_data 	)	          	,
  .				  rgmii_clk_0				(rgmii_clk_0	)				,
  . 			  eth0_rx_de  				(eth0_rx_de  	)	        	,
  . 			  eth0_rx_vs  				(eth0_rx_vs  	)	
   );



endmodule

