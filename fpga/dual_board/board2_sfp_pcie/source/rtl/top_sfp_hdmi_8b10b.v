//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：http://www.openedv.com/forum.php
//淘宝店铺：https://zhengdianyuanzi.tmall.com
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2023-2033
//All rights reserved                                  
//----------------------------------------------------------------------------------------
// File name:           top_sfp_hdmi_8b10b
// Created by:          正点原子
// Created date:        2023年2月3日14:17:02
// Version:             V1.0
// Descriptions:        top_sfp_hdmi_8b10b
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module top_sfp_hdmi_8b10b(    
    input                 sys_clk,                   //系统时钟
    input                 sys_rst_n,                 //系统复位，低电平有效
    //摄像头接口                                           
    input                 cam_pclk,                  //cmos 数据像素时钟
    input                 cam_vsync,                 //cmos 场同步信号
    input                 cam_href,                  //cmos 行同步信号
    input   [7:0]         cam_data,                  //cmos 数据
    output                cam_rst_n,                 //cmos 复位信号，低电平有效
    output                cam_pwdn,                  //电源休眠模式选择 0：正常模式 1：电源休眠模式
    output                cam_scl,                   //cmos SCCB_SCL线
    inout                 cam_sda,                   //cmos SCCB_SDA线
    //光口接口
    input                 i_p_refckn_1,              //差分参考时钟
    input                 i_p_refckp_1,
    input                 i_p_l2rxn,                 //差分接收数据
    input                 i_p_l2rxp,
    input                 i_p_l3rxn,                   
    input                 i_p_l3rxp,
    output                o_p_l2txn,                 //差分发送数据
    output                o_p_l2txp,
    output                o_p_l3txn,
    output                o_p_l3txp,
    output  [1:0]         tx_disable,	             //tx端发送禁止使能
    // DDR3                            
    output                mem_rst_n,                       
    output                mem_ck   ,
    output                mem_ck_n ,
    output                mem_cke  ,
    output                mem_ras_n,
    output                mem_cas_n,
    output                mem_we_n , 
    output                mem_odt  ,
    output 	[14:0]        mem_a    ,   
    output 	[2:0]         mem_ba   ,   
    inout 	[3:0]         mem_dqs  ,
    inout 	[3:0]         mem_dqs_n,
    inout 	[31:0]        mem_dq   ,
    output                mem_cs_n ,
    output 	[3:0]         mem_dm   ,
    //hdmi接口                              
    output                tmds_clk_p,               // TMDS 时钟通道
    output                tmds_clk_n,
    output [2:0]          tmds_data_p,              // TMDS 数据通道
    output [2:0]          tmds_data_n 
    );     
//parameter define  
parameter  V_CMOS_DISP = 11'd768;                  //CMOS分辨率--行
parameter  H_CMOS_DISP = 11'd1024;                 //CMOS分辨率--列	
parameter  TOTAL_H_PIXEL = H_CMOS_DISP + 12'd1216; 
parameter  TOTAL_V_PIXEL = V_CMOS_DISP + 12'd504; 
//wire define                          
wire         clk_50m                ;  //50mhz时钟
wire         locked                 ;  //时钟锁定信号
wire         rst_n                  ;  //全局复位 								             
wire         cam_init_done          ;  //摄像头初始化完成							    
wire         wr_en                  ;  //DDR3控制器模块写使能
wire  [15:0] wr_data                ;  //DDR3控制器模块写数据
wire         rdata_req              ;  //DDR3控制器模块读使能
wire  [15:0] rd_data                ;  //DDR3控制器模块读数据
wire         cmos_frame_valid       ;  //数据有效使能信号
wire         ddr3_init_done         ;  //DDR3初始化完成
wire         sys_init_done          ;  //系统初始化完成(DDR初始化+摄像头初始化)
wire         cmos_frame_vsync       ;  //输出帧有效场同步信号
wire         cmos_frame_href        ;  //输出帧有效行同步信号       
wire  [26:0] ddr3_addr_max          ;  //存入DDR3的最大读写地址 
wire         post_frame_vsync       ;
wire         post_frame_hsync       ;
wire         post_frame_de          ;    
wire  [15:0] post_rgb               ;
wire         video_vs               ;
wire         pixel_clk              ;
wire         pixel_clk_5x           ;

//*****************************************************
//**                    main code
//*****************************************************

//待时钟锁定后产生复位信号
assign  rst_n = sys_rst_n & locked;

//系统初始化完成：DDR3初始化完成
assign  sys_init_done = ddr3_init_done;

//存入DDR3的最大读写地址 
assign  ddr3_addr_max =  V_CMOS_DISP*H_CMOS_DISP;

//ov5640 驱动
ov5640_dri u_ov5640_dri(
    .clk                        (clk_50m            ),
    .rst_n                      (rst_n              ),
            
    .cam_pclk                   (cam_pclk           ),
    .cam_vsync                  (cam_vsync          ),
    .cam_href                   (cam_href           ),
    .cam_data                   (cam_data           ),
    .cam_rst_n                  (cam_rst_n          ),
    .cam_pwdn                   (cam_pwdn           ),
    .cam_scl                    (cam_scl            ),
    .cam_sda                    (cam_sda            ),
    .cam_init_done              (cam_init_done      ),
    .capture_start              (ddr3_init_done     ),
    .cmos_h_pixel               (H_CMOS_DISP        ),
    .cmos_v_pixel               (V_CMOS_DISP        ),
    .total_h_pixel              (TOTAL_H_PIXEL      ),
    .total_v_pixel              (TOTAL_V_PIXEL      ),
    .cmos_frame_vsync           (cmos_frame_vsync   ),
    .cmos_frame_href            (cmos_frame_href    ),
    .cmos_frame_valid           (cmos_frame_valid   ),
    .cmos_frame_data            (wr_data            )
); 
	
//光口传输顶层模块   
sfp_8b10b_top u_sfp_8b10b_top(
    //光口接口
    .i_p_refckn_1               (i_p_refckn_1       ),
    .i_p_refckp_1               (i_p_refckp_1       ),
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
	.drp_clk    	    		(clk_50m            ),      //光口输入时钟  
	.clk_in     	   			(cam_pclk           ),      //输入时钟
	.rst_n						(sys_rst_n          ),      //输入复位
    .h_pixel                    (H_CMOS_DISP        ),  //输入行像素
	.vs_in						(cmos_frame_vsync   ),      //输入场信号
	.data_valid_in 				(cmos_frame_valid   ),      //输入数据有效信号
	.data_in					(wr_data            ),      //输入数据

	.vs_out						(post_frame_vsync   ),      //输出场信号
	.data_valid_out				(post_frame_de      ),	    //输出数据有效信号
	.data_out	   				(post_rgb           )       //输出数据
);     

// ddr控制模块
ddr3_ctrl_top u_ddr3_ctrl_top(
    .clk            (clk_50m        ),
    .wd_clk         (cam_pclk       ),
    .rd_clk         (pixel_clk      ),
    .rst_n          (sys_rst_n      ),
    
    .wd_en          (post_frame_de  ),
    .wd_data        (post_rgb       ),
    .rd_en          (rdata_req      ),
    .rd_data        (rd_data        ),
    
    .addr_rd_min    (28'd0              ),
    .addr_rd_max    (ddr3_addr_max[26:1]),
    .rd_burst_len   (H_CMOS_DISP[10:6]  ),
    .addr_wd_min    (28'd0              ),
    .addr_wd_max    (ddr3_addr_max[26:1]),
    .wd_burst_len   (H_CMOS_DISP[10:6]  ),
    .ddr3_init_done (ddr3_init_done     ),
    //用户接口
    .wr_load        (post_frame_vsync),    //输入源更新信号
    .rd_load        (video_vs        ),    //输出源更新信号
    .ddr3_pingpang_en(1'b1           ),    //DDR3 乒乓操作
    .ddr3_read_valid (1'b1           ),    //请求数据输入
    
    .mem_rst_n      (mem_rst_n      ),
    .mem_ck         (mem_ck         ),
    .mem_ck_n       (mem_ck_n       ),
    .mem_cke        (mem_cke        ),
    .mem_ras_n      (mem_ras_n      ),
    .mem_cas_n      (mem_cas_n      ),
    .mem_we_n       (mem_we_n       ),
    .mem_odt        (mem_odt        ),
    .mem_a          (mem_a          ),
    .mem_ba         (mem_ba         ),
    .mem_dqs        (mem_dqs        ),
    .mem_dqs_n      (mem_dqs_n      ),
    .mem_cs_n       (mem_cs_n       ),
    .mem_dq         (mem_dq         ),
    .mem_dm         (mem_dm         )
); 

pll_clk  u_pll_clk(
    .rst            (~sys_rst_n  ),
    .clkin1         (sys_clk     ),
    .clkout0        (pixel_clk   ), //像素时钟65M
    .clkout1        (pixel_clk_5x), //5倍像素时钟325M
    .clkout2        (clk_50m     ), //output 50M
    .lock           (locked  )
);

//HDMI驱动显示模块    
hdmi_top u_hdmi_top(
    .hdmi_clk                  (pixel_clk          ),  
    .hdmi_clk_5                (pixel_clk_5x       ),
    .sys_rst_n                 (sys_init_done & rst_n),
    .tmds_clk_p                (tmds_clk_p         ),
    .tmds_clk_n                (tmds_clk_n         ),
    .tmds_data_p               (tmds_data_p        ),
    .tmds_data_n               (tmds_data_n        ),
    .video_vs                  (video_vs           ),
    .pixel_xpos                ( ),
    .pixel_ypos                ( ),
    .rd_data                   (rd_data            ),
    .rd_en                     (rdata_req          )
);  
    
endmodule