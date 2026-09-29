//原子哥在线教学平台：www.yuanzige.com
//技术支持：http://www.openedv.com/forum.php
//淘宝店铺：https://zhengdianyuanzi.tmall.com
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料�?
//版权所有，盗版必究�?
//Copyright(C) 正点原子 2023-2033
//All rights reserved                                  
//----------------------------------------------------------------------------------------
// File name:           sfp_8b10b_top
// Created by:          正点原子
// Created date:        2023�?�?�?4:17:02
// Version:             V1.0
// Descriptions:        sfp_8b10b_top
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module sfp_8b10b_top(
    output  [1:0]   tx_disable,

    // HSST lane2 fabric interface. The physical transceiver is owned by
    // the PCIe IP so PCIe and SFP share one GTP_HSST_E2 instance.
    input           tx0_user_clk,
    input           tx0_rst_n,
    output  [3:0]   tx0_gt_txcharisk,
    output  [31:0]  tx0_gt_txdata,
    input           rx1_user_clk,
    input           rx1_rst_n,
    input   [3:0]   rx1_gt_rxcharisk,
    input   [31:0]  rx1_gt_rxdata,
    output          i_wtchdg_clr_0,
    output          i_pll_rst_0,

    input           clk_in,
    input           rx_clk_in,
    input           rst_n,
    input   [10:0]  h_pixel,
    input           vs_in,
    input           data_valid_in,
    input   [15:0]  data_in,

    output          o_vs/* synthesis PAP_MARK_DEBUG="1" */,
    output          data_valid_out/* synthesis PAP_MARK_DEBUG="1" */,
    output  [15:0]  data_out/* synthesis PAP_MARK_DEBUG="1" */
    );
//localparam define     
localparam VS_POSE_DATA1	=	32'h55a101bc;
localparam VS_POSE_DATA2	=	32'h55a102bc;	
localparam DATA_START1      =	32'h55a105bc;
localparam DATA_START2      =	32'h55a106bc;
localparam DATA_END1        =	32'h55a107bc;
localparam DATA_END2	    =	32'h55a108bc;
localparam UNUSE_DATA	    =	32'h55a109bc; 

//wire define    
wire [3:0]  rx1_charisk_align/* synthesis syn_keep="1" */;
wire [31:0] rx1_data_align/* synthesis syn_keep="1" */; 
wire        pll_rst_d;
wire        i_p_cfg_rst;

//*****************************************************
//**                    main code
//*****************************************************
 
assign     tx_disable 	 = 2'b00;
assign     i_p_cfg_rst	 = ~rst_n;
assign     i_pll_rst_0	 = pll_rst_d;
assign     pll_rst_d  	 = i_p_cfg_rst;
assign     i_wtchdg_clr_0 = pll_rst_d;

//光口编码模块
sfp_encoder#(
	.VS_POSE_DATA1	    (VS_POSE_DATA1	),
	.VS_POSE_DATA2	    (VS_POSE_DATA2	),	
	.DATA_START1	    (DATA_START1	),
	.DATA_START2	    (DATA_START2	),
	.DATA_END1   	    (DATA_END1   	),
	.DATA_END2	        (DATA_END2	    ),
	.UNUSE_DATA	        (UNUSE_DATA	    )
)
u_sfp_encoder(
	.clk_in             (clk_in         ),  //输入时钟
	.rst_n              (rst_n          ),  //输入复位
    .h_pixel            (h_pixel        ),  //输入行像�?
	.vs_in              (vs_in          ),  //输入场信�?
	.data_valid_in      (data_valid_in  ),  //输入数据有效信号
	.data_in            (data_in        ),  //输入数据
	.tx_clk             (tx0_user_clk   ),  //光口tx端时�?
	.tx_rst_n           (tx0_rst_n      ),  //光口tx端复�?
            
	.gt_txcharisk       (tx0_gt_txcharisk), //光口tx端K码发送信�?
	.gt_txdata	        (tx0_gt_txdata  )   //光口tx端发送数�?
); 

//光口解码模块    
sfp_decode#(
	.VS_POSE_DATA1	    (VS_POSE_DATA1	),
	.VS_POSE_DATA2	    (VS_POSE_DATA2	),	
	.DATA_START1	    (DATA_START1	),
	.DATA_START2	    (DATA_START2	),
	.DATA_END1   	    (DATA_END1   	),
	.DATA_END2	        (DATA_END2	    ),
	.UNUSE_DATA	        (UNUSE_DATA	    )
)
u_sfp_decode(
	.clk_in             (rx_clk_in      ),
	.rst_n              (rst_n          ),
    .rx_clk             (rx1_user_clk   ),
	.rx_rst_n           (rx1_rst_n      ),
	.rx_charisk_align   (rx1_charisk_align),
	.rx_data_align      (rx1_data_align ),
        
	.o_vs               (o_vs           ),
	.data_valid_out     (data_valid_out ),
	.data_out           (data_out       )
);   
    
//光口字对齐模�?   
sfp_data_align u_sfp_data_align(
	.clk_in             (rx1_user_clk   ),  //rx端时�?
	.rst_n			    (rx1_rst_n      ),  //rx端复�?
	.rx_data_in		    (rx1_gt_rxdata  ),  //rx端K码未校正接收信号
	.rx_charisk_in	    (rx1_gt_rxcharisk), //rx端未校正发送数�?
    
	.rx_data_out	    (rx1_data_align ),  //rx端K码校正后接收信号
	.rx_charisk_out	    (rx1_charisk_align) //rx端校正后发送数�?
);   


endmodule
