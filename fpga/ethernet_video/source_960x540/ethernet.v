module ethernet#(
  parameter BOARD_MAC   =  48'ha0_b1_c2_d3_e4_e4,   
  parameter BOARD_IP    = {8'd192,8'd168,8'd1,8'd10}
)
(  
  input	 wire 						   rst_board ,
  input  wire                          eth_rgmii_rxc_0  /* synthesis PAP_MARK_DEBUG="1" */   ,
  input  wire                          eth_rgmii_rx_ctl_0  /* synthesis PAP_MARK_DEBUG="1" */,
  input  wire [3:0]                    eth_rgmii_rxd_0    /* synthesis PAP_MARK_DEBUG="1" */ ,                   
  output wire                          eth_rgmii_txc_0   /* synthesis PAP_MARK_DEBUG="1" */ ,
  output wire                          eth_rgmii_tx_ctl_0 /* synthesis PAP_MARK_DEBUG="1" */,
  output wire [3:0]                    eth_rgmii_txd_0 /* synthesis PAP_MARK_DEBUG="1" */  ,
  output wire             				rgmii_clk_0/* synthesis PAP_MARK_DEBUG="1" */,
  output wire [15:0]                    eth0_rx_data /* synthesis PAP_MARK_DEBUG="1" */  ,
  output wire            				eth0_rx_de  /* synthesis PAP_MARK_DEBUG="1" */,
  output wire            				eth0_rx_vs  /* synthesis PAP_MARK_DEBUG="1" */
   );
   
wire             eth0_img_de/* synthesis PAP_MARK_DEBUG="1" */;
wire [15 : 0]    eth0_img_data/* synthesis PAP_MARK_DEBUG="1" */;
wire             tx_req/* synthesis PAP_MARK_DEBUG="1" */;
wire             udp_tx_done/* synthesis PAP_MARK_DEBUG="1" */;
wire             tx_start_en/* synthesis PAP_MARK_DEBUG="1" */;
wire [31 : 0]    tx_data    /* synthesis PAP_MARK_DEBUG="1" */;
wire [15 : 0]    tx_byte_num/* synthesis PAP_MARK_DEBUG="1" */;
wire             mac_tx_data_valid_0/* synthesis PAP_MARK_DEBUG="1" */;
wire [7  : 0]    mac_tx_data_0      /* synthesis PAP_MARK_DEBUG="1" */;
wire             mac_rx_error_0     /* synthesis PAP_MARK_DEBUG="1" */;
wire             mac_rx_data_valid_0/* synthesis PAP_MARK_DEBUG="1" */;
wire [7  : 0]    mac_rx_data_0      /* synthesis PAP_MARK_DEBUG="1" */;
wire             rec_pkt_done/* synthesis PAP_MARK_DEBUG="1" */;
wire             rec_en      /* synthesis PAP_MARK_DEBUG="1" */;
wire [31 : 0]    rec_data    /* synthesis PAP_MARK_DEBUG="1" */;
wire [15 : 0]    rec_byte_num/* synthesis PAP_MARK_DEBUG="1" */;

wire pll_lock;
wire rgmii_rxc_phase;
wire rgmii_rxc_phase_d;
wire rgmii_clk_90p ;
pll_phase pll_phase (
  .clkin1(eth_rgmii_rxc_0),        // input
  .pll_lock(pll_lock),    // output
  .clkout0(rgmii_rxc_phase),      // output
  .clkout1(rgmii_rxc_phase_d)       // output
);  
   
eth_img_rec
//#(
//parameter integer PIXEL_WIDTH = 32                                   ,
//parameter integer VIDEO_LENGTH = 16'd960                             ,
//parameter integer VIDEO_HIGTH  = 16'd540                             
//)
eth0_img_rec(
.eth_rx_clk   (rgmii_clk_0  ),//input wire                         
.rstn         (rst_board    ),//input wire                         
.udp_date_rcev(rec_data     ),//input wire [31: 0]   
.udp_date_en  (rec_en       ),//input wire                         
.img_data_en  (eth0_rx_de  ),//output reg                         
.img_data_vs  (eth0_rx_vs  ),//output reg                         
.img_data     (eth0_rx_data) //output reg [15: 0]   
 );  
   
   
//ETH0_GMII_RGMII
gmii_to_rgmii eth0_gmii_to_rgmii(
         .rst                      (  ~rst_board              ),//input        rst,
        .rgmii_clk                 (  rgmii_clk_0          ),//output       rgmii_clk,
        .rgmii_clk_90p             (  rgmii_clk_90p      ),//input        rgmii_clk_90p,
  
        .mac_tx_data_valid         (  mac_tx_data_valid_0     ),//input        mac_tx_data_valid,
        .mac_tx_data               (  mac_tx_data_0        ),//input [7:0]  mac_tx_data,
    
        .mac_rx_error              (  mac_rx_error_0                   ),//output       mac_rx_error,
        .mac_rx_data_valid         (  mac_rx_data_valid_0  ),//output       mac_rx_data_valid,
        .mac_rx_data               (  mac_rx_data_0)        ,//output [7:0] mac_rx_data,
                                                         
        .rgmii_rxc                 (  rgmii_rxc_phase         ),//input        rgmii_rxc,
        .rgmii_rx_ctl              (  eth_rgmii_rx_ctl_0       ),//input        rgmii_rx_ctl,
        .rgmii_rxd                 (  eth_rgmii_rxd_0         ),//input [3:0]  rgmii_rxd,
              
        .rgmii_rxc_phase_d         (  rgmii_rxc_phase_d          ),                                           
        .rgmii_txc                 (  eth_rgmii_txc_0          ),//output       rgmii_txc,
        .rgmii_tx_ctl              (  eth_rgmii_tx_ctl_0       ),//output       rgmii_tx_ctl,
        .rgmii_txd                 (  eth_rgmii_txd_0          ) //output [3:0] rgmii_txd 
    );

//UDP通信
udp_top                                             
   #(
    .BOARD_MAC     (BOARD_MAC),      //参数例化
    .BOARD_IP      (BOARD_IP )
    )
u_udp(
    .rst_n         (rst_board   ),  //input       复位信号，低电平有效            
    //GMII接口                                
    .gmii_rx_clk   (rgmii_clk_0         ),  //input       GMII接收数据时钟                    
    .gmii_rx_dv    (mac_rx_data_valid_0 ),  //input       GMII输入数据有效信号                
    .gmii_rxd      (mac_rx_data_0       ),  //input [7:0] GMII输入数据                              
    .gmii_tx_clk   (rgmii_clk_0         ),  //input       GMII发送数据时钟            
    .gmii_tx_en    (mac_tx_data_valid_0 ),  //output      GMII输出数据有效信号                  
    .gmii_txd      (mac_tx_data_0       ),  //output[7:0] GMII输出数据              
    //用户接口                                  
    .rec_pkt_done  (rec_pkt_done        ),  //output      以太网单包数据接收完成信号          
    .rec_en        (rec_en              ),  //output      以太网接收的数据使能信号            
    .rec_data      (rec_data            ),  //output[31:0]以太网接收的数据                    
    .rec_byte_num  (rec_byte_num        ),  //output[15:0]以太网接收的有效字节数 单位:byte  
    
    .tx_start_en   (tx_start_en         ),  //input       以太网开始发送信号                  
    .tx_data       (tx_data             ),  //input [31:0]以太网待发送数据                    
    .tx_byte_num   (tx_byte_num         ),  //input [15:0]以太网发送的有效字节数 单位:byte   
    .des_mac       (DES_MAC             ),  //input [47:0]发送的目标MAC地址            
    .des_ip        (DES_IP              ),  //input [31:0]发送的目标IP地址              
    .tx_done       (udp_tx_done         ),  //output      以太网发送完成信号                  
    .tx_req        (tx_req              )   //output      读数据请求信号                      
    );    
   
   
   
   
endmodule