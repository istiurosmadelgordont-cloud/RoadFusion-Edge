// *********************************************************************
// 
// Copyright (C) 2021-20xx CrazyBird Corporation
// 
// Filename     :   bilateral_filter_proc.v
// Author       :   CrazyBird
// Email        :   CrazyBirdLin@qq.com
// 
// Description  :   
// 
// Modification History
// Date         By          Version         Change Description
//----------------------------------------------------------------------
// 2026/10/08   Codex       1.3             Register range LUT before spatial weighting
// 2022/03/16   CrazyBird   1.0             Original
// 2026/10/04   Codex       1.2             Add optional aligned Y-guided range weights
// 2026/10/04   Codex       1.1             Correct weights, rounded mean and pipelined division
// 
// *********************************************************************
module bilateral_filter_proc
#(
    parameter   RUNTIME_CONTROL                  =   0,
    parameter   [10:0]  IMG_HDISP = 11'd1920,            //  1920*1080
    parameter   [10:0]  IMG_VDISP = 11'd1080,
    parameter           RANGE_SIGMA = 0,             //  0: legacy LUT; 16/32/48: Gaussian Y denoise
    parameter           GUIDE_ENABLE = 0,             //  0: ordinary; 1: original Y guide
    parameter   [10:0]  DELAY_NUM = 11'd280             //  Horizontal blanking clocks for 1080p60
)
(
    input   [159 : 0]       cfg_data                         ,
    input  wire                 clk             ,
    input  wire                 rst_n           ,
    
    //  Image data prepared to be processed
    input  wire                 per_img_vsync   ,       //  Prepared Image data vsync valid signal
    input  wire                 per_img_href    ,       //  Prepared Image data href vaild  signal
    input  wire     [7:0]       per_img_gray    ,       //  Prepared Image brightness input
    
    input  wire     [7:0]       per_img_guide   ,       //  Y belonging to the same input pixel

    //  Image data has been processed
    output reg                  post_img_vsync  ,       //  processed Image data vsync valid signal
    output reg                  post_img_href   ,       //  processed Image data href vaild  signal
    output reg      [7:0]       post_img_gray           //  processed Image brightness output
);
//----------------------------------------------------------------------
//  Generate 8Bit 3X3 Matrix
wire                            matrix_img_vsync;
wire                            matrix_img_href;
wire                            matrix_top_edge_flag;
wire                            matrix_bottom_edge_flag;
wire                            matrix_left_edge_flag;
wire                            matrix_right_edge_flag;
wire            [7:0]           matrix_p11;
wire            [7:0]           matrix_p12;
wire            [7:0]           matrix_p13;
wire            [7:0]           matrix_p21;
wire            [7:0]           matrix_p22;
wire            [7:0]           matrix_p23;
wire            [7:0]           matrix_p31; 
wire            [7:0]           matrix_p32;
wire            [7:0]           matrix_p33;

Matrix_Generate_3X3_8Bit
#(
    .IMG_HDISP  (IMG_HDISP  ),
    .IMG_VDISP  (IMG_VDISP  ),
    .DELAY_NUM  (DELAY_NUM  )
)
u_Matrix_Generate_3X3_8Bit
(
    //  global clock & reset
    .clk                    (clk                    ),
    .rst_n                  (rst_n                  ),
    
    //  Image data prepared to be processed
    .per_img_vsync          (per_img_vsync          ),      //  Prepared Image data vsync valid signal
    .per_img_href           (per_img_href           ),      //  Prepared Image data href vaild  signal
    .per_img_gray           (per_img_gray           ),      //  Prepared Image brightness input
    
    //  Image data has been processed
    .matrix_img_vsync       (matrix_img_vsync       ),      //  processed Image data vsync valid signal
    .matrix_img_href        (matrix_img_href        ),      //  processed Image data href vaild  signal
    .matrix_top_edge_flag   (matrix_top_edge_flag   ),      //  processed Image top edge
    .matrix_bottom_edge_flag(matrix_bottom_edge_flag),      //  processed Image bottom edge
    .matrix_left_edge_flag  (matrix_left_edge_flag  ),      //  processed Image left edge
    .matrix_right_edge_flag (matrix_right_edge_flag ),      //  processed Image right edge
    .matrix_p11             (matrix_p11             ),      //  3X3 Matrix output
    .matrix_p12             (matrix_p12             ),
    .matrix_p13             (matrix_p13             ),
    .matrix_p21             (matrix_p21             ),
    .matrix_p22             (matrix_p22             ),
    .matrix_p23             (matrix_p23             ),
    .matrix_p31             (matrix_p31             ),  
    .matrix_p32             (matrix_p32             ),
    .matrix_p33             (matrix_p33             )
);

// The target t and guide Y use identical line buffers and window centres.
wire            [7:0]           guide_p11;
wire            [7:0]           range_p11;
wire            [7:0]           guide_p12;
wire            [7:0]           range_p12;
wire            [7:0]           guide_p13;
wire            [7:0]           range_p13;
wire            [7:0]           guide_p21;
wire            [7:0]           range_p21;
wire            [7:0]           guide_p22;
wire            [7:0]           range_p22;
wire            [7:0]           guide_p23;
wire            [7:0]           range_p23;
wire            [7:0]           guide_p31;
wire            [7:0]           range_p31;
wire            [7:0]           guide_p32;
wire            [7:0]           range_p32;
wire            [7:0]           guide_p33;
wire            [7:0]           range_p33;

generate
if (GUIDE_ENABLE) begin : g_guide_matrix
Matrix_Generate_3X3_8Bit
#(
    .IMG_HDISP  (IMG_HDISP  ),
    .IMG_VDISP  (IMG_VDISP  ),
    .DELAY_NUM  (DELAY_NUM  )
)
u_guide_matrix
(
    //  global clock & reset
    .clk                    (clk                    ),
    .rst_n                  (rst_n                  ),

    //  Image data prepared to be processed
    .per_img_vsync          (per_img_vsync          ),      //  Prepared Image data vsync valid signal
    .per_img_href           (per_img_href           ),      //  Prepared Image data href vaild  signal
    .per_img_gray           (per_img_guide          ),      //  Prepared Image brightness input

    //  Image data has been processed
    .matrix_img_vsync       (                       ),      //  processed Image data vsync valid signal
    .matrix_img_href        (                       ),      //  processed Image data href vaild  signal
    .matrix_top_edge_flag   (                       ),      //  processed Image top edge
    .matrix_bottom_edge_flag(                       ),      //  processed Image bottom edge
    .matrix_left_edge_flag  (                       ),      //  processed Image left edge
    .matrix_right_edge_flag (                       ),      //  processed Image right edge
    .matrix_p11             (guide_p11              ),      //  3X3 Matrix output
    .matrix_p12             (guide_p12              ),
    .matrix_p13             (guide_p13              ),
    .matrix_p21             (guide_p21              ),
    .matrix_p22             (guide_p22              ),
    .matrix_p23             (guide_p23              ),
    .matrix_p31             (guide_p31              ),
    .matrix_p32             (guide_p32              ),
    .matrix_p33             (guide_p33              )
);

end
else begin : g_target_guide
    assign guide_p11 = matrix_p11;
    assign guide_p12 = matrix_p12;
    assign guide_p13 = matrix_p13;
    assign guide_p21 = matrix_p21;
    assign guide_p22 = matrix_p22;
    assign guide_p23 = matrix_p23;
    assign guide_p31 = matrix_p31;
    assign guide_p32 = matrix_p32;
    assign guide_p33 = matrix_p33;
end
endgenerate

assign range_p11 = GUIDE_ENABLE ? guide_p11 : matrix_p11;
assign range_p12 = GUIDE_ENABLE ? guide_p12 : matrix_p12;
assign range_p13 = GUIDE_ENABLE ? guide_p13 : matrix_p13;
assign range_p21 = GUIDE_ENABLE ? guide_p21 : matrix_p21;
assign range_p22 = GUIDE_ENABLE ? guide_p22 : matrix_p22;
assign range_p23 = GUIDE_ENABLE ? guide_p23 : matrix_p23;
assign range_p31 = GUIDE_ENABLE ? guide_p31 : matrix_p31;
assign range_p32 = GUIDE_ENABLE ? guide_p32 : matrix_p32;
assign range_p33 = GUIDE_ENABLE ? guide_p33 : matrix_p33;

//----------------------------------------------------------------------
reg             [7:0]           p11_minus_p22_abs;
reg             [7:0]           p21_minus_p22_abs;
reg             [7:0]           p31_minus_p22_abs;
reg             [7:0]           p12_minus_p22_abs;
reg             [7:0]           p22_minus_p22_abs;
reg             [7:0]           p32_minus_p22_abs;
reg             [7:0]           p13_minus_p22_abs;
reg             [7:0]           p23_minus_p22_abs;
reg             [7:0]           p33_minus_p22_abs;

always @(posedge clk)
begin
    if(range_p11 > range_p22)
        p11_minus_p22_abs <= range_p11 - range_p22;
    else
        p11_minus_p22_abs <= range_p22 - range_p11;
    
    if(range_p21 > range_p22)
        p21_minus_p22_abs <= range_p21 - range_p22;
    else
        p21_minus_p22_abs <= range_p22 - range_p21;
    
    if(range_p31 > range_p22)
        p31_minus_p22_abs <= range_p31 - range_p22;
    else
        p31_minus_p22_abs <= range_p22 - range_p31;
    
    if(range_p12 > range_p22)
        p12_minus_p22_abs <= range_p12 - range_p22;
    else
        p12_minus_p22_abs <= range_p22 - range_p12;
    
    p22_minus_p22_abs <= 8'b0;
    
    if(range_p32 > range_p22)
        p32_minus_p22_abs <= range_p32 - range_p22;
    else
        p32_minus_p22_abs <= range_p22 - range_p32;
    
    if(range_p13 > range_p22)
        p13_minus_p22_abs <= range_p13 - range_p22;
    else
        p13_minus_p22_abs <= range_p22 - range_p13;
    
    if(range_p23 > range_p22)
        p23_minus_p22_abs <= range_p23 - range_p22;
    else
        p23_minus_p22_abs <= range_p22 - range_p23;
    
    if(range_p33 > range_p22)
        p33_minus_p22_abs <= range_p33 - range_p22;
    else
        p33_minus_p22_abs <= range_p22 - range_p33;
end

wire            [9:0]           similary_p11_p22;
wire            [9:0]           similary_p21_p22;
wire            [9:0]           similary_p31_p22;
wire            [9:0]           similary_p12_p22;
wire            [9:0]           similary_p22_p22;
wire            [9:0]           similary_p32_p22;
wire            [9:0]           similary_p13_p22;
wire            [9:0]           similary_p23_p22;
wire            [9:0]           similary_p33_p22;

wire [159 : 0] range_config;

frame_config_delay#(
    .DATA_WIDTH                     (160                             ),
    .DELAY_CLOCKS                   (IMG_HDISP + DELAY_NUM + 3       )
)u_range_config(
    .clk                            (clk                             ),
    .rst_n                          (rst_n                           ),
    .pre_vs                         (per_img_vsync                   ),
    .pre_config                     (RUNTIME_CONTROL ? cfg_data : 160'd0),
    .post_config                    (range_config                    )
);
generate
if (RANGE_SIGMA != 0) begin : g_denoise_similarity
Bilateral_Range_LUT #(
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),.SIGMA(RANGE_SIGMA))u0_range(
    .clk                            (clk                             ),
    .cfg_data                       (range_config                    ),
        .Pre_Data(p11_minus_p22_abs), .Post_Data(similary_p11_p22)
);
Bilateral_Range_LUT #(
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),.SIGMA(RANGE_SIGMA))u1_range(
    .clk                            (clk                             ),
    .cfg_data                       (range_config                    ),
        .Pre_Data(p21_minus_p22_abs), .Post_Data(similary_p21_p22)
);
Bilateral_Range_LUT #(
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),.SIGMA(RANGE_SIGMA))u2_range(
    .clk                            (clk                             ),
    .cfg_data                       (range_config                    ),
        .Pre_Data(p31_minus_p22_abs), .Post_Data(similary_p31_p22)
);
Bilateral_Range_LUT #(
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),.SIGMA(RANGE_SIGMA))u3_range(
    .clk                            (clk                             ),
    .cfg_data                       (range_config                    ),
        .Pre_Data(p12_minus_p22_abs), .Post_Data(similary_p12_p22)
);
Bilateral_Range_LUT #(
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),.SIGMA(RANGE_SIGMA))u4_range(
    .clk                            (clk                             ),
    .cfg_data                       (range_config                    ),
        .Pre_Data(p22_minus_p22_abs), .Post_Data(similary_p22_p22)
);
Bilateral_Range_LUT #(
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),.SIGMA(RANGE_SIGMA))u5_range(
    .clk                            (clk                             ),
    .cfg_data                       (range_config                    ),
        .Pre_Data(p32_minus_p22_abs), .Post_Data(similary_p32_p22)
);
Bilateral_Range_LUT #(
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),.SIGMA(RANGE_SIGMA))u6_range(
    .clk                            (clk                             ),
    .cfg_data                       (range_config                    ),
        .Pre_Data(p13_minus_p22_abs), .Post_Data(similary_p13_p22)
);
Bilateral_Range_LUT #(
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),.SIGMA(RANGE_SIGMA))u7_range(
    .clk                            (clk                             ),
    .cfg_data                       (range_config                    ),
        .Pre_Data(p23_minus_p22_abs), .Post_Data(similary_p23_p22)
);
Bilateral_Range_LUT #(
    .RUNTIME_CONTROL                (RUNTIME_CONTROL                 ),.SIGMA(RANGE_SIGMA))u8_range(
    .clk                            (clk                             ),
    .cfg_data                       (range_config                    ),
        .Pre_Data(p33_minus_p22_abs), .Post_Data(similary_p33_p22)
);
end
else if (GUIDE_ENABLE) begin : g_joint_similarity
Joint_Similarity_LUT u0_Joint_Similarity_LUT
(
   .Pre_Data    (p11_minus_p22_abs  ),
   .Post_Data   (similary_p11_p22   )
);

Joint_Similarity_LUT u1_Joint_Similarity_LUT
(
   .Pre_Data    (p21_minus_p22_abs  ),
   .Post_Data   (similary_p21_p22   )
);

Joint_Similarity_LUT u2_Joint_Similarity_LUT
(
   .Pre_Data    (p31_minus_p22_abs  ),
   .Post_Data   (similary_p31_p22   )
);

Joint_Similarity_LUT u3_Joint_Similarity_LUT
(
   .Pre_Data    (p12_minus_p22_abs  ),
   .Post_Data   (similary_p12_p22   )
);

Joint_Similarity_LUT u4_Joint_Similarity_LUT
(
   .Pre_Data    (p22_minus_p22_abs  ),
   .Post_Data   (similary_p22_p22   )
);

Joint_Similarity_LUT u5_Joint_Similarity_LUT
(
   .Pre_Data    (p32_minus_p22_abs  ),
   .Post_Data   (similary_p32_p22   )
);

Joint_Similarity_LUT u6_Joint_Similarity_LUT
(
   .Pre_Data    (p13_minus_p22_abs  ),
   .Post_Data   (similary_p13_p22   )
);

Joint_Similarity_LUT u7_Joint_Similarity_LUT
(
   .Pre_Data    (p23_minus_p22_abs  ),
   .Post_Data   (similary_p23_p22   )
);

Joint_Similarity_LUT u8_Joint_Similarity_LUT
(
   .Pre_Data    (p33_minus_p22_abs  ),
   .Post_Data   (similary_p33_p22   )
);

end
else begin : g_ordinary_similarity
Similary_LUT u0_Similary_LUT
(
   .Pre_Data    (p11_minus_p22_abs  ),
   .Post_Data   (similary_p11_p22   )
);

Similary_LUT u1_Similary_LUT
(
   .Pre_Data    (p21_minus_p22_abs  ),
   .Post_Data   (similary_p21_p22   )
);

Similary_LUT u2_Similary_LUT
(
   .Pre_Data    (p31_minus_p22_abs  ),
   .Post_Data   (similary_p31_p22   )
);

Similary_LUT u3_Similary_LUT
(
   .Pre_Data    (p12_minus_p22_abs  ),
   .Post_Data   (similary_p12_p22   )
);

Similary_LUT u4_Similary_LUT
(
   .Pre_Data    (p22_minus_p22_abs  ),
   .Post_Data   (similary_p22_p22   )
);

Similary_LUT u5_Similary_LUT
(
   .Pre_Data    (p32_minus_p22_abs  ),
   .Post_Data   (similary_p32_p22   )
);

Similary_LUT u6_Similary_LUT
(
   .Pre_Data    (p13_minus_p22_abs  ),
   .Post_Data   (similary_p13_p22   )
);

Similary_LUT u7_Similary_LUT
(
   .Pre_Data    (p23_minus_p22_abs  ),
   .Post_Data   (similary_p23_p22   )
);

Similary_LUT u8_Similary_LUT
(
   .Pre_Data    (p33_minus_p22_abs  ),
   .Post_Data   (similary_p33_p22   )
);

end
endgenerate

//----------------------------------------------------------------------
//      [g11,g12,g13]   [109,115,109]
//  g = [g21,g22,g23] = [115,122,115]
//      [g31,g32,g33]   [109,115,109]
localparam g11 = 7'd109;
localparam g12 = 7'd115;
localparam g13 = 7'd109;
localparam g21 = 7'd115;
localparam g22 = 7'd122;
localparam g23 = 7'd115;
localparam g31 = 7'd109;
localparam g32 = 7'd115;
localparam g33 = 7'd109;

// Register both LUT branches before the spatial multipliers.
reg     [9 : 0]         legacy_similarity_11_d1          ;
reg     [9 : 0]         legacy_similarity_12_d1          ;
reg     [9 : 0]         legacy_similarity_13_d1          ;
reg     [9 : 0]         legacy_similarity_21_d1          ;
reg     [9 : 0]         legacy_similarity_22_d1          ;
reg     [9 : 0]         legacy_similarity_23_d1          ;
reg     [9 : 0]         legacy_similarity_31_d1          ;
reg     [9 : 0]         legacy_similarity_32_d1          ;
reg     [9 : 0]         legacy_similarity_33_d1          ;

always@(posedge clk)begin
    legacy_similarity_11_d1          <=  similary_p11_p22;
    legacy_similarity_12_d1          <=  similary_p12_p22;
    legacy_similarity_13_d1          <=  similary_p13_p22;
    legacy_similarity_21_d1          <=  similary_p21_p22;
    legacy_similarity_22_d1          <=  similary_p22_p22;
    legacy_similarity_23_d1          <=  similary_p23_p22;
    legacy_similarity_31_d1          <=  similary_p31_p22;
    legacy_similarity_32_d1          <=  similary_p32_p22;
    legacy_similarity_33_d1          <=  similary_p33_p22;
end

reg     [9 : 0]         similarity_11_d1                 ;
reg     [9 : 0]         similarity_12_d1                 ;
reg     [9 : 0]         similarity_13_d1                 ;
reg     [9 : 0]         similarity_21_d1                 ;
reg     [9 : 0]         similarity_22_d1                 ;
reg     [9 : 0]         similarity_23_d1                 ;
reg     [9 : 0]         similarity_31_d1                 ;
reg     [9 : 0]         similarity_32_d1                 ;
reg     [9 : 0]         similarity_33_d1                 ;

always@(posedge clk)begin
    similarity_11_d1                 <=  RANGE_SIGMA != 0 ? similary_p11_p22 :
                                            legacy_similarity_11_d1;
    similarity_12_d1                 <=  RANGE_SIGMA != 0 ? similary_p12_p22 :
                                            legacy_similarity_12_d1;
    similarity_13_d1                 <=  RANGE_SIGMA != 0 ? similary_p13_p22 :
                                            legacy_similarity_13_d1;
    similarity_21_d1                 <=  RANGE_SIGMA != 0 ? similary_p21_p22 :
                                            legacy_similarity_21_d1;
    similarity_22_d1                 <=  RANGE_SIGMA != 0 ? similary_p22_p22 :
                                            legacy_similarity_22_d1;
    similarity_23_d1                 <=  RANGE_SIGMA != 0 ? similary_p23_p22 :
                                            legacy_similarity_23_d1;
    similarity_31_d1                 <=  RANGE_SIGMA != 0 ? similary_p31_p22 :
                                            legacy_similarity_31_d1;
    similarity_32_d1                 <=  RANGE_SIGMA != 0 ? similary_p32_p22 :
                                            legacy_similarity_32_d1;
    similarity_33_d1                 <=  RANGE_SIGMA != 0 ? similary_p33_p22 :
                                            legacy_similarity_33_d1;
end

//----------------------------------------------------------------------
//  Each neighbour uses its own spatial coefficient and range similarity.
//  Keep this existing weight stage in fabric FFs between the spatial and pixel
//  multipliers; DSP register absorption would join the two routed DSP paths.
reg             [16:0]          weight11 /* synthesis syn_preserve=1 */;
reg             [16:0]          weight12 /* synthesis syn_preserve=1 */;
reg             [16:0]          weight13 /* synthesis syn_preserve=1 */;
reg             [16:0]          weight21 /* synthesis syn_preserve=1 */;
reg             [16:0]          weight22 /* synthesis syn_preserve=1 */;
reg             [16:0]          weight23 /* synthesis syn_preserve=1 */;
reg             [16:0]          weight31 /* synthesis syn_preserve=1 */;
reg             [16:0]          weight32 /* synthesis syn_preserve=1 */;
reg             [16:0]          weight33 /* synthesis syn_preserve=1 */;

always @(posedge clk)
begin
    weight11 <= similarity_11_d1 * g11;
    weight12 <= similarity_12_d1 * g12;
    weight13 <= similarity_13_d1 * g13;
    weight21 <= similarity_21_d1 * g21;
    weight22 <= similarity_22_d1 * g22;
    weight23 <= similarity_23_d1 * g23;
    weight31 <= similarity_31_d1 * g31;
    weight32 <= similarity_32_d1 * g32;
    weight33 <= similarity_33_d1 * g33;
end

//----------------------------------------------------------------------
//  Four clocks align the original pixels with their unnormalised weights.
reg             [7:0]           matrix_p11_r1 [0:3];
reg             [7:0]           matrix_p12_r1 [0:3];
reg             [7:0]           matrix_p13_r1 [0:3];
reg             [7:0]           matrix_p21_r1 [0:3];
reg             [7:0]           matrix_p22_r1 [0:3];
reg             [7:0]           matrix_p23_r1 [0:3];
reg             [7:0]           matrix_p31_r1 [0:3];
reg             [7:0]           matrix_p32_r1 [0:3];
reg             [7:0]           matrix_p33_r1 [0:3];

always @(posedge clk)
begin
    matrix_p11_r1[0] <= matrix_p11;
    matrix_p11_r1[1] <= matrix_p11_r1[0];
    matrix_p11_r1[2] <= matrix_p11_r1[1];
    matrix_p11_r1[3] <= matrix_p11_r1[2];
    matrix_p12_r1[0] <= matrix_p12;
    matrix_p12_r1[1] <= matrix_p12_r1[0];
    matrix_p12_r1[2] <= matrix_p12_r1[1];
    matrix_p12_r1[3] <= matrix_p12_r1[2];
    matrix_p13_r1[0] <= matrix_p13;
    matrix_p13_r1[1] <= matrix_p13_r1[0];
    matrix_p13_r1[2] <= matrix_p13_r1[1];
    matrix_p13_r1[3] <= matrix_p13_r1[2];
    matrix_p21_r1[0] <= matrix_p21;
    matrix_p21_r1[1] <= matrix_p21_r1[0];
    matrix_p21_r1[2] <= matrix_p21_r1[1];
    matrix_p21_r1[3] <= matrix_p21_r1[2];
    matrix_p22_r1[0] <= matrix_p22;
    matrix_p22_r1[1] <= matrix_p22_r1[0];
    matrix_p22_r1[2] <= matrix_p22_r1[1];
    matrix_p22_r1[3] <= matrix_p22_r1[2];
    matrix_p23_r1[0] <= matrix_p23;
    matrix_p23_r1[1] <= matrix_p23_r1[0];
    matrix_p23_r1[2] <= matrix_p23_r1[1];
    matrix_p23_r1[3] <= matrix_p23_r1[2];
    matrix_p31_r1[0] <= matrix_p31;
    matrix_p31_r1[1] <= matrix_p31_r1[0];
    matrix_p31_r1[2] <= matrix_p31_r1[1];
    matrix_p31_r1[3] <= matrix_p31_r1[2];
    matrix_p32_r1[0] <= matrix_p32;
    matrix_p32_r1[1] <= matrix_p32_r1[0];
    matrix_p32_r1[2] <= matrix_p32_r1[1];
    matrix_p32_r1[3] <= matrix_p32_r1[2];
    matrix_p33_r1[0] <= matrix_p33;
    matrix_p33_r1[1] <= matrix_p33_r1[0];
    matrix_p33_r1[2] <= matrix_p33_r1[1];
    matrix_p33_r1[3] <= matrix_p33_r1[2];
end

//----------------------------------------------------------------------
//  Preserve the full weighted numerator; do not truncate nine normalised weights.
reg             [24:0]          mult_p11_w11;
reg             [24:0]          mult_p12_w12;
reg             [24:0]          mult_p13_w13;
reg             [24:0]          mult_p21_w21;
reg             [24:0]          mult_p22_w22;
reg             [24:0]          mult_p23_w23;
reg             [24:0]          mult_p31_w31;
reg             [24:0]          mult_p32_w32;
reg             [24:0]          mult_p33_w33;
reg             [18:0]          weight_sum_tmp1;
reg             [18:0]          weight_sum_tmp2;
reg             [18:0]          weight_sum_tmp3;

always @(posedge clk)
begin
    mult_p11_w11 <= matrix_p11_r1[3] * weight11;
    mult_p12_w12 <= matrix_p12_r1[3] * weight12;
    mult_p13_w13 <= matrix_p13_r1[3] * weight13;
    mult_p21_w21 <= matrix_p21_r1[3] * weight21;
    mult_p22_w22 <= matrix_p22_r1[3] * weight22;
    mult_p23_w23 <= matrix_p23_r1[3] * weight23;
    mult_p31_w31 <= matrix_p31_r1[3] * weight31;
    mult_p32_w32 <= matrix_p32_r1[3] * weight32;
    mult_p33_w33 <= matrix_p33_r1[3] * weight33;
    weight_sum_tmp1 <= {2'b0, weight11} + {2'b0, weight12} + {2'b0, weight13};
    weight_sum_tmp2 <= {2'b0, weight21} + {2'b0, weight22} + {2'b0, weight23};
    weight_sum_tmp3 <= {2'b0, weight31} + {2'b0, weight32} + {2'b0, weight33};
end

reg             [26:0]          sum_result_tmp1;

reg             [26:0]          sum_result_tmp2;

reg             [26:0]          sum_result_tmp3;
reg             [19:0]          weight_sum;

always @(posedge clk)
begin
    sum_result_tmp1 <= {2'b0, mult_p11_w11} + {2'b0, mult_p12_w12} + {2'b0, mult_p13_w13};
    sum_result_tmp2 <= {2'b0, mult_p21_w21} + {2'b0, mult_p22_w22} + {2'b0, mult_p23_w23};
    sum_result_tmp3 <= {2'b0, mult_p31_w31} + {2'b0, mult_p32_w32} + {2'b0, mult_p33_w33};
    weight_sum <= {1'b0, weight_sum_tmp1} + {1'b0, weight_sum_tmp2} + {1'b0, weight_sum_tmp3};
end

reg             [27:0]          sum_result;
reg             [19:0]          weight_sum_r1;

always @(posedge clk)
begin
    sum_result    <= {1'b0, sum_result_tmp1} + {1'b0, sum_result_tmp2} + {1'b0, sum_result_tmp3};
    weight_sum_r1 <= weight_sum;
end

//----------------------------------------------------------------------
//  Round once: (sum(weight * pixel) + sum(weight) / 2) / sum(weight).
//  The result is 0..255, so eight quotient stages are sufficient.
//  Each stage performs one compare/subtract and accepts one pixel per clock.
wire            [27:0]          rounded_numerator;
wire            [27:0]          first_divisor;

assign rounded_numerator = sum_result + {8'b0, (weight_sum_r1 >> 1)};
assign first_divisor     = {8'b0, weight_sum_r1} << 7;

reg             [27:0]          div_remainder  [0:7];
reg             [19:0]          div_denominator[0:7];
reg             [7:0]           div_quotient   [0:7];
reg             [7:0]           div_zero;

always @(posedge clk or negedge rst_n)
begin
    if(!rst_n)
    begin
        div_remainder[0]   <= 28'b0;
        div_denominator[0] <= 20'b0;
        div_quotient[0]    <= 8'b0;
        div_zero[0]        <= 1'b0;
    end
    else
    begin
        div_denominator[0] <= weight_sum_r1;
        div_zero[0]        <= (weight_sum_r1 == 20'b0);

        if(rounded_numerator >= first_divisor)
        begin
            div_remainder[0] <= rounded_numerator - first_divisor;
            div_quotient[0]  <= 8'h80;
        end
        else
        begin
            div_remainder[0] <= rounded_numerator;
            div_quotient[0]  <= 8'b0;
        end
    end
end

genvar stage;
generate
    for(stage = 1; stage < 8; stage = stage + 1)
    begin : divider_stage
        wire [27:0] shifted_divisor;
        assign shifted_divisor = {8'b0, div_denominator[stage-1]} << (7-stage);

        always @(posedge clk or negedge rst_n)
        begin
            if(!rst_n)
            begin
                div_remainder[stage]   <= 28'b0;
                div_denominator[stage] <= 20'b0;
                div_quotient[stage]    <= 8'b0;
                div_zero[stage]        <= 1'b0;
            end
            else
            begin
                div_denominator[stage] <= div_denominator[stage-1];
                div_zero[stage]        <= div_zero[stage-1];

                if(div_remainder[stage-1] >= shifted_divisor)
                begin
                    div_remainder[stage] <= div_remainder[stage-1] - shifted_divisor;
                    div_quotient[stage]  <= div_quotient[stage-1] | (8'h01 << (7-stage));
                end
                else
                begin
                    div_remainder[stage] <= div_remainder[stage-1];
                    div_quotient[stage]  <= div_quotient[stage-1];
                end
            end
        end
    end
endgenerate

//----------------------------------------------------------------------
//  Matrix to output: 16 registers, including the final output register.
//  The preceding line buffers are additional to this arithmetic latency.
localparam C_CLK_LATENCY = 16;

reg     [C_CLK_LATENCY-2:0]      matrix_img_vsync_r1;
reg     [C_CLK_LATENCY-2:0]      matrix_img_href_r1;
reg     [C_CLK_LATENCY-2:0]      matrix_edge_flag_r1;
reg     [7:0]                   matrix_center_r1 [0:C_CLK_LATENCY-2];

always @(posedge clk or negedge rst_n)
begin : signal_alignment
    integer i;

    if(!rst_n)
    begin
        matrix_img_vsync_r1 <= {(C_CLK_LATENCY-1){1'b0}};
        matrix_img_href_r1  <= {(C_CLK_LATENCY-1){1'b0}};
        matrix_edge_flag_r1 <= {(C_CLK_LATENCY-1){1'b0}};

        for(i = 0; i < C_CLK_LATENCY-1; i = i + 1)
            matrix_center_r1[i] <= 8'b0;
    end
    else
    begin
        matrix_img_vsync_r1 <= {matrix_img_vsync_r1[C_CLK_LATENCY-3:0], matrix_img_vsync};
        matrix_img_href_r1  <= {matrix_img_href_r1[C_CLK_LATENCY-3:0], matrix_img_href};
        matrix_edge_flag_r1 <= {matrix_edge_flag_r1[C_CLK_LATENCY-3:0],
                               matrix_top_edge_flag | matrix_bottom_edge_flag |
                               matrix_left_edge_flag | matrix_right_edge_flag};

        matrix_center_r1[0] <= matrix_p22;
        for(i = 1; i < C_CLK_LATENCY-1; i = i + 1)
            matrix_center_r1[i] <= matrix_center_r1[i-1];
    end
end

//----------------------------------------------------------------------
//  Keep frame edges unchanged; use the centre pixel if the divisor is zero.
always @(posedge clk or negedge rst_n)
begin
    if(!rst_n)
    begin
        post_img_vsync <= 1'b0;
        post_img_href  <= 1'b0;
        post_img_gray  <= 8'b0;
    end
    else
    begin
        post_img_vsync <= matrix_img_vsync_r1[C_CLK_LATENCY-2];
        post_img_href  <= matrix_img_href_r1[C_CLK_LATENCY-2];

        if(!matrix_img_href_r1[C_CLK_LATENCY-2])
            post_img_gray <= 8'b0;
        else if(matrix_edge_flag_r1[C_CLK_LATENCY-2] || div_zero[7])
            post_img_gray <= matrix_center_r1[C_CLK_LATENCY-2];
        else
            post_img_gray <= div_quotient[7];
    end
end

endmodule

