module  matrix_generate_3x3#(
    parameter DATA_WIDTH = 8,
    parameter DATA_DEPTH = 1920
)
(
    input                           clk,  
    input                           rst_n,

    input                           pre_frame_vsync,
    input                           pre_frame_href,
    input                           pre_frame_clken,
    input      [DATA_WIDTH - 1 :0]  pre_img_y,
    
    output                          matrix_frame_vsync,
    output                          matrix_frame_href,
    output                          matrix_frame_clken,
    output reg [DATA_WIDTH - 1 :0]  matrix_p11,
    output reg [DATA_WIDTH - 1 :0]  matrix_p12, 
    output reg [DATA_WIDTH - 1 :0]  matrix_p13,
    output reg [DATA_WIDTH - 1 :0]  matrix_p21, 
    output reg [DATA_WIDTH - 1 :0]  matrix_p22, 
    output reg [DATA_WIDTH - 1 :0]  matrix_p23,
    output reg [DATA_WIDTH - 1 :0]  matrix_p31, 
    output reg [DATA_WIDTH - 1 :0]  matrix_p32, 
    output reg [DATA_WIDTH - 1 :0]  matrix_p33
);

//wire define
wire    [DATA_WIDTH - 1 : 0]    row1_data;  
wire    [DATA_WIDTH - 1 : 0]    row2_data;  
wire    [DATA_WIDTH - 1 : 0]    row3_data;  
wire                            read_frame_href;
wire                            read_frame_clken;

//reg define
reg     [1:0]                   pre_frame_vsync_r;
reg     [1:0]                   pre_frame_href_r;
reg     [1:0]                   pre_frame_clken_r;

//*****************************************************
//**                    main code
//*****************************************************

// Mask unavailable earlier rows; do not reset RAM arrays, preserving block-RAM inference.
reg [1:0] available_rows;
reg read_href_d1;
wire [DATA_WIDTH-1:0] valid_row1 = available_rows >= 2 ? row1_data : {DATA_WIDTH{1'b0}};
wire [DATA_WIDTH-1:0] valid_row2 = available_rows >= 1 ? row2_data : {DATA_WIDTH{1'b0}};
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        available_rows  <= 0;
        read_href_d1    <= 0;
    end
    else begin
        read_href_d1 <= read_frame_href;

        if (!pre_frame_vsync_r[0]) begin
            available_rows <= 0;
        end
        else if (read_href_d1 && !read_frame_href && available_rows < 2) begin
            available_rows <= available_rows + 1'b1;
        end
    end
end

assign read_frame_href    = pre_frame_href_r[0] ;
assign read_frame_clken   = pre_frame_clken_r[0];
assign matrix_frame_vsync = pre_frame_vsync_r[1];
assign matrix_frame_href  = pre_frame_href_r[1] ;
assign matrix_frame_clken = pre_frame_clken_r[1];


one_column_ram #(
    .DATA_WIDTH(DATA_WIDTH),
    .DATA_DEPTH(DATA_DEPTH)
)u_one_column_ram(
    .clock      (clk),
    .rst_n      (rst_n),
    .sclr       (~pre_frame_vsync),   
    .clken      (pre_frame_clken),
    .shiftin    (pre_img_y),

    .taps0x     (row3_data),
    .taps1x     (row2_data),
    .taps2x     (row1_data)
);

//将同步信号延迟两拍，用于同步化处理
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        pre_frame_vsync_r <= 0;
        pre_frame_href_r  <= 0;
        pre_frame_clken_r <= 0;
    end
    else begin
        pre_frame_vsync_r <= { pre_frame_vsync_r[0], pre_frame_vsync };
        pre_frame_href_r  <= { pre_frame_href_r[0],  pre_frame_href  };
        pre_frame_clken_r <= { pre_frame_clken_r[0], pre_frame_clken };
    end
end

//在同步处理后的控制信号下，输出图像矩阵
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        {matrix_p11, matrix_p12, matrix_p13} <= 24'h0;
        {matrix_p21, matrix_p22, matrix_p23} <= 24'h0;
        {matrix_p31, matrix_p32, matrix_p33} <= 24'h0;
    end
    else if(read_frame_href) begin
        if(read_frame_clken) begin
            {matrix_p11, matrix_p12, matrix_p13} <= {matrix_p12, matrix_p13, valid_row1};
            {matrix_p21, matrix_p22, matrix_p23} <= {matrix_p22, matrix_p23, valid_row2};
            {matrix_p31, matrix_p32, matrix_p33} <= {matrix_p32, matrix_p33, row3_data};
        end
        else begin
            {matrix_p11, matrix_p12, matrix_p13} <= {matrix_p11, matrix_p12, matrix_p13};
            {matrix_p21, matrix_p22, matrix_p23} <= {matrix_p21, matrix_p22, matrix_p23};
            {matrix_p31, matrix_p32, matrix_p33} <= {matrix_p31, matrix_p32, matrix_p33};
        end
    end
    else begin
        {matrix_p11, matrix_p12, matrix_p13} <= 24'h0;
        {matrix_p21, matrix_p22, matrix_p23} <= 24'h0;
        {matrix_p31, matrix_p32, matrix_p33} <= 24'h0;
    end
end

endmodule 