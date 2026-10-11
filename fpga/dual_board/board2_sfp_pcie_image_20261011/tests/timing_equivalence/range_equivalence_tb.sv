`timescale 1ns/1ps
module range_equivalence_tb;
    reg clk = 0;
    always #5 clk = ~clk;
    reg [159:0] cfg = 0;
    reg [7:0] difference = 0;
    wire [9:0] actual;
    wire [9:0] expected;
    integer n;
    integer checked = 0;
    reg test_passed = 0;
    Bilateral_Range_LUT #(.RUNTIME_CONTROL(1)) dut(
        .clk(clk), .cfg_data(cfg), .Pre_Data(difference), .Post_Data(actual)
    );
    Bilateral_Range_LUT_reference #(.RUNTIME_CONTROL(1)) ref_lut(
        .clk(clk), .cfg_data(cfg), .Pre_Data(difference), .Post_Data(expected)
    );
    always @(posedge clk)begin
        #1;
        if(actual !== expected)
            $fatal(1,"LUT mismatch sigma=%0d difference=%0d new=%0d old=%0d",cfg[78:73],difference,actual,expected);
        checked = checked + 1;
    end
    initial begin
        for(n=0;n<16384;n=n+1)begin
            @(negedge clk);
            cfg[78:73] = n[13:8];
            difference = n[7:0];
        end
        repeat(2) @(negedge clk);
        test_passed = 1;
        $display("PASS registered range weights: all64 sigma values x256 differences, %0d checks",checked);
        $finish;
    end
endmodule
