`timescale 1ns / 1ps

module fifo_reset_tb;

localparam                      IMAGE_PACKETS = 64800;
localparam                      IMAGE_PIXELS  = 2073600;
localparam      [31 : 0]        IMAGE_BASE    = 32'h01000000;
localparam      [31 : 0]        STATUS_BASE   = 32'h08000000;

reg                             clk = 0;
reg                             pix = 0;
reg                             rstn = 0;
reg                             sync_rstn = 0;
reg                             rxvalid = 0;
reg                             rxlast = 0;
reg             [127 : 0]       rxdata = 0;
reg             [3 : 0]         rxkeep = 0;
reg             [7 : 0]         rxuser = 0;
reg                             ready = 1;
wire                            valid;
wire                            last;
wire                            user;
wire            [127 : 0]       data;
wire                            valid_ref;
wire                            last_ref;
wire                            user_ref;
wire            [127 : 0]       data_ref;
wire                            vs_o;
wire                            hs_o;
wire                            de_o;
reg                             vs_stage = 0;
reg                             de_stage = 0;
reg                             vs = 0;
reg                             de = 0;
reg                             previous_vs = 0;
reg                             previous_de = 0;
integer                         input_pixels = 0;
integer                         pixel_cycle = 0;
integer                         vs_fall_cycle = -1;
integer                         observed_backporch = 0;
integer                         ready_cycle = 0;
integer                         beat = 0;
integer                         image_packets = 0;
integer                         completions = 0;
integer                         stop_completions = 0;
integer                         payload_pixels = 0;
integer                         lane;
integer                         slot;
integer                         timeout;
reg             [31 : 0]        address;
reg             [31 : 0]        token_word;
reg             [31 : 0]        completion_token;
reg             [31 : 0]        observed_word;
reg             [31 : 0]        expected_word;
reg                             is_marker;
reg                             is_stop;
reg                             stalled = 0;
reg             [127 : 0]       stalled_data;
reg                             stalled_last;
reg                             test_passed = 0;

always #4.000 clk = ~clk;
always #3.367 pix = ~pix;

function [31 : 0] swap;
    input       [31 : 0]        value;
    begin
        swap = {value[7 : 0], value[15 : 8], value[23 : 16], value[31 : 24]};
    end
endfunction

function [15 : 0] pixel_value;
    input integer               index;
    begin
        if(index == 0)begin
            pixel_value = 16'ha55a;
        end
        else if(index == IMAGE_PIXELS - 1)begin
            pixel_value = 16'h7d3c;
        end
        else begin
            pixel_value = (index * 13) ^ (index >> 6) ^ 16'h3b17;
        end
    end
endfunction

wire            [15 : 0]        video_data = pixel_value(input_pixels);

// This is the exact sync generator copied from the target, with its actual
// 1080p defaults. Match the two VS/DE output registers in hdmi_ddr_pcie.v.
sync_generator user_sync_gen (
    .clk                        (pix),
    .rstn                       (sync_rstn),
    .vs_out                     (vs_o),
    .hs_out                     (hs_o),
    .de_out                     (de_o),
    .de_re                      (),
    .x_act                      (),
    .y_act                      ()
);

always@(posedge pix)begin
    vs_stage <= vs_o;
    de_stage <= de_o;
    vs       <= vs_stage;
    de       <= de_stage;
end

pcie_dma_ctrl #(
    .IMAGE_TLP_COUNT             (IMAGE_PACKETS),
    .IMAGE_CONTROL_ENABLE        (1)
) dut (
    .clk                        (clk),
    .pix_clk_out                (pix),
    .rstn                       (rstn),
    .axis_master_tvalid          (rxvalid),
    .axis_master_tready          (),
    .axis_master_tdata           (rxdata),
    .axis_master_tkeep           (rxkeep),
    .axis_master_tlast           (rxlast),
    .axis_master_tuser           (rxuser),
    .ep_bus_num                 (8'h01),
    .ep_dev_num                 (5'h00),
    .AXIS_S_TREADY               (ready),
    .AXIS_S_TVALID               (valid),
    .AXIS_S_TDATA                (data),
    .AXIS_S_TLAST                (last),
    .AXIS_S_TUSER                (user),
    .img_cmd_valid              (),
    .img_cmd_addr               (),
    .img_cmd_data               (),
    .img_dma_addr0              (),
    .img_dma_addr1              (),
    .img_dma_addr2              (),
    .img_dma_addr3              (),
    .img_dma_status_addr        (),
    .img_dma_config_valid       (),
    .img_dma_status_valid       (),
    .img_ack_addr               (32'd0),
    .img_ack_addr_valid         (1'b0),
    .hdmi_data_in               (video_data),
    .vs_in                      (vs),
    .de_in                      (de),
    .video_enhance_lightdown_num (),
    .video_enhance_lightdown_sw  (),
    .video_enhance_darkup_num    (),
    .video_enhance_darkup_sw     ()
);

pcie_dma_ctrl_reference #(
    .IMAGE_TLP_COUNT             (IMAGE_PACKETS),
    .IMAGE_CONTROL_ENABLE        (1)
) reference (
    .clk                        (clk),
    .pix_clk_out                (pix),
    .rstn                       (rstn),
    .axis_master_tvalid          (rxvalid),
    .axis_master_tready          (),
    .axis_master_tdata           (rxdata),
    .axis_master_tkeep           (rxkeep),
    .axis_master_tlast           (rxlast),
    .axis_master_tuser           (rxuser),
    .ep_bus_num                 (8'h01),
    .ep_dev_num                 (5'h00),
    .AXIS_S_TREADY               (ready),
    .AXIS_S_TVALID               (valid_ref),
    .AXIS_S_TDATA                (data_ref),
    .AXIS_S_TLAST                (last_ref),
    .AXIS_S_TUSER                (user_ref),
    .img_cmd_valid              (),
    .img_cmd_addr               (),
    .img_cmd_data               (),
    .img_dma_addr0              (),
    .img_dma_addr1              (),
    .img_dma_addr2              (),
    .img_dma_addr3              (),
    .img_dma_status_addr        (),
    .img_dma_config_valid       (),
    .img_dma_status_valid       (),
    .img_ack_addr               (32'd0),
    .img_ack_addr_valid         (1'b0),
    .hdmi_data_in               (video_data),
    .vs_in                      (vs),
    .de_in                      (de),
    .video_enhance_lightdown_num (),
    .video_enhance_lightdown_sw  (),
    .video_enhance_darkup_num    (),
    .video_enhance_darkup_sw     ()
);

task check;
    input bit                   condition;
    input string                message;
    begin
        if(!condition)begin
            $fatal(1, "%s at %t", message, $time);
        end
    end
endtask

task tick;
    input integer               count;
    begin
        repeat(count) @(negedge clk);
    end
endtask

task cmd;
    input       [11 : 0]        command_address;
    input       [31 : 0]        value;
    begin
        @(negedge clk);
        rxvalid            = 1;
        rxlast             = 0;
        rxkeep             = 7;
        rxuser             = 8'h10;
        rxdata             = 0;
        rxdata[31 : 24]    = 8'h40;
        rxdata[9 : 0]      = 1;
        rxdata[39 : 32]    = 8'h0f;
        rxdata[75 : 64]    = command_address;
        @(negedge clk);
        rxlast             = 1;
        rxkeep             = 1;
        rxdata             = {96'd0, swap(value)};
        @(negedge clk);
        rxvalid            = 0;
        rxlast             = 0;
        tick(4);
    end
endtask

always@(negedge clk)begin
    ready_cycle = ready_cycle + 1;
    ready       = (ready_cycle % 31 >= 3);
end

always@(posedge pix)begin
    pixel_cycle = pixel_cycle + 1;
    if(rstn)begin
        if(previous_vs && !vs)begin
            vs_fall_cycle = pixel_cycle;
        end
        if(de && !previous_de && input_pixels == 0)begin
            observed_backporch = pixel_cycle - vs_fall_cycle;
            check(vs_fall_cycle >= 0, "First DE had no preceding VS fall");
            check(observed_backporch == 79392, "Actual 1080p VS-to-DE gap changed");
            check(!dut.fifo_wr_rst && dut.frame_active_pix,
                "New FIFO reset did not release before first active pixel");
            check(!reference.fifo_wr_rst && reference.frame_active_pix,
                "Oracle FIFO reset did not release before first active pixel");
            $display("PASS actual top 1080p VS fall to first DE: %0d pixel clocks", observed_backporch);
        end
        if(de)begin
            check(input_pixels < IMAGE_PIXELS, "Unexpected second active frame");
            check(dut.fifo_wr_en && reference.fifo_wr_en,
                $sformatf("An active RGB565 pixel was discarded/reset/overflowed: n=%0d rst=%b active=%b full=%b ptr=%0d/%0d refen=%b",
                    input_pixels, dut.fifo_wr_rst, dut.frame_active_pix, dut.fifo_wr_full,
                    dut.u_hdmi_pcie_fifo.wr_pointer, dut.u_hdmi_pcie_fifo.rd_pointer, reference.fifo_wr_en));
            input_pixels <= input_pixels + 1;
        end
    end
    previous_vs = vs;
    previous_de = de;
end

// Verify every externally transmitted RGB565 pixel, and compare the complete
// old/new AXIS trace cycle by cycle under identical periodic backpressure.
always@(posedge clk)begin
    if(rstn)begin
        check(valid === valid_ref, "Old/new DMA valid timing differs");
        if(valid)begin
            check(data === data_ref && last === last_ref && user === user_ref,
                "Old/new DMA external packet differs");
        end
        if(stalled)begin
            check(valid && data === stalled_data && last === stalled_last,
                "AXIS packet changed during backpressure");
        end
        stalled      = valid && !ready;
        stalled_data = data;
        stalled_last = last;

        if(valid && ready)begin
            if(beat == 0)begin
                check(!last && data[31 : 24] == 8'h40 && data[9 : 0] == 16,
                    "Invalid MWr32 header");
                address   = data[95 : 64];
                is_marker = address == IMAGE_BASE + IMAGE_PACKETS * 64;
                is_stop   = address == STATUS_BASE;
                if(!is_marker && !is_stop)begin
                    check(address == IMAGE_BASE + image_packets * 64,
                        "Image DMA packet address discontinuity");
                    check(image_packets < IMAGE_PACKETS, "Too many image packets");
                end
                beat = 1;
            end
            else begin
                check(last === (beat == 4), "Incorrect image/completion TLAST");
                if(is_marker || is_stop)begin
                    if(beat == 1)begin
                        token_word = swap(data[31 : 0]);
                    end
                    for(lane = 0; lane < 4; lane = lane + 1)begin
                        check(swap(data[lane * 32 +: 32]) === token_word,
                            "Nonuniform frame/STOP completion token");
                    end
                end
                else begin
                    for(lane = 0; lane < 4; lane = lane + 1)begin
                        observed_word = swap(data[lane * 32 +: 32]);
                        expected_word = {pixel_value(payload_pixels + 1), pixel_value(payload_pixels)};
                        check(observed_word === expected_word,
                            $sformatf("Lost/reordered RGB565 pixel pair %0d: %h vs %h",
                                payload_pixels, observed_word, expected_word));
                        payload_pixels = payload_pixels + 2;
                    end
                end
                if(last)begin
                    if(is_stop)begin
                        check(token_word == 32'h13579bdf, "Incorrect ordered STOP cookie");
                        stop_completions = stop_completions + 1;
                    end
                    else if(is_marker)begin
                        check(image_packets == IMAGE_PACKETS && payload_pixels == IMAGE_PIXELS,
                            "Frame marked complete before its last pixel");
                        check(token_word[1 : 0] == 0 && token_word[31 : 2] != 0,
                            "Invalid first-buffer completion token");
                        completion_token = token_word;
                        completions      = completions + 1;
                    end
                    else begin
                        image_packets = image_packets + 1;
                        if(image_packets % 8192 == 0)begin
                            $display("REAL FIFO progress: %0d image packets", image_packets);
                        end
                    end
                    beat = 0;
                end
                else begin
                    beat = beat + 1;
                end
            end
        end
    end
end

initial begin
    tick(10);
    rstn = 1;
    tick(10);
    for(slot = 0; slot < 4; slot = slot + 1)begin
        cmd(12'h110, IMAGE_BASE + slot * 32'h01000000);
    end
    cmd(12'h180, STATUS_BASE);
    cmd(12'h190, 1);
    check(dut.running && reference.running, "DMA START failed");
    @(negedge pix);
    sync_rstn = 1;

    wait(input_pixels == IMAGE_PIXELS);
    @(negedge pix);
    sync_rstn = 0;
    timeout = 0;
    while(completions == 0 && timeout < 10000)begin
        tick(1);
        timeout = timeout + 1;
    end
    check(completions == 1, "Full image completion missing");
    check(dut.u_hdmi_pcie_fifo.write_count == IMAGE_PIXELS &&
          reference.u_hdmi_pcie_fifo.write_count == IMAGE_PIXELS,
        "New/oracle FIFO did not accept every input pixel exactly once");
    check(dut.u_hdmi_pcie_fifo.read_count == IMAGE_PIXELS &&
          reference.u_hdmi_pcie_fifo.read_count == IMAGE_PIXELS,
        "New/oracle FIFO read count differs from exact frame size");
    cmd(12'h170, completion_token);
    check(dut.buffer_state[0] == 0 && reference.buffer_state[0] == 0,
        "Frame token release failed");
    cmd(12'h130, 32'h13579bdf);
    timeout = 0;
    while(stop_completions == 0 && timeout < 1000)begin
        tick(1);
        timeout = timeout + 1;
    end
    tick(10);
    check(stop_completions == 1 && !dut.running && !reference.running && !valid && !valid_ref,
        "STOP did not complete and quiesce");
    for(slot = 0; slot < 4; slot = slot + 1)begin
        check(dut.buffer_state[slot] == 0 && reference.buffer_state[slot] == 0,
            "STOP left a buffer owned");
    end
    $display("REAL FIFO TEST PASSED pixels=%0d image_packets=%0d first=a55a last=7d3c gap=%0d",
        payload_pixels, image_packets, observed_backporch);
    test_passed = 1;
    $finish;
end

initial begin
    #18000000;
    $fatal(1, "Full 1080p FIFO reset test timeout");
end

endmodule
