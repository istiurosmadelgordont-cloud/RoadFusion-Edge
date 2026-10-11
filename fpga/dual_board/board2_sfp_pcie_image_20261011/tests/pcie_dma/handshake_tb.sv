`timescale 1ns/1ps
module handshake_tb;
    parameter IMAGE_PACKETS=2;
    localparam FRAME_BYTES=(IMAGE_PACKETS+1)*64;
    reg clk=0,pix=0,rstn=0;
    always #5 clk=~clk;
    always #7 pix=~pix;
    reg rxvalid=0,rxlast=0;
    reg [127:0] rxdata=0;
    reg [3:0] rxkeep=0;
    reg [7:0] rxuser=0;
    wire rxready,valid,last,user;
    wire [127:0] data;
    reg ready=1,vs=0,de=0;
    reg random_backpressure=0;
    reg [31:0] random_state=32'h819acf35;
    always @(negedge clk) if(random_backpressure) begin
        random_state={random_state[30:0],random_state[31]^random_state[21]^random_state[1]^random_state[0]};
        ready=(random_state[2:0]!=0);
    end
    wire [7:0] light,dark;
    wire light_sw,dark_sw;
    pcie_dma_ctrl #(.IMAGE_TLP_COUNT(IMAGE_PACKETS),.IMAGE_CONTROL_ENABLE(1)) dut(
        .clk(clk),.pix_clk_out(pix),.rstn(rstn),
        .axis_master_tvalid(rxvalid),.axis_master_tready(rxready),
        .axis_master_tdata(rxdata),.axis_master_tkeep(rxkeep),
        .axis_master_tlast(rxlast),.axis_master_tuser(rxuser),
        .ep_bus_num(8'h01),.ep_dev_num(5'h00),
        .img_cmd_valid(),.img_cmd_addr(),.img_cmd_data(),
        .img_dma_addr0(),.img_dma_addr1(),.img_dma_addr2(),.img_dma_addr3(),
        .img_dma_status_addr(),.img_dma_config_valid(),.img_dma_status_valid(),
        .img_ack_addr(32'd0),.img_ack_addr_valid(1'b0),
        .AXIS_S_TREADY(ready),.AXIS_S_TVALID(valid),.AXIS_S_TDATA(data),
        .AXIS_S_TLAST(last),.AXIS_S_TUSER(user),
        .hdmi_data_in(16'h1234),.vs_in(vs),.de_in(de),
        .video_enhance_lightdown_num(light),.video_enhance_lightdown_sw(light_sw),
        .video_enhance_darkup_num(dark),.video_enhance_darkup_sw(dark_sw));
    function [31:0] swap(input [31:0] v);
        swap={v[7:0],v[15:8],v[23:16],v[31:24]};
    endfunction
    function [31:0] base(input integer i);
        base=32'h01000000+i*32'h01000000;
    endfunction
    task tick(input integer n); repeat(n) @(negedge clk); endtask
    task cmd(input [11:0] addr,input [31:0] value);
        @(negedge clk);
        rxvalid=1;rxlast=0;rxkeep=7;rxuser=8'h10;rxdata=0;
        rxdata[31:24]=8'h40;rxdata[9:0]=1;rxdata[39:32]=8'h0f;
        rxdata[75:64]=addr;
        @(negedge clk);rxlast=1;rxkeep=1;rxdata={96'd0,swap(value)};
        @(negedge clk);rxvalid=0;rxlast=0;tick(2);
    endtask
    task frame;
        @(negedge clk);vs=1;tick(12);vs=0;tick(10);
    endtask
    task check(input bit cond,input string msg);
        if(!cond) $fatal(1,"%s at %t",msg,$time);
    endtask

    integer beat=0,slot=-1,packets=0,completions=0,stop_completions=0;
    integer images[0:3];
    reg host_held[0:3];
    reg [31:0] tokens[0:3];
    reg [31:0] address,token_word,stop_word;
    reg [127:0] stalled_data;
    reg stalled=0,stalled_last;
    integer j,k,n,vs_header_collisions=0,vs_last_collisions=0,stop_last_collisions=0;
    integer vs_header_before,vs_last_before,stop_last_before;
    reg expect_vs_header_edge=0,expect_vs_last_edge=0,expect_stop_last_edge=0;
    reg test_passed=0;
    bit is_marker,is_stop;
    // Packet monitor observes external AXIS, not just internal state.
    always @(posedge clk) if(rstn) begin
        if(expect_vs_header_edge) begin
            check(valid && ready && dut.tx_handshake && dut.vs_rising && !last && beat==0,
                "Target edge did not contain actual VS/header handshake");
            expect_vs_header_edge=0;
        end
        if(expect_vs_last_edge) begin
            check(valid && ready && dut.tx_handshake && dut.vs_rising && last && beat==4 && dut.packet_kind==0,
                "Target edge did not contain actual VS/image TLAST handshake");
            expect_vs_last_edge=0;
        end
        if(expect_stop_last_edge) begin
            check(valid && ready && dut.tx_handshake && dut.stop_pulse && last && beat==4 &&
                dut.packet_kind==0 && dut.stop_cookie==32'habcdef04,
                "Target edge did not contain actual STOP/image TLAST handshake");
            expect_stop_last_edge=0;
        end
        if(valid && ready && dut.vs_rising && beat==0) vs_header_collisions++;
        if(valid && ready && dut.vs_rising && last) vs_last_collisions++;
        if(valid && ready && dut.stop_pulse && last) stop_last_collisions++;
        if(stalled) begin
            if(!valid || data!==stalled_data || last!==stalled_last)
                $fatal(1,"AXIS changed while backpressured");
        end
        stalled=valid && !ready;stalled_data=data;stalled_last=last;
        if(valid && ready) begin
            if(beat==0) begin
                if(last || data[31:24]!=8'h40 || data[9:0]!=16)
                    $fatal(1,"Invalid MWr32 header");
                address=data[95:64];is_stop=address==32'h08000000;
                slot=-1;
                for(j=0;j<4;j=j+1)
                    if(address>=base(j) && address<base(j)+FRAME_BYTES) slot=j;
                if(!is_stop && slot<0) $fatal(1,"Out-of-range DMA address %h",address);
                if(!is_stop && host_held[slot]) $fatal(1,"Write into RC-held buffer %0d",slot);
                is_marker=!is_stop && address==base(slot)+IMAGE_PACKETS*64;
                if(!is_stop && !is_marker) begin
                    // An aborted frame can restart at its base address.
                    if(address==base(slot)) images[slot]=0;
                    if(address!=base(slot)+images[slot]*64) $fatal(1,"Image packet address discontinuity");
                    images[slot]=images[slot]+1;
                    if(IMAGE_PACKETS>2 && images[slot]%8192==0)
                        $display("FULL FRAME progress: %0d image packets",images[slot]);
                end
                beat=1;
            end else begin
                if(last !== (beat==4)) $fatal(1,"Incorrect TLAST/duplicated beat %0d",beat);
                if(is_marker || is_stop) begin
                    if(beat==1) token_word=swap(data[31:0]);
                    for(k=0;k<4;k=k+1)
                        if(swap(data[k*32+:32])!==token_word) $fatal(1,"Nonuniform completion token");
                end else begin
                    for(k=0;k<4;k=k+1)
                        if(swap(data[k*32+:32])!==(images[slot]-1)*4+beat-1)
                            $fatal(1,"Image payload reordered/duplicated/lost at beat %0d",beat);
                end
                if(last) begin
                    packets=packets+1;beat=0;
                    if(is_stop) begin stop_word=token_word;stop_completions=stop_completions+1;end
                    else if(is_marker) begin
                        if(token_word[1:0]!=slot || token_word[31:2]==0 || images[slot]!=IMAGE_PACKETS)
                            $fatal(1,"Invalid/incomplete frame completion");
                        tokens[slot]=token_word;host_held[slot]=1;completions=completions+1;
                    end
                end else beat=beat+1;
            end
        end
    end
    task wait_done(input integer n);
        integer timeout;
        begin
            timeout=0;
            while(completions<n && timeout<(IMAGE_PACKETS+1)*30+500) begin tick(1);timeout++;end
            check(completions==n,"Expected frame completion missing");tick(3);
        end
    endtask
    task release_slot(input integer n);
        cmd(12'h170,tokens[n]);host_held[n]=0;
    endtask
    task stop_run(input [31:0] cookie);
        integer count_before,timeout;
        begin
            count_before=stop_completions;cmd(12'h130,cookie);timeout=0;
            while(stop_completions==count_before && timeout<300) begin tick(1);timeout++;end
            check(stop_completions==count_before+1 && stop_word==cookie,"STOP ack missing or wrong");
            tick(10);check(!dut.running && !dut.stop_pending && !valid,"STOP not quiescent");
            for(n=0;n<4;n=n+1) begin host_held[n]=0;images[n]=0;end
        end
    endtask
    integer before_count,before_packets,old_invalid;
    reg [31:0] stale_token;
    initial begin
        for(n=0;n<4;n=n+1) begin images[n]=0;host_held[n]=0;tokens[n]=0;end
        tick(5);rstn=1;tick(5);
        cmd(12'h190,1);check(!dut.running,"START without config accepted");
        cmd(12'h110,32'hffffffc0);check(dut.alloc_addr_state==0,"Wrapping address accepted");
        cmd(12'h110,base(0)+1);check(dut.alloc_addr_state==0,"Unaligned address accepted");
        // Aliases must not commit a configuration.
        for(n=0;n<4;n=n+1) cmd(12'h110,base(0));
        check(!dut.rc_cfg_ep_flag,"Overlapping buffers accepted");cmd(12'h1a0,1);
        for(n=0;n<4;n=n+1) cmd(12'h110,base(n));
        cmd(12'h180,base(0));check(!dut.status_addr_valid,"Overlapping status accepted");
        cmd(12'h180,32'h08000000);
        check(!dut.running,"Address registration automatically started DMA");
        cmd(12'h190,1);check(dut.running,"START failed");
        if(IMAGE_PACKETS>2) begin
            frame();wait_done(1);
            check(address==base(0)+IMAGE_PACKETS*64,"Full-frame tail address wrong");
            release_slot(0);check(dut.buffer_state[0]==0,"Full-frame release failed");
            stop_run(32'h12345678);
            $display("FULL FRAME TEST PASSED image_packets=%0d completions=%0d packets=%0d",IMAGE_PACKETS,completions,packets);
            test_passed=1;
            $finish;
        end
        for(n=0;n<4;n=n+1) begin frame();wait_done(n+1);end
        before_packets=packets;frame();tick(150);
        check(packets==before_packets,"No-free frame generated DMA");
        for(n=0;n<4;n=n+1) check(dut.buffer_state[n]==2,"Held slot lost ownership");
        old_invalid=dut.invalid_releases;
        cmd(12'h170,tokens[2]+4);check(dut.invalid_releases==old_invalid+1,"Wrong token accepted");
        stale_token=tokens[2];release_slot(2);
        frame();wait_done(5);check(tokens[2]!=stale_token,"Reused token");
        cmd(12'h170,stale_token);check(dut.buffer_state[2]==2,"Stale release freed new frame");
        release_slot(2);cmd(12'h170,tokens[2]);check(dut.buffer_state[2]==0,"Duplicate release changed state");
        // Empty FIFO stalls safely, a release of WRITING must fail.
        dut.u_hdmi_pcie_fifo.supply=0;frame();tick(20);
        check(dut.buffer_state[2]==1,"Slot not reserved");
        cmd(12'h170,dut.active_token);check(dut.buffer_state[2]==1,"Released WRITING slot");
        dut.u_hdmi_pcie_fifo.supply=1;wait_done(6);
        $display("PASS allocation, all-held drop, out-of-order/stale/duplicate release, empty FIFO");

        stop_run(32'h12345678);cmd(12'h190,1);
        // VS and header acceptance on the SAME edge, followed by full drain.
        vs_header_before=vs_header_collisions;
        ready=0;frame();wait(valid);@(negedge clk);vs=1;
        tick(2);expect_vs_header_edge=1;ready=1;tick(10);vs=0;tick(20);
        check(completions==6,"Aborted frame got completion");
        check(beat==0 && !valid,"Header/VS collision corrupted packet framing");
        check(!expect_vs_header_edge && vs_header_collisions==vs_header_before+1,
            "VS/header case did not add exactly one local collision");
        frame();wait_done(7);
        $display("PASS VS/header handshake collision and recovery");

        stop_run(32'h12345679);cmd(12'h190,1);ready=0;frame();wait(valid);
        cmd(12'h130,32'h24681357);tick(20);
        check(dut.stop_pending && valid,"STOP dropped stalled header");
        before_count=stop_completions;ready=1;
        while(stop_completions==before_count) tick(1);
        check(stop_word==32'h24681357 && completions==7,"STOP drain produced wrong completion");
        tick(20);check(!valid && beat==0 && !dut.running,"STOP did not drain");
        for(n=0;n<4;n=n+1) begin host_held[n]=0;images[n]=0;end
        $display("PASS STOP under header backpressure and ordered cookie");

        cmd(12'h190,1);frame();
        // Hold the last image payload beat, then collide VS with its handshake.
        vs_last_before=vs_last_collisions;
        wait(valid && dut.packet_kind==0 && last);@(negedge clk);ready=0;vs=1;
        tick(2);expect_vs_last_edge=1;ready=1;tick(10);vs=0;tick(30);
        check(completions==7 && beat==0,"VS/TLAST collision emitted false completion");
        check(!expect_vs_last_edge && vs_last_collisions==vs_last_before+1,
            "VS/TLAST case did not add exactly one local collision");
        frame();wait_done(8);
        stop_run(32'h12345680);cmd(12'h190,1);
        frame();wait(valid && dut.packet_kind==1);@(negedge clk);ready=0;
        cmd(12'h130,32'h55667788);tick(10);ready=1;
        while(stop_word!=32'h55667788) tick(1);
        check(completions==9,"Presented marker was truncated on STOP");
        tick(10);for(n=0;n<4;n=n+1) begin host_held[n]=0;images[n]=0;end
        $display("PASS VS/TLAST collision and STOP during completion packet");

        cmd(12'h190,1);dut.u_hdmi_pcie_fifo.supply=0;frame();de=1;
        dut.u_hdmi_pcie_fifo.inject_full=1;tick(20);
        de=0;dut.u_hdmi_pcie_fifo.inject_full=0;tick(20);
        check(!dut.frame_active,"FIFO overflow did not cancel frame");
        dut.u_hdmi_pcie_fifo.supply=1;tick(40);check(completions==9,"Overflow frame marked valid");
        frame();wait_done(10);stop_run(32'habcdef01);
        cmd(12'h190,1);random_backpressure=1;
        for(n=0;n<32;n=n+1) begin
            frame();wait_done(11+n);release_slot(slot);
        end
        random_backpressure=0;ready=1;stop_run(32'habcdef03);
        $display("PASS 32 frames with reproducible random backpressure and payload scoreboard");
        cmd(12'h190,1);frame();wait(valid && last);@(negedge clk);ready=0;
        stop_last_before=stop_last_collisions;
        fork
            cmd(12'h130,32'habcdef04);
            begin
                // Follow the decoded pulse, including the RX input-register
                // latency; do not infer the collision from an external beat.
                wait(dut.stop_pulse);
                @(negedge clk);expect_stop_last_edge=1;ready=1;
            end
        join
        while(stop_word!=32'habcdef04) tick(1);
        check(!expect_stop_last_edge && stop_last_collisions==stop_last_before+1 && completions==42,
            "STOP/TLAST case did not add exactly one local simultaneous handshake");
        tick(10);
        $display("PASS STOP/TLAST simultaneous handshake");
        // Sequence exhaustion is fail-closed, never ABA token wrap.
        @(negedge clk);dut.allocation_sequence=30'h3fffffff;
        cmd(12'h190,1);before_packets=packets;frame();tick(100);
        check(packets==before_packets,"Sequence wrapped");stop_run(32'habcdef02);
        $display("PASS overflow cancellation and token exhaustion");
        $display("ALL TESTS PASSED image_packets=%0d completions=%0d packets=%0d",IMAGE_PACKETS,completions,packets);
        test_passed=1;
        $finish;
    end
    initial begin #(64'd5000000000); $fatal(1,"Global simulation timeout");end
endmodule
