`timescale 1ns / 1ps

module atmosphere_temporal_tb;

    reg                 clk             = 1'b0      ;
    reg                 rst_n           = 1'b0      ;
    reg     [23 : 0]    raw_A_rgb       = 24'd0     ;
    reg                 raw_valid       = 1'b0      ;
    wire    [23 : 0]    smoothed_A_rgb              ;
    wire                initialized                 ;
    wire                update_pulse                ;
    reg                 video_valid     = 1'b0      ;
    reg     [23 : 0]    active_A_rgb                 ;
    reg                 previous_video_valid = 1'b0 ;
    integer             video_samples   = 0         ;
    integer             oracle_pulse_count          ;
    wire    [23 : 0]    reference_A_rgb             ;
    wire                reference_initialized       ;
    wire                reference_update_pulse      ;

    integer             tested_frames   = 0         ;
    integer             updates_seen    = 0         ;
    integer             test_passed     = 0         ;
    integer             scenario                    ;
    integer             frame_index                 ;
    integer             channel                     ;
    integer             previous_value              ;
    integer             current_value               ;
    integer             target_value                ;
    integer             frame_count                 ;
    integer             all_converged               ;
    integer             magnitude                   ;
    integer             fast_mode                   ;
    integer             reference_step              ;
    integer             random_state    = 32'h3a52d7e1;
    reg     [23 : 0]    previous_rgb                ;
    reg     [23 : 0]    target_rgb                  ;
    reg     [23 : 0]    jitter_rgb                  ;
    reg     [23 : 0]    frozen_rgb                  ;
    reg     [23 : 0]    positive_trace[0 : 79]      ;

    always #3.367 clk = ~clk;

    atmosphere_temporal dut(
        .clk                    (clk                ),
        .rst_n                  (rst_n              ),
        .raw_A_rgb              (raw_A_rgb          ),
        .raw_valid              (raw_valid          ),
        .smoothed_A_rgb         (smoothed_A_rgb     ),
        .initialized            (initialized        ),
        .update_pulse           (update_pulse       )
    );

    // The old four-clock implementation is an oracle for the frame algorithm.
    // The fifth blanking clock may differ, but all active video must agree.
    atmosphere_temporal_reference reference(
        .clk                    (clk                    ),
        .rst_n                  (rst_n                  ),
        .raw_A_rgb              (raw_A_rgb              ),
        .raw_valid              (raw_valid              ),
        .smoothed_A_rgb         (reference_A_rgb        ),
        .initialized            (reference_initialized  ),
        .update_pulse           (reference_update_pulse )
    );

    always@(posedge clk)begin
        #0.1;
        if(rst_n && video_valid)begin
            require(smoothed_A_rgb === reference_A_rgb,
                    "scene pipeline must preserve reference RGB throughout active video");
            require(initialized === reference_initialized,
                    "scene pipeline must preserve initialization state throughout active video");
            require(update_pulse === reference_update_pulse,
                    "scene pipeline must not publish while active video is valid");
        end
    end

    always@(posedge clk)begin
        #0.1;
        if(!rst_n)begin
            previous_video_valid = 1'b0;
        end
        else begin
            if(video_valid)begin
                if(previous_video_valid)begin
                    require(smoothed_A_rgb === active_A_rgb, "A must remain stable within active video");
                end
                active_A_rgb = smoothed_A_rgb;
                video_samples = video_samples + 1;
            end
            previous_video_valid = video_valid;
        end
    end

    task automatic require;
        input condition;
        input string description;
        begin
            if(!condition)begin
                $error("FAIL frame=%0d A=%06h: %s", tested_frames, smoothed_A_rgb, description);
                $fatal(1);
            end
        end
    endtask

    task automatic reset_dut;
        begin
            @(negedge clk);
            rst_n     = 1'b0;
            raw_valid = 1'b0;
            video_valid = 1'b0;
            repeat(3) @(negedge clk);
            require(smoothed_A_rgb === 24'he6e6e6, "reset A must be RGB 230");
            require(initialized === 1'b0, "reset must clear initialization");
            require(update_pulse === 1'b0, "reset must clear publication pulse");
            rst_n = 1'b1;
            repeat(2) @(negedge clk);
        end
    endtask

    task automatic send_frame;
        input [23 : 0] sample;
        reg   [23 : 0] old_rgb;
        reg             was_initialized;
        integer         pulse_count;
        integer         cycle;
        begin
            old_rgb         = smoothed_A_rgb;
            was_initialized = initialized;
            pulse_count     = 0;
            oracle_pulse_count = 0;

            @(negedge clk);
            video_valid = 1'b0;
            raw_A_rgb = sample;
            raw_valid = 1'b1;

            for(cycle = 0; cycle < 24; cycle = cycle + 1)begin
                @(posedge clk);
                #0.1;
                if(update_pulse)begin
                    pulse_count  = pulse_count + 1;
                    updates_seen = updates_seen + 1;
                end
                if(was_initialized && cycle < 5)begin
                    require(smoothed_A_rgb === old_rgb, "RGB must commit atomically after five preparation clocks");
                end
                if(reference_update_pulse)begin
                    oracle_pulse_count = oracle_pulse_count + 1;
                end
                @(negedge clk);
                raw_valid = 1'b0;
                if(cycle >= 5)begin
                    video_valid = 1'b1;
                end
            end

            require(initialized === 1'b1, "valid frame must initialize or retain initialization");
            require(smoothed_A_rgb === reference_A_rgb, "A must match oracle after blanking");
            require(pulse_count == oracle_pulse_count, "frame update count must match oracle");
            require(pulse_count <= 1, "each frame may produce at most one update pulse");
            if(was_initialized)begin
                require(pulse_count == (smoothed_A_rgb != old_rgb), "update pulse must identify an actual byte change");
            end
            else begin
                require(smoothed_A_rgb === sample, "first valid frame must initialize all channels exactly");
                require(pulse_count == 1, "first real estimate must publish once");
            end

            tested_frames = tested_frames + 1;
        end
    endtask

    function automatic integer absolute;
        input integer value;
        begin
            absolute = value < 0 ? -value : value;
        end
    endfunction

    function automatic [31 : 0] next_random;
        input [31 : 0] value;
        begin
            next_random = {value[30 : 0], value[31] ^ value[21] ^ value[1] ^ value[0]};
        end
    endfunction

    initial begin
        reset_dut;

        raw_A_rgb = 24'hffffff;
        repeat(30) @(negedge clk);
        require(smoothed_A_rgb === 24'he6e6e6 && !initialized, "raw bus changes without valid must be ignored");

        // Use the exact static-image captures supplied by the user.
        send_frame(24'he2e2eb);
        frozen_rgb = smoothed_A_rgb;
        for(frame_index = 0; frame_index < 200; frame_index = frame_index + 1)begin
            case(frame_index % 4)
                0 : send_frame(24'he2e2eb);
                1 : send_frame(24'he3e2eb);
                2 : send_frame(24'he4e3ea);
                3 : send_frame(24'he3e3e9);
            endcase
            require(smoothed_A_rgb === frozen_rgb, "observed static-frame A jitter must be completely suppressed");
        end

        // Persistent small shifts must move, even though individual EMA steps
        // are initially below half a gray code. Q8 state must retain them.
        reset_dut;
        send_frame(24'h646464);
        send_frame(24'h676167);
        require(smoothed_A_rgb === 24'h646464, "sub-half-code first EMA step should keep output byte");
        send_frame(24'h676167);
        require(smoothed_A_rgb === 24'h656365, "fractional state must accumulate in both directions");
        repeat(20) send_frame(24'h676167);
        require(smoothed_A_rgb === 24'h656365, "dead band may retain at most two-code residual");

        // An isolated white outlier is limited to two codes, not a direct jump.
        reset_dut;
        send_frame(24'h646464);
        send_frame(24'hffffff);
        require(smoothed_A_rgb === 24'h666666, "one large outlier must be limited to two codes");
        repeat(10) send_frame(24'h646464);
        require(smoothed_A_rgb === 24'h666666, "return within dead band must not bounce back");

        // Confirmed large changes accelerate only on the third consecutive frame.
        reset_dut;
        send_frame(24'h646464);
        send_frame(24'hc8c8c8);
        require(smoothed_A_rgb === 24'h666666, "large frame one must use normal step");
        send_frame(24'hc8c8c8);
        require(smoothed_A_rgb === 24'h686868, "large frame two must use normal step");
        send_frame(24'hc8c8c8);
        require(smoothed_A_rgb === 24'h6c6c6c, "large frame three may use fast step");
        send_frame(24'h141414);
        require(smoothed_A_rgb === 24'h6a6a6a, "direction reversal must restart scene confirmation");

        // Whole-range transitions must be monotonic, bounded, and converge
        // to the two-code dead band; cover opposite signs within one RGB tuple.
        reset_dut;
        send_frame(24'h00ff80);
        target_rgb  = 24'hff0000;
        frame_count = 0;
        for(frame_index = 0; frame_index < 130; frame_index = frame_index + 1)begin
            previous_rgb = smoothed_A_rgb;
            send_frame(target_rgb);
            all_converged = 1;
            for(channel = 0; channel < 3; channel = channel + 1)begin
                previous_value = previous_rgb[channel * 8 +: 8];
                current_value  = smoothed_A_rgb[channel * 8 +: 8];
                target_value   = target_rgb[channel * 8 +: 8];
                require(absolute(current_value - previous_value) <= 4, "per-frame update must never exceed four codes");
                require(absolute(target_value - current_value) <= absolute(target_value - previous_value),
                        "constant scene must approach target monotonically without wraparound");
                if(absolute(target_value - current_value) > 2)begin
                    all_converged = 0;
                end
            end
            if(all_converged && frame_count == 0)begin
                frame_count = frame_index + 1;
            end
        end
        for(channel = 0; channel < 3; channel = channel + 1)begin
            require(absolute(integer'(smoothed_A_rgb[channel * 8 +: 8]) -
                             integer'(target_rgb[channel * 8 +: 8])) <= 2,
                    "full-range transition must converge within two codes in 130 frames");
        end
        require(frame_count > 0, "whole RGB must settle inside the dead band");
        $display("Full-range transition converged in %0d frames: %06h", frame_count, smoothed_A_rgb);

        // All 256 possible starting values retain exactly while raw A varies
        // by up to two codes, including clamped zero and 255 neighborhoods.
        for(scenario = 0; scenario < 256; scenario = scenario + 1)begin
            reset_dut;
            target_rgb = {3{8'(scenario)}};
            send_frame(target_rgb);
            for(frame_index = -2; frame_index <= 2; frame_index = frame_index + 1)begin
                current_value = scenario + frame_index;
                if(current_value < 0)begin
                    current_value = 0;
                end
                else if(current_value > 255)begin
                    current_value = 255;
                end
                send_frame({3{8'(current_value)}});
                require(smoothed_A_rgb === target_rgb, "two-code dead band must hold throughout byte range");
            end
        end

        // Independent RGB large-change directions converge without coupling.
        // Random target sequences exercise unsigned/signed boundary handling.
        for(scenario = 0; scenario < 40; scenario = scenario + 1)begin
            random_state = next_random(random_state);
            reset_dut;
            send_frame(random_state[23 : 0]);
            random_state = next_random(random_state);
            target_rgb = random_state[23 : 0];

            for(frame_index = 0; frame_index < 130; frame_index = frame_index + 1)begin
                previous_rgb = smoothed_A_rgb;
                send_frame(target_rgb);
                for(channel = 0; channel < 3; channel = channel + 1)begin
                    previous_value = previous_rgb[channel * 8 +: 8];
                    current_value  = smoothed_A_rgb[channel * 8 +: 8];
                    target_value   = target_rgb[channel * 8 +: 8];
                    require(absolute(current_value - previous_value) <= 4, "random scene must honor per-frame cap");
                    require(absolute(target_value - current_value) <= absolute(target_value - previous_value),
                            "random constant scene must not overshoot");
                end
            end

            frozen_rgb = smoothed_A_rgb;
            for(channel = 0; channel < 3; channel = channel + 1)begin
                require(absolute(integer'(frozen_rgb[channel * 8 +: 8]) -
                                 integer'(target_rgb[channel * 8 +: 8])) <= 2,
                        "random constant scene must converge");
            end

            for(frame_index = 0; frame_index < 50; frame_index = frame_index + 1)begin
                for(channel = 0; channel < 3; channel = channel + 1)begin
                    random_state = next_random(random_state);
                    current_value = integer'(frozen_rgb[channel * 8 +: 8]) + (random_state[3 : 0] % 5) - 2;
                    if(current_value < 0)begin
                        current_value = 0;
                    end
                    else if(current_value > 255)begin
                        current_value = 255;
                    end
                    jitter_rgb[channel * 8 +: 8] = 8'(current_value);
                end
                send_frame(jitter_rgb);
                require(smoothed_A_rgb === frozen_rgb, "RGB jitter after convergence must remain stable");
            end
        end

        // Zero and 255 are exact for initial valid frames; resetting while a
        // scene update is in flight must cancel it instead of publishing stale A.
        reset_dut;
        send_frame(24'h000000);
        require(smoothed_A_rgb === 24'h000000, "initial black frame must preserve zero");
        reset_dut;
        send_frame(24'hffffff);
        require(smoothed_A_rgb === 24'hffffff, "initial white frame must preserve 255");
        @(negedge clk);
        video_valid = 1'b0;
        raw_A_rgb = 24'h000000;
        raw_valid = 1'b1;
        @(negedge clk);
        raw_valid = 1'b0;
        reset_dut;
        repeat(10) @(negedge clk);
        require(smoothed_A_rgb === 24'he6e6e6 && !initialized && !update_pulse,
                "reset during update must clear all pending work");

        test_passed = 1;
        $display("PASS atmosphere_temporal: frames=%0d updates=%0d active_video_samples=%0d",
                 tested_frames, updates_seen, video_samples);
        $finish;
    end

    initial begin
        #10000000;
        $fatal(1, "simulation watchdog expired");
    end

endmodule
