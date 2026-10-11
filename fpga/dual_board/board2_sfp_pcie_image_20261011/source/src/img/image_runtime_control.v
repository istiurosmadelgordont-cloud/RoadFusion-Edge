// Submit a stable five-word packet, then TOGGLE apply_toggle.
// Keep the words stable until ack_toggle matches apply_toggle.
module image_runtime_control#(
    parameter [159 : 0]       RESET_CONFIG                     =   160'h0080c040020301900000410000f38034000001c0
)(
    input                       clk                              ,
    input                       rst_n                            ,
    input                       frame_start                      ,
    input       [159 : 0]       requested_config                 ,
    input                       apply_toggle                     ,
    output      [159 : 0]       frame_config                     ,
    output reg  [159 : 0]       active_config                    ,
    output                      pending                          ,
    output reg                  ack_toggle
);

reg                     apply_meta                       ;
reg                     apply_sync                       ;
reg                     accepted_toggle                  ;
reg     [1 : 0]         settle_count                     ;
reg                     ready                            ;
reg     [159 : 0]       held_config                      ;

function [159 : 0] sanitize_config;
    input   [159 : 0]       value                            ;
    reg     [159 : 0]       cleaned                          ;
    begin
        cleaned                          =  0;
        cleaned[8 : 0]                   =  value[8 : 0];
        // Removed A diagnostics [10:9] stay zero. Tone controls use new bits.
        cleaned[12 : 11]                 =  value[12 : 11];
        cleaned[95 : 80]                 =  value[95 : 80];
        cleaned[39 : 32]                 =  value[39 : 32] == 0 ? 8'd1 : value[39 : 32];
        cleaned[47 : 40]                 =  value[47 : 40] < cleaned[39 : 32] ? cleaned[39 : 32] : value[47 : 40];
        cleaned[55 : 48]                 =  value[55 : 48];
        cleaned[72 : 64]                 =  value[72 : 64] > 256 ? 9'd256 : value[72 : 64];
        cleaned[78 : 73]                 =  value[78 : 73] == 16 || value[78 : 73] == 48 ? value[78 : 73] : 6'd32;
        cleaned[111 : 96]                =  value[111 : 96] == 0 ? 16'd1 : value[111 : 96];
        cleaned[114 : 112]               =  value[114 : 112] > 4 ? 3'd4 : value[114 : 112];
        cleaned[121 : 115]               =  value[121 : 115] > 64 ? 7'd64 : value[121 : 115];
        cleaned[135 : 128]               =  value[135 : 128];
        cleaned[143 : 136]               =  value[143 : 136];
        cleaned[152 : 144]               =  value[152 : 144] > 256 ? 9'd256 : value[152 : 144];
        sanitize_config                  =  cleaned;
    end
endfunction

assign  pending                          =   ready || settle_count != 0 || apply_sync != ack_toggle;
// Present the new packet BEFORE the sampling edge used by the video pipeline.

assign  frame_config                     =   frame_start && ready ? held_config : active_config;

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        apply_meta                       <=  0;
        apply_sync                       <=  0;
        accepted_toggle                  <=  0;
        settle_count                     <=  0;
        ready                            <=  0;
        held_config                      <=  RESET_CONFIG;
        active_config                    <=  RESET_CONFIG;
        ack_toggle                       <=  0;
    end
    else begin
        apply_meta                       <=  apply_toggle;
        apply_sync                       <=  apply_meta;
        if(!ready && settle_count == 0 && apply_sync != ack_toggle)begin
            accepted_toggle                  <=  apply_sync;
            settle_count                     <=  2;
        end
        else if(settle_count != 0)begin
            settle_count                     <=  settle_count - 1'b1;
            if(settle_count == 1)begin
                held_config                      <=  sanitize_config(requested_config);
                ready                            <=  1;
            end
        end
        if(frame_start && ready)begin
            active_config                    <=  held_config;
            ack_toggle                       <=  accepted_toggle;
            ready                            <=  0;
        end
    end
end

endmodule
