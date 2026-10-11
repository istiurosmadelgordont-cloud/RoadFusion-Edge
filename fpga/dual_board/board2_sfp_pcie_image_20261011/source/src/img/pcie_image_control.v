// BAR1 full-DW controls. The snapshot crosses to the pixel clock as a held bus.
// Replies are 64-byte MWr packets sent through a packet-locked posted arbiter.
module pcie_image_control#(
    parameter [159 : 0] RESET_CONFIG                     =   160'h0080c040020301900000410000f38034000001c0,
    parameter [15 : 0]  IMAGE_TLP_COUNT                  =   16'd64800
)(
    input                   clk                              ,
    input                   rst_n                            ,
    input                   cmd_valid                        ,
    input   [11 : 0]        cmd_addr                         ,
    input   [31 : 0]        cmd_data                         ,
    input   [31 : 0]        dma_addr0                        ,
    input   [31 : 0]        dma_addr1                        ,
    input   [31 : 0]        dma_addr2                        ,
    input   [31 : 0]        dma_addr3                        ,
    input   [31 : 0]        dma_status_addr                  ,
    input                   dma_config_valid                 ,
    input                   dma_status_valid                 ,
    input   [7 : 0]         ep_bus_num                       ,
    input   [4 : 0]         ep_dev_num                       ,
    input                   image_rst_n                      ,
    input                   image_ready                      ,
    input                   image_cfg_ack                    ,
    input                   image_cfg_pending                ,
    input   [159 : 0]       image_cfg_active                 ,
    output reg [159 : 0]    cfg_data                         ,
    output reg              cfg_apply                        ,
    output reg [31 : 0]     ack_addr                         ,
    output reg              ack_addr_valid                   ,
    output                  cfg_busy                         ,
    output                  tx_tvalid                        ,
    output  [127 : 0]       tx_tdata                         ,
    output                  tx_tlast                         ,
    output                  tx_tuser                         ,
    input                   tx_tready
);

localparam [11 : 0] CMD_WORD0                        =   12'h200;
localparam [11 : 0] CMD_WORD1                        =   12'h204;
localparam [11 : 0] CMD_WORD2                        =   12'h208;
localparam [11 : 0] CMD_WORD3                        =   12'h20c;
localparam [11 : 0] CMD_WORD4                        =   12'h210;
localparam [11 : 0] CMD_COMMIT                       =   12'h214;
localparam [11 : 0] CMD_ACK_ADDR                     =   12'h220;
localparam [11 : 0] CMD_QUERY                        =   12'h224;
localparam [31 : 0] ACK_MAGIC                        =   32'h494d4743;
localparam [31 : 0] STATUS_RESET                     =   32'd0;
localparam [31 : 0] STATUS_ACCEPTED                  =   32'd1;
localparam [31 : 0] STATUS_APPLIED                   =   32'd2;
localparam [31 : 0] STATUS_BUSY                      =   32'd3;
localparam [31 : 0] STATUS_BAD_COMMAND               =   32'd4;
localparam [31 : 0] STATUS_BAD_ACK_ADDRESS           =   32'd5;
localparam [31 : 0] STATUS_QUEUE_OVERFLOW            =   32'd6;
localparam [31 : 0] STATUS_WAITING_INPUT             =   32'd7;
localparam [31 : 0] STATUS_RESTORING                 =   32'd8;
localparam [32 : 0] BUFFER_BYTES                     =   ({17'd0, IMAGE_TLP_COUNT} + 33'd1) << 6;
localparam [1 : 0]  TX_IDLE                          =   2'd0;
localparam [1 : 0]  TX_HEADER                        =   2'd1;
localparam [1 : 0]  TX_PAYLOAD                       =   2'd2;

reg     [159 : 0]       shadow_config                    ;
reg     [159 : 0]       active_snapshot                  ;
reg                     in_flight                        ;
reg                     ever_committed                   ;
reg                     restoring                        ;
reg     [31 : 0]        accepted_cookie                  ;
reg     [31 : 0]        applied_cookie                   ;
reg     [31 : 0]        rejected_count_base              ;
reg     [31 : 0]        overflow_count_base              ;
reg     [31 : 0]        last_rejected_cookie_base        ;
reg     [1 : 0]         ack_settle                       ;
reg     [2 : 0]         boot_settle                      ;
reg     [1 : 0]         ready_settle                     ;
reg                     image_ack_meta                   ;
reg                     image_ack_sync                   ;
reg                     image_reset_meta                 ;
reg                     image_reset_sync                 ;
reg                     image_ready_meta                 ;
reg                     image_ready_sync                 ;
reg                     image_pending_meta               ;
reg                     image_pending_sync               ;
reg                     previous_image_ready             ;

reg     [511 : 0]       reply_fifo [0 : 7]               ;
reg     [2 : 0]         reply_wr_pointer                 ;
reg     [2 : 0]         reply_rd_pointer                 ;
reg     [3 : 0]         reply_count                      ;
reg     [1 : 0]         tx_state                         ;
reg     [1 : 0]         tx_index                         ;
reg     [511 : 0]       tx_payload                       ;
reg     [127 : 0]       tx_header                        ;
reg     [31 : 0]        reply_sequence                   ;

reg                     command_reply                    ;
reg     [31 : 0]        command_status                   ;
reg                     command_rejected                 ;
reg                     commit_accepted                  ;
reg                     address_accepted                 ;
reg     [31 : 0]        reply_flags                      ;
reg     [511 : 0]       command_record                   ;
reg     [511 : 0]       applied_record                   ;
reg                     command_reply_s1                 ;
reg                     applied_reply_s1                 ;
reg                     command_rejected_s1              ;
reg     [1 : 0]         lost_records_s1                  ;
reg     [31 : 0]        lost_cookie_s1                   ;
reg     [511 : 0]       command_record_data_s1           ;
reg     [511 : 0]       applied_record_s1                ;
reg     [31 : 0]        command_cookie_s1                ;
reg     [31 : 0]        applied_cookie_s1                ;
reg     [1 : 0]         pushed_records                   ;
reg     [1 : 0]         lost_records                     ;
reg     [31 : 0]        lost_cookie                      ;
reg     [3 : 0]         free_records                     ;
wire                    reply_pop                        ;
wire                    apply_complete                   ;
wire                    image_available                  ;
wire                    address_ok                       ;
wire                    image_lost                       ;
wire    [31 : 0]        rejected_count                   ;
wire    [31 : 0]        overflow_count                   ;
wire    [31 : 0]        last_rejected_cookie             ;
wire    [511 : 0]       command_record_s1                ;

function separated_range;
    input   [31 : 0]       candidate                        ;
    input   [31 : 0]       reserved_address                 ;
    input   [32 : 0]       reserved_bytes                   ;
    begin
        separated_range                  =  reserved_address == 0 ||
            {1'b0, candidate} + 33'd64 <= {1'b0, reserved_address} ||
            {1'b0, reserved_address} + reserved_bytes <= {1'b0, candidate};
    end
endfunction

function [31 : 0] endian32;
    input   [31 : 0]       value                            ;
    begin
        endian32                         =  {value[7 : 0], value[15 : 8], value[23 : 16], value[31 : 24]};
    end
endfunction

function [127 : 0] make_header;
    input   [31 : 0]       address                          ;
    input   [7 : 0]        bus_number                       ;
    input   [4 : 0]        device_number                    ;
    reg     [127 : 0]      value                            ;
    begin
        value                            =  0;
        value[9 : 0]                     =  10'd16;
        value[31 : 24]                   =  8'h40;
        value[39 : 32]                   =  8'hff;
        value[47 : 40]                   =  8'h02;
        value[63 : 48]                   =  {bus_number, device_number, 3'b000};
        value[95 : 64]                   =  address;
        make_header                      =  value;
    end
endfunction

function [511 : 0] make_record;
    input   [31 : 0]       cookie                           ;
    input   [31 : 0]       status                           ;
    input   [31 : 0]       flags                            ;
    input   [159 : 0]      active                           ;
    input   [31 : 0]       accepted                         ;
    input   [31 : 0]       applied                          ;
    input   [31 : 0]       rejected                         ;
    input   [31 : 0]       overflow                         ;
    input   [31 : 0]       last_rejected                    ;
    reg     [511 : 0]      value                            ;
    begin
        value                            =  0;
        value[31 : 0]                    =  ACK_MAGIC;
// Sequence is filled when this queued record starts transmission.
        value[63 : 32]                   =  0;
        value[95 : 64]                   =  cookie;
        value[127 : 96]                  =  status;
        value[159 : 128]                 =  flags;
        value[319 : 160]                 =  active;
        value[351 : 320]                 =  accepted;
        value[383 : 352]                 =  applied;
        value[415 : 384]                 =  rejected;
        value[447 : 416]                 =  overflow;
        value[479 : 448]                 =  last_rejected;
        value[511 : 480]                 =  0;
        make_record                      =  value;
    end
endfunction

assign  image_available                 =   image_reset_sync && image_ready_sync && ready_settle == 3;
assign  image_lost                      =   previous_image_ready && !(image_reset_sync && image_ready_sync);
assign  apply_complete                  =   in_flight && image_available && ack_settle == 1 &&
                                           image_ack_sync == cfg_apply;
assign  cfg_busy                        =   in_flight || reply_count != 0 || tx_state != TX_IDLE ||
                                           command_reply_s1 || applied_reply_s1;
assign  reply_pop                       =   tx_state == TX_IDLE && reply_count != 0;
assign  address_ok                      =   cmd_data != 0 && cmd_data[5 : 0] == 0 &&
                                           {1'b0, cmd_data} + 33'd64 <= 33'h100000000 &&
                                           separated_range(cmd_data, dma_addr0, BUFFER_BYTES) &&
                                           separated_range(cmd_data, dma_addr1, BUFFER_BYTES) &&
                                           separated_range(cmd_data, dma_addr2, BUFFER_BYTES) &&
                                           separated_range(cmd_data, dma_addr3, BUFFER_BYTES) &&
                                           separated_range(cmd_data, dma_status_addr, 33'd64);

// Account rejection and queue-loss events one clock later. The visible
// totals include that pending stage, so a following QUERY sees the same count
// and cookie as before. Queue loss retains priority over rejection at that edge.
assign  rejected_count                  =   rejected_count_base + {31'd0, command_rejected_s1};
assign  overflow_count                  =   overflow_count_base + {30'd0, lost_records_s1};
assign  last_rejected_cookie            =   lost_records_s1 != 0 ? lost_cookie_s1 :
                                           command_rejected_s1 ? command_cookie_s1 : last_rejected_cookie_base;
// The S1 record and rejected_count both include the last sampled rejection.
// Fill DW12 at the existing queue boundary, after that event was registered.
// Current address validation cannot feed the count carry at this boundary.
assign  command_record_s1               =   {command_record_data_s1[511 : 416], rejected_count,
                                            command_record_data_s1[383 : 0]};

// The header and each 128-bit payload beat remain stable during backpressure.
assign  tx_tvalid                       =   rst_n && tx_state != TX_IDLE;
assign  tx_tdata                        =   tx_state == TX_HEADER ? tx_header :
    {endian32(tx_payload[tx_index * 128 + 96 +: 32]), endian32(tx_payload[tx_index * 128 + 64 +: 32]),
     endian32(tx_payload[tx_index * 128 + 32 +: 32]), endian32(tx_payload[tx_index * 128 + 0 +: 32])};
assign  tx_tlast                        =   tx_state == TX_PAYLOAD && tx_index == 3;
assign  tx_tuser                        =   0;

always@(*)begin
    command_reply                       =   0;
    command_status                      =   STATUS_RESET;
    command_rejected                    =   0;
    commit_accepted                     =   0;
    address_accepted                    =   0;
    if(cmd_valid)begin
        case(cmd_addr)
            CMD_COMMIT:begin
                command_reply           =   ack_addr_valid;
                if(cmd_data == 0 || !ack_addr_valid || boot_settle != 0)begin
                    command_status      =   STATUS_BAD_COMMAND;
                    command_rejected    =   1;
                end
                else if(in_flight)begin
                    command_status      =   STATUS_BUSY;
                    command_rejected    =   1;
                end
                else begin
                    command_status      =   STATUS_ACCEPTED;
                    commit_accepted     =   1;
                end
            end
            CMD_ACK_ADDR:begin
                if(!cfg_busy && address_ok)begin
                    address_accepted    =   1;
                end
                else begin
                    command_reply       =   ack_addr_valid;
                    command_status      =   cfg_busy ? STATUS_BUSY : STATUS_BAD_ACK_ADDRESS;
                    command_rejected    =   1;
                end
            end
            CMD_QUERY:begin
                command_reply           =   ack_addr_valid;
                if(cmd_data == 0)begin
                    command_status      =   STATUS_BAD_COMMAND;
                    command_rejected    =   1;
                end
                else if(overflow_count != 0)begin
                    command_status      =   STATUS_QUEUE_OVERFLOW;
                end
                else if(restoring)begin
                    command_status      =   STATUS_RESTORING;
                end
                else if(in_flight)begin
                    command_status      =   image_available ? STATUS_ACCEPTED : STATUS_WAITING_INPUT;
                end
                else begin
                    command_status      =   ever_committed ? STATUS_APPLIED : STATUS_RESET;
                end
            end
            default:begin
            end
        endcase
    end

    reply_flags                         =   0;
    reply_flags[0]                      =   image_reset_sync;
    reply_flags[1]                      =   image_ready_sync;
    reply_flags[2]                      =   image_pending_sync;
    reply_flags[3]                      =   in_flight || commit_accepted;
    reply_flags[4]                      =   ack_addr_valid;
    reply_flags[5]                      =   overflow_count != 0;
    reply_flags[6]                      =   ever_committed || commit_accepted;
    reply_flags[7]                      =   restoring;
    command_record                      =   make_record(cmd_data, command_status, reply_flags, active_snapshot,
        commit_accepted ? cmd_data : accepted_cookie, applied_cookie,
        32'd0, overflow_count, command_rejected ? cmd_data : last_rejected_cookie);
    reply_flags[2]                      =   0;
    reply_flags[3]                      =   0;
    reply_flags[7]                      =   0;
    applied_record                      =   make_record(accepted_cookie, STATUS_APPLIED, reply_flags,
        image_cfg_active, accepted_cookie, accepted_cookie, rejected_count, overflow_count, last_rejected_cookie);

end

// Register command/APPLIED reply events before queue arbitration. Address
// checks do not drive queue pointers and overflow-counter enables directly.
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        command_reply_s1                <=  0;
        applied_reply_s1                <=  0;
        command_rejected_s1             <=  0;
        lost_records_s1                 <=  0;
        lost_cookie_s1                  <=  0;
        command_record_data_s1          <=  0;
        applied_record_s1               <=  0;
        command_cookie_s1               <=  0;
        applied_cookie_s1               <=  0;
    end
    else begin
        command_reply_s1                <=  command_reply;
        applied_reply_s1                <=  apply_complete && ack_addr_valid;
        command_rejected_s1             <=  command_rejected;
        lost_records_s1                 <=  lost_records;
        lost_cookie_s1                  <=  lost_cookie;
        command_record_data_s1          <=  command_record;
        command_cookie_s1               <=  cmd_data;
        applied_record_s1               <=  applied_record;
        applied_cookie_s1               <=  accepted_cookie;
    end
end

always@(*)begin
    free_records                        =   8 - reply_count + reply_pop;
    pushed_records                      =   0;
    lost_records                        =   0;
    lost_cookie                         =   0;
    if(command_reply_s1)begin
        if(free_records != 0)begin
            pushed_records              =   1;
        end
        else begin
            lost_records                =   1;
            lost_cookie                 =   command_cookie_s1;
        end
    end
    if(applied_reply_s1)begin
        if(free_records > pushed_records)begin
            pushed_records              =   pushed_records + 1'b1;
        end
        else begin
            lost_records                =   lost_records + 1'b1;
            lost_cookie                 =   applied_cookie_s1;
        end
    end
end

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        image_ack_meta                  <=  0;
        image_ack_sync                  <=  0;
        image_reset_meta                <=  0;
        image_reset_sync                <=  0;
        image_ready_meta                <=  0;
        image_ready_sync                <=  0;
        image_pending_meta              <=  0;
        image_pending_sync              <=  0;
        previous_image_ready            <=  0;
        boot_settle                     <=  4;
        ready_settle                    <=  0;
    end
    else begin
        image_ack_meta                  <=  image_cfg_ack;
        image_ack_sync                  <=  image_ack_meta;
        image_reset_meta                <=  image_rst_n;
        image_reset_sync                <=  image_reset_meta;
        image_ready_meta                <=  image_ready;
        image_ready_sync                <=  image_ready_meta;
        image_pending_meta              <=  image_cfg_pending;
        image_pending_sync              <=  image_pending_meta;
        previous_image_ready            <=  image_reset_sync && image_ready_sync;
        if(boot_settle != 0)
            boot_settle                 <=  boot_settle - 1'b1;
        if(image_reset_sync && image_ready_sync)begin
            if(ready_settle != 3)
                ready_settle            <=  ready_settle + 1'b1;
        end
        else begin
            ready_settle                <=  0;
        end
    end
end

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        shadow_config                   <=  RESET_CONFIG;
        cfg_data                        <=  RESET_CONFIG;
        cfg_apply                       <=  0;
        active_snapshot                 <=  RESET_CONFIG;
        ack_addr                        <=  0;
        ack_addr_valid                  <=  0;
        in_flight                       <=  0;
        ever_committed                  <=  0;
        restoring                       <=  0;
        accepted_cookie                 <=  0;
        applied_cookie                  <=  0;
        rejected_count_base             <=  0;
        overflow_count_base             <=  0;
        last_rejected_cookie_base       <=  0;
        ack_settle                      <=  0;
    end
    else begin
        if(boot_settle != 0 && !ever_committed)
            cfg_apply                   <=  image_ack_sync;
        if(cmd_valid)begin
            case(cmd_addr)
                CMD_WORD0:shadow_config[31 : 0]       <=  cmd_data;
                CMD_WORD1:shadow_config[63 : 32]      <=  cmd_data;
                CMD_WORD2:shadow_config[95 : 64]      <=  cmd_data;
                CMD_WORD3:shadow_config[127 : 96]     <=  cmd_data;
                CMD_WORD4:shadow_config[159 : 128]    <=  cmd_data;
                default:begin
                end
            endcase
        end
        if(address_accepted)begin
            ack_addr                    <=  cmd_data;
            ack_addr_valid              <=  1;
        end
        if(commit_accepted)begin
            cfg_data                    <=  shadow_config;
            cfg_apply                   <=  image_available ? !image_ack_sync : 1'b1;
            accepted_cookie             <=  cmd_data;
            in_flight                   <=  1;
            ever_committed              <=  1;
            restoring                   <=  0;
            ack_settle                  <=  0;
        end
        else if(image_lost && ever_committed)begin
            // Pixel reset clears its ACK to zero. Restore the last committed
            // snapshot even when the previous transaction used toggle zero.
            cfg_apply                   <=  1;
            in_flight                   <=  1;
            restoring                   <=  1;
            applied_cookie              <=  0;
            active_snapshot             <=  RESET_CONFIG;
            ack_settle                  <=  0;
        end
        else if(in_flight)begin
            if(!image_available || image_ack_sync != cfg_apply)begin
                ack_settle              <=  0;
                if(!image_reset_sync)
                    cfg_apply           <=  1;
            end
            else if(ack_settle == 0)begin
                ack_settle              <=  2;
            end
            else if(ack_settle == 1)begin
                active_snapshot         <=  image_cfg_active;
                applied_cookie          <=  accepted_cookie;
                in_flight               <=  0;
                restoring               <=  0;
                ack_settle              <=  0;
            end
            else begin
                ack_settle              <=  ack_settle - 1'b1;
            end
        end
        rejected_count_base             <=  rejected_count_base + {31'd0, command_rejected_s1};
        overflow_count_base             <=  overflow_count_base + {30'd0, lost_records_s1};
        if(command_rejected_s1)begin
            last_rejected_cookie_base   <=  command_cookie_s1;
        end
        if(lost_records_s1 != 0)begin
            last_rejected_cookie_base   <=  lost_cookie_s1;
        end
    end
end

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        reply_wr_pointer                <=  0;
        reply_rd_pointer                <=  0;
        reply_count                     <=  0;
        tx_state                        <=  TX_IDLE;
        tx_index                        <=  0;
        tx_payload                      <=  0;
        tx_header                       <=  0;
        reply_sequence                  <=  0;
    end
    else begin
        reply_count                     <=  reply_count + pushed_records - reply_pop;
        reply_wr_pointer                <=  reply_wr_pointer + pushed_records;
        if(command_reply_s1 && free_records != 0)
            reply_fifo[reply_wr_pointer] <=  command_record_s1;
        if(applied_reply_s1 && free_records > (command_reply_s1 && free_records != 0))
            reply_fifo[(reply_wr_pointer + (command_reply_s1 && free_records != 0)) & 3'b111] <=
                applied_record_s1;
        case(tx_state)
            TX_IDLE:begin
                if(reply_pop)begin
                    tx_payload          <=  reply_fifo[reply_rd_pointer];
                    reply_sequence      <=  reply_sequence + 1'b1;
                    tx_payload[63 : 32] <=  reply_sequence + 1'b1;
                    tx_payload[511 : 480] <= ~(reply_sequence + 1'b1);
                    tx_header           <=  make_header(ack_addr, ep_bus_num, ep_dev_num);
                    reply_rd_pointer    <=  reply_rd_pointer + 1'b1;
                    tx_state            <=  TX_HEADER;
                end
            end
            TX_HEADER:begin
                if(tx_tready)begin
                    tx_state            <=  TX_PAYLOAD;
                    tx_index            <=  0;
                end
            end
            TX_PAYLOAD:begin
                if(tx_tready)begin
                    if(tx_index == 3)begin
                        tx_state        <=  TX_IDLE;
                    end
                    else begin
                        tx_index        <=  tx_index + 1'b1;
                    end
                end
            end
            default:begin
                tx_state                <=  TX_IDLE;
            end
        endcase
    end
end

endmodule
