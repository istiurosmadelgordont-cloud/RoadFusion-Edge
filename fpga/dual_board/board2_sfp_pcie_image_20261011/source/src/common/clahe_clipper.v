// Adapted from 09_hdmi_loop_clahe_dehalo clipper_8col.v.
// DONOR_MODE=1 uses the first donor CDF mapping; mode 0 uses exact rounded normalization.
`timescale 1ns/1ps

/*
 * clipper_8col.v - 8열 순차 클리핑 + CDF 생성 FSM
 *
 * ST_CLIP     (256클럭): clip_limit 초과분 합산
 * ST_CDF_WRITE(256클럭): 재분배 → CDF 누적 → 정규화 → BRAM 기록
 * 8열 × 512클럭 ≈ 4,112클럭 (타일 행 주기 ~129,600클럭 내)
 *
 * bin_idx 1클럭 선행 요청, proc_idx/bin_data로 지연 처리 (LUTRAM 조합 지연 흡수)
 */

module clahe_clipper #(parameter TILE_PIXELS = 32400, parameter DONOR_MODE = 0) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,

    input  wire [15:0] hist_data,      // 요청 빈 카운트 (비동기 LUTRAM)
    output wire [7:0]  rd_bin,         // 빈 인덱스 선행 요청 (1클럭 앞)
    input  wire        hist_row_done,  // 타일 행 완료 → 처리 시작

    input  wire [15:0] clip_limit,     // 클리핑 한계값 (권장 400~600)
    input  wire [2:0]  tile_row_in,    // 현재 타일 행 번호 (0~7)

    output reg  [2:0]  rd_col,         // histogram 현재 처리 열

    output wire        cdf_wr_en,
    output wire [5:0]  cdf_wr_tile,    // {tile_row[2:0], col_idx[2:0]}
    output wire [7:0]  cdf_wr_bin,
    output wire [7:0]  cdf_wr_data,    // 정규화 CDF (0~255)

    output wire        hist_clear_start,
    output wire        frame_cdf_done  // 행 7 완료 → cdf_store page 전환
);

    localparam NUM_COLS = 8;

    localparam ST_IDLE      = 3'd0;
    localparam ST_CLIP      = 3'd1;
    localparam ST_CDF_WRITE = 3'd2;
    localparam ST_NEXT_COL  = 3'd3;
    localparam ST_DONE      = 3'd4;

    reg [2:0]  state;
    reg [8:0]  bin_idx;       // 선행 요청용 (0~256, 9비트 — 256 포함)
    reg [7:0]  requested_bin_d1; // synchronous histogram RAM request metadata
    reg [7:0]  proc_idx;      // bin_idx 1클럭 지연: 실제 연산 대상 빈
    reg [2:0]  col_idx;
    reg [2:0]  tile_row_reg;
    reg [19:0] excess_total; // 초과분 총합 (최대 32,400 → 15비트, 20비트 마진)
    reg [23:0] cdf_accum;       // CDF 누적기 (배열 없이 순차 처리 — 합성 병목 제거)
    reg [7:0] residual_accum;
    reg [15:0] bin_clipped;
    reg [15:0] bin_excess;
    reg [15:0] redist_base;
    reg [7:0]  cdf_bin;
    reg [15:0] bin_data;       // hist_data 1클럭 지연 샘플

    // The synchronous histogram lookup and bin_data register add two
    // request clocks. Clipping uses request indices 2..257; a further
    // redistribution register makes CDF processing use indices 3..258.
    // Align the count and bin before the normalization pipeline.
    reg [23:0] cdf_next_ff;
    reg [7:0]  proc_idx_r;
    reg        wr_active;

    wire [15:0] redist_step = excess_total[19:8]; // excess / 256
    wire [7:0]  redist_rem  = excess_total[7:0];  // excess % 256

    // Register clipping before redistribution and the CDF feedback adder.
    wire [15:0] clipped_val    = bin_clipped;
    wire [8:0] residual_sum = {1'b0, residual_accum} + {1'b0, redist_rem};
    wire [15:0] redist_val     = redist_base
                               + (DONOR_MODE ? (cdf_bin < redist_rem) : residual_sum[8]);
    wire [23:0] next_cdf_combo = cdf_accum + redist_val;

    // Exact rounded CDF normalization, one quotient bit per stage. This
    // changes table generation by seven clocks, not the pixel-path latency.
    localparam NORMALIZE_DENOMINATOR = DONOR_MODE && TILE_PIXELS == 32400 ? 65536 : TILE_PIXELS;
    wire [23:0] numerator = DONOR_MODE && TILE_PIXELS == 32400 ?
                           {8'b0, cdf_next_ff[15:0]} * 24'd516 :
                           {8'b0, cdf_next_ff[15:0]} * 24'd255 + (DONOR_MODE ? 0 : TILE_PIXELS/2);
    reg [23:0] div_remainder [0:7];
    reg [7:0]  div_quotient [0:7];
    reg [5:0]  div_tile [0:7];
    reg [7:0]  div_bin [0:7];
    reg [7:0]  div_valid;
    reg [7:0]  div_saturated;
    reg [6:0]  done_pipe;
    integer divide_stage;

    assign rd_bin = bin_idx[7:0];
    assign hist_clear_start = (state == ST_DONE);
    assign frame_cdf_done = done_pipe[6];
    assign cdf_wr_en = div_valid[7];
    assign cdf_wr_tile = div_tile[7];
    assign cdf_wr_bin = div_bin[7];
    assign cdf_wr_data = div_saturated[7] ? 8'd255 : div_quotient[7];

    always @(posedge clk) begin
        if (!rst_n || clear) begin
            div_valid <= 0;
            div_saturated <= 0;
            done_pipe <= 0;
            for (divide_stage = 0; divide_stage < 8; divide_stage = divide_stage+1) begin
                div_remainder[divide_stage] <= 0;
                div_quotient[divide_stage] <= 0;
                div_tile[divide_stage] <= 0;
                div_bin[divide_stage] <= 0;
            end
        end else begin
            div_valid <= {div_valid[6:0], wr_active};
            div_saturated <= {div_saturated[6:0], cdf_next_ff > TILE_PIXELS};
            done_pipe <= {done_pipe[5:0], state == ST_DONE && tile_row_reg == 7};
            div_tile[0] <= {tile_row_reg, col_idx};
            div_bin[0] <= proc_idx_r;
            if (numerator >= (NORMALIZE_DENOMINATOR << 7)) begin
                div_remainder[0] <= numerator - (NORMALIZE_DENOMINATOR << 7);
                div_quotient[0] <= 8'h80;
            end else begin
                div_remainder[0] <= numerator;
                div_quotient[0] <= 0;
            end
            for (divide_stage = 1; divide_stage < 8; divide_stage = divide_stage+1) begin
                div_tile[divide_stage] <= div_tile[divide_stage-1];
                div_bin[divide_stage] <= div_bin[divide_stage-1];
                if (div_remainder[divide_stage-1] >= (NORMALIZE_DENOMINATOR << (7-divide_stage))) begin
                    div_remainder[divide_stage] <= div_remainder[divide_stage-1] -
                                                  (NORMALIZE_DENOMINATOR << (7-divide_stage));
                    div_quotient[divide_stage] <= div_quotient[divide_stage-1] |
                                                 (8'b1 << (7-divide_stage));
                end else begin
                    div_remainder[divide_stage] <= div_remainder[divide_stage-1];
                    div_quotient[divide_stage] <= div_quotient[divide_stage-1];
                end
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n || clear) begin
            residual_accum   <= 0;
            state            <= ST_IDLE;
            bin_idx          <= 9'b0;
            requested_bin_d1 <= 8'b0;
            proc_idx         <= 8'b0;
            col_idx          <= 3'b0;
            tile_row_reg     <= 3'b0;
            excess_total    <= 20'b0;
            cdf_accum          <= 24'b0;
            bin_clipped       <= 0;
            bin_excess        <= 0;
            redist_base       <= 0;
            cdf_bin           <= 0;
            bin_data          <= 16'b0;
            rd_col         <= 3'b0;
            cdf_next_ff       <= 24'b0;
            proc_idx_r       <= 8'b0;
            wr_active        <= 1'b0;
        end else begin
            requested_bin_d1 <= bin_idx[7:0];
            proc_idx    <= requested_bin_d1;
            bin_data     <= hist_data;
            bin_clipped  <= hist_data > clip_limit ? clip_limit : hist_data;
            bin_excess   <= hist_data > clip_limit ? hist_data - clip_limit : 16'd0;
            redist_base  <= clipped_val + redist_step;
            cdf_bin      <= proc_idx;
            cdf_next_ff  <= next_cdf_combo;
            proc_idx_r  <= cdf_bin;
            wr_active   <= 1'b0;

            // wr_active, cdf_next_ff and proc_idx_r feed the same divider
            // pipeline slot. All 256 bins, including bin 0 and bin 255,
            // retain their own tile/bin metadata while the FSM advances.

            case (state)
                ST_IDLE: begin
                    if (hist_row_done) begin
                        state         <= ST_CLIP;
                        bin_idx       <= 9'b0;
                        col_idx       <= 3'b0;
                        tile_row_reg  <= tile_row_in;
                        excess_total <= 20'b0;
                        rd_col      <= 3'b0;
                    end
                end

                ST_CLIP: begin
                    if (bin_idx < 9'd257)
                        bin_idx <= bin_idx + 1;

                    if (bin_idx > 1) begin  // 첫 클럭: 파이프라인 준비, 무시
                        excess_total <= excess_total + bin_excess;
                    end

                    if (bin_idx == 9'd257) begin
                        bin_idx <= 9'b0;
                        cdf_accum <= 24'b0;
                        residual_accum <= 0;
                        state   <= ST_CDF_WRITE;
                    end
                end

                ST_CDF_WRITE: begin
                    if (bin_idx < 9'd258)
                        bin_idx <= bin_idx + 1;

                    if (bin_idx > 2) begin  // 첫 클럭: 파이프라인 준비, 무시
                        cdf_accum   <= next_cdf_combo;
                        residual_accum <= residual_sum[7:0];
                        wr_active <= 1'b1;
                    end

                    if (bin_idx == 9'd258)
                        state <= ST_NEXT_COL;
                end

                ST_NEXT_COL: begin
                    // bin 255's registered write lands on the first cycle of
                    // this state; wr_active's default clear above already
                    // prevents any spill into the next column's bin 0.
                    if (col_idx == NUM_COLS - 1) begin
                        col_idx <= 3'b0;
                        state   <= ST_DONE;
                    end else begin
                        col_idx       <= col_idx + 1;
                        rd_col      <= col_idx + 1;
                        excess_total <= 20'b0;
                        bin_idx       <= 9'b0;
                        cdf_accum       <= 24'b0;
                        state         <= ST_CLIP;
                    end
                end

                ST_DONE: begin
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
