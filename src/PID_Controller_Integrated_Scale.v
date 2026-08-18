/*
 * Module: PID_Controller_Integrated_Scale
 *==============================================================================
 * PID Controller Bit-Width Analysis Table
 *==============================================================================
 *
 * INPUT STAGE:
 * ------------
 * desire_in            8-bit    -128 to 127        Signed input
 * input_in             8-bit    -128 to 127        Signed input
 * error_9bit           9-bit    -256 to 255         desire - input
 *
 * P-TERM STAGE:
 * -------------
 * KP_VALUE             3-bit    Unsigned (0 to 7)   Gain = 2
 * pterm_product         12-bit   9 + 3 = 12          error x Kp
 * pterm_scaled          8-bit    After shift          Output to sum
 *
 * I-TERM STAGE:
 * -------------
 * window_reg(i)         9-bit    -256 to 255          Each error value
 * window_sum             13-bit   9 + log2(16) = 13    Sum of 16 errors
 * KI_VALUE               3-bit    Unsigned (0 to 7)    Gain = 1
 * iterm_product           16-bit   13 + 3 = 16          window_sum x Ki
 * iterm_scaled            8-bit    After shift           Output to sum
 *
 * D-TERM STAGE:
 * -------------
 * error_prev              9-bit    -256 to 255          Previous error
 * error_diff               10-bit   9 + 1 = 10           error[n]-error[n-1]
 * KD_VALUE                 3-bit    Unsigned (0 to 7)    Gain = 1
 * dterm_product              13-bit   10 + 3 = 13          error_diff x Kd
 * dterm_scaled               8-bit    After shift          Output to sum
 *
 * SUM STAGE:
 * ----------
 * pid_sum                    10-bit   8 + log2(3) ~= 10    P+I+D
 * pid_sum_scaled              8-bit    After shift          Final output
 *
 * SHIFT PARAMETERS:
 * ------------------
 * Pterm_SHIFT_AMOUNT = 1, Iterm_SHIFT_AMOUNT = 7, Dterm_SHIFT_AMOUNT = 3,
 * Sum_SHIFT_AMOUNT = 0
 *
 * Author: Yuxuan Huo
 * Date:   12/09/2025
 *==============================================================================
 */
`default_nettype none

module PID_Controller_Integrated_Scale #(
    parameter DATA_WIDTH         = 8,
    parameter GAIN_WIDTH         = 3,
    parameter MAX_WINDOW_SIZE    = 16,
    parameter Error_SHIFT_AMOUNT = 0,
    parameter Pterm_SHIFT_AMOUNT = 1,
    parameter Iterm_SHIFT_AMOUNT = 7,
    parameter Dterm_SHIFT_AMOUNT = 3,
    parameter Sum_SHIFT_AMOUNT   = 0
) (
    input  wire                    clk,
    input  wire                    rst_n,      // active low, matches TT convention
    input  wire                    ena,        // TT enable, high when design is powered
    input  wire [DATA_WIDTH-1:0]   ds_in,      // Desired Signal in
    input  wire [DATA_WIDTH-1:0]   bs_in,      // Brain Signal in
    input  wire                    data_valid, // must set as '1'
    output reg  [DATA_WIDTH-1:0]   stim_signal_out,
    output reg                     stim_valid
);

    // Fixed gain parameters (3-bit signed, positive magnitude)
    localparam signed [GAIN_WIDTH-1:0] KP_VALUE = 3'sd2;  // Kp = 2
    localparam signed [GAIN_WIDTH-1:0] KI_VALUE = 3'sd1;  // Ki = 1
    localparam signed [GAIN_WIDTH-1:0] KD_VALUE = 3'sd1;  // Kd = 1
    localparam [7:0] WINDOW_SIZE_VALUE = 8'd16;

    // I-term: Sliding window storage (9-bit signed error values)
    reg signed [8:0] window_reg [0:MAX_WINDOW_SIZE-1];
    reg [7:0]         fill_count;
    reg [7:0]         curr_window_size;
    reg signed [12:0]  window_sum;   // 9-bit + log2(16) = 13-bit

    // D-term: Previous error storage (9-bit)
    reg signed [8:0] error_prev;

    integer idx;

    always @(posedge clk or negedge rst_n) begin : pid_calc
        // Local combinational variables (blocking assignment, mirrors VHDL variables)
        reg signed [DATA_WIDTH-1:0] desire_signed;
        reg signed [DATA_WIDTH-1:0] input_signed;
        reg signed [8:0]            error_9bit;

        reg signed [11:0] pterm_product;   // 9-bit x 3-bit = 12-bit
        reg signed [DATA_WIDTH-1:0] pterm_scaled;

        reg signed [8:0]  old_data;
        reg signed [12:0] temp_sum;
        reg signed [15:0] iterm_product;   // 13-bit x 3-bit = 16-bit
        reg signed [DATA_WIDTH-1:0] iterm_scaled;

        reg signed [9:0]  error_diff;      // 9-bit - 9-bit = 10-bit
        reg signed [12:0] dterm_product;   // 10-bit x 3-bit = 13-bit
        reg signed [DATA_WIDTH-1:0] dterm_scaled;

        reg signed [9:0]  pid_sum;         // 8+8+8 needs 10-bit
        reg signed [DATA_WIDTH-1:0] pid_sum_scaled;

        if (!rst_n) begin
            for (idx = 0; idx < MAX_WINDOW_SIZE; idx = idx + 1)
                window_reg[idx] <= 9'sd0;
            fill_count       <= 8'd0;
            curr_window_size <= 8'd0;
            window_sum        <= 13'sd0;
            error_prev         <= 9'sd0;
            stim_signal_out     <= {DATA_WIDTH{1'b0}};
            stim_valid            <= 1'b0;

        end else if (ena) begin

            curr_window_size <= WINDOW_SIZE_VALUE;

            if (data_valid) begin

                // ========================================
                // STEP 1: Error Calculation (9-bit, non-saturating)
                // ========================================
                desire_signed = $signed(ds_in);
                input_signed  = $signed(bs_in);
                error_9bit    = {desire_signed[DATA_WIDTH-1], desire_signed} -
                                 {input_signed[DATA_WIDTH-1], input_signed};

                // ========================================
                // STEP 2: P-TERM CALCULATION
                // ========================================
                pterm_product = error_9bit * KP_VALUE;
                pterm_scaled  = pterm_product >>> Pterm_SHIFT_AMOUNT;

                // ========================================
                // STEP 3: I-TERM CALCULATION (sliding window)
                // ========================================
                old_data = window_reg[MAX_WINDOW_SIZE - WINDOW_SIZE_VALUE];

                for (idx = 0; idx < MAX_WINDOW_SIZE - 1; idx = idx + 1)
                    window_reg[idx] <= window_reg[idx + 1];
                window_reg[MAX_WINDOW_SIZE - 1] <= error_9bit;

                if (fill_count < curr_window_size) begin
                    temp_sum   = window_sum + {{4{error_9bit[8]}}, error_9bit};
                    fill_count <= fill_count + 1'b1;
                end else begin
                    temp_sum = window_sum - {{4{old_data[8]}}, old_data} +
                                             {{4{error_9bit[8]}}, error_9bit};
                end
                window_sum <= temp_sum;

                iterm_product = temp_sum * KI_VALUE;
                iterm_scaled  = iterm_product >>> Iterm_SHIFT_AMOUNT;

                // ========================================
                // STEP 4: D-TERM CALCULATION (differential)
                // ========================================
                error_diff    = {error_9bit[8], error_9bit} -
                                 {error_prev[8], error_prev};
                dterm_product = error_diff * KD_VALUE;
                dterm_scaled  = dterm_product >>> Dterm_SHIFT_AMOUNT;

                error_prev <= error_9bit;

                // ========================================
                // STEP 5: SUM P + I + D
                // ========================================
                pid_sum = {{2{pterm_scaled[DATA_WIDTH-1]}}, pterm_scaled} +
                          {{2{iterm_scaled[DATA_WIDTH-1]}}, iterm_scaled} +
                          {{2{dterm_scaled[DATA_WIDTH-1]}}, dterm_scaled};

                // ========================================
                // STEP 6: Final shift for output
                // ========================================
                pid_sum_scaled = pid_sum >>> Sum_SHIFT_AMOUNT;

                stim_signal_out <= pid_sum_scaled;
                stim_valid        <= 1'b1;

            end else begin
                stim_valid <= 1'b1;   // preserved as-is from original VHDL
            end
        end
    end

endmodule