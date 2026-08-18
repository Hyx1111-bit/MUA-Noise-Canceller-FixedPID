/*
 * Module: Fixed_PID_Controller
 *==============================================================================
 * Top-level template for Tiny Tapeout project.
 * Integrates: Data_Receiver (x2), Unsigned2Signed_Converter (x2),
 *             PID_Controller_Integrated_Scale, Signed2Unsigned_Converter.
 *
 * Wiring status:
 *   - BS/DS reception + de-glitch register stage: done
 *   - U2S conversion (BS/DS): done
 *   - PID input wiring: done
 *   - PID output -> S2U conversion: done
 *   - Valid chain: valid_tie (=1) feeds Data_Receiver.valid_in only;
 *     every downstream valid_out/data_valid_out is chained into the
 *     next stage's valid_in/data_valid_in. Final S2U data_valid_out
 *     is unused (not routed to any pin).
 *   - uio_out / uio_oe: not used, tied to 0
 *
 * Author: Yuxuan Huo
 * Date:   07/19/2026
 *==============================================================================
 */
`default_nettype none

module tt_um_hyx1111bit_fixed_pid (
    input  wire [7:0] ui_in,    // Dedicated inputs  - Brain Signal
    input  wire [7:0] uio_in,   // IOs: Input path   - Desired Signal

    output wire [7:0] uo_out,   // Stimulus Signal (PID Output Signal)
    output wire [7:0] uio_out,  // IOs: Output path (unused, tied to 0)
    output wire [7:0] uio_oe,   // IOs: Enable path (unused, tied to 0)

    input  wire        ena,     // always 1 when the design is powered
    input  wire        clk,     // clock
    input  wire        rst_n    // reset_n - low to reset
);

    //--------------------------------------------------------------------
    // Constant tie signal: source of validity for both channels
    //--------------------------------------------------------------------
    wire valid_tie = 1'b1;

    //--------------------------------------------------------------------
    // Internal wires: Data_Receiver outputs
    //--------------------------------------------------------------------
    wire [7:0] bs_recv_data;
    wire       bs_recv_valid;

    wire [7:0] ds_recv_data;
    wire       ds_recv_valid;

    //--------------------------------------------------------------------
    // De-glitch register stage (rising-edge latch on clk)
    //--------------------------------------------------------------------
    reg [7:0] bs_reg;
    reg [7:0] ds_reg;

    //--------------------------------------------------------------------
    // Internal wires: Unsigned2Signed_Converter outputs
    //--------------------------------------------------------------------
    wire [7:0] bs_signed;
    wire       bs_signed_valid;

    wire [7:0] ds_signed;
    wire       ds_signed_valid;

    //--------------------------------------------------------------------
    // Internal wires: PID_Controller output
    //--------------------------------------------------------------------
    wire [7:0] pid_stim_out;
    wire       pid_stim_valid;

    //--------------------------------------------------------------------
    // Internal wires: Signed2Unsigned_Converter output
    //--------------------------------------------------------------------
    wire [7:0] stim_unsigned;
    wire       stim_unsigned_valid;   // not routed to any pin

    //--------------------------------------------------------------------
    // Submodule instance: Data_Receiver (Brain Signal channel)
    //--------------------------------------------------------------------
    Data_Receiver u_recv_brain (
        .clk_in    (clk),
        .rst_n     (rst_n),
        .data_in   (ui_in),
        .valid_in  (valid_tie),
        .Data_out  (bs_recv_data),
        .valid_out (bs_recv_valid)
    );

    //--------------------------------------------------------------------
    // Submodule instance: Data_Receiver (Desired Signal channel)
    //--------------------------------------------------------------------
    Data_Receiver u_recv_desired (
        .clk_in    (clk),
        .rst_n     (rst_n),
        .data_in   (uio_in),
        .valid_in  (valid_tie),
        .Data_out  (ds_recv_data),
        .valid_out (ds_recv_valid)
    );

    //--------------------------------------------------------------------
    // De-glitch registers: latch Data_Receiver outputs on clk rising edge
    //--------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bs_reg <= 8'b0;
            ds_reg <= 8'b0;
        end else begin
            bs_reg <= bs_recv_data;
            ds_reg <= ds_recv_data;
        end
    end

    //--------------------------------------------------------------------
    // Submodule instance: Unsigned2Signed_Converter (Brain Signal channel)
    //--------------------------------------------------------------------
    Unsigned2Signed_Converter #(
        .DATA_WIDTH (8)
    ) u_u2s_brain (
        .clk             (clk),
        .rst_n           (rst_n),
        .unsigned_in     (bs_reg),
        .data_valid_in   (bs_recv_valid),
        .signed_out      (bs_signed),
        .data_valid_out  (bs_signed_valid)
    );

    //--------------------------------------------------------------------
    // Submodule instance: Unsigned2Signed_Converter (Desired Signal channel)
    //--------------------------------------------------------------------
    Unsigned2Signed_Converter #(
        .DATA_WIDTH (8)
    ) u_u2s_desired (
        .clk             (clk),
        .rst_n           (rst_n),
        .unsigned_in     (ds_reg),
        .data_valid_in   (ds_recv_valid),
        .signed_out      (ds_signed),
        .data_valid_out  (ds_signed_valid)
    );

    //--------------------------------------------------------------------
    // Submodule instance: PID_Controller_Integrated_Scale
    //--------------------------------------------------------------------
    PID_Controller_Integrated_Scale #(
        .DATA_WIDTH         (8),
        .GAIN_WIDTH          (3),
        .MAX_WINDOW_SIZE      (16),
        .Error_SHIFT_AMOUNT    (0),
        .Pterm_SHIFT_AMOUNT     (1),
        .Iterm_SHIFT_AMOUNT      (7),
        .Dterm_SHIFT_AMOUNT       (3),
        .Sum_SHIFT_AMOUNT          (0)
    ) u_pid (
        .clk             (clk),
        .rst_n           (rst_n),
        .ena             (ena),
        .ds_in           (ds_signed),
        .bs_in           (bs_signed),
        .data_valid      (bs_signed_valid & ds_signed_valid),
        .stim_signal_out (pid_stim_out),
        .stim_valid      (pid_stim_valid)
    );

    //--------------------------------------------------------------------
    // Submodule instance: Signed2Unsigned_Converter
    //--------------------------------------------------------------------
    Signed2Unsigned_Converter #(
        .DATA_WIDTH (8)
    ) u_s2u (
        .clk             (clk),
        .rst_n           (rst_n),
        .signed_in       (pid_stim_out),
        .data_valid_in   (pid_stim_valid),
        .unsigned_out    (stim_unsigned),
        .data_valid_out  (stim_unsigned_valid)
    );

    //--------------------------------------------------------------------
    // Top-level output assignments
    //--------------------------------------------------------------------
    assign uo_out  = stim_unsigned;   // Stimulus signal to DAC
    assign uio_out = 8'b0;            // unused, tied low
    assign uio_oe  = 8'b0;            // unused, all uio pins configured as input

    //--------------------------------------------------------------------
    // List all unused signals to prevent warnings
    //--------------------------------------------------------------------
    wire _unused = &{stim_unsigned_valid, 1'b0};

endmodule