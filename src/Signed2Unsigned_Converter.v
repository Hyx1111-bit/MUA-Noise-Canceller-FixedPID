/*
 * Module: Signed2Unsigned_Converter
 *==============================================================================
 * Description:
 *   Converts signed PID output to unsigned format for DAC input
 *
 * Conversion Method:
 *   Inverse of U2S converter - adds offset to center at midpoint
 *
 *   Signed to Unsigned mapping:
 *     -128 (0x80) ->   0 (0x00) [minimum DAC output]
 *        0 (0x00) -> 128 (0x80) [midpoint DAC output]
 *     +127 (0x7F) -> 255 (0xFF) [maximum DAC output]
 *
 *   Formula: unsigned_out = signed_in + 128
 *
 * Use Case:
 *   PID controller outputs signed data (-128 to +127)
 *   DAC expects unsigned data (0 to 255)
 *   This module bridges between PID and DAC
 *
 * Physical Meaning (example with 5V DAC):
 *   Signed=-128 -> Unsigned=0   -> DAC outputs 0V    (min stimulation)
 *   Signed=0    -> Unsigned=128 -> DAC outputs 2.5V  (baseline)
 *   Signed=+127 -> Unsigned=255 -> DAC outputs 5V    (max stimulation)
 *
 * Interface:
 *   signed_in      : 8-bit signed input from PID (-128 to +127)
 *   data_valid_in  : High when input data is valid
 *   unsigned_out   : 8-bit unsigned output (0 to 255)
 *   data_valid_out : High when output data is valid
 *
 * Timing:
 *   Single-cycle conversion with registered output
 *   1 clock cycle latency
 *
 * Author: Yuxuan Huo
 * Date:   11/14/2025
 *==============================================================================
 */
`default_nettype none

module Signed2Unsigned_Converter #(
    parameter DATA_WIDTH = 8  // Data bit width (default 8-bit)
) (
    input  wire                    clk,
    input  wire                    rst_n,          // Active low reset (TT convention)

    // Input interface (signed from PID)
    input  wire [DATA_WIDTH-1:0]   signed_in,
    input  wire                    data_valid_in,

    // Output interface (unsigned to DAC)
    output reg  [DATA_WIDTH-1:0]   unsigned_out,
    output reg                     data_valid_out
);

    // Offset constant for conversion (2^(N-1) where N is DATA_WIDTH)
    localparam [DATA_WIDTH-1:0] OFFSET = (1 << (DATA_WIDTH-1));

    //--------------------------------------------------------------------
    // Signed to Unsigned Conversion Process
    //
    // Implements: unsigned_out = signed_in + 2^(N-1)
    // For 8-bit: unsigned_out = signed_in + 128
    //
    // Maps the signed range [-2^(N-1), 2^(N-1)-1] to
    // unsigned range [0, 2^N-1]
    //--------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin : conversion_proc
        reg [DATA_WIDTH-1:0] temp_result;

        if (!rst_n) begin
            unsigned_out   <= {DATA_WIDTH{1'b0}};
            data_valid_out <= 1'b0;

        end else begin
            if (data_valid_in) begin
                // Direct conversion (bit pattern add, wraps like VHDL unsigned+offset)
                temp_result = signed_in + OFFSET;

                unsigned_out   <= temp_result;
                data_valid_out <= 1'b1;
            end else begin
                data_valid_out <= 1'b0;
            end
        end
    end

endmodule