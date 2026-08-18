/*
 * Module: Unsigned2Signed_Converter
 *==============================================================================
 * Description:
 *   Converts unsigned ADC data to signed format for use in PID control
 *   error calculation.
 *
 * Conversion Method:
 *   Uses offset binary mapping where ADC midpoint (128) maps to 0
 *
 *   Unsigned -> Signed mapping:
 *     0   (0x00) -> -128 (0x80) [minimum]
 *     128 (0x80) ->    0 (0x00) [zero/midpoint]
 *     255 (0xFF) ->  127 (0x7F) [maximum]
 *
 *   Formula: signed_out = unsigned_in - 128
 *
 * Use Case:
 *   ADC outputs unsigned 8-bit data (0-255)
 *   PID controller needs signed data centered at zero
 *   This module bridges between ADC and error calculation
 *
 * Example Conversions:
 *   ADC=0   -> Signed=-128 (maximum negative brain signal)
 *   ADC=128 -> Signed=0    (baseline/zero reference)
 *   ADC=255 -> Signed=+127 (maximum positive brain signal)
 *
 * Interface:
 *   unsigned_in    : 8-bit unsigned input from ADC (0-255)
 *   data_valid_in  : High when input data is valid
 *   signed_out     : 8-bit signed output (-128 to +127)
 *   data_valid_out : High when output data is valid
 *
 * Timing:
 *   Single-cycle conversion with registered output
 *   1 clock cycle latency
 *
 * Author: Yuxuan Huo
 * Date:   11/14/2025
 * Update: 11/14/2025 - Fixed timing violation by using variable
 *==============================================================================
 */
`default_nettype none

module Unsigned2Signed_Converter #(
    parameter DATA_WIDTH = 8  // Data bit width (default 8-bit)
) (
    input  wire                    clk,
    input  wire                    rst_n,           // Active low reset (TT convention)

    // Input interface (unsigned from ADC)
    input  wire [DATA_WIDTH-1:0]   unsigned_in,
    input  wire                    data_valid_in,

    // Output interface (signed for PID)
    output reg  [DATA_WIDTH-1:0]   signed_out,
    output reg                     data_valid_out
);

    // Offset constant for conversion (2^(N-1) where N is DATA_WIDTH)
    localparam [DATA_WIDTH-1:0] OFFSET = (1 << (DATA_WIDTH-1));

    //--------------------------------------------------------------------
    // Unsigned to Signed Conversion Process
    //
    // Implements: signed_out = unsigned_in - 2^(N-1)
    // For 8-bit: signed_out = unsigned_in - 128
    //
    // Maps the unsigned range [0, 2^N-1] to signed range
    // [-2^(N-1), 2^(N-1)-1]
    //--------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin : conversion_proc
        reg [DATA_WIDTH-1:0] temp_result;

        if (!rst_n) begin
            signed_out     <= {DATA_WIDTH{1'b0}};
            data_valid_out <= 1'b0;

        end else begin
            if (data_valid_in) begin
                // Direct conversion (unsigned subtraction, wraps like VHDL unsigned())
                temp_result = unsigned_in - OFFSET;

                signed_out     <= temp_result;
                data_valid_out <= 1'b1;
            end else begin
                data_valid_out <= 1'b0;
            end
        end
    end

endmodule