/*
 * Module: Data_Receiver (Slave)
 *==============================================================================
 * Description:
 *   Receives parallel 8-bit data from a master module.
 *   Latches data on clk_in rising edge when valid_in is asserted.
 *
 * Features:
 *   - Synchronous reception on master clock (clk_in)
 *   - 8-bit parallel data input
 *   - Data-ready flag output
 *   - Active-low reset (rst_n), consistent with Tiny Tapeout convention
 *
 * Author: Yuxuan Huo
 * Date:   11/11/2025
 *==============================================================================
 */
`default_nettype none

module Data_Receiver (
    // Clock and control from master
    input  wire        clk_in,    // Sampling clock from master
    input  wire        rst_n,     // Reset (active low)

    // Data from master
    input  wire [7:0]  data_in,   // 8-bit parallel data
    input  wire        valid_in,  // Data valid signal

    // Received data output
    output wire [7:0]  Data_out,  // Latched data
    output wire        valid_out  // Data ready flag
);

    reg [7:0] data_latched;
    reg       ready_flag;

    //--------------------------------------------------------------------
    // Data reception process
    // Latches data_in on rising edge of clk_in when valid_in = 1
    //--------------------------------------------------------------------
    always @(posedge clk_in or negedge rst_n) begin
        if (!rst_n) begin
            data_latched <= 8'b0;
            ready_flag   <= 1'b0;
        end else begin
            if (valid_in) begin
                data_latched <= data_in;
                ready_flag   <= 1'b1;
            end else begin
                ready_flag   <= 1'b0;
            end
        end
    end

    // Output assignments
    assign Data_out  = data_latched;
    assign valid_out = ready_flag;

endmodule