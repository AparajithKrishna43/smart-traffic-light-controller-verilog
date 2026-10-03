// ==============================================================================
// Module: counter_unit.v
// Part of: Smart Traffic Light Controller Using Verilog HDL
// Description: Manages accurate timing intervals for each light phase and
//              generates phase timeout flags. Resets synchronously on state change.
// ==============================================================================

`timescale 1ns / 1ps

module counter_unit #(
    parameter TIMER_WIDTH = 6
)(
    input  wire                   clk,
    input  wire                   rst,
    input  wire                   timer_reset,
    input  wire [TIMER_WIDTH-1:0] timer_threshold,
    output reg  [TIMER_WIDTH-1:0] timer_count,
    output wire                   timer_expired
);

    assign timer_expired = (timer_count >= timer_threshold - 1'b1);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            timer_count <= {TIMER_WIDTH{1'b0}};
        end else if (timer_reset) begin
            timer_count <= {TIMER_WIDTH{1'b0}};
        end else if (!timer_expired) begin
            timer_count <= timer_count + 1'b1;
        end
    end

endmodule
