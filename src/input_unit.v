// ==============================================================================
// Module: input_unit.v
// Part of: Smart Traffic Light Controller Using Verilog HDL
// Description: Synchronizes, latches, and prioritizes external asynchronous
//              inputs (pedestrian crossing button, emergency vehicle sensors).
// ==============================================================================

`timescale 1ns / 1ps

module input_unit (
    input  wire clk,
    input  wire rst,
    input  wire ped_req_in,
    input  wire amb_req_N_in,
    input  wire ped_serviced,
    input  wire amb_serviced,
    output reg  ped_req_latched,
    output reg  amb_req_latched
);

    // Double flip-flop synchronizers to prevent metastability
    reg ped_meta, ped_sync;
    reg amb_meta, amb_sync;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ped_meta        <= 1'b0;
            ped_sync        <= 1'b0;
            amb_meta        <= 1'b0;
            amb_sync        <= 1'b0;
            ped_req_latched <= 1'b0;
            amb_req_latched <= 1'b0;
        end else begin
            // 2FF synchronizers
            ped_meta <= ped_req_in;
            ped_sync <= ped_meta;

            amb_meta <= amb_req_N_in;
            amb_sync <= amb_meta;

            // Ambulance request latching
            if (amb_sync)
                amb_req_latched <= 1'b1;
            else if (amb_serviced)
                amb_req_latched <= 1'b0;

            // Pedestrian request latching
            if (ped_sync)
                ped_req_latched <= 1'b1;
            else if (ped_serviced)
                ped_req_latched <= 1'b0;
        end
    end

endmodule
