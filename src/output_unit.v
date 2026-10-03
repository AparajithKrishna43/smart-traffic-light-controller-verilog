// ==============================================================================
// Module: output_unit.v
// Part of: Smart Traffic Light Controller Using Verilog HDL
// Description: Decodes FSM state into 3-bit Red-Yellow-Green drive signals
//              for all 8 intersection directions (N, S, E, W, NW, NE, SW, SE).
// ==============================================================================

`timescale 1ns / 1ps

module output_unit (
    input  wire [3:0]  current_state,
    output reg  [2:0]  light_N,
    output reg  [2:0]  light_S,
    output reg  [2:0]  light_E,
    output reg  [2:0]  light_W,
    output reg  [2:0]  light_NW,
    output reg  [2:0]  light_NE,
    output reg  [2:0]  light_SW,
    output reg  [2:0]  light_SE,
    output wire [23:0] lights_bus
);

    localparam RED    = 3'b100;
    localparam YELLOW = 3'b010;
    localparam GREEN  = 3'b001;

    localparam S_G1_GREEN = 4'd0;
    localparam S_G1_YEL   = 4'd1;
    localparam S_G2_GREEN = 4'd2;
    localparam S_G2_YEL   = 4'd3;
    localparam S_G3_GREEN = 4'd4;
    localparam S_G3_YEL   = 4'd5;
    localparam S_ALL_RED  = 4'd6;
    localparam S_PED_RED  = 4'd7;
    localparam S_AMB_N_G  = 4'd8;
    localparam S_AMB_N_Y  = 4'd9;

    assign lights_bus = {light_N, light_S, light_E, light_W,
                         light_NW, light_NE, light_SW, light_SE};

    always @(*) begin
        // Safe default: All lights RED
        light_N  = RED;
        light_S  = RED;
        light_E  = RED;
        light_W  = RED;
        light_NW = RED;
        light_NE = RED;
        light_SW = RED;
        light_SE = RED;

        case (current_state)
            S_G1_GREEN: begin
                light_N  = GREEN;
                light_S  = GREEN;
                light_E  = GREEN;
                light_W  = GREEN;
            end

            S_G1_YEL: begin
                light_N  = YELLOW;
                light_S  = YELLOW;
                light_E  = YELLOW;
                light_W  = YELLOW;
            end

            S_G2_GREEN: begin
                light_NW = GREEN;
                light_SE = GREEN;
            end

            S_G2_YEL: begin
                light_NW = YELLOW;
                light_SE = YELLOW;
            end

            S_G3_GREEN: begin
                light_NE = GREEN;
                light_SW = GREEN;
            end

            S_G3_YEL: begin
                light_NE = YELLOW;
                light_SW = YELLOW;
            end

            S_ALL_RED, S_PED_RED: begin
                // All RED maintained
            end

            S_AMB_N_G: begin
                light_N  = GREEN;
            end

            S_AMB_N_Y: begin
                light_N  = YELLOW;
            end

            default: begin
                // All RED maintained
            end
        endcase
    end

endmodule
