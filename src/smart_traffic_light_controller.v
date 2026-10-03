// ==============================================================================
// Project: Smart Traffic Light Controller Using Verilog HDL
// Authors: Aparajith Krishna S, Sabarish V L, Akash Sriram Prasad
// Affiliation: Department of Electronics and Communication Engineering,
//              SRM Institute of Science and Technology, Kattankulathur
// Description: Multi-directional 8-way FSM Traffic Light Controller featuring
//              dynamic priority handling for Ambulances (North priority) and
//              Pedestrians (All-Red safety halt).
// ==============================================================================

`timescale 1ns / 1ps

module smart_traffic_light_controller #(
    // Timing intervals (in clock cycles)
    parameter T_G1_GREEN = 6'd10, // Straight directions green time (N, S, E, W)
    parameter T_G2_GREEN = 6'd8,  // Diagonal NW-SE green time
    parameter T_G3_GREEN = 6'd8,  // Diagonal SW-NE green time
    parameter T_YELLOW   = 6'd3,  // Yellow clearance duration
    parameter T_ALL_RED  = 6'd2,  // Inter-phase all-red safety interval
    parameter T_PED      = 6'd6,  // Pedestrian crossing all-red duration
    parameter T_AMB      = 6'd8   // North ambulance priority green duration
)(
    input  wire        clk,        // System clock
    input  wire        rst,        // Active-high asynchronous reset
    input  wire        ped_req,    // Pedestrian crossing request (active-high)
    input  wire        amb_req_N,  // Emergency ambulance request from North (active-high)

    // Individual 3-bit outputs for 8 traffic directions
    // Bit mapping: [2] = RED, [1] = YELLOW, [0] = GREEN
    output reg  [2:0]  light_N,    // North
    output reg  [2:0]  light_S,    // South
    output reg  [2:0]  light_E,    // East
    output reg  [2:0]  light_W,    // West
    output reg  [2:0]  light_NW,   // Northwest
    output reg  [2:0]  light_NE,   // Northeast
    output reg  [2:0]  light_SW,   // Southwest
    output reg  [2:0]  light_SE,   // Southeast

    // Packed 24-bit output bus {N, S, E, W, NW, NE, SW, SE}
    output wire [23:0] lights_bus,

    // Status and telemetry outputs
    output reg  [3:0]  current_state,
    output reg  [5:0]  timer,
    output wire        ped_active,
    output wire        amb_active
);

    // ==========================================================================
    // 3-Bit One-Hot Light Color Encodings
    // ==========================================================================
    localparam RED    = 3'b100;
    localparam YELLOW = 3'b010;
    localparam GREEN  = 3'b001;

    // ==========================================================================
    // FSM State Definitions
    // ==========================================================================
    localparam S_G1_GREEN = 4'd0;  // Group 1 Green  (N, S, E, W Green; Diagonals Red)
    localparam S_G1_YEL   = 4'd1;  // Group 1 Yellow (N, S, E, W Yellow; Diagonals Red)
    localparam S_G2_GREEN = 4'd2;  // Group 2 Green  (NW, SE Green; Others Red)
    localparam S_G2_YEL   = 4'd3;  // Group 2 Yellow (NW, SE Yellow; Others Red)
    localparam S_G3_GREEN = 4'd4;  // Group 3 Green  (SW, NE Green; Others Red)
    localparam S_G3_YEL   = 4'd5;  // Group 3 Yellow (SW, NE Yellow; Others Red)
    localparam S_ALL_RED  = 4'd6;  // Intermediate all-red safety clearance
    localparam S_PED_RED  = 4'd7;  // Pedestrian crossing priority (All-Red)
    localparam S_AMB_N_G  = 4'd8;  // Ambulance priority North Green (Others Red)
    localparam S_AMB_N_Y  = 4'd9;  // Ambulance clearance North Yellow

    // State registers
    reg [3:0] next_state;
    reg [3:0] return_state;        // Target scheduled state to resume after priority
    reg [3:0] next_normal_state;   // Next planned cycle state after yellow/all-red

    // Request latching registers (sampled and held until serviced)
    reg latched_ped_req;
    reg latched_amb_req;

    // Status assignments
    assign lights_bus = {light_N, light_S, light_E, light_W,
                         light_NW, light_NE, light_SW, light_SE};
    assign ped_active = (current_state == S_PED_RED);
    assign amb_active = (current_state == S_AMB_N_G) || (current_state == S_AMB_N_Y);

    // ==========================================================================
    // Request Latching Logic
    // Ensures asynchronous pulses are not missed while servicing a cycle
    // ==========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            latched_ped_req <= 1'b0;
            latched_amb_req <= 1'b0;
        end else begin
            // Latch new requests
            if (amb_req_N)
                latched_amb_req <= 1'b1;
            else if (current_state == S_AMB_N_G && timer == (T_AMB - 1))
                latched_amb_req <= 1'b0; // Cleared once service begins completing

            if (ped_req)
                latched_ped_req <= 1'b1;
            else if (current_state == S_PED_RED && timer == (T_PED - 1))
                latched_ped_req <= 1'b0; // Cleared once service begins completing
        end
    end

    // ==========================================================================
    // FSM State Transition & Timer Sequential Logic
    // ==========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            current_state     <= S_G1_GREEN;
            timer             <= 6'd0;
            return_state      <= S_G2_GREEN;
            next_normal_state <= S_G2_GREEN;
        end else begin
            // Check for immediate high-priority preemption (Ambulance highest priority)
            if ((amb_req_N || latched_amb_req) && 
                (current_state != S_AMB_N_G) && (current_state != S_AMB_N_Y)) begin
                // Save resume point if preemption happens during normal operation
                if (current_state == S_G1_GREEN || current_state == S_G1_YEL)
                    return_state <= S_G2_GREEN;
                else if (current_state == S_G2_GREEN || current_state == S_G2_YEL)
                    return_state <= S_G3_GREEN;
                else if (current_state == S_G3_GREEN || current_state == S_G3_YEL)
                    return_state <= S_G1_GREEN;

                current_state <= S_AMB_N_G;
                timer         <= 6'd0;
            end
            // Check for Pedestrian priority (if not servicing Ambulance)
            else if ((ped_req || latched_ped_req) && 
                     (current_state != S_PED_RED) && 
                     (current_state != S_AMB_N_G) && 
                     (current_state != S_AMB_N_Y)) begin
                // Record the scheduled state after pedestrian crossing
                if (current_state == S_G1_GREEN || current_state == S_G1_YEL)
                    return_state <= S_G2_GREEN;
                else if (current_state == S_G2_GREEN || current_state == S_G2_YEL)
                    return_state <= S_G3_GREEN;
                else if (current_state == S_G3_GREEN || current_state == S_G3_YEL)
                    return_state <= S_G1_GREEN;

                current_state <= S_PED_RED;
                timer         <= 6'd0;
            end
            // Normal timer progression and state machine transitions
            else begin
                case (current_state)
                    // ----------------------------------------------------------
                    // Group 1: Straight Directions (N, S, E, W)
                    // ----------------------------------------------------------
                    S_G1_GREEN: begin
                        if (timer >= (T_G1_GREEN - 1)) begin
                            current_state     <= S_G1_YEL;
                            timer             <= 6'd0;
                            next_normal_state <= S_G2_GREEN;
                        end else begin
                            timer <= timer + 1'b1;
                        end
                    end

                    S_G1_YEL: begin
                        if (timer >= (T_YELLOW - 1)) begin
                            current_state <= S_ALL_RED;
                            timer         <= 6'd0;
                        end else begin
                            timer <= timer + 1'b1;
                        end
                    end

                    // ----------------------------------------------------------
                    // Group 2: Diagonals NW-SE
                    // ----------------------------------------------------------
                    S_G2_GREEN: begin
                        if (timer >= (T_G2_GREEN - 1)) begin
                            current_state     <= S_G2_YEL;
                            timer             <= 6'd0;
                            next_normal_state <= S_G3_GREEN;
                        end else begin
                            timer <= timer + 1'b1;
                        end
                    end

                    S_G2_YEL: begin
                        if (timer >= (T_YELLOW - 1)) begin
                            current_state <= S_ALL_RED;
                            timer         <= 6'd0;
                        end else begin
                            timer <= timer + 1'b1;
                        end
                    end

                    // ----------------------------------------------------------
                    // Group 3: Diagonals SW-NE
                    // ----------------------------------------------------------
                    S_G3_GREEN: begin
                        if (timer >= (T_G3_GREEN - 1)) begin
                            current_state     <= S_G3_YEL;
                            timer             <= 6'd0;
                            next_normal_state <= S_G1_GREEN;
                        end else begin
                            timer <= timer + 1'b1;
                        end
                    end

                    S_G3_YEL: begin
                        if (timer >= (T_YELLOW - 1)) begin
                            current_state <= S_ALL_RED;
                            timer         <= 6'd0;
                        end else begin
                            timer <= timer + 1'b1;
                        end
                    end

                    // ----------------------------------------------------------
                    // Safe Inter-phase All-Red Clearance Interval
                    // ----------------------------------------------------------
                    S_ALL_RED: begin
                        if (timer >= (T_ALL_RED - 1)) begin
                            current_state <= next_normal_state;
                            timer         <= 6'd0;
                        end else begin
                            timer <= timer + 1'b1;
                        end
                    end

                    // ----------------------------------------------------------
                    // Special Priority 1: Pedestrian Crossing (All-Red)
                    // ----------------------------------------------------------
                    S_PED_RED: begin
                        if (timer >= (T_PED - 1)) begin
                            // Return safely to scheduled state
                            current_state <= return_state;
                            timer         <= 6'd0;
                        end else begin
                            timer <= timer + 1'b1;
                        end
                    end

                    // ----------------------------------------------------------
                    // Special Priority 2: North Ambulance Priority (North Green)
                    // ----------------------------------------------------------
                    S_AMB_N_G: begin
                        if (timer >= (T_AMB - 1)) begin
                            current_state <= S_AMB_N_Y;
                            timer         <= 6'd0;
                        end else begin
                            timer <= timer + 1'b1;
                        end
                    end

                    S_AMB_N_Y: begin
                        if (timer >= (T_YELLOW - 1)) begin
                            // Return to scheduled state after yellow clearance
                            current_state <= return_state;
                            timer         <= 6'd0;
                        end else begin
                            timer <= timer + 1'b1;
                        end
                    end

                    default: begin
                        current_state <= S_G1_GREEN;
                        timer         <= 6'd0;
                    end
                endcase
            end
        end
    end

    // ==========================================================================
    // Output Decoder Logic: Maps FSM State to Light Output Vectors
    // ==========================================================================
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
            // Group 1 Green: Straight directions (North, South, East, West)
            S_G1_GREEN: begin
                light_N  = GREEN;
                light_S  = GREEN;
                light_E  = GREEN;
                light_W  = GREEN;
                light_NW = RED;
                light_NE = RED;
                light_SW = RED;
                light_SE = RED;
            end

            // Group 1 Yellow: Straight directions transition to yellow
            S_G1_YEL: begin
                light_N  = YELLOW;
                light_S  = YELLOW;
                light_E  = YELLOW;
                light_W  = YELLOW;
                light_NW = RED;
                light_NE = RED;
                light_SW = RED;
                light_SE = RED;
            end

            // Group 2 Green: NW and SE Diagonals
            S_G2_GREEN: begin
                light_N  = RED;
                light_S  = RED;
                light_E  = RED;
                light_W  = RED;
                light_NW = GREEN;
                light_SE = GREEN;
                light_NE = RED;
                light_SW = RED;
            end

            // Group 2 Yellow: NW and SE Diagonals yellow
            S_G2_YEL: begin
                light_N  = RED;
                light_S  = RED;
                light_E  = RED;
                light_W  = RED;
                light_NW = YELLOW;
                light_SE = YELLOW;
                light_NE = RED;
                light_SW = RED;
            end

            // Group 3 Green: SW and NE Diagonals
            S_G3_GREEN: begin
                light_N  = RED;
                light_S  = RED;
                light_E  = RED;
                light_W  = RED;
                light_NW = RED;
                light_SE = RED;
                light_NE = GREEN;
                light_SW = GREEN;
            end

            // Group 3 Yellow: SW and NE Diagonals yellow
            S_G3_YEL: begin
                light_N  = RED;
                light_S  = RED;
                light_E  = RED;
                light_W  = RED;
                light_NW = RED;
                light_SE = RED;
                light_NE = YELLOW;
                light_SW = YELLOW;
            end

            // Safe Inter-phase All-Red clearance
            S_ALL_RED: begin
                light_N  = RED;
                light_S  = RED;
                light_E  = RED;
                light_W  = RED;
                light_NW = RED;
                light_NE = RED;
                light_SW = RED;
                light_SE = RED;
            end

            // Pedestrian crossing state: All vehicle signals remain RED
            S_PED_RED: begin
                light_N  = RED;
                light_S  = RED;
                light_E  = RED;
                light_W  = RED;
                light_NW = RED;
                light_NE = RED;
                light_SW = RED;
                light_SE = RED;
            end

            // North Ambulance Priority: North is GREEN, all others RED
            S_AMB_N_G: begin
                light_N  = GREEN;
                light_S  = RED;
                light_E  = RED;
                light_W  = RED;
                light_NW = RED;
                light_NE = RED;
                light_SW = RED;
                light_SE = RED;
            end

            // North Ambulance Clearance: North transitions through YELLOW
            S_AMB_N_Y: begin
                light_N  = YELLOW;
                light_S  = RED;
                light_E  = RED;
                light_W  = RED;
                light_NW = RED;
                light_NE = RED;
                light_SW = RED;
                light_SE = RED;
            end

            default: begin
                light_N  = RED;
                light_S  = RED;
                light_E  = RED;
                light_W  = RED;
                light_NW = RED;
                light_NE = RED;
                light_SW = RED;
                light_SE = RED;
            end
        endcase
    end

endmodule
