// ==============================================================================
// Testbench: traffic_light_controller_tb.v
// Project: Smart Traffic Light Controller Using Verilog HDL
// Authors: Aparajith Krishna S, Sabarish V L, Akash Sriram Prasad
// Affiliation: Department of Electronics and Communication Engineering,
//              SRM Institute of Science and Technology, Kattankulathur
// Description: Comprehensive self-checking testbench covering reset sequence,
//              cyclical 8-way operation, pedestrian priority preemption,
//              ambulance high-priority override, and resume state verification.
// ==============================================================================

`timescale 1ns / 1ps

module traffic_light_controller_tb;

    // Clock and stimulus signals
    reg clk;
    reg rst;
    reg ped_req;
    reg amb_req_N;

    // Monitored outputs
    wire [2:0]  light_N, light_S, light_E, light_W;
    wire [2:0]  light_NW, light_NE, light_SW, light_SE;
    wire [23:0] lights_bus;
    wire [3:0]  current_state;
    wire [5:0]  timer;
    wire        ped_active;
    wire        amb_active;

    // Light encoding parameters
    localparam RED    = 3'b100;
    localparam YELLOW = 3'b010;
    localparam GREEN  = 3'b001;

    // State definitions
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

    // Simulation timing constants
    localparam T_G1 = 6'd6;
    localparam T_G2 = 6'd5;
    localparam T_G3 = 6'd5;
    localparam T_Y  = 6'd2;
    localparam T_AR = 6'd2;
    localparam T_P  = 6'd4;
    localparam T_A  = 6'd5;

    // Verification counters
    integer pass_count = 0;
    integer fail_count = 0;

    // Instantiate Unit Under Test (UUT)
    smart_traffic_light_controller #(
        .T_G1_GREEN(T_G1),
        .T_G2_GREEN(T_G2),
        .T_G3_GREEN(T_G3),
        .T_YELLOW(T_Y),
        .T_ALL_RED(T_AR),
        .T_PED(T_P),
        .T_AMB(T_A)
    ) uut (
        .clk(clk),
        .rst(rst),
        .ped_req(ped_req),
        .amb_req_N(amb_req_N),
        .light_N(light_N),
        .light_S(light_S),
        .light_E(light_E),
        .light_W(light_W),
        .light_NW(light_NW),
        .light_NE(light_NE),
        .light_SW(light_SW),
        .light_SE(light_SE),
        .lights_bus(lights_bus),
        .current_state(current_state),
        .timer(timer),
        .ped_active(ped_active),
        .amb_active(amb_active)
    );

    // Clock generator: 10ns period (100 MHz reference)
    always #5 clk = ~clk;

    // Assertion task
    task check_condition(input [127:0] test_name, input condition);
        begin
            if (condition) begin
                $display("[PASS] @ %0t ns | %0s", $time, test_name);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] @ %0t ns | %0s", $time, test_name);
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        // Waveform dumping for GTKWave / ModelSim / Vivado
        $dumpfile("traffic_light_controller.vcd");
        $dumpvars(0, traffic_light_controller_tb);

        $display("==================================================================");
        $display("STARTING TESTBENCH: Smart Traffic Light Controller (Verilog HDL)");
        $display("Authors: Aparajith Krishna S, Sabarish V L, Akash Sriram Prasad");
        $display("==================================================================");

        // Initialize signals
        clk       = 0;
        rst       = 1;
        ped_req   = 0;
        amb_req_N = 0;

        // ----------------------------------------------------------------------
        // Test 1: Reset Behavior & Startup State
        // ----------------------------------------------------------------------
        #25;
        rst = 0;
        #10;
        check_condition("Test 1: Reset -> Initial S_G1_GREEN State", current_state == S_G1_GREEN);
        check_condition("Test 1: N, S, E, W are GREEN", 
            (light_N == GREEN) && (light_S == GREEN) && (light_E == GREEN) && (light_W == GREEN));
        check_condition("Test 1: Diagonals are RED",
            (light_NW == RED) && (light_NE == RED) && (light_SW == RED) && (light_SE == RED));

        // ----------------------------------------------------------------------
        // Test 2: Full Regular Cyclical Sequencing
        // ----------------------------------------------------------------------
        // Wait for Group 1 Yellow
        wait(current_state == S_G1_YEL);
        check_condition("Test 2: Group 1 Yellow reached", 
            (light_N == YELLOW) && (light_S == YELLOW) && (light_E == YELLOW) && (light_W == YELLOW));

        // Wait for Group 2 Green (NW-SE)
        wait(current_state == S_G2_GREEN);
        check_condition("Test 2: Group 2 Green reached (NW, SE Green)",
            (light_NW == GREEN) && (light_SE == GREEN) && (light_N == RED) && (light_E == RED));

        // Wait for Group 3 Green (SW-NE)
        wait(current_state == S_G3_GREEN);
        check_condition("Test 2: Group 3 Green reached (SW, NE Green)",
            (light_SW == GREEN) && (light_NE == GREEN) && (light_NW == RED) && (light_N == RED));

        // Wait for cycle back to Group 1 Green
        wait(current_state == S_G1_GREEN);
        check_condition("Test 2: Cycle completed back to Group 1 Green", current_state == S_G1_GREEN);

        // ----------------------------------------------------------------------
        // Test 3: Pedestrian Priority Request (All-Red Preemption)
        // ----------------------------------------------------------------------
        #20;
        $display("[INFO] Triggering Pedestrian Request (ped_req = 1)...");
        @(posedge clk);
        ped_req = 1;
        @(posedge clk);
        ped_req = 0;

        @(posedge clk);
        check_condition("Test 3: Preempted to S_PED_RED", current_state == S_PED_RED);
        check_condition("Test 3: ped_active flag is asserted", ped_active == 1'b1);
        check_condition("Test 3: All 8 traffic directions are RED",
            (light_N == RED) && (light_S == RED) && (light_E == RED) && (light_W == RED) &&
            (light_NW == RED) && (light_NE == RED) && (light_SW == RED) && (light_SE == RED));

        // Wait for pedestrian duration to complete
        wait(current_state != S_PED_RED);
        check_condition("Test 3: Safely resumed after pedestrian duration", current_state != S_PED_RED);
        check_condition("Test 3: ped_active flag deasserted", ped_active == 1'b0);

        // ----------------------------------------------------------------------
        // Test 4: Ambulance Request from North (High Priority Override)
        // ----------------------------------------------------------------------
        #30;
        $display("[INFO] Triggering North Ambulance Request (amb_req_N = 1)...");
        @(posedge clk);
        amb_req_N = 1;
        @(posedge clk);
        amb_req_N = 0;

        @(posedge clk);
        check_condition("Test 4: Preempted to S_AMB_N_G", current_state == S_AMB_N_G);
        check_condition("Test 4: amb_active flag is asserted", amb_active == 1'b1);
        check_condition("Test 4: North is GREEN", light_N == GREEN);
        check_condition("Test 4: All other 7 directions are RED",
            (light_S == RED) && (light_E == RED) && (light_W == RED) &&
            (light_NW == RED) && (light_NE == RED) && (light_SW == RED) && (light_SE == RED));

        // Wait for ambulance yellow clearance
        wait(current_state == S_AMB_N_Y);
        check_condition("Test 4: North transitions through clearance YELLOW", light_N == YELLOW);

        // Wait for resume
        wait(current_state != S_AMB_N_Y && current_state != S_AMB_N_G);
        check_condition("Test 4: Resumed normal operation after ambulance clearance", amb_active == 1'b0);

        // ----------------------------------------------------------------------
        // Test 5: Simultaneous Ambulance & Pedestrian Arbitration
        // ----------------------------------------------------------------------
        #40;
        $display("[INFO] Triggering Simultaneous Pedestrian and Ambulance Requests...");
        @(posedge clk);
        ped_req   = 1;
        amb_req_N = 1;
        @(posedge clk);
        ped_req   = 0;
        amb_req_N = 0;

        @(posedge clk);
        check_condition("Test 5: Ambulance overrides Pedestrian (Emergency priority)", 
            current_state == S_AMB_N_G);

        #150;
        $display("==================================================================");
        $display("TESTBENCH SUMMARY: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("==================================================================");

        if (fail_count == 0)
            $display(">>> ALL VERIFICATION TESTS PASSED SUCCESSFULLY! <<<");
        else
            $display(">>> SOME TESTS FAILED. PLEASE REVIEW LOGS. <<<");

        $finish;
    end

endmodule
