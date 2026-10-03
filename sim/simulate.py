#!/usr/bin/env python3
"""
Python Cycle-Accurate Simulation & VCD Generator for
Smart Traffic Light Controller Using Verilog HDL
Authors: Aparajith Krishna S, Sabarish V L, Akash Sriram Prasad
Affiliation: Department of Electronics and Communication Engineering,
             SRM Institute of Science and Technology, Kattankulathur
"""

import sys
import os

# Light Encodings (3-bit one-hot)
RED    = 0b100
YELLOW = 0b010
GREEN  = 0b001

LIGHT_STR = {RED: "RED   ", YELLOW: "YELLOW", GREEN: "GREEN "}

# States
S_G1_GREEN = 0
S_G1_YEL   = 1
S_G2_GREEN = 2
S_G2_YEL   = 3
S_G3_GREEN = 4
S_G3_YEL   = 5
S_ALL_RED  = 6
S_PED_RED  = 7
S_AMB_N_G  = 8
S_AMB_N_Y  = 9

STATE_NAMES = {
    S_G1_GREEN: "S_G1_GREEN (Straight N/S/E/W)",
    S_G1_YEL:   "S_G1_YEL   (Straight N/S/E/W)",
    S_G2_GREEN: "S_G2_GREEN (Diagonals NW/SE)",
    S_G2_YEL:   "S_G2_YEL   (Diagonals NW/SE)",
    S_G3_GREEN: "S_G3_GREEN (Diagonals SW/NE)",
    S_G3_YEL:   "S_G3_YEL   (Diagonals SW/NE)",
    S_ALL_RED:  "S_ALL_RED  (Safety Interval)",
    S_PED_RED:  "S_PED_RED  (Pedestrian Halt)",
    S_AMB_N_G:  "S_AMB_N_G  (Ambulance North)",
    S_AMB_N_Y:  "S_AMB_N_Y  (Ambulance Clearance)"
}

class TrafficLightSimulator:
    def __init__(self, t_g1=6, t_g2=5, t_g3=5, t_yellow=2, t_all_red=2, t_ped=4, t_amb=5):
        self.t_g1 = t_g1
        self.t_g2 = t_g2
        self.t_g3 = t_g3
        self.t_yellow = t_yellow
        self.t_all_red = t_all_red
        self.t_ped = t_ped
        self.t_amb = t_amb

        self.reset()
        self.vcd_lines = []
        self.vcd_prev_state = None

    def reset(self):
        self.current_state = S_G1_GREEN
        self.timer = 0
        self.return_state = S_G2_GREEN
        self.next_normal_state = S_G2_GREEN
        self.latched_ped = False
        self.latched_amb = False
        self.lights = {dir_name: RED for dir_name in ["N", "S", "E", "W", "NW", "NE", "SW", "SE"]}
        self.update_outputs()

    def update_outputs(self):
        # Default all RED
        for k in self.lights:
            self.lights[k] = RED

        s = self.current_state
        if s == S_G1_GREEN:
            for k in ["N", "S", "E", "W"]: self.lights[k] = GREEN
        elif s == S_G1_YEL:
            for k in ["N", "S", "E", "W"]: self.lights[k] = YELLOW
        elif s == S_G2_GREEN:
            for k in ["NW", "SE"]: self.lights[k] = GREEN
        elif s == S_G2_YEL:
            for k in ["NW", "SE"]: self.lights[k] = YELLOW
        elif s == S_G3_GREEN:
            for k in ["NE", "SW"]: self.lights[k] = GREEN
        elif s == S_G3_YEL:
            for k in ["NE", "SW"]: self.lights[k] = YELLOW
        elif s == S_AMB_N_G:
            self.lights["N"] = GREEN
        elif s == S_AMB_N_Y:
            self.lights["N"] = YELLOW
        # S_ALL_RED and S_PED_RED keep all RED

    def step(self, ped_req, amb_req_N):
        # Latch requests
        if amb_req_N:
            self.latched_amb = True
        elif self.current_state == S_AMB_N_G and self.timer == (self.t_amb - 1):
            self.latched_amb = False

        if ped_req:
            self.latched_ped = True
        elif self.current_state == S_PED_RED and self.timer == (self.t_ped - 1):
            self.latched_ped = False

        # Priority 1: Ambulance
        if (amb_req_N or self.latched_amb) and self.current_state not in (S_AMB_N_G, S_AMB_N_Y):
            if self.current_state in (S_G1_GREEN, S_G1_YEL):
                self.return_state = S_G2_GREEN
            elif self.current_state in (S_G2_GREEN, S_G2_YEL):
                self.return_state = S_G3_GREEN
            elif self.current_state in (S_G3_GREEN, S_G3_YEL):
                self.return_state = S_G1_GREEN

            self.current_state = S_AMB_N_G
            self.timer = 0

        # Priority 2: Pedestrian
        elif (ped_req or self.latched_ped) and self.current_state not in (S_PED_RED, S_AMB_N_G, S_AMB_N_Y):
            if self.current_state in (S_G1_GREEN, S_G1_YEL):
                self.return_state = S_G2_GREEN
            elif self.current_state in (S_G2_GREEN, S_G2_YEL):
                self.return_state = S_G3_GREEN
            elif self.current_state in (S_G3_GREEN, S_G3_YEL):
                self.return_state = S_G1_GREEN

            self.current_state = S_PED_RED
            self.timer = 0

        # Regular timer operation
        else:
            s = self.current_state
            if s == S_G1_GREEN:
                if self.timer >= (self.t_g1 - 1):
                    self.current_state = S_G1_YEL
                    self.timer = 0
                    self.next_normal_state = S_G2_GREEN
                else:
                    self.timer += 1
            elif s == S_G1_YEL:
                if self.timer >= (self.t_yellow - 1):
                    self.current_state = S_ALL_RED
                    self.timer = 0
                else:
                    self.timer += 1
            elif s == S_G2_GREEN:
                if self.timer >= (self.t_g2 - 1):
                    self.current_state = S_G2_YEL
                    self.timer = 0
                    self.next_normal_state = S_G3_GREEN
                else:
                    self.timer += 1
            elif s == S_G2_YEL:
                if self.timer >= (self.t_yellow - 1):
                    self.current_state = S_ALL_RED
                    self.timer = 0
                else:
                    self.timer += 1
            elif s == S_G3_GREEN:
                if self.timer >= (self.t_g3 - 1):
                    self.current_state = S_G3_YEL
                    self.timer = 0
                    self.next_normal_state = S_G1_GREEN
                else:
                    self.timer += 1
            elif s == S_G3_YEL:
                if self.timer >= (self.t_yellow - 1):
                    self.current_state = S_ALL_RED
                    self.timer = 0
                else:
                    self.timer += 1
            elif s == S_ALL_RED:
                if self.timer >= (self.t_all_red - 1):
                    self.current_state = self.next_normal_state
                    self.timer = 0
                else:
                    self.timer += 1
            elif s == S_PED_RED:
                if self.timer >= (self.t_ped - 1):
                    self.current_state = self.return_state
                    self.timer = 0
                else:
                    self.timer += 1
            elif s == S_AMB_N_G:
                if self.timer >= (self.t_amb - 1):
                    self.current_state = S_AMB_N_Y
                    self.timer = 0
                else:
                    self.timer += 1
            elif s == S_AMB_N_Y:
                if self.timer >= (self.t_yellow - 1):
                    self.current_state = self.return_state
                    self.timer = 0
                else:
                    self.timer += 1

        self.update_outputs()

    def generate_vcd_header(self):
        return [
            "$date", "   October 2026", "$end",
            "$version", "   Smart Traffic Light Controller Sim v1.0", "$end",
            "$timescale 1ns $end",
            "$scope module traffic_light_controller_tb $end",
            "$var wire 1 ! clk $end",
            "$var wire 1 \" rst $end",
            "$var wire 1 # ped_req $end",
            "$var wire 1 $ amb_req_N $end",
            "$var wire 4 % current_state [3:0] $end",
            "$var wire 6 & timer [5:0] $end",
            "$var wire 3 ' light_N [2:0] $end",
            "$var wire 3 ( light_S [2:0] $end",
            "$var wire 3 ) light_E [2:0] $end",
            "$var wire 3 * light_W [2:0] $end",
            "$var wire 3 + light_NW [2:0] $end",
            "$var wire 3 , light_NE [2:0] $end",
            "$var wire 3 - light_SW [2:0] $end",
            "$var wire 3 . light_SE [2:0] $end",
            "$upscope $end",
            "$enddefinitions $end",
            "$dumpvars",
            "b0 !", "b0 \"", "b0 #", "b0 $",
            "b0000 %", "b000000 &",
            "b100 '", "b100 (", "b100 )", "b100 *",
            "b100 +", "b100 ,", "b100 -", "b100 .",
            "$end"
        ]

def run_testbench():
    sim = TrafficLightSimulator()
    print("=" * 76)
    print(" Smart Traffic Light Controller (Verilog HDL) - Cycle-Accurate Simulator")
    print(" Authors: Aparajith Krishna S, Sabarish V L, Akash Sriram Prasad")
    print(" SRM Institute of Science and Technology, Kattankulathur")
    print("=" * 76)

    passed = 0
    failed = 0

    def assert_test(cond, msg, cycle):
        nonlocal passed, failed
        if cond:
            print(f"[PASS] Cycle {cycle:3d}: {msg}")
            passed += 1
        else:
            print(f"[FAIL] Cycle {cycle:3d}: {msg}")
            failed += 1

    vcd_records = sim.generate_vcd_header()
    time_ns = 0

    # Test 1: Reset & Start
    assert_test(sim.current_state == S_G1_GREEN, "Reset -> Initial S_G1_GREEN", 0)
    assert_test(sim.lights["N"] == GREEN and sim.lights["NW"] == RED, "Straight GREEN, Diagonals RED", 0)

    # Run for 25 cycles of regular operation
    print("\n--- Running Normal Sequential Operation (G1 Straight -> G2 NW/SE -> G3 SW/NE) ---")
    g2_visited = False
    g3_visited = False
    for c in range(1, 26):
        sim.step(ped_req=False, amb_req_N=False)
        time_ns += 10
        if sim.current_state == S_G2_GREEN:
            g2_visited = True
        if sim.current_state == S_G3_GREEN:
            g3_visited = True

    assert_test(g2_visited, "Transitioned to Group 2 Diagonals (NW-SE)", 25)
    assert_test(g3_visited, "Transitioned to Group 3 Diagonals (SW-NE)", 25)

    # Test 3: Pedestrian Priority
    print("\n--- Testing Pedestrian Request (ped_req = 1) ---")
    sim.step(ped_req=True, amb_req_N=False)
    time_ns += 10
    assert_test(sim.current_state == S_PED_RED, "Immediate preemption to S_PED_RED", 26)
    all_red = all(v == RED for v in sim.lights.values())
    assert_test(all_red, "All 8 directions forced to RED during Pedestrian phase", 26)

    # Wait out pedestrian time (4 cycles)
    for c in range(27, 31):
        sim.step(ped_req=False, amb_req_N=False)
        time_ns += 10
    assert_test(sim.current_state != S_PED_RED, "Safely resumed normal sequencing after pedestrian interval", 31)

    # Test 4: Ambulance Request North Priority
    print("\n--- Testing North Ambulance Priority (amb_req_N = 1) ---")
    sim.step(ped_req=False, amb_req_N=True)
    time_ns += 10
    assert_test(sim.current_state == S_AMB_N_G, "Immediate preemption to S_AMB_N_G", 32)
    assert_test(sim.lights["N"] == GREEN and sim.lights["S"] == RED and sim.lights["NW"] == RED,
                "North signal GREEN, all other 7 directions RED", 32)

    # Wait for ambulance passage + yellow clearance
    amb_yellow_reached = False
    for c in range(33, 40):
        sim.step(ped_req=False, amb_req_N=False)
        time_ns += 10
        if sim.current_state == S_AMB_N_Y:
            amb_yellow_reached = True

    assert_test(amb_yellow_reached, "Ambulance transitions through North YELLOW clearance", 40)
    assert_test(sim.current_state not in (S_AMB_N_G, S_AMB_N_Y), "Safely resumed regular cycle after ambulance clearance", 40)

    # Test 5: Conflict Resolution
    print("\n--- Testing Simultaneous Ambulance and Pedestrian Requests ---")
    sim.step(ped_req=True, amb_req_N=True)
    time_ns += 10
    assert_test(sim.current_state == S_AMB_N_G, "Emergency Ambulance takes priority over Pedestrian halt", 41)

    print("\n" + "=" * 76)
    print(f" SIMULATION COMPLETED: {passed} PASSED, {failed} FAILED")
    print("=" * 76)

    # Write out a VCD file
    vcd_path = os.path.join(os.path.dirname(__file__), "..", "traffic_light_controller.vcd")
    with open(vcd_path, "w") as f:
        f.write("\n".join(vcd_records) + "\n")
    print(f"[INFO] VCD trace file written to: {os.path.abspath(vcd_path)}")

    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(run_testbench())
