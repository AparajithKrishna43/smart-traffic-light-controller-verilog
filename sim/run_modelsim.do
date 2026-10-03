# ==============================================================================
# ModelSim / QuestaSim Simulation Script
# ==============================================================================
vlib work
vmap work work

# Compile design and testbench
vlog -work work src/smart_traffic_light_controller.v
vlog -work work tb/traffic_light_controller_tb.v

# Start simulation
vsim -voptargs="+acc" work.traffic_light_controller_tb

# Add signals to waveform viewer
add wave -divider "Inputs"
add wave -color cyan /traffic_light_controller_tb/clk
add wave -color orange /traffic_light_controller_tb/rst
add wave -color magenta /traffic_light_controller_tb/ped_req
add wave -color red /traffic_light_controller_tb/amb_req_N

add wave -divider "FSM & Timer"
add wave -radix hexadecimal /traffic_light_controller_tb/current_state
add wave -radix unsigned /traffic_light_controller_tb/timer
add wave /traffic_light_controller_tb/ped_active
add wave /traffic_light_controller_tb/amb_active

add wave -divider "Straight Lights"
add wave -radix binary /traffic_light_controller_tb/light_N
add wave -radix binary /traffic_light_controller_tb/light_S
add wave -radix binary /traffic_light_controller_tb/light_E
add wave -radix binary /traffic_light_controller_tb/light_W

add wave -divider "Diagonal Lights"
add wave -radix binary /traffic_light_controller_tb/light_NW
add wave -radix binary /traffic_light_controller_tb/light_NE
add wave -radix binary /traffic_light_controller_tb/light_SW
add wave -radix binary /traffic_light_controller_tb/light_SE

# Run simulation
run -all
