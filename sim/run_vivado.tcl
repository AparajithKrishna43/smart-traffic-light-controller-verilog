# ==============================================================================
# Xilinx Vivado Non-Project Batch Simulation Script
# ==============================================================================
set proj_name "smart_traffic_light_controller"

# Create in-memory project
create_project -in_memory -part xc7a35tcpg236-1

# Read design sources
read_verilog src/smart_traffic_light_controller.v
read_verilog tb/traffic_light_controller_tb.v

# Set top module
set_property top traffic_light_controller_tb [get_filesets sim_1]

# Run behavioral simulation
launch_simulation -sim_type behavioral

# Run for 2000ns
run 2000ns
