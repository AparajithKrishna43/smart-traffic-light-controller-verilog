#!/usr/bin/env bash
set -e

echo "=============================================================================="
echo "Compiling and Running Smart Traffic Light Controller with Icarus Verilog"
echo "=============================================================================="

if ! command -v iverilog &> /dev/null; then
    echo "[ERROR] iverilog not found. Please install iverilog or run 'python3 sim/simulate.py'"
    exit 1
fi

iverilog -o sim/traffic_tb.vvp -g2012 src/smart_traffic_light_controller.v tb/traffic_light_controller_tb.v
vvp sim/traffic_tb.vvp

echo ""
echo "Simulation finished. VCD waveform saved to traffic_light_controller.vcd."
echo "View waveform with: gtkwave traffic_light_controller.vcd"
