@echo off
echo ==============================================================================
echo Compiling and Running Smart Traffic Light Controller with Icarus Verilog
echo ==============================================================================

where iverilog >nul 2>nul
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Icarus Verilog (iverilog) is not found in your PATH.
    echo Please install Icarus Verilog or use python sim\simulate.py
    exit /b 1
)

iverilog -o sim\traffic_tb.vvp -g2012 src\smart_traffic_light_controller.v tb\traffic_light_controller_tb.v
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Compilation failed!
    exit /b 1
)

vvp sim\traffic_tb.vvp

echo.
echo Simulation finished. VCD waveform saved to traffic_light_controller.vcd.
echo View waveform with: gtkwave traffic_light_controller.vcd
