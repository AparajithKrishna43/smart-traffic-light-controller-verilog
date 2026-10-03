# Running on EDA Playground (Zero-Install Simulation)

You can run this project in under 30 seconds on [EDA Playground](https://www.edaplayground.com/) without installing any local software:

1. Open [edaplayground.com](https://www.edaplayground.com/).
2. In the **Testbench + Design** dropdown on the left panel, select **Verilog / SystemVerilog**.
3. Choose the simulator: **Icarus Verilog 0.10.0** or **Aldec Riviera Pro**.
4. Check **Open EPWave after run** to view waveforms directly in your browser.
5. In the **testbench** window on the left:
   - Paste the contents of [`tb/traffic_light_controller_tb.v`](../tb/traffic_light_controller_tb.v).
6. In the **design** window on the right:
   - Paste the contents of [`src/smart_traffic_light_controller.v`](../src/smart_traffic_light_controller.v).
7. Click **Run**.
8. View the test results in the console log and the timing waveforms in EPWave.
