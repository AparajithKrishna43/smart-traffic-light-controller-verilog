# System Architecture & Design Specification

## Overview

The **Smart Traffic Light Controller** is modeled as a synchronous digital system designed using **Verilog HDL** based on a **Finite State Machine (FSM)** architecture. The system orchestrates traffic flow across eight discrete directions at a complex intersection while providing deterministic priority preemption for emergency vehicles (ambulances) and pedestrian safety halts.

---

## 1. Functional Block Architecture

In accordance with Section V of the paper, the hardware is partitioned into four primary functional blocks:

```mermaid
flowchart TD
    subgraph Inputs["External Signals"]
        CLK["clk (System Clock)"]
        RST["rst (Active-High Reset)"]
        PED["ped_req (Pedestrian Button)"]
        AMB["amb_req_N (Ambulance Sensor - North)"]
    end

    subgraph InputUnit["1. Input Unit"]
        SYNC["2-FF Synchronizers & Request Latching Logic"]
    end

    subgraph ControlUnit["2. Control Unit (FSM)"]
        FSM["Finite State Machine<br/>- Cyclical Phases (G1, G2, G3)<br/>- Priority Overrides (S_AMB_N_G, S_PED_RED)<br/>- State History & Resume Logic"]
    end

    subgraph CounterUnit["3. Timer / Counter Unit"]
        TIMER["Synchronous Counter & Comparator<br/>- T_G1_GREEN, T_G2_GREEN, T_G3_GREEN<br/>- T_YELLOW, T_ALL_RED<br/>- T_PED, T_AMB"]
    end

    subgraph OutputUnit["4. Output Unit"]
        DECODER["Light Output Decoder<br/>3-Bit One-Hot Drive (Red, Yellow, Green)"]
    end

    subgraph Intersections["8 Traffic Directions"]
        N["North Light [2:0]"]
        S["South Light [2:0]"]
        E["East Light [2:0]"]
        W["West Light [2:0]"]
        NW["Northwest Light [2:0]"]
        NE["Northeast Light [2:0]"]
        SW["Southwest Light [2:0]"]
        SE["Southeast Light [2:0]"]
    end

    CLK --> InputUnit
    RST --> InputUnit
    PED --> InputUnit
    AMB --> InputUnit

    InputUnit -->|Synchronized & Latched Requests| ControlUnit
    ControlUnit <-->|Phase Start / Timeout| CounterUnit
    ControlUnit -->|Current State [3:0]| OutputUnit

    OutputUnit --> N & S & E & W
    OutputUnit --> NW & NE & SW & SE
```

---

## 2. Port Definitions

| Signal Name | Direction | Width | Description |
| :--- | :--- | :--- | :--- |
| `clk` | Input | 1-bit | System master clock (all transitions occur on `posedge clk`). |
| `rst` | Input | 1-bit | Active-high asynchronous reset; initializes controller to `S_G1_GREEN`. |
| `ped_req` | Input | 1-bit | Active-high request signal triggered by pedestrian crosswalk pushbutton. |
| `amb_req_N` | Input | 1-bit | Active-high sensor input detecting an approaching ambulance from the North. |
| `light_N` | Output | 3-bit | North light status: `[2]=RED, [1]=YELLOW, [0]=GREEN`. |
| `light_S` | Output | 3-bit | South light status: `[2]=RED, [1]=YELLOW, [0]=GREEN`. |
| `light_E` | Output | 3-bit | East light status: `[2]=RED, [1]=YELLOW, [0]=GREEN`. |
| `light_W` | Output | 3-bit | West light status: `[2]=RED, [1]=YELLOW, [0]=GREEN`. |
| `light_NW` | Output | 3-bit | Northwest light status: `[2]=RED, [1]=YELLOW, [0]=GREEN`. |
| `light_NE` | Output | 3-bit | Northeast light status: `[2]=RED, [1]=YELLOW, [0]=GREEN`. |
| `light_SW` | Output | 3-bit | Southwest light status: `[2]=RED, [1]=YELLOW, [0]=GREEN`. |
| `light_SE` | Output | 3-bit | Southeast light status: `[2]=RED, [1]=YELLOW, [0]=GREEN`. |
| `lights_bus` | Output | 24-bit | Concatenated bus `{light_N, light_S, light_E, light_W, light_NW, light_NE, light_SW, light_SE}`. |
| `current_state` | Output | 4-bit | Diagnostic telemetry indicating active FSM state index. |
| `timer` | Output | 6-bit | Diagnostic telemetry showing elapsed clock ticks in current phase. |
| `ped_active` | Output | 1-bit | High when pedestrian crossing phase (`S_PED_RED`) is active. |
| `amb_active` | Output | 1-bit | High when ambulance priority phase (`S_AMB_N_G` or `S_AMB_N_Y`) is active. |

---

## 3. Light Encoding Standard

Each traffic light is represented by a 3-bit one-hot code ensuring fail-safe visualization and physical LED / relay driving:

| Constant | Binary Value | Meaning | Description |
| :--- | :--- | :--- | :--- |
| `RED` | `3'b100` | Stop | Active red signal; vehicular movement halted. |
| `YELLOW` | `3'b010` | Caution | Active yellow signal; vehicular clearance phase. |
| `GREEN` | `3'b001` | Go | Active green signal; vehicular right-of-way granted. |

---

## 4. Direction Grouping & Safety Mutual Exclusion

To prevent intersection collision courses, lanes are partitioned into non-conflicting groups:

1. **Group 1 (Cardinal / Straight)**: North, South, East, West (`light_N`, `light_S`, `light_E`, `light_W`).
2. **Group 2 (Diagonal Northwest-Southeast)**: Northwest, Southeast (`light_NW`, `light_SE`).
3. **Group 3 (Diagonal Southwest-Northeast)**: Northeast, Southwest (`light_NE`, `light_SW`).
4. **Intermediate All-Red**: All 8 directions are held at `RED` for $T_{\text{ALL\_RED}}$ clock ticks between phase switches to clear any straggling vehicles.
