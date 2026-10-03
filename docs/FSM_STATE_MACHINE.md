# Finite State Machine (FSM) Specification

## 1. FSM States

The controller defines 10 discrete states mapped onto a 4-bit state register:

| State Constant | Binary | Value | Description |
| :--- | :--- | :--- | :--- |
| `S_G1_GREEN` | `4'b0000` | `4'd0` | Group 1 (N, S, E, W) Green; all diagonals Red. |
| `S_G1_YEL`   | `4'b0001` | `4'd1` | Group 1 (N, S, E, W) Yellow; all diagonals Red. |
| `S_G2_GREEN` | `4'b0010` | `4'd2` | Group 2 (NW, SE) Green; all other directions Red. |
| `S_G2_YEL`   | `4'b0011` | `4'd3` | Group 2 (NW, SE) Yellow; all other directions Red. |
| `S_G3_GREEN` | `4'b0100` | `4'd4` | Group 3 (SW, NE) Green; all other directions Red. |
| `S_G3_YEL`   | `4'b0101` | `4'd5` | Group 3 (SW, NE) Yellow; all other directions Red. |
| `S_ALL_RED`  | `4'b0110` | `4'd6` | Inter-phase safety clearance; all 8 directions Red. |
| `S_PED_RED`  | `4'b0111` | `4'd7` | Pedestrian crossing priority; all 8 directions Red. |
| `S_AMB_N_G`  | `4'b1000` | `4'd8` | Ambulance emergency priority; North Green, all others Red. |
| `S_AMB_N_Y`  | `4'b1001` | `4'd9` | Ambulance clearance; North Yellow, all others Red. |

---

## 2. State Transition Diagram

```mermaid
stateDiagram-v2
    [*] --> S_G1_GREEN : rst = 1

    S_G1_GREEN --> S_G1_YEL : timer >= T_G1_GREEN - 1
    S_G1_YEL --> S_ALL_RED : timer >= T_YELLOW - 1
    
    S_ALL_RED --> S_G2_GREEN : if next = G2 & timer >= T_ALL_RED - 1
    S_G2_GREEN --> S_G2_YEL : timer >= T_G2_GREEN - 1
    S_G2_YEL --> S_ALL_RED : timer >= T_YELLOW - 1

    S_ALL_RED --> S_G3_GREEN : if next = G3 & timer >= T_ALL_RED - 1
    S_G3_GREEN --> S_G3_YEL : timer >= T_G3_GREEN - 1
    S_G3_YEL --> S_ALL_RED : timer >= T_YELLOW - 1

    S_ALL_RED --> S_G1_GREEN : if next = G1 & timer >= T_ALL_RED - 1

    %% Priority Overrides
    S_G1_GREEN --> S_AMB_N_G : amb_req_N = 1
    S_G2_GREEN --> S_AMB_N_G : amb_req_N = 1
    S_G3_GREEN --> S_AMB_N_G : amb_req_N = 1
    S_ALL_RED --> S_AMB_N_G : amb_req_N = 1
    S_PED_RED --> S_AMB_N_G : amb_req_N = 1

    S_G1_GREEN --> S_PED_RED : ped_req = 1 & !amb_req_N
    S_G2_GREEN --> S_PED_RED : ped_req = 1 & !amb_req_N
    S_G3_GREEN --> S_PED_RED : ped_req = 1 & !amb_req_N
    S_ALL_RED --> S_PED_RED : ped_req = 1 & !amb_req_N

    %% Priority Resumption
    S_AMB_N_G --> S_AMB_N_Y : timer >= T_AMB - 1
    S_AMB_N_Y --> S_G1_GREEN : resume if return = G1
    S_AMB_N_Y --> S_G2_GREEN : resume if return = G2
    S_AMB_N_Y --> S_G3_GREEN : resume if return = G3

    S_PED_RED --> S_G1_GREEN : timer >= T_PED - 1 (if return = G1)
    S_PED_RED --> S_G2_GREEN : timer >= T_PED - 1 (if return = G2)
    S_PED_RED --> S_G3_GREEN : timer >= T_PED - 1 (if return = G3)
```

---

## 3. Signal Output Truth Table

Light values: `G = GREEN (3'b001)`, `Y = YELLOW (3'b010)`, `R = RED (3'b100)`.

| State | North | South | East | West | NW | NE | SW | SE | Active Condition |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| `S_G1_GREEN` | **G** | **G** | **G** | **G** | R | R | R | R | Straight lanes moving |
| `S_G1_YEL`   | **Y** | **Y** | **Y** | **Y** | R | R | R | R | Straight lanes stopping |
| `S_G2_GREEN` | R | R | R | R | **G** | R | R | **G** | NW-SE diagonals moving |
| `S_G2_YEL`   | R | R | R | R | **Y** | R | R | **Y** | NW-SE diagonals stopping |
| `S_G3_GREEN` | R | R | R | R | R | **G** | **G** | R | SW-NE diagonals moving |
| `S_G3_YEL`   | R | R | R | R | R | **Y** | **Y** | R | SW-NE diagonals stopping |
| `S_ALL_RED`  | R | R | R | R | R | R | R | R | Intersection clear interval |
| `S_PED_RED`  | R | R | R | R | R | R | R | R | Pedestrians crossing |
| `S_AMB_N_G`  | **G** | R | R | R | R | R | R | R | North ambulance corridor |
| `S_AMB_N_Y`  | **Y** | R | R | R | R | R | R | R | North ambulance clearance |

---

## 4. Priority Resolution & Preemption Rules

1. **Ambulance Highest Precedence**:
   - `amb_req_N` has strict priority over all other states, including active pedestrian halts (`S_PED_RED`).
   - Forces immediate transition to `S_AMB_N_G` within exactly one clock cycle.
   - Saves the interrupted regular phase in `return_state`.
   - Before resuming normal traffic, traverses `S_AMB_N_Y` (yellow clearance) to prevent sudden broadside collisions.

2. **Pedestrian Safety Precedence**:
   - `ped_req` preempts normal vehicular green and yellow phases.
   - Transitions directly to `S_PED_RED`, holding all 8 vehicle signals at `RED` for $T_{\text{PED}}$ clock ticks.
   - Resumes seamlessly at the next scheduled green group upon timeout.
