# Haptic Suit Upgrade Plan: 2-Arm to 4-Arm Scale-Up

This document outlines the strategy to upgrade the Haptic Suit system from a single-microcontroller, 2-arm configuration to a dual-microcontroller, 4-arm configuration using a Master-Slave architecture.

## 1. Architecture Overview

### Hardware Logic
*   **Total Arms:** 4 (Arm 1, Arm 2, Arm 3, Arm 4)
*   **Microcontrollers:** 2x Adafruit Feather M0
    *   **Master (WiFi):** Handles Arm 1 & Arm 2. Manages WiFi communication with the Host (ROS). Controls the Slave.
    *   **Slave (No WiFi):** Handles Arm 3 & Arm 4. Receives commands from Master. Sends sensor data to Master.
*   **Inter-Board Communication:** UART (Serial1) or I2C. UART is recommended for simplicity (TX/RX lines).

### Data Flow
1.  **Host (ROS)** sends UDP packet with targets for all 4 arms to **Master**.
2.  **Master** parses packet:
    *   Applies targets for Arm 1 & 2 to local motors/PID.
    *   Forwards targets for Arm 3 & 4 to **Slave** via UART.
3.  **Slave** receives targets:
    *   Applies targets for Arm 3 & 4 to local motors/PID.
    *   Reads local IMUs (Arm 3 & 4).
    *   Sends IMU feedback (current roll) back to **Master** (optional, for telemetry).

## 2. Firmware Implementation Plan

### 2.1. Slave Firmware (`HapticSuitSlave.ino`)
*   **Base:** Clone `HapticSuitController.ino`.
*   **Modifications:**
    *   Remove WiFi functionality (`WiFi101`, `WiFiUdp`).
    *   Keep `PID` and `IMU` logic.
    *   Set `NUMBER_ARMS` to 2.
    *   **Loop Logic:**
        *   Listen on `Serial1` (Hardware Serial) for incoming packets from Master (e.g., `<roll3, roll4, throt3, throt4>`).
        *   Update local setpoints.
        *   Run PID loop for local motors.
        *   Send feedback string to Master if required.

### 2.2. Master Firmware (`HapticSuitMaster.ino`)
*   **Base:** Current `HapticSuitController.ino`.
*   **Modifications:**
    *   Keep WiFi functionality.
    *   Set `NUMBER_ARMS` to 2 (for local control of Arm 1 & 2).
    *   **UDP Parsing:** Update to accept 8 values (Roll 1-4, Throttle 1-4) or maintain 4-command structure if kinematic logic handles mapping differently.
    *   **Loop Logic:**
        *   Apply `roll1`, `roll2` locally.
        *   Construct a serial packet for `roll3`, `roll4`, etc., and send via `Serial1` to Slave.
        *   (Optional) Parse feedback from Slave to send full telemetry back to Host.

## 3. Host / ROS Implementation Plan

### 3.1. Update `drone_ws/src/flight_controller/src/suit_controller_*.py`
*   **State Variables:** Expand `self.throttle` and `self.angle` lists from 2 to 4 elements.
*   **Command Construction:** Update `_send_command` to format the UDP string for 4 arms.
    *   *New Format Suggestion:* `"r1,r2,r3,r4,t1,t2,t3,t4"` (Comma separated).
    *   *Legacy Compatibility:* Ensure the Master firmware parses this correctly.

### 3.2. Kinematics Integration
*   **Source:** `kinematics/matlab/FullEquations/kinematic_equations.m`
*   **Action:** Translate the MATLAB logic into Python for the ROS controller.
    *   The MATLAB script calculates forces `f1..f4` and torques `t1..t4` based on desired wrench/movement.
    *   Implement an `InverseKinematics` class in Python that takes desired body torque/force and outputs the 4 motor angles/throttles.

## 4. Hardware Wiring Plan (Master-Slave)

1.  **Common Ground:** Connect GND of Master and Slave together.
2.  **UART Connection:**
    *   Master TX -> Slave RX
    *   Master RX -> Slave TX
3.  **IMU Addressing:**
    *   **Master Chain:** IMU 1 (0x4B), IMU 2 (0x4A).
    *   **Slave Chain:** IMU 3 (0x4B), IMU 4 (0x4A). (Since they are on separate I2C buses of different controllers, address conflicts are avoided).

## 5. Execution Steps

1.  **Phase 1 (Slave):** Create and test `HapticSuitSlave` on a standalone board. Verify it accepts Serial commands.
2.  **Phase 2 (Master):** Update `HapticSuitController` to `HapticSuitMaster`. Implement UDP expansion and Serial forwarding.
3.  **Phase 3 (Integration):** Wire Master and Slave. Test full signal chain (WiFi -> Master -> UART -> Slave).
4.  **Phase 4 (ROS):** Update Python scripts to send 4-arm commands.

