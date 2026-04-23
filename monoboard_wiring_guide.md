# 4-Arm Haptic Suit — Full Pin / Wiring Layout (UNO R4 WiFi + TCA9548A + 4× GY-521 (MPU6050) + 2× TB6612 + 4× ESC)

This document is the full bench-test wiring plan for validating all 4 arms before mounting on the drone.

## System Overview (What’s Connected)
- Microcontroller: **Arduino UNO R4 WiFi**
- IMU bus: **TCA9548A I2C multiplexer** + **4× GY-521 (MPU6050)** (one per arm)
- Arm rotation: **4× DC gearmotors** driven by **2× TB6612FNG** (2 motors per TB6612)
- Thrust: **4× ESC signal outputs** (one per arm, for brushless motors)

---

## A) IMU / I2C Wiring (3.3V, using Qwiic)

### A1. UNO R4 WiFi → TCA9548A (Qwiic)
Use the Qwiic cable between the UNO R4 WiFi Qwiic port and the TCA9548A (if your TCA board has Qwiic).
If your TCA board does NOT have a Qwiic connector, wire these 4 lines manually:

| Signal | UNO R4 WiFi (Qwiic port) | TCA9548A |
|---|---|---|
| 3.3V | Qwiic 3.3V | VCC (3.3V) |
| GND | Qwiic GND | GND |
| SDA | Qwiic SDA (I2C0) | SDA |
| SCL | Qwiic SCL (I2C0) | SCL |

Notes:
- This keeps the entire IMU bus at **3.3V**.
- In Arduino code, Qwiic I2C on UNO R4 WiFi is typically accessed via **Wire1**.

### A2. TCA9548A → 4× GY-521 (one per channel)
Recommended channel mapping:

| Arm | TCA9548A Channel | Pins on TCA9548A |
|---|---:|---|
| Arm 1 | 0 | SDA0 / SCL0 |
| Arm 2 | 1 | SDA1 / SCL1 |
| Arm 3 | 2 | SDA2 / SCL2 |
| Arm 4 | 3 | SDA3 / SCL3 |

For each IMU, connect:

| TCA9548A | GY-521 |
|---|---|
| SDAx | SDA |
| SCLx | SCL |
| 3.3V (VCC rail) | VCC (or VIN, depends on breakout) |
| GND | GND |

Where `x` is the channel number (0–3).

**Important rules**
- Keep each IMU on its own matching SDAx/SCLx pair (don’t cross SDA0 with SCL1, etc.).
- Power all IMUs from **3.3V** unless your breakout explicitly supports 5V input.

---

## B) Arm Rotation Motors (2× TB6612FNG driving 4 DC motors)

### B1. Shared Power + Enable (both TB6612 boards)
Each TB6612 board has:
- VCC = logic supply (5V logic)
- VM = motor power supply (external motor voltage)
- GND = ground
- STBY = standby enable (must be HIGH to drive motors)

Wire BOTH TB6612 boards like this:

| TB6612 Pin | Connect To |
|---|---|
| VCC (logic) | UNO **5V** |
| GND | UNO **GND** |
| VM (motor power) | External motor supply **+** (e.g., 6–12V as required by your DC motors) |
| Motor supply GND | External motor supply **−** (must be tied to UNO GND) |
| STBY | UNO **D13** (shared; tie STBY from both boards together to D13) |

**Grounding requirement**
- External motor supply GND MUST be connected to UNO GND (common ground), or direction/PWM control will behave unpredictably.

### B2. Control Pins Per Arm (Rotation motor control)
Assume:
- TB6612 #1 drives Arm 1 and Arm 2 rotation motors
- TB6612 #2 drives Arm 3 and Arm 4 rotation motors

PWM pins are used for speed control.
DIR pins set direction.

| Arm | TB6612 Board / Channel | PWM Pin | DIR1 | DIR2 |
|---|---|---:|---:|---:|
| Arm 1 rotation | TB6612 #1 / A (PWMA, AIN1, AIN2) | **D3** | **D10** | **D11** |
| Arm 2 rotation | TB6612 #1 / B (PWMB, BIN1, BIN2) | **D5** | **D12** | **D7** |
| Arm 3 rotation | TB6612 #2 / A (PWMA, AIN1, AIN2) | **D6** | **A0** | **A1** |
| Arm 4 rotation | TB6612 #2 / B (PWMB, BIN1, BIN2) | **D9** | **A2** | **A3** |

Motor output terminals:
- TB6612 channel A outputs: **A01 / A02**
- TB6612 channel B outputs: **B01 / B02**

So:
- Arm 1 DC motor → TB6612 #1 A01/A02
- Arm 2 DC motor → TB6612 #1 B01/B02
- Arm 3 DC motor → TB6612 #2 A01/A02
- Arm 4 DC motor → TB6612 #2 B01/B02

---

## C) ESC Signal Outputs (4 thrust motors)

Each ESC typically needs:
- Signal (PWM/servo pulse)
- Signal ground (GND reference)
Power to the ESC comes from the main battery (not from the Arduino).

### C1. UNO → ESC Signal Pins
| Arm | ESC Signal Pin (UNO R4 WiFi) |
|---|---:|
| Arm 1 thruster | **D2** |
| Arm 2 thruster | **D4** |
| Arm 3 thruster | **D8** |
| Arm 4 thruster | **A4** (used as digital) |

### C2. ESC Wiring Notes
- ESC **signal ground** must connect to **UNO GND**.
- ESC main power leads connect to the **main battery**.
- If an ESC has a BEC/5V output, **do NOT connect multiple BEC 5V outputs together**. For bench testing, it’s usually best to connect only **signal + ground** from each ESC to the Arduino and power the Arduino separately via USB-C (or one known-good regulator).

---

## D) Power / Grounding (Do Not Skip)

### D1. Power Domains
1) **IMU domain (3.3V)**
- UNO 3.3V → TCA9548A VCC → IMUs

2) **Logic domain (5V)**
- UNO 5V → TB6612 VCC (logic)

3) **DC motor domain (VM)**
- External motor supply (e.g., 6–12V) → TB6612 VM

4) **ESC / brushless domain**
- Main battery → ESC power leads

### D2. Single Common Ground
ALL of these must share ground:
- UNO GND
- TCA9548A GND
- All IMU GNDs
- TB6612 GND
- External DC motor supply GND
- ESC signal ground (and typically ESC power GND)

If you don’t share ground across these domains, you will see:
- flaky I2C reads
- motors that twitch / run wrong direction
- ESC signals that don’t arm reliably

---

## E) “At a Glance” Pin Summary

### E1. I2C (via Qwiic / Wire1)
- Qwiic SDA/SCL/3.3V/GND → TCA9548A SDA/SCL/VCC/GND
- TCA9548A CH0..CH3 → IMU #1..#4 SDA/SCL + 3.3V + GND

### E2. TB6612 (rotation)
- STBY: D13
- Arm 1: PWM D3, DIR D10/D11
- Arm 2: PWM D5, DIR D12/D7
- Arm 3: PWM D6, DIR A0/A1
- Arm 4: PWM D9, DIR A2/A3

### E3. ESC signals (thrust)
- Arm 1: D2
- Arm 2: D4
- Arm 3: D8
- Arm 4: A4

---

## F) Bench Bring-Up Order (Recommended)
1) **IMUs only**
   - Power UNO + mux + IMUs
   - Verify mux channel switching and IMU readout for all 4 arms

2) **Rotation motors**
   - Add TB6612 logic + VM motor power
   - Test each arm rotation motor briefly (low PWM) + confirm IMU angle changes

3) **ESC signals (NO PROPS)**
   - Connect signal+ground to ESCs
   - Arm/calibrate ESCs one-by-one
   - Verify each responds to low throttle pulses

---

## G) Labeling Convention (Avoid Confusion Later)
- Arm 1 = Channel 0 = ESC1 = Rotation Motor 1
- Arm 2 = Channel 1 = ESC2 = Rotation Motor 2
- Arm 3 = Channel 2 = ESC3 = Rotation Motor 3
- Arm 4 = Channel 3 = ESC4 = Rotation Motor 4

Physically label each arm harness with: `ARM1 / ARM2 / ARM3 / ARM4`.

---