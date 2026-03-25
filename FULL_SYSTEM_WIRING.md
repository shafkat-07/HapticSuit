## Full 4-Arm Haptic Suit Wiring Guide

This guide details how to wire the entire system: 2 Microcontrollers (Master & Slave), 4 IMUs, and 4 ESCs (Motors).

### 1. Power & Common Connections (Breadboard)
**Goal:** Share Ground (GND) and ensure both boards are powered.

1.  **GND Rail:** Use the long **Blue (-) Rail** on your breadboard as the common ground.
2.  **Master GND:** Connect **Master GND** to the Blue Rail.
3.  **Slave GND:** Connect **Slave GND** to the Blue Rail.
4.  **Power:** Connect your battery or power source to the **Bat** or **USB** pins as appropriate for your setup. If using a single battery, connect its (-) to the Blue Rail and (+) to the `Bat` pin on *both* boards.

---

### 2. Communication (Master <-> Slave)
**Goal:** Allow Master to send commands to Slave.

| Master Pin | Slave Pin | Wire Color (Suggested) |
| :--- | :--- | :--- |
| **TX (Pin 1)** | **RX (Pin 0)** | Green |
| **RX (Pin 0)** | **TX (Pin 1)** | Yellow |

---

### 3. IMU Connections (Sensors)
**Goal:** Connect 2 IMUs to Master and 2 IMUs to Slave.
*Note: IMUs use I2C. Address 0x4B is the default. Address 0x4A is selected by connecting the IMU's ADR pin to 3V.*

#### **Master IMUs (Arm 1 & 2)**
*   **SDA:** Connect both IMU SDA pins to **Master SDA**.
*   **SCL:** Connect both IMU SCL pins to **Master SCL**.
*   **Power:** 3V and GND to all.
*   **Addressing:**
    *   **IMU 1 (Arm 1):** Leave ADR pin disconnected (Address 0x4B).
    *   **IMU 2 (Arm 2):** Connect **ADR pin to 3V** (Address 0x4A).

#### **Slave IMUs (Arm 3 & 4)**
*   **SDA:** Connect both IMU SDA pins to **Slave SDA**.
*   **SCL:** Connect both IMU SCL pins to **Slave SCL**.
*   **Power:** 3V and GND to all.
*   **Addressing:**
    *   **IMU 3 (Arm 3):** Leave ADR pin disconnected (Address 0x4B).
    *   **IMU 4 (Arm 4):** Connect **ADR pin to 3V** (Address 0x4A).

---

### 4. Motor Connections (ESCs)
**Goal:** Connect the PWM signal wires from the ESCs to the correct microcontroller pins.

#### **Master Board (Controls Arm 1 & 2)**
| Arm | Component | Pin on Master |
| :--- | :--- | :--- |
| **Arm 1** | ESC Signal (PWM) | **Pin 9** |
| **Arm 1** | Motor Direction 1 | **Pin 12** |
| **Arm 1** | Motor Direction 2 | **Pin 6** |
| **Arm 1** | PWM Speed Control | **Pin 5** |
| | | |
| **Arm 2** | ESC Signal (PWM) | **Pin 10** |
| **Arm 2** | Motor Direction 1 | **Pin 14** |
| **Arm 2** | Motor Direction 2 | **Pin 13** |
| **Arm 2** | PWM Speed Control | **Pin 11** |

#### **Slave Board (Controls Arm 3 & 4)**
| Arm | Component | Pin on Slave |
| :--- | :--- | :--- |
| **Arm 3** | ESC Signal (PWM) | **Pin 9** |
| **Arm 3** | Motor Direction 1 | **Pin 12** |
| **Arm 3** | Motor Direction 2 | **Pin 6** |
| **Arm 3** | PWM Speed Control | **Pin 5** |
| | | |
| **Arm 4** | ESC Signal (PWM) | **Pin 10** |
| **Arm 4** | Motor Direction 1 | **Pin 14** |
| **Arm 4** | Motor Direction 2 | **Pin 13** |
| **Arm 4** | PWM Speed Control | **Pin 11** |

*Note: The pin numbers for Arm 3/4 are identical to Arm 1/2 because they are on a DIFFERENT board (the Slave board).*

---

### Summary Checklist (Master/Slave Layout)
- [ ] **GND Shared:** Master, Slave, and all components connected to Blue Rail.
- [ ] **Serial Crossed:** Master TX->Slave RX, Master RX->Slave TX.
- [ ] **I2C Separated:** Master has 2 IMUs, Slave has 2 IMUs.
- [ ] **Motors Split:** Master controls ESC 1&2, Slave controls ESC 3&4.

---

## Uno R4 WiFi Single‑Board Variant (Monoboard)

The new **monoboard** architecture replaces the dual‑Feather Master/Slave layout with a **single Arduino UNO R4 WiFi** plus:

- 1× **TCA9548A** I2C multiplexer
- 4× **BNO086** IMUs (one per arm)
- 2× **TB6612FNG** motor drivers (rotation motors, 2 per board)
- 4× **ESCs** (thrust motors)

For full bench wiring details, see `[monoboard_wiring_guide.md](monoboard_wiring_guide.md)`. The key pin mappings that the firmware uses are summarized here.

### 1. IMU / I2C (UNO R4 Qwiic → TCA9548A → 4× BNO086)

- UNO R4 WiFi **Qwiic port** (3.3 V, I2C `Wire1`) connects to the **TCA9548A**:
  - Qwiic 3.3V → TCA9548A VCC
  - Qwiic GND → TCA9548A GND
  - Qwiic SDA → TCA9548A SDA
  - Qwiic SCL → TCA9548A SCL
- TCA9548A channel mapping to IMUs:
  - Arm 1 IMU → channel 0 (SDA0/SCL0)
  - Arm 2 IMU → channel 1 (SDA1/SCL1)
  - Arm 3 IMU → channel 2 (SDA2/SCL2)
  - Arm 4 IMU → channel 3 (SDA3/SCL3)
- In firmware, IMUs are accessed over **`Wire1`** with a TCA channel‑select helper before each read.

### 2. Rotation Motors (2× TB6612FNG)

Both TB6612 boards share:

- VCC (logic) → UNO **5V**
- GND → UNO **GND**
- VM (motor supply) → external motor voltage (6–12 V as needed)
- STBY → UNO **D13** (both boards tied together)

Per‑arm control pins (matching `monoboard_wiring_guide.md`, and used by the Uno firmware):

| Arm | TB6612 Channel | PWM pin | DIR1 | DIR2 |
| --- | -------------- | ------: | ---: | ---: |
| Arm 1 rotation | TB6612 #1 / A | **D3** | **D10** | **D11** |
| Arm 2 rotation | TB6612 #1 / B | **D5** | **D12** | **D7** |
| Arm 3 rotation | TB6612 #2 / A | **D6** | **A0** | **A1** |
| Arm 4 rotation | TB6612 #2 / B | **D9** | **A2** | **A3** |

The Uno R4 single‑board firmware configures these as:

- `STBY` output (D13) driven HIGH
- One PWM + two DIR pins per arm for the TB6612 driver.

### 3. ESC Thrust Outputs

Each ESC receives:

- Signal (PWM/servo pulse) from an Arduino digital pin
- Signal ground tied to **UNO GND**

UNO R4 WiFi pin mapping (used directly in the Uno firmware):

| Arm | ESC Signal Pin |
| --- | -------------: |
| Arm 1 thruster | **D2** |
| Arm 2 thruster | **D4** |
| Arm 3 thruster | **D8** |
| Arm 4 thruster | **A4** (used as digital) |

### 4. Power and Grounding (Monoboard)

The monoboard wiring keeps the same **single common ground** requirement:

- UNO GND
- TCA9548A GND
- All IMU GNDs
- Both TB6612 GNDs
- External DC motor supply GND
- ESC signal ground (and ESC power GND)

Refer to `[monoboard_wiring_guide.md](monoboard_wiring_guide.md)` for the recommended bring‑up order (IMUs → rotation motors → ESCs) and additional safety notes.

