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

### Summary Checklist
- [ ] **GND Shared:** Master, Slave, and all components connected to Blue Rail.
- [ ] **Serial Crossed:** Master TX->Slave RX, Master RX->Slave TX.
- [ ] **I2C Separated:** Master has 2 IMUs, Slave has 2 IMUs.
- [ ] **Motors Split:** Master controls ESC 1&2, Slave controls ESC 3&4.

