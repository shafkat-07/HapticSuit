# Firmware

To flash the firmware you need to use the [Arduino IDE](https://www.arduino.cc/en/software). This also includes our PID class shown below:

```c
#include <math.h>
#include <Arduino.h>

// A class to compute the control signal
class PID{
  private:
    float Kp ;
    float Ki ;
    float Kd ;
    float dt ;
    float integral;
    float previous_error;

  public:

  // Constructor
  PID()
  {
    Kp = 0;
    Ki = 0;
    Kd = 0;
    dt = 0;

    // Variables used by the controller
    integral = 0;
    previous_error = 0;
  };

  // Constructor
  PID(float Kp_in, float Ki_in, float Kd_in, float rate_in)
  {
    Kp = Kp_in;
    Ki = Ki_in;
    Kd = Kd_in;
    dt = 1.0  / rate_in;

    // Variables used by the controller
    integral = 0;
    previous_error = 0;
  }

  // A function to set the parameters
  void set_parameters(float Kp_in, float Ki_in, float Kd_in, float rate_in)
  {
    Kp = Kp_in;
    Ki = Ki_in;
    Kd = Kd_in;
    dt = 1.0  / rate_in;
  }

  // A function to compute the control signal
  float get_output(float setpoint, float current_output)
  {

    // Create the output
    float output;

    // Run the controller
    float error = setpoint - current_output;
    integral = integral + error * dt;
    float derivative = (error - previous_error)/dt;
    output = Kp*error + Ki*integral + Kd*derivative;
    previous_error = error;

    // Return the output
    return output;
  }
  
};
```

This was used to control both the angle of each of the arms. Please refer to the Arduino documentation for further information on how to flash each of the different boards.

---

## Board Layouts

There are now two firmware layouts in this repository:

- **Legacy Master/Slave (dual Feather M0)**  
  - `firmware/HapticSuitMaster/HapticSuitMaster.ino`  
  - `firmware/HapticSuitSlave/HapticSuitSlave.ino`  
  - Master receives UDP commands from ROS and forwards a subset of commands to the Slave via UART.

- **Monoboard Uno R4 WiFi (single board, 4 arms)**  
  - `firmware/HapticSuitUnoR4Single/HapticSuitUnoR4Single.ino`  
  - Runs the full 4‑arm haptic suit on a single **Arduino UNO R4 WiFi**.  
  - Uses the UNO R4 Qwiic port (`Wire1`) with a **TCA9548A** I2C multiplexer to talk to **4× BNO086** IMUs (one per arm).  
  - Drives **2× TB6612FNG** boards for arm rotation and **4× ESCs** for thrust, with pins matching `monoboard_wiring_guide.md` and `FULL_SYSTEM_WIRING.md`.

Both layouts share the same high‑level UDP interface from ROS:  
`angle1, angle2, throttle1, throttle2` on UDP port `8888`.  
The Uno R4 single‑board firmware internally maps these two channels to four physical arms (1 & 3 share channel 1, 2 & 4 share channel 2).

### Flashing the Uno R4 WiFi Monoboard Firmware

1. Install the **Arduino UNO R4** board support in the Arduino IDE (via Boards Manager).
2. Open `firmware/HapticSuitUnoR4Single/HapticSuitUnoR4Single.ino` in the IDE.
3. Install required libraries if needed:
   - `SparkFun BNO080` Arduino library
   - `WiFiS3` (for UNO R4 WiFi networking)
4. Select the correct UNO R4 WiFi board and serial port.
5. Flash the sketch.

For wiring details (IMUs, TB6612s, ESCs, power domains, and bring‑up order), see:

- `monoboard_wiring_guide.md`
- `FULL_SYSTEM_WIRING.md` (section “Uno R4 WiFi Single‑Board Variant (Monoboard)”)
