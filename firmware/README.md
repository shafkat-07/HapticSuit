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