#include <sh2.h>
#include <sh2_SensorValue.h>
#include <sh2_err.h>
#include <sh2_hal.h>
#include <sh2_util.h>
#include <shtp.h>


#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <WiFiS3.h>    // UNO R4 WiFi network driver
#include <WiFiUdp.h>
#include <SparkFun_BNO08x_Arduino_Library.h>


// Local PID helper
#include "PID.cpp"

// --- Configuration ---

// Number of arms (full 4‑arm suit)
#define NUM_ARMS 4

// WiFi configuration (used when ENABLE_WIFI is true)
#define WIFI_SSID     "NU-IoT"
#define WIFI_PASSWORD "nkzvswmm"
#define ENABLE_WIFI   false

// UDP port used by ROS suit_controller_* scripts and wifi_test_sender.py
#define UDP_PORT 8888

// Control loop configuration
#define LOOP_RATE 100           // Hz
#define ROLL_MAX 270
#define ROLL_MIN -270
#define ROTATION_MAX_SPEED 255
#define ROTATION_MIN_SPEED 20
#define MOTOR_SPEED_MAX 100
#define MOTOR_SPEED_MIN 0

// TCA9548A I2C multiplexer address
#define TCA_ADDR 0x70

// --- Global state ---

int wifi_status = WL_IDLE_STATUS;

WiFiUDP Udp;
unsigned int localPort = UDP_PORT;
char packetBuffer[128];

// ESC (thrust) output pins, matching monoboard_wiring_guide.md
const int escPins[NUM_ARMS]  = {2, 4, 8, A4};

// TB6612 rotation motor pins, matching monoboard_wiring_guide.md
const int pwmPins[NUM_ARMS]  = {3, 5, 6, 9};
const int dir1Pins[NUM_ARMS] = {10, 12, A0, A2};
const int dir2Pins[NUM_ARMS] = {11, 7,  A1, A3};
const int stbyPin            = 13; // STBY tied to both TB6612 boards

// IMUs (via TCA9548A on Wire1), one per arm
BNO08x imu_array[NUM_ARMS];

// IMU / control state
float current_roll[NUM_ARMS];
float previous_roll[NUM_ARMS];
float global_roll[NUM_ARMS];
int   roll_state[NUM_ARMS];

// Commanded targets from ROS / UDP
int roll_setpoint[NUM_ARMS]   = {0, 0, 0, 0};
int motor_throttle[NUM_ARMS]  = {0, 0, 0, 0};

// Control loop timing
const float dt = 1.0f / LOOP_RATE;

// PID controllers (one per arm)
PID pid[NUM_ARMS];

// ESC servo objects
Servo esc_array[NUM_ARMS];

// --- Helpers ---

void printWiFiStatus()
{
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// Select a TCA9548A channel on Wire1 (0–7)
void tcaSelect(uint8_t channel)
{
  if (channel > 7) return;
  Wire1.beginTransmission(TCA_ADDR);
  Wire1.write(1 << channel);
  Wire1.endTransmission();
}

// Control the spinning rotation motors via TB6612
void set_motor(int rotation_direction, int pwm, int pwm_pin, int dir_pin1, int dir_pin2)
{
  analogWrite(pwm_pin, pwm);
  if (rotation_direction == 1)
  {
    digitalWrite(dir_pin1, HIGH);
    digitalWrite(dir_pin2, LOW);
  }
  else if (rotation_direction == -1)
  {
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, HIGH);
  }
  else
  {
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, LOW);
  }
}

// Computes the global roll value (i.e removes the 180 discontinuity)
double update_global_roll(double cur_reading, double prev_reading, int* state)
{
  int current_state = *state;
  double diff = cur_reading - prev_reading;

  // If the sign has changed
  if (abs(diff) > 180)
  {
    if (diff > 0)
    {
      // Update the state
      current_state = current_state - 1;
    }
    else
    {
      // Update the state
      current_state = current_state + 1;
    }
  }

  // Save the state
  *state = current_state;

  float global_roll_val = (current_state * 360) + cur_reading;

  return global_roll_val;
}

// Verify safe rotation bounds
bool rotation_within_bounds(double current_roll, double current_direction)
{
  bool safe = true;
  if ((current_roll < ROLL_MIN) && (current_direction <= -1))
  {
    safe = false;
  }
  if ((current_roll > ROLL_MAX) && (current_direction >= 1))
  {
    safe = false;
  }
  return safe;
}

// --- Setup & Loop ---

void setup()
{
  // Allow time for startup
  delay(1000);

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("Uno R4 Monoboard Haptic Suit Setup Starting");

  // Init control state
  Serial.println("Init variables");
  for (int k = 0; k < NUM_ARMS; k++)
  {
    current_roll[k]  = 0.0f;
    previous_roll[k] = 0.0f;
    global_roll[k]   = 0.0f;
    roll_state[k]    = 0;
    roll_setpoint[k] = 0;
    motor_throttle[k] = 0;
  }

  // Configure TB6612 and ESC pins
  Serial.println("Attaching motors / configuring pins");
  analogWriteResolution(8);

  pinMode(stbyPin, OUTPUT);
  digitalWrite(stbyPin, HIGH); // enable both TB6612 boards

  for (int k = 0; k < NUM_ARMS; k++)
  {
    // ESCs
    esc_array[k].attach(escPins[k], 1000, 2000);
    esc_array[k].write(0);

    // Rotation motor pins
    pinMode(pwmPins[k], OUTPUT);
    pinMode(dir1Pins[k], OUTPUT);
    pinMode(dir2Pins[k], OUTPUT);

    Serial.print("  Arm "); Serial.print(k + 1); Serial.println(" Configured:");
    Serial.print("    - ESC (Throttle) on Pin "); Serial.println(escPins[k]);
    Serial.print("    - Rotation Motor on Pins: PWM=");
    Serial.print(pwmPins[k]);
    Serial.print(", DIR1=");
    Serial.print(dir1Pins[k]);
    Serial.print(", DIR2=");
    Serial.println(dir2Pins[k]);
  }

  // PID configuration
  Serial.println("Creating PID controllers");
  for (int k = 0; k < NUM_ARMS; k++)
  {
    pid[k].set_parameters(10.0f, 0.0f, 0.0f, LOOP_RATE);
  }

  // I2C / IMU setup (Wire1 + TCA9548A)
  Serial.println("Initializing I2C on Wire1 and connecting to IMUs via TCA9548A");
  Wire1.begin();
  Wire1.setClock(400000); // 400 kHz
  delay(10);

  bool imu_success[NUM_ARMS] = {false, false, false, false};

  for (int k = 0; k < NUM_ARMS; k++)
  {
    Serial.print("Attempting to connect to IMU for Arm ");
    Serial.print(k + 1);
    Serial.print(" on TCA channel ");
    Serial.println(k);

    tcaSelect(k);
    imu_success[k] = imu_array[k].begin(BNO08x_DEFAULT_ADDRESS, Wire1);

    if (imu_success[k] == false)
    {
      Serial.print("  FAILED to connect to IMU on channel ");
      Serial.println(k);
    }
    else
    {
      Serial.print("  SUCCESS: IMU connected on channel ");
      Serial.println(k);
    }
    delay(100);
  }

  Serial.println("Enabling rotation vectors on IMUs");
  for (int k = 0; k < NUM_ARMS; k++)
  {
    if (imu_success[k])
    {
      tcaSelect(k);
      imu_array[k].enableRotationVector(5); // 5 ms update
      delay(10);
    }
  }

  // WiFi / UDP setup (optional)
  if (ENABLE_WIFI)
  {
    Serial.println("Setting up WiFi (UNO R4 WiFiS3)");

    while (wifi_status != WL_CONNECTED)
    {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(WIFI_SSID);

      wifi_status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

      // wait 5 seconds for connection:
      delay(5000);
    }

    Serial.println("Connected to WiFi");
    printWiFiStatus();

    // Start UDP
    Udp.begin(localPort);
  }

  Serial.println("Setup finished\n----------------------\n");
}

void loop()
{
  long start_time = micros();

  // If WiFi is available - check for UDP packet from ROS / wifi_test_sender.py
  if (ENABLE_WIFI)
  {
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      int len = Udp.read(packetBuffer, sizeof(packetBuffer) - 1);
      if (len > 0) packetBuffer[len] = 0;

      // Expected format from ROS nodes:
      // "angle1, angle2, throttle1, throttle2"
      int a1, a2, t1, t2;
      if (sscanf(packetBuffer, "%d , %d , %d , %d", &a1, &a2, &t1, &t2) == 4)
      {
        a1 = constrain(a1, ROLL_MIN, ROLL_MAX);
        a2 = constrain(a2, ROLL_MIN, ROLL_MAX);
        t1 = constrain(t1, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
        t2 = constrain(t2, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

        // Map two channels into four physical arms:
        // Arms 1 & 3 share (a1, t1)
        // Arms 2 & 4 share (a2, t2)
        roll_setpoint[0] = a1;
        roll_setpoint[2] = a1;
        roll_setpoint[1] = a2;
        roll_setpoint[3] = a2;

        motor_throttle[0] = t1;
        motor_throttle[2] = t1;
        motor_throttle[1] = t2;
        motor_throttle[3] = t2;
      }

      // Send a simple acknowledgement
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write("Cmd Ack");
      Udp.endPacket();
    }
  }

  // Get the latest IMU data (BNO08x API)
  for (int k = 0; k < NUM_ARMS; k++)
  {
    tcaSelect(k);

    // getSensorEvent() updates sensorValue and returns true when new data arrives
    if (imu_array[k].getSensorEvent())
    {
      // We enabled the rotation vector, so getRoll() is valid here
      current_roll[k] = imu_array[k].getRoll(); // already in degrees
      global_roll[k]  = update_global_roll(current_roll[k], previous_roll[k], &roll_state[k]);
      previous_roll[k] = current_roll[k];
    }
  }

  // Rotation control loop for each arm
  for (int k = 0; k < NUM_ARMS; k++)
  {
    float motor_power = pid[k].get_output(roll_setpoint[k], global_roll[k]);

    int motor_direction = (motor_power > 0) - (motor_power < 0);
    motor_power = min(fabs(motor_power), (float)ROTATION_MAX_SPEED);

    // Thresholding small outputs
    if (motor_power < ROTATION_MIN_SPEED)
    {
      motor_power = 0;
      motor_direction = 0;
    }

    // Safety: bound rotation
    if (!rotation_within_bounds(global_roll[k], motor_direction))
    {
      motor_power = 0;
    }

    set_motor(motor_direction, (int)motor_power, pwmPins[k], dir1Pins[k], dir2Pins[k]);
  }

  // ESC throttle outputs
  for (int k = 0; k < NUM_ARMS; k++)
  {
    esc_array[k].write(motor_throttle[k]);
  }

  // Maintain fixed loop rate
  while ((micros() - start_time) <= (dt * 1.0e6f))
  {
    ;
  }
}

