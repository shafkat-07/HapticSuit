#include <Wire.h>
#include <Servo.h>
#include <WiFiS3.h>    // UNO R4 WiFi network driver
#include <WiFiUdp.h>
#include <MadgwickAHRS.h>


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
Madgwick fusion[NUM_ARMS];

static const uint8_t MPU6050_ADDR_PRIMARY = 0x68;
static const uint8_t MPU6050_ADDR_AD0_HIGH = 0x69;

static const uint8_t REG_WHO_AM_I = 0x75;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

static const uint8_t WHO_AM_I_EXPECTED = 0x68;

static const float ACCEL_LSB_PER_G = 16384.0f;
static const float GYRO_LSB_PER_DPS = 131.0f;

static uint8_t g_mpuAddress[NUM_ARMS];

static float g_biasGx[NUM_ARMS];
static float g_biasGy[NUM_ARMS];
static float g_biasGz[NUM_ARMS];

bool imu_success[NUM_ARMS] = {false, false, false, false};

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

static bool i2cWrite8(uint8_t devAddr, uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(devAddr);
  Wire1.write(reg);
  Wire1.write(value);
  return Wire1.endTransmission() == 0;
}

static bool i2cReadBytes(uint8_t devAddr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire1.beginTransmission(devAddr);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) {
    return false;
  }
  const size_t got = Wire1.requestFrom(devAddr, len);
  if (got != len) {
    return false;
  }
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire1.read();
  }
  return true;
}

static bool mpuProbe(uint8_t addr, uint8_t *whoAmIOut) {
  uint8_t w = 0;
  if (!i2cReadBytes(addr, REG_WHO_AM_I, &w, 1)) {
    return false;
  }
  *whoAmIOut = w;
  return (w == WHO_AM_I_EXPECTED);
}

static bool mpuInit(uint8_t addr) {
  if (!i2cWrite8(addr, REG_PWR_MGMT_1, 0x00)) {
    return false;
  }
  delay(10);
  if (!i2cWrite8(addr, REG_ACCEL_CONFIG, 0x00)) {
    return false;
  }
  if (!i2cWrite8(addr, REG_GYRO_CONFIG, 0x00)) {
    return false;
  }
  return true;
}

static void remapImuFrame(float axIn, float ayIn, float azIn, float gxIn, float gyIn, float gzIn,
 float *axOut, float *ayOut, float *azOut, float *gxOut, float *gyOut, float *gzOut) {
  *axOut = axIn;
  *ayOut = ayIn;
  *azOut = azIn;
  *gxOut = gxIn;
  *gyOut = gyIn;
  *gzOut = gzIn;
}

static bool readMpuScaled(uint8_t devAddr, float *axg, float *ayg, float *azg, float *gxd, float *gyd, float *gzd) {
  uint8_t buf[14];
  if (!i2cReadBytes(devAddr, REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
    return false;
  }
  const int16_t rax = static_cast<int16_t>((buf[0] << 8) | buf[1]);
  const int16_t ray = static_cast<int16_t>((buf[2] << 8) | buf[3]);
  const int16_t raz = static_cast<int16_t>((buf[4] << 8) | buf[5]);
  const int16_t rgx = static_cast<int16_t>((buf[8] << 8) | buf[9]);
  const int16_t rgy = static_cast<int16_t>((buf[10] << 8) | buf[11]);
  const int16_t rgz = static_cast<int16_t>((buf[12] << 8) | buf[13]);

  *axg = rax / ACCEL_LSB_PER_G;
  *ayg = ray / ACCEL_LSB_PER_G;
  *azg = raz / ACCEL_LSB_PER_G;
  *gxd = rgx / GYRO_LSB_PER_DPS;
  *gyd = rgy / GYRO_LSB_PER_DPS;
  *gzd = rgz / GYRO_LSB_PER_DPS;
  return true;
}

static void calibrateGyroBias(uint8_t mpuAddr, float *biasGx, float *biasGy, float *biasGz,
 uint16_t settleMs, uint16_t sampleCount) {
  delay(settleMs);
  double sx = 0.0, sy = 0.0, sz = 0.0;
  uint16_t ok = 0;
  for (uint16_t i = 0; i < sampleCount; i++) {
    float ax, ay, az, gx, gy, gz;
    if (readMpuScaled(mpuAddr, &ax, &ay, &az, &gx, &gy, &gz)) {
      sx += gx;
      sy += gy;
      sz += gz;
      ok++;
    }
    delay(5);
  }
  if (ok == 0) {
    *biasGx = *biasGy = *biasGz = 0.0f;
    return;
  }
  *biasGx = static_cast<float>(sx / ok);
  *biasGy = static_cast<float>(sy / ok);
  *biasGz = static_cast<float>(sz / ok);
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

  for (int k = 0; k < NUM_ARMS; k++)
  {
    Serial.print("Attempting to connect to IMU for Arm ");
    Serial.print(k + 1);
    Serial.print(" on TCA channel ");
    Serial.println(k);

    tcaSelect(k);
    delay(10);

    uint8_t who = 0;
    if (mpuProbe(MPU6050_ADDR_PRIMARY, &who)) {
      g_mpuAddress[k] = MPU6050_ADDR_PRIMARY;
      imu_success[k] = true;
    } else if (mpuProbe(MPU6050_ADDR_AD0_HIGH, &who)) {
      g_mpuAddress[k] = MPU6050_ADDR_AD0_HIGH;
      imu_success[k] = true;
    } else {
      Serial.print("  FAILED to connect to IMU on channel ");
      Serial.println(k);
      imu_success[k] = false;
      continue;
    }

    if (!mpuInit(g_mpuAddress[k])) {
      Serial.print("  FAILED to init IMU on channel ");
      Serial.println(k);
      imu_success[k] = false;
      continue;
    }

    Serial.print("  SUCCESS: IMU connected on channel ");
    Serial.println(k);

    Serial.print("  Calibrating gyro bias... ");
    calibrateGyroBias(g_mpuAddress[k], &g_biasGx[k], &g_biasGy[k], &g_biasGz[k], 500, 200);
    Serial.println("Done.");

    fusion[k].begin((float)LOOP_RATE);
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

  // Get the latest IMU data (GY-521 + Madgwick API)
  for (int k = 0; k < NUM_ARMS; k++)
  {
    if (!imu_success[k]) continue;

    tcaSelect(k);

    float axg, ayg, azg, gxd, gyd, gzd;
    if (readMpuScaled(g_mpuAddress[k], &axg, &ayg, &azg, &gxd, &gyd, &gzd)) {
      gxd -= g_biasGx[k];
      gyd -= g_biasGy[k];
      gzd -= g_biasGz[k];

      float axf, ayf, azf, gxf, gyf, gzf;
      remapImuFrame(axg, ayg, azg, gxd, gyd, gzd, &axf, &ayf, &azf, &gxf, &gyf, &gzf);

      fusion[k].updateIMU(gxf, gyf, gzf, axf, ayf, azf);

      current_roll[k] = fusion[k].getRoll(); // already in degrees
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

