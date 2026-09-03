/*
 * GY-521 + TB6612 Closed-Loop Angle Holding Test (Arm 0 / TCA Channel 0)
 *
 * Designed to safely test IMU feedback + DC motor PID closed-loop control.
 *
 * Pin assignments per monoboard_wiring_guide.md:
 *   STBY (both TB6612): D13
 *   Arm 1 (Index 0): PWM D3, DIR1 D10, DIR2 D11 (Mux CH0)
 *   Arm 2 (Index 1): PWM D5, DIR1 D12, DIR2 D7  (Mux CH1)
 *   Arm 3 (Index 2): PWM D6, DIR1 A0,  DIR2 A1  (Mux CH2)
 *   Arm 4 (Index 3): PWM D9, DIR1 A2,  DIR2 A3  (Mux CH3)
 *
 * All unselected arms (1, 2, 3) are held safe (PWM=0, DIR=LOW).
 */

#include <Wire.h>
#include <MadgwickAHRS.h>

// --- Configuration ---
static const uint8_t ACTIVE_ARM = 0; // Testing Arm 1 (Index 0) on TCA Channel 0
static const float SAMPLE_HZ = 100.0f;
static const uint32_t SAMPLE_US = static_cast<uint32_t>(1000000.0f / SAMPLE_HZ);

// Safety limits for bench test
static const uint8_t MAX_TEST_PWM = 85;         // Cap power (~33%) so arm moves gently
static const uint8_t MIN_PWM = 25;              // Minimum PWM to overcome gearbox stiction
static const float RUNAWAY_ANGLE_LIMIT = 45.0f; // Cut off motor if angle error exceeds 45 deg
static const float DEADBAND_DEG = 1.0f;         // Within 1 deg, stop motor to prevent hunting

// --- Pin Definitions (monoboard_wiring_guide.md) ---
static const uint8_t PIN_STBY = 13;
static const int PWM_PINS[4]  = {3,  5,  6,  9};
static const int DIR1_PINS[4] = {10, 12, A0, A2};
static const int DIR2_PINS[4] = {11, 7,  A1, A3};

// --- I2C / MPU Registers ---
static const uint8_t TCA9548A_ADDRESS = 0x70;
static const uint8_t MPU6050_ADDR_PRIMARY = 0x68;
static const uint8_t MPU6050_ADDR_AD0_HIGH = 0x69;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_WHO_AM_I = 0x75;
static const uint8_t WHO_AM_I_EXPECTED = 0x68;

static const float ACCEL_LSB_PER_G = 16384.0f;
static const float GYRO_LSB_PER_DPS = 131.0f;

static uint8_t g_mpuAddress = MPU6050_ADDR_PRIMARY;
static float g_biasGx = 0.0f, g_biasGy = 0.0f, g_biasGz = 0.0f;
static Madgwick fusion;

// --- Control Modes & Variables ---
enum ControlMode { MODE_MONITOR, MODE_PID_HOLD };
static ControlMode g_mode = MODE_MONITOR;

static char g_activeAxis = 'r';     // 'r' = Roll, 'p' = Pitch, 'y' = Yaw
static int8_t g_motorPolarity = 1;  // +1 or -1
static float g_targetAngle = 0.0f;

// Conservative PID parameters for initial bench test
static float g_Kp = 5.0f;
static float g_Ki = 0.0f;
static float g_Kd = 0.25f;
static float g_integral = 0.0f;
static float g_prevAngle = 0.0f;

// --- Low-Level I2C Helpers ---
static void tcaSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire1.beginTransmission(TCA9548A_ADDRESS);
  Wire1.write(static_cast<uint8_t>(1u << ch));
  Wire1.endTransmission();
}

static bool i2cWrite8(uint8_t devAddr, uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(devAddr);
  Wire1.write(reg);
  Wire1.write(val);
  return Wire1.endTransmission() == 0;
}

static bool i2cReadBytes(uint8_t devAddr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire1.beginTransmission(devAddr);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom(devAddr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire1.read();
  return true;
}

static bool mpuInit(uint8_t addr) {
  if (!i2cWrite8(addr, REG_PWR_MGMT_1, 0x00)) return false;
  delay(10);
  if (!i2cWrite8(addr, REG_ACCEL_CONFIG, 0x00)) return false;
  if (!i2cWrite8(addr, REG_GYRO_CONFIG, 0x00)) return false;
  return true;
}

static bool readMpuScaled(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
  uint8_t buf[14];
  if (!i2cReadBytes(g_mpuAddress, REG_ACCEL_XOUT_H, buf, 14)) return false;
  const int16_t rax = static_cast<int16_t>((buf[0] << 8) | buf[1]);
  const int16_t ray = static_cast<int16_t>((buf[2] << 8) | buf[3]);
  const int16_t raz = static_cast<int16_t>((buf[4] << 8) | buf[5]);
  const int16_t rgx = static_cast<int16_t>((buf[8] << 8) | buf[9]);
  const int16_t rgy = static_cast<int16_t>((buf[10] << 8) | buf[11]);
  const int16_t rgz = static_cast<int16_t>((buf[12] << 8) | buf[13]);

  *ax = rax / ACCEL_LSB_PER_G;
  *ay = ray / ACCEL_LSB_PER_G;
  *az = raz / ACCEL_LSB_PER_G;
  *gx = rgx / GYRO_LSB_PER_DPS;
  *gy = rgy / GYRO_LSB_PER_DPS;
  *gz = rgz / GYRO_LSB_PER_DPS;
  return true;
}

static void calibrateGyro() {
  delay(500);
  double sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 200; i++) {
    float ax, ay, az, gx, gy, gz;
    if (readMpuScaled(&ax, &ay, &az, &gx, &gy, &gz)) {
      sx += gx; sy += gy; sz += gz;
    }
    delay(5);
  }
  g_biasGx = static_cast<float>(sx / 200.0);
  g_biasGy = static_cast<float>(sy / 200.0);
  g_biasGz = static_cast<float>(sz / 200.0);
}

// --- Motor Control Functions ---
static void setMotor(int8_t dir, uint8_t pwm) {
  pwm = min(pwm, MAX_TEST_PWM);
  analogWrite(PWM_PINS[ACTIVE_ARM], pwm);
  if (dir > 0) {
    digitalWrite(DIR1_PINS[ACTIVE_ARM], HIGH);
    digitalWrite(DIR2_PINS[ACTIVE_ARM], LOW);
  } else if (dir < 0) {
    digitalWrite(DIR1_PINS[ACTIVE_ARM], LOW);
    digitalWrite(DIR2_PINS[ACTIVE_ARM], HIGH);
  } else {
    digitalWrite(DIR1_PINS[ACTIVE_ARM], LOW);
    digitalWrite(DIR2_PINS[ACTIVE_ARM], LOW);
  }
}

static void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    analogWrite(PWM_PINS[i], 0);
    digitalWrite(DIR1_PINS[i], LOW);
    digitalWrite(DIR2_PINS[i], LOW);
  }
}

// --- Serial Interface ---
static void printHelp() {
  Serial.println(F("\n======================================================="));
  Serial.println(F("         HAPTIC SUIT BENCH TEST: ARM 0 (IMU0)         "));
  Serial.println(F("======================================================="));
  Serial.println(F(" COMMANDS:"));
  Serial.println(F("   m         : Monitor Mode (Motor OFF, move arm by hand)"));
  Serial.println(F("   a <r|p|y> : Set active tracking axis ('a r', 'a p', 'a y')"));
  Serial.println(F("   f         : Jog Forward (dir=+1) 150ms to check polarity"));
  Serial.println(F("   b         : Jog Backward (dir=-1) 150ms to check polarity"));
  Serial.println(F("   p         : Toggle motor polarity (+1 <-> -1)"));
  Serial.println(F("   t <angle> : ENGAGE PID HOLD to target angle (e.g. 't 0', 't 15')"));
  Serial.println(F("   x or s    : EMERGENCY STOP / Disengage PID immediately"));
  Serial.println(F("   h or ?    : Print this menu again"));
  Serial.println(F("=======================================================\n"));
}

static void processSerial() {
  while (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '\r' || cmd == '\n' || cmd == ' ') {
      continue;
    }

    if (cmd == 'x' || cmd == 's') {
      g_mode = MODE_MONITOR;
      stopAllMotors();
      Serial.println(F("\n[!] EMERGENCY STOP: Motor power cut, PID disengaged."));
    } else if (cmd == 'm') {
      g_mode = MODE_MONITOR;
      stopAllMotors();
      Serial.println(F("\n[*] Switched to MONITOR MODE (Motor OFF). Move arm by hand."));
    } else if (cmd == 'f') {
      Serial.println(F("[*] Jogging FORWARD (dir=+1) for 150ms..."));
      setMotor(+1 * g_motorPolarity, 65);
      delay(150);
      setMotor(0, 0);
    } else if (cmd == 'b') {
      Serial.println(F("[*] Jogging BACKWARD (dir=-1) for 150ms..."));
      setMotor(-1 * g_motorPolarity, 65);
      delay(150);
      setMotor(0, 0);
    } else if (cmd == 'p') {
      g_motorPolarity *= -1;
      Serial.print(F("[*] Motor polarity multiplier toggled to: "));
      Serial.println(g_motorPolarity > 0 ? "+1 (Normal)" : "-1 (Inverted)");
      Serial.println(F("    RULE: 'f' should INCREASE your tracked angle. If it DECREASED it, toggle 'p'."));
    } else if (cmd == 'a') {
      char axis = 0;
      while (Serial.available()) {
        char c = Serial.read();
        if (c == 'r' || c == 'p' || c == 'y') {
          axis = c;
          break;
        }
      }
      if (axis) {
        g_activeAxis = axis;
        Serial.print(F("[*] Active tracking axis set to: "));
        Serial.println(g_activeAxis == 'r' ? "ROLL" : (g_activeAxis == 'p' ? "PITCH" : "YAW"));
      } else {
        Serial.println(F("[!] Specify axis: 'a r' for Roll, 'a p' for Pitch, 'a y' for Yaw"));
      }
    } else if (cmd == 't') {
      float angle = Serial.parseFloat();
      g_targetAngle = constrain(angle, -35.0f, 35.0f); // Safe bench bounds
      g_integral = 0.0f;
      float cur = (g_activeAxis == 'r') ? fusion.getRoll() : ((g_activeAxis == 'p') ? fusion.getPitch() : fusion.getYaw());
      g_prevAngle = cur;
      g_mode = MODE_PID_HOLD;
      Serial.print(F("\n[*] >>> ENGAGING PID HOLD <<< Target: "));
      Serial.print(g_targetAngle, 1);
      Serial.println(F(" deg. Press 'x' or 's' at any time to stop!"));
    } else if (cmd == 'h' || cmd == '?') {
      printHelp();
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // 1. Initialize all 4 arm motor pins to safe unpowered state
  for (int i = 0; i < 4; i++) {
    pinMode(PWM_PINS[i], OUTPUT);
    pinMode(DIR1_PINS[i], OUTPUT);
    pinMode(DIR2_PINS[i], OUTPUT);
    analogWrite(PWM_PINS[i], 0);
    digitalWrite(DIR1_PINS[i], LOW);
    digitalWrite(DIR2_PINS[i], LOW);
  }
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH); // Enable TB6612 H-bridge logic

  // 2. Initialize I2C on Wire1 (Qwiic on UNO R4)
  Wire1.begin();
  Wire1.setClock(400000);
  delay(50);

  // 3. Connect to IMU 0 on TCA Channel 0
  Serial.print(F("Connecting to IMU on TCA Channel "));
  Serial.println(ACTIVE_ARM);
  tcaSelect(ACTIVE_ARM);
  delay(15);

  uint8_t who = 0;
  if (i2cReadBytes(MPU6050_ADDR_PRIMARY, REG_WHO_AM_I, &who, 1) && who == WHO_AM_I_EXPECTED) {
    g_mpuAddress = MPU6050_ADDR_PRIMARY;
  } else if (i2cReadBytes(MPU6050_ADDR_AD0_HIGH, REG_WHO_AM_I, &who, 1) && who == WHO_AM_I_EXPECTED) {
    g_mpuAddress = MPU6050_ADDR_AD0_HIGH;
  } else {
    Serial.println(F("[!] ERROR: MPU-6050 not responding on TCA Channel 0. Check Qwiic cable & wiring!"));
    while (1) {
      delay(1000);
    }
  }

  if (!mpuInit(g_mpuAddress)) {
    Serial.println(F("[!] ERROR: MPU-6050 register init failed."));
    while (1) {
      delay(1000);
    }
  }

  Serial.println(F("Calibrating gyro (keep arm completely still)..."));
  calibrateGyro();
  fusion.begin(SAMPLE_HZ);

  Serial.println(F("Arm 0 IMU calibrated successfully. Other arms held safe."));
  printHelp();
}

void loop() {
  const uint32_t t0 = micros();

  processSerial();

  // 1. Sample IMU 0 via TCA Channel 0
  tcaSelect(ACTIVE_ARM);
  float ax, ay, az, gx, gy, gz;
  if (!readMpuScaled(&ax, &ay, &az, &gx, &gy, &gz)) {
    stopAllMotors();
    return;
  }

  gx -= g_biasGx;
  gy -= g_biasGy;
  gz -= g_biasGz;

  fusion.updateIMU(gx, gy, gz, ax, ay, az);

  const float roll  = fusion.getRoll();
  const float pitch = fusion.getPitch();
  const float yaw   = fusion.getYaw();

  const float currentAngle = (g_activeAxis == 'r') ? roll : ((g_activeAxis == 'p') ? pitch : yaw);

  // 2. Control and telemetry
  static uint32_t lastPrint = 0;

  if (g_mode == MODE_MONITOR) {
    stopAllMotors();
    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      Serial.print(F("R: ")); Serial.print(roll, 1);
      Serial.print(F(" | P: ")); Serial.print(pitch, 1);
      Serial.print(F(" | Y: ")); Serial.print(yaw, 1);
      Serial.print(F("  --> [TRACKING: "));
      Serial.print(g_activeAxis == 'r' ? "Roll" : (g_activeAxis == 'p' ? "Pitch" : "Yaw"));
      Serial.print(F(" = ")); Serial.print(currentAngle, 1);
      Serial.println(F(" deg] (Motor OFF)"));
    }
  } else if (g_mode == MODE_PID_HOLD) {
    const float error = g_targetAngle - currentAngle;

    // Safety runaway tripwire: if arm is pushed or driven beyond 45 deg error, cut power
    if (fabs(error) > RUNAWAY_ANGLE_LIMIT) {
      g_mode = MODE_MONITOR;
      stopAllMotors();
      Serial.print(F("\n[!] RUNAWAY PROTECTION TRIPPED: Error = "));
      Serial.print(error, 1);
      Serial.println(F(" deg. Motor turned OFF!"));
      return;
    }

    if (fabs(error) < DEADBAND_DEG) {
      setMotor(0, 0); // Within deadband target, coast motor
    } else {
      g_integral = constrain(g_integral + error * (1.0f / SAMPLE_HZ), -20.0f, 20.0f);

      // Derivative on measurement (negative rate of change) avoids setpoint kick
      const float dAngle = (currentAngle - g_prevAngle) * SAMPLE_HZ;
      g_prevAngle = currentAngle;
      const float derivative = -dAngle;

      float output = (g_Kp * error) + (g_Ki * g_integral) + (g_Kd * derivative);
      output *= g_motorPolarity;

      const int8_t dir = (output > 0) ? 1 : ((output < 0) ? -1 : 0);
      const uint8_t pwm = constrain(static_cast<int>(fabs(output)), MIN_PWM, MAX_TEST_PWM);
      setMotor(dir, pwm);
    }

    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      Serial.print(F("HOLD >> Target: ")); Serial.print(g_targetAngle, 1);
      Serial.print(F(" | Angle: ")); Serial.print(currentAngle, 1);
      Serial.print(F(" | Error: ")); Serial.print(error, 1);
      Serial.println(F(" deg"));
    }
  }

  // 100 Hz timing
  while (static_cast<uint32_t>(micros() - t0) < SAMPLE_US) {
    /* wait */
  }
}
