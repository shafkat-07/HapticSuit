/*
 * GY-521 + TB6612 1-DOF Hinge Angle Closed-Loop Test (Arm 0 / TCA Channel 0)
 *
 * Estimates the arm's hinge angle directly as a scalar rather than decomposing a
 * full orientation into roll/pitch/yaw. The arm is a single revolute joint, so its
 * attitude is always R(h, theta) for one fixed axis h. Projecting gravity into the
 * plane normal to h gives the angle from a single atan2, which is continuous over
 * the whole +/-180 range and has no gimbal-lock singularity. A Z-Y-X Euler
 * decomposition would put an asin on the pitch axis, which saturates and folds
 * back at +/-90 deg.
 *
 * The only degenerate case is h parallel to gravity, where the in-plane gravity
 * component vanishes and the angle is unobservable. That is detected via aPerp and
 * degrades to gyro-only coasting.
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
#include <math.h>

// --- Configuration ---
static const uint8_t ACTIVE_ARM = 0; // Testing Arm 1 (Index 0) on TCA Channel 0
static const float SAMPLE_HZ = 100.0f;
static const uint32_t SAMPLE_US = static_cast<uint32_t>(1000000.0f / SAMPLE_HZ);

// Safety limits for bench test
static const uint8_t MAX_TEST_PWM = 85;         // Cap power (~33%) so arm moves gently
static const uint8_t MIN_PWM = 25;              // Minimum PWM to overcome gearbox stiction
static const float RUNAWAY_ANGLE_LIMIT = 45.0f; // Cut off motor if angle error exceeds 45 deg
static const float DEADBAND_DEG = 1.0f;         // Within 1 deg, stop motor to prevent hunting
static const float TARGET_LIMIT_DEG = 35.0f;    // Safe bench bounds for commanded target

// --- Pin Definitions (monoboard_wiring_guide.md) ---
static const uint8_t PIN_STBY = 13;
static const int PWM_PINS[4]  = {3,  5,  6,  9};
static const int DIR1_PINS[4] = {10, 12, A0, A2};
static const int DIR2_PINS[4] = {11, 7,  A1, A3};

// --- I2C / MPU Registers ---
static const uint8_t TCA9548A_ADDRESS = 0x70;
static const uint8_t MPU6050_ADDR_PRIMARY = 0x68;
static const uint8_t MPU6050_ADDR_AD0_HIGH = 0x69;
static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_CONFIG = 0x1A;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_WHO_AM_I = 0x75;
static const uint8_t WHO_AM_I_EXPECTED = 0x68;

// DLPF_CFG=3 gives 44 Hz accel / 42 Hz gyro bandwidth. Leaving DLPF at 0 (260 Hz)
// aliases propeller blade-pass vibration straight into the 100 Hz control band.
static const uint8_t CFG_DLPF = 0x03;
// With DLPF enabled the gyro output rate is 1 kHz, so divide by 10 for 100 Hz.
static const uint8_t CFG_SMPLRT_DIV = 0x09;
// +/-4 g and +/-500 dps. The stock +/-2 g clips on prop vibration, and clipping
// rectifies into a DC angle bias rather than showing up as noise.
static const uint8_t CFG_ACCEL_FS = 0x08;
static const uint8_t CFG_GYRO_FS = 0x08;
static const float ACCEL_LSB_PER_G = 8192.0f;
static const float GYRO_LSB_PER_DPS = 65.5f;

static uint8_t g_mpuAddress = MPU6050_ADDR_PRIMARY;
static float g_biasGx = 0.0f, g_biasGy = 0.0f, g_biasGz = 0.0f;

// --- Hinge Frame ---
// Right-handed triad (H, U, V). H is the hinge axis in IMU body coordinates, U
// points along the in-plane gravity direction at the mechanical zero, V = H x U.
//
// Measured on arm 0: gravity stays in the IMU x-z plane through a full sweep
// (ay stays near zero while ax and az cover the whole range), so the hinge is the
// IMU y-axis. With the IMU +Z facing up at the mechanical zero this reduces theta
// to atan2(-ax, az) and the hinge rate to gy. Re-measure per arm with 'c' then 'z'.
static float g_hingeH[3] = {0.0f, 1.0f, 0.0f};
static float g_hingeU[3] = {0.0f, 0.0f, 1.0f};
static float g_hingeV[3] = {1.0f, 0.0f, 0.0f};

// --- Angle Estimator ---
// Two-state Kalman filter over (theta, gyro bias about the hinge axis).
static float g_theta = 0.0f;        // deg, wrapped to (-180, 180]
static float g_thetaBias = 0.0f;    // deg/s
static float g_P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
static bool g_estInit = false;

static const float Q_THETA = 0.05f;   // deg^2/s process noise on angle
static const float Q_BIAS = 0.0005f;  // (deg/s)^2/s process noise on bias
static const float R_ACC = 4.0f;      // deg^2 accel noise at aPerp = 1 g
static const float APERP_MIN_G = 0.30f; // Below this, hinge axis is too near gravity
// Zero-rate output on an MPU6050 is a few dps. Anything beyond this is the filter
// absorbing a modelling error into the bias, which would ramp the angle forever.
static const float BIAS_LIMIT_DPS = 8.0f;
// A converged estimate tracks gravity closely, so a large innovation means the
// accelerometer is not measuring gravity or the hinge axis is wrong.
static const float INNOV_GATE_DEG = 45.0f;
static const uint16_t INNOV_REJECT_LIMIT = 100; // 1 s before forcing a re-init

// Multi-turn accumulator so the estimate can span the +/-270 deg mechanical range.
static float g_thetaGlobal = 0.0f;
static float g_thetaPrev = 0.0f;
static int g_turns = 0;

// Diagnostics retained for the monitor printout
static float g_lastAccAngle = 0.0f;
static float g_lastAPerp = 0.0f;
static float g_lastANorm = 0.0f;
static bool g_lastAccUsed = false;
static uint16_t g_innovRejects = 0;

// A rotation about H cannot change gravity's component along H, so the spread of
// a.H over a sweep is a direct measure of how wrong the assumed hinge axis is.
static float g_aHmin = 0.0f;
static float g_aHmax = 0.0f;
static bool g_aHvalid = false;
static const float AH_SPREAD_WARN_G = 0.15f;
static const float AH_DECAY = 0.0005f; // ~20 s relaxation at 100 Hz

// --- Control Modes & Variables ---
enum ControlMode { MODE_MONITOR, MODE_PID_HOLD, MODE_CALIB_AXIS, MODE_CALIB_ZERO };
static ControlMode g_mode = MODE_MONITOR;

static int8_t g_motorPolarity = 1;  // +1 or -1
static float g_targetAngle = 0.0f;

// Conservative PID parameters for initial bench test
static float g_Kp = 5.0f;
static float g_Ki = 0.0f;
static float g_Kd = 0.25f;
static float g_integral = 0.0f;
static float g_prevAngle = 0.0f;

// Jog state. Kept non-blocking so the estimator continues to run at 100 Hz;
// a delay() here would stall the filter for many sample periods.
static uint32_t g_jogUntilMs = 0;
static int8_t g_jogDir = 0;

static bool g_rawDump = false;

// Calibration accumulators
static const uint32_t CALIB_AXIS_MS = 15000;
static const uint32_t CALIB_ZERO_MS = 1500;
static const float CALIB_MIN_RATE_DPS = 5.0f;
static double g_axisM[3][3];
static uint16_t g_axisSamples = 0;
static double g_zeroSum[3];
static uint16_t g_zeroSamples = 0;
static uint32_t g_calibEndMs = 0;

// --- Vector Helpers ---
static inline float dot3(float x, float y, float z, const float v[3]) {
  return x * v[0] + y * v[1] + z * v[2];
}

static bool normalize3(float v[3]) {
  const float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (!(n > 1e-6f)) return false;
  v[0] /= n; v[1] /= n; v[2] /= n;
  return true;
}

static void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

static inline float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

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
  if (!i2cWrite8(addr, REG_CONFIG, CFG_DLPF)) return false;
  if (!i2cWrite8(addr, REG_SMPLRT_DIV, CFG_SMPLRT_DIV)) return false;
  if (!i2cWrite8(addr, REG_ACCEL_CONFIG, CFG_ACCEL_FS)) return false;
  if (!i2cWrite8(addr, REG_GYRO_CONFIG, CFG_GYRO_FS)) return false;
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

// Rejects the attempt if the arm moved during sampling, which would otherwise bake
// a rate into the bias and show up later as a steady angle ramp.
static bool calibrateGyro() {
  static const uint8_t ATTEMPTS = 5;
  static const uint16_t N = 200;
  static const float MAX_VARIANCE = 1.0f;

  for (uint8_t attempt = 0; attempt < ATTEMPTS; attempt++) {
    delay(500);
    double s[3] = {0, 0, 0};
    double ss[3] = {0, 0, 0};
    uint16_t ok = 0;

    for (uint16_t i = 0; i < N; i++) {
      float ax, ay, az, g[3];
      if (readMpuScaled(&ax, &ay, &az, &g[0], &g[1], &g[2])) {
        for (uint8_t k = 0; k < 3; k++) {
          s[k] += g[k];
          ss[k] += static_cast<double>(g[k]) * g[k];
        }
        ok++;
      }
      delay(5);
    }

    if (ok < N / 2) continue;

    float worst = 0.0f;
    float mean[3];
    for (uint8_t k = 0; k < 3; k++) {
      mean[k] = static_cast<float>(s[k] / ok);
      const float var = static_cast<float>(ss[k] / ok) - mean[k] * mean[k];
      if (var > worst) worst = var;
    }

    if (worst <= MAX_VARIANCE) {
      g_biasGx = mean[0];
      g_biasGy = mean[1];
      g_biasGz = mean[2];
      return true;
    }

    Serial.print(F("  [!] Arm moved during calibration (var="));
    Serial.print(worst, 2);
    Serial.println(F("). Hold still, retrying..."));
  }
  return false;
}

// --- Angle Estimator ---
static void estimatorReset() {
  g_estInit = false;
  g_thetaBias = 0.0f;
  g_P[0][0] = 1.0f; g_P[0][1] = 0.0f;
  g_P[1][0] = 0.0f; g_P[1][1] = 1.0f;
  g_turns = 0;
  g_innovRejects = 0;
  g_aHvalid = false;
}

static void estimatorUpdate(float dt, float ax, float ay, float az,
                            float gx, float gy, float gz) {
  const float au = dot3(ax, ay, az, g_hingeU);
  const float av = dot3(ax, ay, az, g_hingeV);
  const float aH = dot3(ax, ay, az, g_hingeH);
  const float aPerp = sqrtf(au * au + av * av);
  const float aNorm = sqrtf(ax * ax + ay * ay + az * az);

  g_lastAPerp = aPerp;
  g_lastANorm = aNorm;
  g_lastAccUsed = false;

  if (!g_aHvalid) {
    g_aHmin = g_aHmax = aH;
    g_aHvalid = true;
  } else {
    // Relax the interval toward the current reading so that a one-off excursion,
    // such as tilting the whole rig by hand, ages out while a persistent
    // inconsistency keeps it open.
    g_aHmin += (aH - g_aHmin) * AH_DECAY;
    g_aHmax += (aH - g_aHmax) * AH_DECAY;
    if (aH < g_aHmin) g_aHmin = aH;
    if (aH > g_aHmax) g_aHmax = aH;
  }

  // Gravity in body coordinates rotates by -theta when the body rotates by
  // +theta about H, hence the negated V component.
  const bool observable = (aPerp > APERP_MIN_G);
  if (observable) {
    g_lastAccAngle = atan2f(-av, au) * RAD_TO_DEG;
  }

  if (!g_estInit) {
    if (!observable) return;
    g_theta = g_lastAccAngle;
    g_thetaPrev = g_theta;
    g_thetaGlobal = g_theta;
    g_estInit = true;
    g_innovRejects = 0;
    return;
  }

  const float omega = dot3(gx, gy, gz, g_hingeH);
  g_theta = wrap180(g_theta + (omega - g_thetaBias) * dt);
  g_P[0][0] += dt * (dt * g_P[1][1] - g_P[0][1] - g_P[1][0] + Q_THETA);
  g_P[0][1] -= dt * g_P[1][1];
  g_P[1][0] -= dt * g_P[1][1];
  g_P[1][1] += Q_BIAS * dt;

  if (observable) {
    // Angle noise scales as 1/aPerp because a fixed accelerometer error in g maps
    // to a larger angle error the shorter the in-plane gravity component is.
    const float perpScale = 1.0f / (aPerp * aPerp);
    // Any deviation of |a| from 1 g is thrust or vibration rather than gravity, so
    // down-weight the correction instead of letting it pull the angle off.
    const float excess = fabsf(aNorm - 1.0f);
    const float R = R_ACC * perpScale * (1.0f + 200.0f * excess * excess);

    const float y = wrap180(g_lastAccAngle - g_theta);

    if (fabsf(y) > INNOV_GATE_DEG) {
      // Applying this would dump the discrepancy into the bias state and ramp the
      // angle. Coast instead, and re-seed if the disagreement is persistent.
      g_innovRejects++;
      if (g_innovRejects >= INNOV_REJECT_LIMIT) {
        g_theta = g_lastAccAngle;
        g_thetaPrev = g_theta;
        g_thetaBias = 0.0f;
        g_turns = 0;
        g_innovRejects = 0;
      }
    } else {
      g_innovRejects = 0;

      const float S = g_P[0][0] + R;
      const float K0 = g_P[0][0] / S;
      const float K1 = g_P[1][0] / S;

      g_theta = wrap180(g_theta + K0 * y);
      g_thetaBias = constrain(g_thetaBias + K1 * y, -BIAS_LIMIT_DPS, BIAS_LIMIT_DPS);

      const float P00 = g_P[0][0];
      const float P01 = g_P[0][1];
      g_P[0][0] -= K0 * P00;
      g_P[0][1] -= K0 * P01;
      g_P[1][0] -= K1 * P00;
      g_P[1][1] -= K1 * P01;

      g_lastAccUsed = true;
    }
  }

  const float diff = g_theta - g_thetaPrev;
  if (diff > 180.0f) g_turns--;
  else if (diff < -180.0f) g_turns++;
  g_thetaPrev = g_theta;
  g_thetaGlobal = g_turns * 360.0f + g_theta;
}

// A rotation about the true hinge axis cannot change gravity's component along it.
// A large observed spread therefore means the assumed axis is wrong and the angle
// carries little or no information, so closed-loop control must not run on it.
static bool axisLooksWrong() {
  return g_aHvalid && (g_aHmax - g_aHmin) > AH_SPREAD_WARN_G;
}

// --- Hinge Frame Calibration ---
// Every gyro sample taken while the arm swings lies along +/-H, so H is the
// principal eigenvector of sum(g g'). The outer product is sign-invariant, which
// makes back-and-forth sweeps perfectly acceptable.
static void axisAccumulate(float gx, float gy, float gz) {
  if (sqrtf(gx * gx + gy * gy + gz * gz) < CALIB_MIN_RATE_DPS) return;
  const float v[3] = {gx, gy, gz};
  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 3; j++) {
      g_axisM[i][j] += static_cast<double>(v[i]) * v[j];
    }
  }
  g_axisSamples++;
}

static bool axisSolve(float out[3]) {
  // Seeding with the highest-energy column guarantees a non-orthogonal start.
  uint8_t best = 0;
  double bestNorm = -1.0;
  for (uint8_t j = 0; j < 3; j++) {
    const double n = g_axisM[0][j] * g_axisM[0][j] +
                     g_axisM[1][j] * g_axisM[1][j] +
                     g_axisM[2][j] * g_axisM[2][j];
    if (n > bestNorm) { bestNorm = n; best = j; }
  }
  if (!(bestNorm > 0.0)) return false;

  float v[3] = {static_cast<float>(g_axisM[0][best]),
                static_cast<float>(g_axisM[1][best]),
                static_cast<float>(g_axisM[2][best])};
  if (!normalize3(v)) return false;

  for (uint8_t it = 0; it < 64; it++) {
    float w[3];
    for (uint8_t i = 0; i < 3; i++) {
      w[i] = static_cast<float>(g_axisM[i][0] * v[0] +
                                g_axisM[i][1] * v[1] +
                                g_axisM[i][2] * v[2]);
    }
    if (!normalize3(w)) return false;
    v[0] = w[0]; v[1] = w[1]; v[2] = w[2];
  }

  out[0] = v[0]; out[1] = v[1]; out[2] = v[2];
  return true;
}

static bool setZeroReference(const float a0[3]) {
  const float d = dot3(a0[0], a0[1], a0[2], g_hingeH);
  float u[3] = {a0[0] - d * g_hingeH[0],
                a0[1] - d * g_hingeH[1],
                a0[2] - d * g_hingeH[2]};
  if (!normalize3(u)) return false;
  g_hingeU[0] = u[0]; g_hingeU[1] = u[1]; g_hingeU[2] = u[2];
  cross3(g_hingeH, g_hingeU, g_hingeV);
  return true;
}

static void printHingeFrame() {
  Serial.println(F("\n--- Hinge frame (paste into the sketch defaults) ---"));
  Serial.print(F("static float g_hingeH[3] = {"));
  Serial.print(g_hingeH[0], 5); Serial.print(F("f, "));
  Serial.print(g_hingeH[1], 5); Serial.print(F("f, "));
  Serial.print(g_hingeH[2], 5); Serial.println(F("f};"));
  Serial.print(F("static float g_hingeU[3] = {"));
  Serial.print(g_hingeU[0], 5); Serial.print(F("f, "));
  Serial.print(g_hingeU[1], 5); Serial.print(F("f, "));
  Serial.print(g_hingeU[2], 5); Serial.println(F("f};"));
  Serial.print(F("static float g_hingeV[3] = {"));
  Serial.print(g_hingeV[0], 5); Serial.print(F("f, "));
  Serial.print(g_hingeV[1], 5); Serial.print(F("f, "));
  Serial.print(g_hingeV[2], 5); Serial.println(F("f};"));
  Serial.println(F("----------------------------------------------------\n"));
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
  Serial.println(F("   c         : Calibrate hinge AXIS (sweep arm by hand 15s)"));
  Serial.println(F("   z         : Capture ZERO reference (hold arm at mech. zero)"));
  Serial.println(F("   n         : Negate hinge axis (flips sign of the angle)"));
  Serial.println(F("   k         : Print current hinge frame constants"));
  Serial.println(F("   d         : Toggle raw accel/gyro dump (identify the axis)"));
  Serial.println(F("   r         : Reset estimator (angle, bias, turns, diagnostics)"));
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
      g_jogDir = 0;
      stopAllMotors();
      Serial.println(F("\n[!] EMERGENCY STOP: Motor power cut, PID disengaged."));
    } else if (cmd == 'm') {
      g_mode = MODE_MONITOR;
      g_jogDir = 0;
      stopAllMotors();
      Serial.println(F("\n[*] Switched to MONITOR MODE (Motor OFF). Move arm by hand."));
    } else if (cmd == 'c') {
      g_mode = MODE_CALIB_AXIS;
      stopAllMotors();
      for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) g_axisM[i][j] = 0.0;
      }
      g_axisSamples = 0;
      g_calibEndMs = millis() + CALIB_AXIS_MS;
      Serial.println(F("\n[*] AXIS CALIBRATION: sweep the arm back and forth"));
      Serial.println(F("    through its full range for the next 15 seconds..."));
    } else if (cmd == 'z') {
      g_mode = MODE_CALIB_ZERO;
      stopAllMotors();
      g_zeroSum[0] = g_zeroSum[1] = g_zeroSum[2] = 0.0;
      g_zeroSamples = 0;
      g_calibEndMs = millis() + CALIB_ZERO_MS;
      Serial.println(F("\n[*] ZERO CAPTURE: hold the arm at mechanical zero (thrust up)..."));
    } else if (cmd == 'n') {
      for (uint8_t i = 0; i < 3; i++) {
        g_hingeH[i] = -g_hingeH[i];
        g_hingeV[i] = -g_hingeV[i];
      }
      estimatorReset();
      Serial.println(F("\n[*] Hinge axis negated; angle sign flipped. Estimator reset."));
    } else if (cmd == 'k') {
      printHingeFrame();
    } else if (cmd == 'd') {
      g_rawDump = !g_rawDump;
      Serial.print(F("\n[*] Raw axis dump "));
      Serial.println(g_rawDump ? F("ON. Rotate the arm: the gyro axis carrying the")
                               : F("OFF."));
      if (g_rawDump) {
        Serial.println(F("    rotation and the accel axes that change are the hinge plane."));
      }
    } else if (cmd == 'r') {
      estimatorReset();
      Serial.println(F("\n[*] Estimator reset (angle, bias, turns, aH spread)."));
    } else if (cmd == 'f' || cmd == 'b') {
      // Drop to monitor so a running PID does not fight the jog for the output.
      g_mode = MODE_MONITOR;
      const int8_t sign = (cmd == 'f') ? 1 : -1;
      Serial.print(F("[*] Jogging "));
      Serial.println(cmd == 'f' ? F("FORWARD (dir=+1) for 150ms...")
                                : F("BACKWARD (dir=-1) for 150ms..."));
      g_jogDir = static_cast<int8_t>(sign * g_motorPolarity);
      g_jogUntilMs = millis() + 150;
    } else if (cmd == 'p') {
      g_motorPolarity *= -1;
      Serial.print(F("[*] Motor polarity multiplier toggled to: "));
      Serial.println(g_motorPolarity > 0 ? "+1 (Normal)" : "-1 (Inverted)");
      Serial.println(F("    RULE: 'f' should INCREASE the hinge angle. If it DECREASED it, toggle 'p'."));
    } else if (cmd == 't') {
      float angle = Serial.parseFloat();
      if (!g_estInit) {
        Serial.println(F("\n[!] REFUSING PID HOLD: angle estimate has not converged yet."));
        continue;
      }
      if (axisLooksWrong()) {
        Serial.println(F("\n[!] REFUSING PID HOLD: hinge axis looks wrong, so the"));
        Serial.println(F("    angle feedback is unreliable. Run 'c' then 'z' first,"));
        Serial.println(F("    or 'r' to clear the diagnostic if the axis is correct."));
        continue;
      }
      g_targetAngle = constrain(angle, -TARGET_LIMIT_DEG, TARGET_LIMIT_DEG);
      g_integral = 0.0f;
      g_prevAngle = g_thetaGlobal;
      g_jogDir = 0;
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
  // parseFloat() otherwise blocks for a full second on a malformed 't' command,
  // which would stall the 100 Hz estimator loop.
  Serial.setTimeout(50);

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
  if (!calibrateGyro()) {
    Serial.println(F("[!] WARNING: gyro bias calibration never settled."));
    Serial.println(F("    Continuing; the filter will estimate the bias online."));
  }

  estimatorReset();

  Serial.println(F("Arm 0 IMU calibrated successfully. Other arms held safe."));
  Serial.print(F("Hinge axis H = ["));
  Serial.print(g_hingeH[0], 3); Serial.print(F(", "));
  Serial.print(g_hingeH[1], 3); Serial.print(F(", "));
  Serial.print(g_hingeH[2], 3);
  Serial.println(F("] (IMU y-axis). Run 'c' then 'z' to re-measure."));
  printHelp();
}

void loop() {
  const uint32_t t0 = micros();

  // Measured dt keeps the filter correct even if a loop overruns.
  static uint32_t lastUs = 0;
  float dt = 1.0f / SAMPLE_HZ;
  if (lastUs != 0) {
    dt = constrain((t0 - lastUs) * 1e-6f, 1e-3f, 0.1f);
  }
  lastUs = t0;

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

  estimatorUpdate(dt, ax, ay, az, gx, gy, gz);
  const float currentAngle = g_thetaGlobal;

  // 2. Control and telemetry
  static uint32_t lastPrint = 0;

  if (g_jogDir != 0) {
    if (static_cast<int32_t>(millis() - g_jogUntilMs) >= 0) {
      g_jogDir = 0;
      setMotor(0, 0);
    } else {
      setMotor(g_jogDir, 65);
    }
  }

  if (g_mode == MODE_CALIB_AXIS) {
    axisAccumulate(gx, gy, gz);
    if (static_cast<int32_t>(millis() - g_calibEndMs) >= 0) {
      float h[3];
      if (g_axisSamples < 100 || !axisSolve(h)) {
        Serial.println(F("\n[!] AXIS CALIBRATION FAILED: not enough motion recorded."));
        Serial.println(F("    Sweep the arm through a wide range and retry 'c'."));
      } else {
        g_hingeH[0] = h[0]; g_hingeH[1] = h[1]; g_hingeH[2] = h[2];

        // U carries the old zero reference and is no longer perpendicular to the
        // new axis, so re-orthogonalize it. A 'z' capture replaces it properly.
        float u[3] = {g_hingeU[0], g_hingeU[1], g_hingeU[2]};
        const float d = dot3(u[0], u[1], u[2], g_hingeH);
        u[0] -= d * g_hingeH[0]; u[1] -= d * g_hingeH[1]; u[2] -= d * g_hingeH[2];
        if (!normalize3(u)) {
          u[0] = 1.0f; u[1] = 0.0f; u[2] = 0.0f;
          const float d2 = dot3(u[0], u[1], u[2], g_hingeH);
          u[0] -= d2 * g_hingeH[0]; u[1] -= d2 * g_hingeH[1]; u[2] -= d2 * g_hingeH[2];
          normalize3(u);
        }
        g_hingeU[0] = u[0]; g_hingeU[1] = u[1]; g_hingeU[2] = u[2];
        cross3(g_hingeH, g_hingeU, g_hingeV);
        estimatorReset();
        Serial.print(F("\n[*] AXIS CALIBRATION DONE ("));
        Serial.print(g_axisSamples);
        Serial.println(F(" samples)."));
        Serial.print(F("    Hinge axis H = ["));
        Serial.print(g_hingeH[0], 4); Serial.print(F(", "));
        Serial.print(g_hingeH[1], 4); Serial.print(F(", "));
        Serial.print(g_hingeH[2], 4); Serial.println(F("]"));
        Serial.println(F("    Now hold the arm at mechanical zero and press 'z'."));
      }
      g_mode = MODE_MONITOR;
    } else if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      Serial.print(F("CALIB AXIS >> samples: "));
      Serial.print(g_axisSamples);
      Serial.print(F("  time left: "));
      Serial.print((g_calibEndMs - millis()) / 1000);
      Serial.println(F("s"));
    }
  } else if (g_mode == MODE_CALIB_ZERO) {
    g_zeroSum[0] += ax; g_zeroSum[1] += ay; g_zeroSum[2] += az;
    g_zeroSamples++;
    if (static_cast<int32_t>(millis() - g_calibEndMs) >= 0) {
      if (g_zeroSamples == 0) {
        Serial.println(F("\n[!] ZERO CAPTURE FAILED: no samples."));
      } else {
        const float a0[3] = {static_cast<float>(g_zeroSum[0] / g_zeroSamples),
                             static_cast<float>(g_zeroSum[1] / g_zeroSamples),
                             static_cast<float>(g_zeroSum[2] / g_zeroSamples)};
        if (!setZeroReference(a0)) {
          Serial.println(F("\n[!] ZERO CAPTURE FAILED: hinge axis is parallel to gravity"));
          Serial.println(F("    at this pose, so the angle is unobservable here."));
        } else {
          estimatorReset();
          Serial.println(F("\n[*] ZERO CAPTURE DONE. Hinge angle is now 0 at this pose."));
          printHingeFrame();
        }
      }
      g_mode = MODE_MONITOR;
    }
  } else if (g_mode == MODE_MONITOR) {
    if (g_jogDir == 0) {
      stopAllMotors();
    }
    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      if (g_rawDump) {
        Serial.print(F("RAW a: [")); Serial.print(ax, 2);
        Serial.print(F(", ")); Serial.print(ay, 2);
        Serial.print(F(", ")); Serial.print(az, 2);
        Serial.print(F("]  g: [")); Serial.print(gx, 1);
        Serial.print(F(", ")); Serial.print(gy, 1);
        Serial.print(F(", ")); Serial.print(gz, 1);
        Serial.println(F("]"));
      } else {
        const float aHspread = g_aHvalid ? (g_aHmax - g_aHmin) : 0.0f;
        Serial.print(F("Angle: ")); Serial.print(g_thetaGlobal, 1);
        Serial.print(F(" (wrap ")); Serial.print(g_theta, 1);
        Serial.print(F(", turns ")); Serial.print(g_turns);
        Serial.print(F(") | acc-only: ")); Serial.print(g_lastAccAngle, 1);
        Serial.print(F(" | bias: ")); Serial.print(g_thetaBias, 2);
        Serial.print(F(" | |a|: ")); Serial.print(g_lastANorm, 2);
        Serial.print(F(" aPerp: ")); Serial.print(g_lastAPerp, 2);
        Serial.print(F(" aH-spread: ")); Serial.print(aHspread, 2);
        if (aHspread > AH_SPREAD_WARN_G) {
          Serial.print(F(" [AXIS WRONG - run 'c']"));
        }
        Serial.println(g_lastAccUsed ? F(" [ACC OK]") : F(" [COASTING]"));
      }
    }
  } else if (g_mode == MODE_PID_HOLD) {
    const float error = g_targetAngle - currentAngle;

    if (axisLooksWrong()) {
      g_mode = MODE_MONITOR;
      stopAllMotors();
      Serial.println(F("\n[!] HOLD ABORTED: hinge axis inconsistency detected."));
      Serial.println(F("    The angle feedback cannot be trusted. Run 'c' then 'z'."));
      return;
    }

    // Safety runaway tripwire: if arm is pushed or driven beyond 45 deg error, cut power
    if (fabsf(error) > RUNAWAY_ANGLE_LIMIT) {
      g_mode = MODE_MONITOR;
      stopAllMotors();
      Serial.print(F("\n[!] RUNAWAY PROTECTION TRIPPED: Error = "));
      Serial.print(error, 1);
      Serial.println(F(" deg. Motor turned OFF!"));
      return;
    }

    // Derivative on measurement (negative rate of change) avoids setpoint kick.
    // Updated every cycle, including inside the deadband, so the term cannot go
    // stale and kick when the arm leaves the deadband.
    const float dAngle = (currentAngle - g_prevAngle) / dt;
    g_prevAngle = currentAngle;
    const float derivative = -dAngle;

    if (fabsf(error) < DEADBAND_DEG) {
      setMotor(0, 0); // Within deadband target, coast motor
    } else {
      g_integral = constrain(g_integral + error * dt, -20.0f, 20.0f);

      float output = (g_Kp * error) + (g_Ki * g_integral) + (g_Kd * derivative);
      output *= g_motorPolarity;

      const int8_t dir = (output > 0) ? 1 : ((output < 0) ? -1 : 0);
      const uint8_t pwm = constrain(static_cast<int>(fabsf(output)), MIN_PWM, MAX_TEST_PWM);
      setMotor(dir, pwm);
    }

    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      Serial.print(F("HOLD >> Target: ")); Serial.print(g_targetAngle, 1);
      Serial.print(F(" | Angle: ")); Serial.print(currentAngle, 1);
      Serial.print(F(" | Error: ")); Serial.print(error, 1);
      Serial.print(F(" deg | bias: ")); Serial.print(g_thetaBias, 2);
      Serial.println(g_lastAccUsed ? F(" dps [ACC OK]") : F(" dps [GYRO ONLY]"));
    }
  }

  // 100 Hz timing
  while (static_cast<uint32_t>(micros() - t0) < SAMPLE_US) {
    /* wait */
  }
}
