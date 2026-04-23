/*
 * GY-521 (MPU-6050) — TCA9548A: 4 IMUs on mux channels 0–3, raw samples + Madgwick fusion roll (BNO-style).
 *
 * Dependency (Arduino Library Manager): "Madgwick" by Arduino (v1.2.0+)
 *   https://github.com/arduino-libraries/MadgwickAHRS
 *   Include: MadgwickAHRS.h  |  Class: Madgwick
 *
 * Hardware: UNO R4 WiFi Wire1 (Qwiic) -> TCA9548A @ 0x70 -> channels 0..3 -> one GY-521 each.
 *
 * Axis remap (MPU-6050 chip frame -> fusion input frame):
 *   Default is identity. To match Android/BNO mounting, permute/sign the assignments
 *   in remapImuFrame() using your bench tests (see comment block there).
 */

#include <Wire.h>
#include <MadgwickAHRS.h>

// --- Library ---
static const uint8_t NUM_IMUS = 4;
static Madgwick fusion[NUM_IMUS];

// --- Timing (match HapticSuitUnoR4Single LOOP_RATE for consistent filter tuning) ---
static const float SAMPLE_HZ = 100.0f;
static const uint32_t SAMPLE_US = static_cast<uint32_t>(1000000.0f / SAMPLE_HZ);

// --- I2C / MPU ---
static const uint8_t TCA9548A_ADDRESS = 0x70;
// One MPU-6050 per mux channel (adjust order here if your wiring differs).
static const uint8_t IMU_MUX_CHANNELS[NUM_IMUS] = {0, 1, 2, 3};
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

static uint8_t g_mpuAddress[NUM_IMUS];

// Gyro bias (deg/s) after calibration, per IMU
static float g_biasGx[NUM_IMUS];
static float g_biasGy[NUM_IMUS];
static float g_biasGz[NUM_IMUS];

static void tcaSelect(uint8_t channel) {
  if (channel > 7) {
    return;
  }
  Wire1.beginTransmission(TCA9548A_ADDRESS);
  Wire1.write(static_cast<uint8_t>(1u << channel));
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

/*
 * Remap chip-frame accel (g) and gyro (deg/s) into the frame expected for fusion.
 * Identity: out = in. For90 deg mounts, swap axes and flip signs, e.g.:
 *   ax_out = ay_in; ay_out = -ax_in; az_out = az_in;
 * Document final values once roll matches mechanical / old BNO behavior.
 */
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

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Wire1.begin();
  Wire1.setClock(400000);
  delay(20);

  for (uint8_t i = 0; i < NUM_IMUS; i++) {
    tcaSelect(IMU_MUX_CHANNELS[i]);
    delay(10);

    uint8_t who = 0;
    if (mpuProbe(MPU6050_ADDR_PRIMARY, &who)) {
      g_mpuAddress[i] = MPU6050_ADDR_PRIMARY;
    } else if (mpuProbe(MPU6050_ADDR_AD0_HIGH, &who)) {
      g_mpuAddress[i] = MPU6050_ADDR_AD0_HIGH;
    } else {
      Serial.print(F("IMU "));
      Serial.print(i);
      Serial.print(F(" (mux ch "));
      Serial.print(IMU_MUX_CHANNELS[i]);
      Serial.println(F("): MPU6050 not found on 0x68/0x69."));
      while (1) {
        delay(1000);
      }
    }

    if (!mpuInit(g_mpuAddress[i])) {
      Serial.print(F("IMU "));
      Serial.print(i);
      Serial.println(F(": MPU6050 init failed."));
      while (1) {
        delay(1000);
      }
    }

    Serial.print(F("IMU "));
    Serial.print(i);
    Serial.print(F(" (mux "));
    Serial.print(IMU_MUX_CHANNELS[i]);
    Serial.println(F("): keep still for gyro bias calibration..."));
    calibrateGyroBias(g_mpuAddress[i], &g_biasGx[i], &g_biasGy[i], &g_biasGz[i], 500, 200);
    Serial.print(F("IMU "));
    Serial.print(i);
    Serial.print(F(": gyro bias (deg/s) "));
    Serial.print(g_biasGx[i], 3);
    Serial.print(F(","));
    Serial.print(g_biasGy[i], 3);
    Serial.print(F(","));
    Serial.println(g_biasGz[i], 3);

    fusion[i].begin(SAMPLE_HZ);
  }

  Serial.println(F("All IMUs ready."));
}

void loop() {
  const uint32_t t0 = micros();

  for (uint8_t i = 0; i < NUM_IMUS; i++) {
    tcaSelect(IMU_MUX_CHANNELS[i]);

    float axg, ayg, azg, gxd, gyd, gzd;
    if (!readMpuScaled(g_mpuAddress[i], &axg, &ayg, &azg, &gxd, &gyd, &gzd)) {
      Serial.print(F("IMU "));
      Serial.print(i);
      Serial.println(F(": sample read failed."));
      delay(100);
      return;
    }

    gxd -= g_biasGx[i];
    gyd -= g_biasGy[i];
    gzd -= g_biasGz[i];

    float axf, ayf, azf, gxf, gyf, gzf;
    remapImuFrame(axg, ayg, azg, gxd, gyd, gzd, &axf, &ayf, &azf, &gxf, &gyf, &gzf);

    fusion[i].updateIMU(gxf, gyf, gzf, axf, ayf, azf);

    const float rollDeg = fusion[i].getRoll();
    const float pitchDeg = fusion[i].getPitch();
    const float yawDeg = fusion[i].getYaw();

    Serial.print(F("IMU"));
    Serial.print(i);
    Serial.print(F(" roll "));
    Serial.print(rollDeg, 2);
    Serial.print(F(" pitch "));
    Serial.print(pitchDeg, 2);
    Serial.print(F(" yaw "));
    Serial.print(yawDeg, 2);
    Serial.print(F(" aRaw "));
    Serial.print(static_cast<int16_t>(axg * ACCEL_LSB_PER_G));
    Serial.print(F(","));
    Serial.print(static_cast<int16_t>(ayg * ACCEL_LSB_PER_G));
    Serial.print(F(","));
    Serial.print(static_cast<int16_t>(azg * ACCEL_LSB_PER_G));
    if (i + 1 < NUM_IMUS) {
      Serial.print(F(" | "));
    }
  }
  Serial.println();

  while (static_cast<uint32_t>(micros() - t0) < SAMPLE_US) {
    /* wait */
  }
}
