/*
 * ESC bench test — 4× brushless ESC signals only (no IMU, no TB6612 rotation).
 *
 * Hardware per monoboard_wiring_guide.md (UNO R4 WiFi):
 *   Arm 1 thruster: D2
 *   Arm 2 thruster: D4
 *   Arm 3 thruster: D8
 *   Arm 4 thruster: A4 (digital)
 *
 * Each ESC: signal pin above + signal GND to UNO GND; motor power from battery.
 * Remove all propellers. Do not parallel multiple BEC 5V outputs — power UNO via USB
 * for bench testing when possible.
 *
 * Set ACTIVE_ESC to the arm indices you have wired (0 = Arm 1 … 3 = Arm 4).
 *
 * Pulse timing matches HapticSuitUnoR4Single.ino: Servo attach1000–2000 µs.
 * Edit ESC_TEST_US if your ESCs need a slightly higher minimum to arm.
 */

#include <Servo.h>

// Which ESCs are connected (each value 0..3, no duplicates)
static const uint8_t ACTIVE_ESC[] = {0, 1, 2, 3};
static const size_t NUM_ACTIVE_ESC =
    sizeof(ACTIVE_ESC) / sizeof(ACTIVE_ESC[0]);

// Monoboard ESC pins — same as escPins[] in HapticSuitUnoR4Single.ino
static const uint8_t kEscPins[] = {2, 4, 8, A4};

// TB6612 STBY (D13): drive HIGH if your bench setup powers the full monoboard
// so both drivers stay out of standby. No effect if D13 is not wired.
static const uint8_t PIN_STBY = 13;

static const char *const kArmNames[] = {
    "Arm 1", "Arm 2", "Arm 3", "Arm 4"};

// Servo / ESC pulse limits (microseconds)
static const int ESC_MIN_US = 1000;
static const int ESC_MAX_US = 2000;
// Conservative test throttle; increase only after each ESC arms cleanly at idle.
static const int ESC_TEST_US = 1100;

// Hold low after upload / before sequence so you can connect battery safely
static const unsigned long ARM_SETTLE_MS = 4000;
// Smooth ramp duration to ESC_TEST_US
static const unsigned long RAMP_UP_MS = 800;
static const unsigned long HOLD_AT_TEST_MS = 1500;
static const unsigned long RAMP_DOWN_MS = 800;
static const unsigned long GAP_BETWEEN_MS = 800;

static Servo esc[NUM_ACTIVE_ESC];

static void setAllActiveUs(int us) {
  us = constrain(us, ESC_MIN_US, ESC_MAX_US);
  for (size_t i = 0; i < NUM_ACTIVE_ESC; i++) {
    esc[i].writeMicroseconds(us);
  }
}

static bool validateActiveEsc() {
  if (NUM_ACTIVE_ESC == 0) {
    Serial.println(F("ERROR: ACTIVE_ESC is empty."));
    return false;
  }
  for (size_t i = 0; i < NUM_ACTIVE_ESC; i++) {
    if (ACTIVE_ESC[i] >= 4) {
      Serial.print(F("ERROR: ACTIVE_ESC["));
      Serial.print(i);
      Serial.println(F("] must be 0..3."));
      return false;
    }
    for (size_t j = i + 1; j < NUM_ACTIVE_ESC; j++) {
      if (ACTIVE_ESC[i] == ACTIVE_ESC[j]) {
        Serial.println(F("ERROR: duplicate entry in ACTIVE_ESC."));
        return false;
      }
    }
  }
  return true;
}

static void rampTo(size_t escIndex, int fromUs, int toUs, unsigned long durationMs) {
  const unsigned long t0 = millis();
  while (true) {
    unsigned long elapsed = millis() - t0;
    if (elapsed >= durationMs) {
      esc[escIndex].writeMicroseconds(toUs);
      break;
    }
    int u = fromUs + (int)((long)(toUs - fromUs) * (long)elapsed / (long)durationMs);
    esc[escIndex].writeMicroseconds(constrain(u, ESC_MIN_US, ESC_MAX_US));
    delay(10);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
  }
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println();
  Serial.println(F("=========================================="));
  Serial.println(F("ESC-only test (monoboard pins)"));
  Serial.println(F("See monoboard_wiring_guide.md section C"));
  Serial.print(F("NUM_ACTIVE_ESC="));
  Serial.println(NUM_ACTIVE_ESC);
  for (size_t i = 0; i < NUM_ACTIVE_ESC; i++) {
    uint8_t arm = ACTIVE_ESC[i];
    Serial.print(F("  ACTIVE_ESC["));
    Serial.print(i);
    Serial.print(F("]="));
    Serial.print(arm);
    Serial.print(F(" ("));
    Serial.print(kArmNames[arm]);
    Serial.print(F(") pin "));
    Serial.println(kEscPins[arm]);
  }

  if (!validateActiveEsc()) {
    while (1) {
      digitalWrite(LED_BUILTIN, (millis() / 200) % 2);
      delay(10);
    }
  }

  for (size_t i = 0; i < NUM_ACTIVE_ESC; i++) {
    uint8_t arm = ACTIVE_ESC[i];
    esc[i].attach(kEscPins[arm], ESC_MIN_US, ESC_MAX_US);
    esc[i].writeMicroseconds(ESC_MIN_US);
  }

  Serial.print(F("ESC_MIN_US="));
  Serial.print(ESC_MIN_US);
  Serial.print(F(" ESC_TEST_US="));
  Serial.println(ESC_TEST_US);
  Serial.println(F("NO PROPS. Connect battery to ESCs; UNO can stay on USB."));
  Serial.println(F("Type any character + Enter to arm idle + run test sequence."));
  Serial.println(F("=========================================="));

  while (Serial.available() == 0) {
    digitalWrite(LED_BUILTIN, (millis() / 100) % 2);
  }
  while (Serial.available()) {
    Serial.read();
    delay(2);
  }
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.print(F("Holding "));
  Serial.print(ESC_MIN_US);
  Serial.print(F(" us for "));
  Serial.print(ARM_SETTLE_MS);
  Serial.println(F(" ms (ESC init / arm at idle)..."));
  delay(ARM_SETTLE_MS);
}

void loop() {
  for (size_t i = 0; i < NUM_ACTIVE_ESC; i++) {
    uint8_t arm = ACTIVE_ESC[i];
    Serial.print(F("--- "));
    Serial.print(kArmNames[arm]);
    Serial.print(F(" (pin "));
    Serial.print(kEscPins[arm]);
    Serial.println(F(") ramp up / hold / ramp down ---"));

    setAllActiveUs(ESC_MIN_US);
    delay(GAP_BETWEEN_MS);

    rampTo(i, ESC_MIN_US, ESC_TEST_US, RAMP_UP_MS);
    delay(HOLD_AT_TEST_MS);
    rampTo(i, ESC_TEST_US, ESC_MIN_US, RAMP_DOWN_MS);

    setAllActiveUs(ESC_MIN_US);
    delay(GAP_BETWEEN_MS);
  }

  Serial.println(F("--- Full cycle done. Pausing 3 s ---"));
  delay(3000);
}
