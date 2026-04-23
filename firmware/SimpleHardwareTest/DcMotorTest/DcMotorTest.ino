/*
 * DC motor bench test — TB6612 only (no IMU, no ESC).
 *
 * Hardware per monoboard_wiring_guide.md (UNO R4 WiFi + 2x TB6612FNG):
 *   STBY (both drivers): D13
 *   Arm 1: PWM D3,  DIR D10 / D11
 *   Arm 2: PWM D5,  DIR D12 / D7
 *   Arm 3: PWM D6,  DIR A0 / A1
 *   Arm 4: PWM D9,  DIR A2 / A3
 *
 * Set ACTIVE_ARMS to the arm indices you have wired (0 = Arm 1 … 3 = Arm 4).
 * Default is two motors: Arm 1 and Arm 2.
 *
 * Ensure VM motor supply and UNO GND are common. Start with motors unloaded
 * and a conservative TEST_SPEED.
 */

// Which motors are connected (each value 0..3, no duplicates)
static const uint8_t ACTIVE_ARMS[] = {0, 1, 2, 3};
static const size_t NUM_ACTIVE_ARMS = sizeof(ACTIVE_ARMS) / sizeof(ACTIVE_ARMS[0]);

// TB6612 standby — must be HIGH for any motor output
static const uint8_t PIN_STBY = 13;

struct MotorPins {
  const char *name;
  uint8_t pwm;
  uint8_t in1;
  uint8_t in2;
};

static const MotorPins kMotors[] = {
    {"Arm 1", 3, 10, 11},
    {"Arm 2", 9, 12, 7},
    {"Arm 3", 6, A0, A1},
    {"Arm 4", 5, A2, A3},
};

// Keep low until you trust wiring and mechanical setup (0–255)
static const uint8_t TEST_SPEED = 80;

static const unsigned long STEP_MS = 2000;
static const unsigned long GAP_MS = 500;

static void motorCoast(const MotorPins &m) {
  analogWrite(m.pwm, 0);
  digitalWrite(m.in1, LOW);
  digitalWrite(m.in2, LOW);
}

static void motorStopAllActive() {
  for (size_t i = 0; i < NUM_ACTIVE_ARMS; i++) {
    motorCoast(kMotors[ACTIVE_ARMS[i]]);
  }
}

// TB6612: IN1=H, IN2=L vs IN1=L, IN2=H selects direction; PWM on PWMA/PWMB sets speed.
static void motorRun(const MotorPins &m, bool forward, uint8_t speed) {
  if (forward) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
  } else {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
  }
  analogWrite(m.pwm, speed);
}

static bool validateActiveArms() {
  if (NUM_ACTIVE_ARMS == 0) {
    Serial.println(F("ERROR: ACTIVE_ARMS is empty."));
    return false;
  }
  for (size_t i = 0; i < NUM_ACTIVE_ARMS; i++) {
    if (ACTIVE_ARMS[i] >= 4) {
      Serial.print(F("ERROR: ACTIVE_ARMS["));
      Serial.print(i);
      Serial.println(F("] must be 0..3."));
      return false;
    }
    for (size_t j = i + 1; j < NUM_ACTIVE_ARMS; j++) {
      if (ACTIVE_ARMS[i] == ACTIVE_ARMS[j]) {
        Serial.println(F("ERROR: duplicate entry in ACTIVE_ARMS."));
        return false;
      }
    }
  }
  return true;
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
  Serial.println(F("DC motor test (TB6612 only)"));
  Serial.println(F("STBY=D13; see monoboard_wiring_guide.md"));
  Serial.print(F("NUM_ACTIVE_ARMS="));
  Serial.println(NUM_ACTIVE_ARMS);
  for (size_t i = 0; i < NUM_ACTIVE_ARMS; i++) {
    Serial.print(F("  ACTIVE_ARMS["));
    Serial.print(i);
    Serial.print(F("]="));
    Serial.print(ACTIVE_ARMS[i]);
    Serial.print(F(" ("));
    Serial.print(kMotors[ACTIVE_ARMS[i]].name);
    Serial.println(F(")"));
  }

  if (!validateActiveArms()) {
    while (1) {
      digitalWrite(LED_BUILTIN, (millis() / 200) % 2);
      delay(10);
    }
  }

  for (size_t i = 0; i < NUM_ACTIVE_ARMS; i++) {
    const MotorPins &m = kMotors[ACTIVE_ARMS[i]];
    pinMode(m.pwm, OUTPUT);
    pinMode(m.in1, OUTPUT);
    pinMode(m.in2, OUTPUT);
    motorCoast(m);
  }

  Serial.println(F("Type any character + Enter to start sequence."));
  Serial.println(F("=========================================="));

  while (Serial.available() == 0) {
    digitalWrite(LED_BUILTIN, (millis() / 100) % 2);
  }
  while (Serial.available()) {
    Serial.read();
    delay(2);
  }
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.print(F("TEST_SPEED="));
  Serial.println(TEST_SPEED);
}

void loop() {
  Serial.println(F("--- Forward each arm ---"));
  for (size_t i = 0; i < NUM_ACTIVE_ARMS; i++) {
    const MotorPins &m = kMotors[ACTIVE_ARMS[i]];
    Serial.print(F("  "));
    Serial.print(m.name);
    Serial.println(F(" forward"));
    motorRun(m, true, TEST_SPEED);
    delay(STEP_MS);
    motorCoast(m);
    delay(GAP_MS);
  }

  motorStopAllActive();
  delay(GAP_MS);

  Serial.println(F("--- Reverse each arm ---"));
  for (size_t i = 0; i < NUM_ACTIVE_ARMS; i++) {
    const MotorPins &m = kMotors[ACTIVE_ARMS[i]];
    Serial.print(F("  "));
    Serial.print(m.name);
    Serial.println(F(" reverse"));
    motorRun(m, false, TEST_SPEED);
    delay(STEP_MS);
    motorCoast(m);
    delay(GAP_MS);
  }

  motorStopAllActive();
  Serial.println(F("--- Cycle complete. Pausing 3 s ---"));
  delay(3000);
}
