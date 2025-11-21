/*
  Haptic Suit - Dual Motor Hardware Test
  Tests both Motor A (Arm 1) and Motor B (Arm 2)
  
  Wiring Check:
  - Motor A: Pins 5 (PWM), 12, 6
  - Motor B: Pins 11 (PWM), 14(A0), 13
*/

// --- Motor A Definitions ---
const int PWMA = 5;
const int AIN1 = 12;
const int AIN2 = 6;

// --- Motor B Definitions ---
const int PWMB = 11;
const int BIN1 = 14; // labeled "A0" on the board
const int BIN2 = 13; // labeled "13" (Red LED)

// --- Settings ---
const int MOTOR_SPEED = 100; // Safe speed (0-255)

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("--- Dual Motor Test Start ---");

  // Configure Motor A Pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  // Configure Motor B Pins
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  stopAll();
  delay(2000);
}

void loop() {
  // 1. Test Motor A
  Serial.println("Testing Motor A (Forward)...");
  runMotorA(MOTOR_SPEED, true);
  delay(1000);
  stopAll();
  delay(500);

  Serial.println("Testing Motor A (Reverse)...");
  runMotorA(MOTOR_SPEED, false);
  delay(1000);
  stopAll();
  delay(1000);

  // 2. Test Motor B
  Serial.println("Testing Motor B (Forward)...");
  runMotorB(MOTOR_SPEED, true);
  delay(1000);
  stopAll();
  delay(500);

  Serial.println("Testing Motor B (Reverse)...");
  runMotorB(MOTOR_SPEED, false);
  delay(1000);
  stopAll();
  delay(2000);
}

// --- Helper Functions ---

void runMotorA(int speed, bool forward) {
  digitalWrite(AIN1, forward ? HIGH : LOW);
  digitalWrite(AIN2, forward ? LOW : HIGH);
  analogWrite(PWMA, speed);
}

void runMotorB(int speed, bool forward) {
  digitalWrite(BIN1, forward ? HIGH : LOW);
  digitalWrite(BIN2, forward ? LOW : HIGH);
  analogWrite(PWMB, speed);
}

void stopAll() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
  
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
}