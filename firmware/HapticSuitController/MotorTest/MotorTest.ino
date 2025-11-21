/*
  Haptic Suit - Motor Hardware Test
  Based on pinout from HapticSuitController.ino
  
  This sketch verifies:
  1. The Feather M0 is alive.
  2. The wiring to the TB6612 is correct.
  3. The Motor Power supply is working.
*/

// --- Pin Definitions (Matches HapticSuitController.ino) ---
const int PIN_PWM_SPEED = 5;    // Controls how fast
const int PIN_DIR_1     = 12;   // Controls direction leg 1
const int PIN_DIR_2     = 6;    // Controls direction leg 2

// --- Settings ---
const int MOTOR_SPEED   = 100;  // Speed from 0 (Stop) to 255 (Max)
// Note: If your battery is 11.1V and motor is 6V, keep this below 130!

void setup() {
  // 1. Setup Serial Monitor for debugging
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Wait for USB connection
  
  Serial.println("--- Haptic Suit Motor Test Start ---");

  // 2. Configure Pins
  pinMode(PIN_PWM_SPEED, OUTPUT);
  pinMode(PIN_DIR_1, OUTPUT);
  pinMode(PIN_DIR_2, OUTPUT);

  // 3. Initial State: Stopped
  stopMotor();
  delay(2000);
}

void loop() {
  // Step 1: Spin Forward
  Serial.println("Spinning FORWARD...");
  spinForward(MOTOR_SPEED);
  delay(2000); // Run for 2 seconds

  // Step 2: Stop
  Serial.println("Stopping...");
  stopMotor();
  delay(1000); // Rest for 1 second

  // Step 3: Spin Backward
  Serial.println("Spinning BACKWARD...");
  spinBackward(MOTOR_SPEED);
  delay(2000); // Run for 2 seconds

  // Step 4: Stop
  Serial.println("Stopping...");
  stopMotor();
  delay(1000); // Rest for 1 second
}

// --- Helper Functions ---

void spinForward(int speed) {
  // To move forward: One pin HIGH, the other LOW
  digitalWrite(PIN_DIR_1, HIGH);
  digitalWrite(PIN_DIR_2, LOW);
  analogWrite(PIN_PWM_SPEED, speed);
}

void spinBackward(int speed) {
  // To move backward: Swap the HIGH and LOW
  digitalWrite(PIN_DIR_1, LOW);
  digitalWrite(PIN_DIR_2, HIGH);
  analogWrite(PIN_PWM_SPEED, speed);
}

void stopMotor() {
  // To stop: Both LOW (Coast) or Speed 0
  digitalWrite(PIN_DIR_1, LOW);
  digitalWrite(PIN_DIR_2, LOW);
  analogWrite(PIN_PWM_SPEED, 0);
}