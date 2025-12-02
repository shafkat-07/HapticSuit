void setup() {
  // Start the Hardware Serial (TX=1, RX=0) to listen to Master
  Serial1.begin(9600);
  
  // Setup the built-in red LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // Check if Master sent any data
  if (Serial1.available()) {
    char cmd = Serial1.read();
    
    // If Master says '1', turn LED ON. If '0', turn LED OFF.
    if (cmd == '1') {
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (cmd == '0') {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

