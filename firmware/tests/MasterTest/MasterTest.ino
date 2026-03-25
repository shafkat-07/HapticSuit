void setup() {
  // Start the Hardware Serial (TX=1, RX=0) to talk to Slave
  Serial1.begin(9600);
}

void loop() {
  // Send '1' to turn Slave LED ON
  Serial1.write('1');
  delay(1000); // Wait 1 second
  
  // Send '0' to turn Slave LED OFF
  Serial1.write('0');
  delay(1000); // Wait 1 second
}

