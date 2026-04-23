#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

BNO080 myIMU;
#define TCA9548A_ADDRESS 0x70
#define BNO080_ADDRESS 0x4B

// Function to select channel on the TCA9548A multiplexer
void tcaSelect(uint8_t i) {
  if (i > 7) return;

  Wire1.beginTransmission(TCA9548A_ADDRESS);
  Wire1.write(1 << i);
  Wire1.endTransmission();
}

void checkDevices() {
  byte error;

  Serial.print("Checking Mux (0x70)... ");
  Wire1.beginTransmission(TCA9548A_ADDRESS);
  error = Wire1.endTransmission();
  if (error == 0) {
    Serial.println("FOUND!");
  } else {
    Serial.print("NOT FOUND! (Error code: ");
    Serial.print(error);
    Serial.println(")");
  }

  Serial.print("Checking IMU (0x4B)... ");
  Wire1.beginTransmission(BNO080_ADDRESS);
  error = Wire1.endTransmission();
  if (error == 0) {
    Serial.println("FOUND!");
  } else {
    Serial.print("NOT FOUND! (Error code: ");
    Serial.print(error);
    Serial.println(")");
  }
}

// Function to cleanly flush out any pending data from the BNO080
void flushBNO080() {
  Serial.println("--- Attempting to flush BNO080 I2C buffer ---");
  
  // Try up to 10 times to clear the buffer
  for (int attempts = 0; attempts < 10; attempts++) {
    Wire1.requestFrom((uint8_t)BNO080_ADDRESS, (size_t)4);
    if (Wire1.available() < 4) {
      Serial.println("Flush: No header available, bus seems clear.");
      return;
    }
    
    uint8_t ls_byte = Wire1.read();
    uint8_t ms_byte = Wire1.read();
    uint8_t channel = Wire1.read();
    uint8_t sequence = Wire1.read();
    
    // Calculate length (MSB has continuation bit 15 which we ignore)
    uint16_t length = (((uint16_t)ms_byte & 0x7F) << 8) | ls_byte;
    
    if (length == 0 || length > 1024) {
       Serial.println("Flush: Empty or invalid packet length, stopping flush.");
       return;
    }
    
    Serial.print("Flush: Found pending packet of length ");
    Serial.println(length);
    
    int bytesToRead = length - 4;
    
    // Read the rest of the packet in chunks
    while (bytesToRead > 0) {
      int chunk = (bytesToRead > 28) ? 28 : bytesToRead;
      Wire1.requestFrom((uint8_t)BNO080_ADDRESS, (size_t)(chunk + 4));
      
      // Consume all bytes off the bus
      while (Wire1.available()) {
        Wire1.read();
      }
      
      bytesToRead -= chunk;
    }
    
    Serial.println("Flush: Packet chunk cleared. Checking for more...");
    delay(10);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Start serial for debugging
  Serial.begin(115200);
  
  // Wait for the Serial Monitor to be opened
  while (!Serial) {
    digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
  }
  
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("\n\n\n==========================================");
  Serial.println("BOARD IS AWAKE AND CONNECTED.");
  Serial.println("TYPE ANY CHARACTER AND PRESS ENTER TO START.");
  Serial.println("==========================================");

  while (Serial.available() == 0) {
    digitalWrite(LED_BUILTIN, (millis() / 100) % 2);
  }
  
  while (Serial.available()) {
    Serial.read();
    delay(2);
  }
  
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("\n\n------------------------------------------");
  Serial.println("Starting Diagnostic Single IMU Test (BNO080)");
  Serial.println("------------------------------------------");

  // Enable debugging output from the library
  myIMU.enableDebugging(Serial);

  bool initialized = false;
  
  // Try up to 5 times to initialize, resetting the Arduino's I2C peripheral completely each time
  for (int attempt = 1; attempt <= 5; attempt++) {
    Serial.print("\n--- Initialization Attempt ");
    Serial.print(attempt);
    Serial.println(" ---");
    
    // Complete I2C peripheral reset
    Wire1.end();
    delay(10);
    Wire1.begin();
    
    // 400kHz is highly recommended for BNO080 to prevent SHTP buffer overflows and clock stretching bugs
    Wire1.setClock(400000); 
    delay(50);
    
    Serial.println("Selecting MUX Channel 0...");
    tcaSelect(0);
    delay(100);
    
    checkDevices();

    // The BNO080 can get stuck if it is trying to send a packet but we send an I2C Write (like softReset).
    // So we first flush any pending reads.
    flushBNO080();
    delay(50);

    // Manually send a soft-reset command because the library's begin() doesn't wait long enough 
    // for the 200ms boot sequence of the BNO080.
    Serial.println("Sending Manual Soft Reset Command...");
    Wire1.beginTransmission(BNO080_ADDRESS);
    Wire1.write(5); // Packet Length LSB (4 header + 1 payload)
    Wire1.write(0); // Packet Length MSB
    Wire1.write(1); // Channel: Executable (1)
    Wire1.write(0); // Sequence Number
    Wire1.write(1); // Reset Byte (1)
    uint8_t err = Wire1.endTransmission();
    
    if (err == 0) {
      Serial.println("Manual soft reset accepted! Waiting 500ms for IMU to boot...");
      delay(500);
      
      // Flush the massive 284 byte advertisement packet that comes after a boot
      Serial.println("Flushing post-boot advertisement packet...");
      flushBNO080();
      delay(50);
    } else {
      Serial.print("Manual soft reset failed! (Error ");
      Serial.print(err);
      Serial.println(")");
      // Even if it failed, we'll still try myIMU.begin() just in case
    }
    
    Serial.println("Calling myIMU.begin()...");
    if (myIMU.begin(BNO080_ADDRESS, Wire1)) {
      initialized = true;
      break;
    } else {
      Serial.println("myIMU.begin() failed.");
      delay(1000); // Wait a second before trying the whole sequence again
    }
  }

  if (initialized) {
    Serial.println("\n==========================================");
    Serial.println("SUCCESS: BNO080 detected and initialized!");
    Serial.println("==========================================");
  } else {
    Serial.println("\n==========================================");
    Serial.println("FATAL ERROR: BNO080 could not be initialized.");
    Serial.println("==========================================");
    while (1) {
      digitalWrite(LED_BUILTIN, (millis() / 50) % 2); 
      delay(10); 
    }
  }

  Serial.println("Enabling rotation vector at 50ms interval...");
  myIMU.enableRotationVector(50);
  
  Serial.println("IMU initialized successfully. Reading data...");
}

void loop() {
  if (myIMU.dataAvailable() == true) {
    float roll = (myIMU.getRoll()) * 180.0 / PI; 
    float pitch = (myIMU.getPitch()) * 180.0 / PI;
    float yaw = (myIMU.getYaw()) * 180.0 / PI;

    Serial.print("Roll: ");
    Serial.print(roll, 2);
    Serial.print("\tPitch: ");
    Serial.print(pitch, 2);
    Serial.print("\tYaw: ");
    Serial.println(yaw, 2);
  }
  
  delay(10);
}