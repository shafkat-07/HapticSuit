#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO08x

BNO08x myIMU;

// For the Feather M0, we often need to define the I2C pins explicitly if the default Wire fails
#define IMU_SDA 20
#define IMU_SCL 21

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10); // Wait for USB

  Serial.println("BNO086 Basic Test");

  // Initialize I2C with a fast clock
  Wire.begin();
  Wire.setClock(100000); 

  // Connect to the IMU
  // If your scanner found 0x4A, change the address below0!
  if (myIMU.begin(0x4B) == false) {
    Serial.println("BNO086 not detected at default I2C address. Check wiring or try 0x4A.");
    while (1);
  }

  Serial.println("BNO086 Connected!");

  // Enable Rotation Vector
  setReports();
}

void setReports() {
  // Send data every 50ms
  if (myIMU.enableGameRotationVector(50) == true) {
    Serial.println("Rotation vector enabled");
  } else {
    Serial.println("Could not enable rotation vector");
  }
}

void loop() {
  // Look for reports from the IMU
  if (myIMU.getSensorEvent() == true) {
    
    // Get the Rotation Vector (Quaternions converted to Euler)
    // Note: The BNO08x library separates getting the event from reading the data
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      
      float roll = (myIMU.getRoll()) * 180.0 / PI;
      float pitch = (myIMU.getPitch()) * 180.0 / PI;
      float yaw = (myIMU.getYaw()) * 180.0 / PI;

      Serial.print("Roll: ");
      Serial.print(roll, 1);
      Serial.print(" Pitch: ");
      Serial.print(pitch, 1);
      Serial.print(" Yaw: ");
      Serial.println(yaw, 1);
    }
  }
}