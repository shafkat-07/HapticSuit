#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "SparkFun_BNO080_Arduino_Library.h" 

// Include the PID class
#include "PID.cpp"

// Define the number of LOCAL motors (Arm 1 & 2)
#define NUMBER_ARMS 2 
// Define the IMU start address
#define IMU_START_ADDRESS 0x4B
// Define the wifi SSID and password (set to "" for no password)
#define WIFI_SSID "NU-IoT"
#define WIFI_PASSWORD "nkzvswmm"
// Define if you want to enable wifi (makes debugging easier)
#define ENABLE_WIFI false
// Define the UDP communication port
#define UDP_PORT 8888
// Define the gear ratio of (1:150) ~70RPM
#define PPR 1050
// Define the looping frequency (HZ)
#define LOOP_RATE 100
// Define the min and max for roll and motor speed
#define ROLL_MAX 270
#define ROLL_MIN -270
#define ROTATION_MAX_SPEED 255
#define ROTATION_MIN_SPEED 20
#define MOTOR_SPEED_MAX 100
#define MOTOR_SPEED_MIN 0

// Define the wifi status
int status = WL_IDLE_STATUS;

// Define variable for communication
unsigned int localPort = UDP_PORT;     
char packetBuffer[SOCKET_BUFFER_UDP_SIZE + 1]; 
char  ReplyBuffer[] = "acknowledged";

// Define the communication protocal
WiFiUDP Udp;

// Define variables for the motors
Servo esc_array[NUMBER_ARMS];
int   roll_setpoint[4]  = {0,0,0,0}; // Stores setpoints for ALL 4 arms
int   motor_throttle[4] = {0,0,0,0}; // Stores throttle for ALL 4 arms

// Create the IMU (imu1 goes to 0x4B, imu2 goes to 0x4A)
BNO080 imu_array[NUMBER_ARMS];

// Create variables to hold the IMU data (Only for local arms 1 & 2)
float current_roll[NUMBER_ARMS];
float previous_roll[NUMBER_ARMS];
float global_roll[NUMBER_ARMS];
int   roll_state[NUMBER_ARMS];

// Declare the pins for Local Arms
const int pwn_pin[] = {5,  11};
const int direction_pin1[] = {12, 14};
const int direction_pin2[] = {6,  13};

// Get the loop rate
const float dt = 1.0 / LOOP_RATE;

// PID class instances
PID pid[NUMBER_ARMS];

// Define setup
void setup()
{
  // Allow time for startup
  delay(1000);

  // Configure communication
  Serial.begin(115200);   // Debug
  Serial1.begin(115200);  // Communication to Slave

  // Start the setup
  delay(5000);
  Serial.println();
  Serial.println("Master Program Setup Starting");

  // Init variables
  Serial.println("Init variables");
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    current_roll[k] = 0;
    previous_roll[k] = 0;
    global_roll[k] = 0;
    roll_state[k] = 0;
  }

  // Start wifi
  if (ENABLE_WIFI)
  {
    Serial.println("Setting up WIFI");
    // Configure pins for Adafruit ATWINC1500 Feather
    WiFi.setPins(8, 7, 4, 2);

    // check for the presence of the shield:
    if (WiFi.status() == WL_NO_SHIELD)
    {
      while (1)
      {
        Serial.println("WiFi shield not present");
        delay(10);
      }
    }

    // attempt to connect to WiFi network:
    while ( status != WL_CONNECTED) {
      Serial.println("Attempting to connect to SSID: " + String(WIFI_SSID) + "\n");
      // Connect to the network
      if(strlen(WIFI_PASSWORD) == 0)
      {
        // No password
        status = WiFi.begin(WIFI_SSID);
      } 
      else
      {
        // Password
        status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      }
        
      // wait 10 seconds for connection:
      delay(10000);
    }
    
    // Print you are connected to wifi and print the status (includes the ip address)
    Serial.println("Connected to wifi");
    printWiFiStatus();

    // Start a UDP port to accept commands
    Udp.begin(localPort);
  }

  // Setup the esc motors
  Serial.println("Attaching motors");
  analogWriteResolution(8);
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    esc_array[k].attach(9+k, 1000, 2000);
    esc_array[k].write(0);

    // Set the different pins
    pinMode(pwn_pin[k],OUTPUT);
    pinMode(direction_pin1[k],OUTPUT);
    pinMode(direction_pin2[k],OUTPUT);

    // Print configuration for verification
    Serial.print("  Arm "); Serial.print(k+1); Serial.println(" Configured:");
    Serial.print("    - ESC (Throttle) on Pin "); Serial.println(9+k);
    Serial.print("    - Orientation Motor on Pins: PWM="); Serial.print(pwn_pin[k]);
    Serial.print(", DIR1="); Serial.print(direction_pin1[k]);
    Serial.print(", DIR2="); Serial.println(direction_pin2[k]);
  }
  delay(10);


  // Set the different pins to control the motors
  Serial.println("Creating controllers");
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    // Set the different PID paramters
    pid[k].set_parameters(10, 0, 0, dt);
  }
  delay(10);

  //Increase I2C data rate to 400kHz
  Serial.println("Increasing I2C rate");
  Wire.begin();
  Wire.setClock(50000);
  delay(10);

  // Start the IMU sensors
  Serial.println("Connecting to IMUs");
  bool imu_success[NUMBER_ARMS] = {false};

  int imu_address = IMU_START_ADDRESS;
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    Serial.print("Attempting to connect to IMU at 0x");
    Serial.println(imu_address, HEX);

    // Try to connect
    imu_success[k] = imu_array[k].begin(imu_address);
    
    if (imu_success[k] == false)
    {
       Serial.print("  FAILED to connect to IMU at 0x");
       Serial.println(imu_address, HEX);
       Serial.println("  Continuing anyway...");
    }
    else
    {
       Serial.print("  SUCCESS: Connected to IMU at 0x");
       Serial.println(imu_address, HEX);
    }
    
    imu_address = imu_address - 1;
    delay(100);
  }
  
  Serial.println("Enabling rotation Vectors on IMUs");
  // Set IMU's update to 5ms
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    if (imu_success[k]) {
      imu_array[k].enableRotationVector(5);
    }
    delay(10); 
  }

  // Print finished
  Serial.println("Setup finished\n----------------------\n");
}

void loop()
{

  // Start the loop timer
  long start_time = micros();

  // If wifi is available - Check if there is a UDP packet
  if (ENABLE_WIFI)
  {
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {

      // Print the package size and ip address and port
      // Serial.println("Received packet of size (" +  String(Udp.remoteIP()) + ") from " +  String(Udp.remotePort()) + ":");

      // Read the packet into packetBufffer
      int len = Udp.read(packetBuffer, SOCKET_BUFFER_UDP_SIZE);
      if (len > 0) packetBuffer[len] = 0;
      // Serial.println("Contents:");
      // Serial.println(packetBuffer);

      // EXPECTED PACKET: "r1,r2,r3,r4,t1,t2,t3,t4"
      // Split and parse
      char *token;
      char *ptr = packetBuffer;
      
      // Parse Roll Setpoints 1-4
      for(int i=0; i<4; i++) {
        token = strtok_r(ptr, ",", &ptr); // Use ptr for first call? No, strtok_r logic is tricky.
        // Re-implementing simplified parsing logic to be safe or use existing sub_str helper
        // But sub_str copies string every time. Let's stick to sub_str for now to match style, but extended.
      }
      
      // Re-parsing logic using sub_str approach for safety matching existing code
      // We expect 8 values now.
      char *m1 = sub_str(packetBuffer, 1); // r1
      char *m2 = sub_str(packetBuffer, 2); // r2
      char *m3 = sub_str(packetBuffer, 3); // r3
      char *m4 = sub_str(packetBuffer, 4); // r4
      char *m5 = sub_str(packetBuffer, 5); // t1
      char *m6 = sub_str(packetBuffer, 6); // t2
      char *m7 = sub_str(packetBuffer, 7); // t3
      char *m8 = sub_str(packetBuffer, 8); // t4

      if (m1 && m2 && m3 && m4 && m5 && m6 && m7 && m8) {
        // Arm 1 & 2 (Local)
        roll_setpoint[0] = max(min(atoi(m1), ROLL_MAX), ROLL_MIN);
        roll_setpoint[1] = max(min(atoi(m2), ROLL_MAX), ROLL_MIN);
        motor_throttle[0] = max(min(atoi(m5), MOTOR_SPEED_MAX), MOTOR_SPEED_MIN);
        motor_throttle[1] = max(min(atoi(m6), MOTOR_SPEED_MAX), MOTOR_SPEED_MIN);

        // Arm 3 & 4 (Remote/Slave)
        roll_setpoint[2] = max(min(atoi(m3), ROLL_MAX), ROLL_MIN);
        roll_setpoint[3] = max(min(atoi(m4), ROLL_MAX), ROLL_MIN);
        motor_throttle[2] = max(min(atoi(m7), MOTOR_SPEED_MAX), MOTOR_SPEED_MIN);
        motor_throttle[3] = max(min(atoi(m8), MOTOR_SPEED_MAX), MOTOR_SPEED_MIN);

        // Forward to Slave via Serial1
        // Format: "r3,r4,t3,t4\n"
        String slaveCmd = String(roll_setpoint[2]) + "," + 
                          String(roll_setpoint[3]) + "," + 
                          String(motor_throttle[2]) + "," + 
                          String(motor_throttle[3]) + "\n";
        Serial1.print(slaveCmd);
        
        // Serial.println("Local: " + String(roll_setpoint[0]) + "," + String(roll_setpoint[1]));
        // Serial.println("Fwd: " + slaveCmd);
      }

      // Send a reply
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write("Cmd Ack");
      Udp.endPacket();
    }
  }

  // Get the latest IMU data (Local Arms)
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    if (imu_array[k].dataAvailable() == true)
    {
      // Get the current roll
      current_roll[k] = (imu_array[k].getRoll()) * 180.0 / PI; 
      // Keep track of the global roll value
      global_roll[k] = update_global_roll(current_roll[k], previous_roll[k], &roll_state[k]);
      // Update the previous roll
      previous_roll[k] = current_roll[k];
    }
  }

  // Loop through the LOCAL motors
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    // Compute the roll output
    float motor_power = pid[k].get_output(roll_setpoint[k], global_roll[k]);

    // Get the sign of the motor_power
    int motor_direction = (motor_power > 0) - (motor_power < 0);
    motor_power = min(abs(motor_power), ROTATION_MAX_SPEED);

    // Check if the motor power is less than a threshold
    if (motor_power < ROTATION_MIN_SPEED)
    {
      motor_power = 0;
      motor_direction = 0;
    }

    // Validate the safe rotation
    if (!rotation_within_bounds(global_roll[k], motor_direction))
    {
      motor_power = 0;
    }

    // Set the motor to rotate
    set_motor(motor_direction, motor_power, pwn_pin[k], direction_pin1[k], direction_pin2[k]);

  }
  
  // Sets the motor throttle
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    esc_array[k].write(motor_throttle[k]);
  }
  
  // Maintain the loop rate
  while ((micros() - start_time) <= (dt * 1.0e6))
  {
    ;
  }
}

void printWiFiStatus() 
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// Control the spinning motors
// rotation_direction --> Direction motors spin in, either 1 or -1
//pwm                 --> The pwm value sent to the motor (higher == rotate faster)
void set_motor(int rotation_direction, int pwm, int pwn_pin, int dir_pin1, int dir_pin2)
{
  analogWrite(pwn_pin, pwm);
  if(rotation_direction == 1){
    digitalWrite(dir_pin1, HIGH);
    digitalWrite(dir_pin2, LOW);
  }
  else if(rotation_direction == -1){
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, HIGH);
  }
  else{
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, LOW); 
  }  
}

// Segment the strings
char* sub_str (char* input_string, int segment_number) 
{
  char *act, *sub, *ptr;
  static char copy[100];
  int i;
 
  strcpy(copy, input_string);
  for (i = 1, act = copy; i <= segment_number; i++, act = NULL) 
  {
    sub = strtok_r(act, ",", &ptr);
    if (sub == NULL) break;
  }
  return sub;
}

// Computes the global roll value (i.e removes the 180 discontinuty)
double update_global_roll(double cur_reading, double prev_reading, int* state)
{
  int current_state = *state;
  double diff = cur_reading - prev_reading;
  // If the sign has changed
  if (abs(diff) > 180)
  {
    if (diff > 0)
    {
      // Update the state
      current_state = current_state - 1;
    }
    else
    {
      // Update the state
      current_state = current_state + 1;
    }
  }
  // Save the state
  *state = current_state;

  float global_roll = (current_state * 360) + cur_reading;

  return global_roll;
}

// Need to write a verify safety function here
bool rotation_within_bounds(double current_roll, double current_direction)
{
  bool safe = true;
  if ((current_roll < ROLL_MIN) and (current_direction <= -1))
  {
    safe = false;
  }
  if ((current_roll > ROLL_MAX) and (current_direction >= 1))
  {
    safe = false;
  }
  return safe;
}

