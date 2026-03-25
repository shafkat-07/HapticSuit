#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "SparkFun_BNO080_Arduino_Library.h" // IMU Library enabled

// Include the PID class
#include "PID.cpp"

// Define the number of motors
#define NUMBER_ARMS 2
// Define the wifi SSID and password
#define WIFI_SSID "NU-IoT"
#define WIFI_PASSWORD "nkzvswmm"
// Define if you want to enable wifi (makes debugging easier)
#define ENABLE_WIFI true
// Define the UDP communication port
#define UDP_PORT 8888
// Define the looping frequency (HZ)
#define LOOP_RATE 100
// Define the min and max for roll and motor speed
#define ROLL_MAX 270
#define ROLL_MIN -270
#define MOTOR_SPEED_MAX 100
#define MOTOR_SPEED_MIN 0
#define SOCKET_BUFFER_UDP_SIZE 255 

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
int   roll_setpoint[NUMBER_ARMS]  = {0,0};
int   motor_throttle[NUMBER_ARMS] = {0,0};

// Create the IMU (imu1 goes to 0x4B, imu2 goes to 0x4A)
BNO080 imu_array[NUMBER_ARMS]; // Assuming you have attached both, or we test just one.

// Create variables to hold the IMU data
float current_roll[NUMBER_ARMS];
float previous_roll[NUMBER_ARMS];
float global_roll[NUMBER_ARMS];
int   roll_state[NUMBER_ARMS];

// Declare the pins
const int pwn_pin[] = {5,  11};
const int direction_pin1[] = {12, 14};
const int direction_pin2[] = {6,  13};

// Get the loop rate
const float dt = 1.0 / LOOP_RATE;

// Define setup
void setup()
{
  // Allow time for startup
  delay(1000);

  // Configure communication
  Serial.begin(115200);

  // Start the setup
  delay(5000);
  Serial.println();
  Serial.println("WiFi + IMU Test Program Setup Starting");

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
        status = WiFi.begin(WIFI_SSID);
      } 
      else
      {
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

  //Increase I2C data rate to 100kHz
  Serial.println("Increasing I2C rate");
  Wire.begin();
  Wire.setClock(100000);
  delay(10);

  // Start the IMU sensors
  Serial.println("Connecting to IMUs");
  // We will try to connect to both, but proceed even if one fails for testing purposes
  // 0x4B (Open Jumper)
  if (imu_array[0].begin(0x4B) == false)
  {
    Serial.println("BNO080 not detected at 0x4B. Check wiring or ADDR jumper.");
  } else {
    Serial.println("BNO080 detected at 0x4B!");
    imu_array[0].enableRotationVector(50); // Update every 50ms
  }
  
  // 0x4A (Closed Jumper)
  if (imu_array[1].begin(0x4A) == false)
  {
    Serial.println("BNO080 not detected at 0x4A. Check wiring or ADDR jumper.");
  } else {
    Serial.println("BNO080 detected at 0x4A!");
    imu_array[1].enableRotationVector(50); // Update every 50ms
  }

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
      Serial.println("Received packet of size (" +  String(Udp.remoteIP()) + ") from " +  String(Udp.remotePort()) + ":");

      // Read the packet into packetBufffer
      int len = Udp.read(packetBuffer, SOCKET_BUFFER_UDP_SIZE);
      if (len > 0) packetBuffer[len] = 0;
      Serial.println("Contents:");
      Serial.println(packetBuffer);

      // We just acknowledge and send back the current IMU data
      String reply = "Command Received. ";
      
      // Check IMU 0
      if (imu_array[0].dataAvailable() == true)
      {
        float roll = (imu_array[0].getRoll()) * 180.0 / PI;
        reply += "IMU(0x4B) Roll: " + String(roll, 2) + " ";
      } else {
        reply += "IMU(0x4B) No Data ";
      }

      // Check IMU 1
      if (imu_array[1].dataAvailable() == true)
      {
        float roll = (imu_array[1].getRoll()) * 180.0 / PI;
        reply += "IMU(0x4A) Roll: " + String(roll, 2);
      } else {
         reply += "IMU(0x4A) No Data";
      }
      
      Serial.println("Sending Reply: " + reply);

      // Send a reply
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(reply.c_str());
      Udp.endPacket();
    }
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
