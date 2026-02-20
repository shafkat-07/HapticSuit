#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <Servo.h>
#include <Wire.h>

// How many motors
#define NMOTORS 2

int status = WL_IDLE_STATUS;
//#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[]     = "ViconNetwork";   
char pass[]     = "LessLabUVA1234";  
int keyIndex = 0;            // your network key Index number (needed only for WEP)

unsigned int localPort = 8888;      // local port to listen on

char packetBuffer[SOCKET_BUFFER_UDP_SIZE + 1]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

WiFiUDP Udp;

Servo esc_1;
Servo esc_2;
int throttle1 = 0;
int throttle2 = 0;
int timer = 0;

//70 rpm, around 1:150 gear ratio
const int PPR = 1050;

int angle[NMOTORS] = {0,0};

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU1; //Open I2C ADR jumper - goes to address 0x4B
BNO080 myIMU2; //Closed I2C ADR jumper - goes to address 0x4A

// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umin, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umin(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float uminIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umin = uminIn, umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(float target, float deltaT, int &pwr, int &dir){
    // error
    int e = target;
    
//    Serial.print("target ");
//    Serial.print(target);
//    Serial.print(",");
//    
//    Serial.print("e ");
//    Serial.print(e);
//    Serial.print(",");
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }

    pwr = map(pwr, 0, umax, 0, umax-umin) + umin;
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  
    // store previous error
    eprev = e;

//    Serial.print("pwr ");
//    Serial.print(pwr);
//    Serial.print(",");
  }
  
};


//pins
const int pwm[] = {5,11};
const int in1[] = {12,14};
const int in2[] = {6,13};

// Globals
long prevT = 0;

float roll[] = {0.0, 0.0};


// PID class instances
SimplePID pid[NMOTORS];

void setup() {
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);

  analogWriteResolution(8);

  esc_1.attach(9, 1000, 2000);
  esc_2.attach(10, 1000, 2000);

  esc_1.write(0);
  esc_2.write(0);
  
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
//    status = WiFi.begin(ssid);
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWiFiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);

  Wire.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  for(int k = 0; k < NMOTORS; k++){
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);

//    pid[0].setParams(4,0.2,0,50,255);
//    pid[1].setParams(2,0.1,0,20,255);
    
//    pid[0].setParams(6,0.108,0,0,255);
//    pid[1].setParams(4,0.072,0,0,255);

    pid[0].setParams(4,0.072,0,0,255);
    pid[1].setParams(2,0.036,0,0,255);
  }
  Serial.println("here0\n");

  //Start 2 sensors
  myIMU1.begin(0x4B);
  myIMU2.begin(0x4A);

  Serial.println("here1\n");
  myIMU1.enableRotationVector(5); //Send data update every 5ms
  myIMU2.enableRotationVector(5); //Send data update every 5ms
  Serial.println("here2\n");
}

void loop() {

  Serial.println("loop\n");

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, SOCKET_BUFFER_UDP_SIZE);
    if (len > 0) packetBuffer[len] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    char *m1 = subStr(packetBuffer, 1);
    char *m2 = subStr(packetBuffer, 2);
    char *m3 = subStr(packetBuffer, 3);
    char *m4 = subStr(packetBuffer, 4);

    Serial.println("Values:");
    Serial.println(m1);
    Serial.println(m2);
    Serial.println(m3);
    Serial.println(m4);
    
    throttle1 = atoi(&m1[0]);
    throttle2 = atoi(&m2[0]);
    angle[0] = atoi(&m3[0]);
    angle[1] = atoi(&m4[0]);

    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    //Udp.write(ReplyBuffer);
    Udp.write("m1: ");
    Udp.print(throttle1);
    Udp.write("\t m2: ");
    Udp.print(throttle2);
    Udp.write("\t m3: ");
    Udp.print(angle[0]);
    Udp.write("\t m4: ");
    Udp.println(angle[1]);
    Udp.endPacket();
  }

  //Look for reports from the IMU1
  if (myIMU1.dataAvailable() == true)
  {
    roll[0] = (myIMU1.getRoll()) * 180.0 / PI; // Convert roll to degrees
  }

  if (myIMU2.dataAvailable() == true)
  {
    roll[1] = (myIMU2.getRoll()) * 180.0 / PI; // Convert roll to degrees
  }

   Serial.println(String("roll[1]:\t") + String(roll[1], 2) + "\n"); 

  // set target position
  float target[NMOTORS];
  for(int k=0; k<NMOTORS; k++){
    target[k] = map (angle[k], 0, 180, 0, PPR) - map (roll[k], 0, 180, 0, PPR);
  }

  
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(target[k],deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
  }

//  Serial.println(throttle1);
//  Serial.println(throttle2);
  esc_1.write(throttle1);
  esc_2.write(throttle2);

//  Serial.print(millis());
//  Serial.print(", ");
//  Serial.print("roll1 ");
//  Serial.print(roll[0]);
//  Serial.print(", ");
//  Serial.print("roll2 ");
//  Serial.println(roll[1]);
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// pwm -> pin for outputting speed
// in1 -> controls direction (high, low == clockwise); (low, high  == anticlockwise); (low, low  == stop)
// in2 -> controls direction
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW); 
  }  
}

char* subStr (char* input_string, int segment_number) 
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
