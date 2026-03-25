#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include "SparkFun_BNO080_Arduino_Library.h" 

// Include the PID class
#include "PID.cpp"

// Define the number of motors - Slave controls 2 arms (Arm 3 & 4)
#define NUMBER_ARMS 2

// Define the IMU start address (note they go in decreasing order from this point)
#define IMU_START_ADDRESS 0x4B

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

// Define variable for communication
char packetBuffer[255]; 

// Define variables for the motors
Servo esc_array[NUMBER_ARMS];
int   roll_setpoint[NUMBER_ARMS]  = {0};
int   motor_throttle[NUMBER_ARMS] = {0};

// Create the IMU (imu1 goes to 0x4B, imu2 goes to 0x4A)
BNO080 imu_array[NUMBER_ARMS];

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

// PID class instances
PID pid[NUMBER_ARMS];

// Define setup
void setup()
{
  // Allow time for startup
  delay(1000);

  // Configure communication
  Serial.begin(115200); // Debug Serial
  Serial1.begin(115200); // Communication with Master

  // Start the setup
  delay(5000);
  Serial.println();
  Serial.println("Slave Program Setup Starting");

  // Init variables
  Serial.println("Init variables");
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    current_roll[k] = 0;
    previous_roll[k] = 0;
    global_roll[k] = 0;
    roll_state[k] = 0;
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

    // Print configuration for verification (Slave handles Arms 3 & 4)
    Serial.print("  Arm "); Serial.print(k+3); Serial.println(" Configured:");
    Serial.print("    - ESC (Throttle) on Pin "); Serial.println(9+k);
    Serial.print("    - Orientation Motor on Pins: PWM="); Serial.print(pwn_pin[k]);
    Serial.print(", DIR1="); Serial.print(direction_pin1[k]);
    Serial.print(", DIR2="); Serial.println(direction_pin2[k]);
  }
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
  Wire.setClock(100000);
  delay(10);

  // Start the IMU sensors
  Serial.println("Connecting to IMUs");
  bool imu_success[NUMBER_ARMS] = {false};

  // Do this until connection is successful
  while (imu_success[0] == false)
  {
    // Connect to the IMU
    int imu_address = IMU_START_ADDRESS;
    for(int k = 0; k < NUMBER_ARMS; k++)
    {
      delay(100);
      imu_success[k] = imu_array[k].begin(imu_address);
      imu_address = imu_address -1;
      delay(100);
    }

    if (imu_success[0] == false)
    {
      Serial.println("BNO080 not detected at 0x4B I2C address. Check your jumpers and the hookup guide. Freezing...");
    }
    else {
        Serial.println("IMU at 0x4B connected successfully.");
    }
    
    if (imu_success[1] == false && NUMBER_ARMS > 1) {
       // Optional: Print status of second IMU
       // Serial.println("BNO080 not detected at 0x4A...");
    } else if (NUMBER_ARMS > 1) {
       Serial.println("IMU at 0x4A connected successfully.");
    }

    delay(1000);
  }
  
  Serial.println("Enabling rotation Vectors on IMUs");
  // Set IMU's update to 5ms
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    imu_array[k].enableRotationVector(5);
    delay(10); 
  }

  // Print finished
  Serial.println("Setup finished\n----------------------\n");
}

void loop()
{

  // Start the loop timer
  long start_time = micros();

  // Check if there is data from Master
  if (Serial1.available())
  {
    // Read the packet until newline
    int len = Serial1.readBytesUntil('\n', packetBuffer, 254);
    
    if (len > 0)
    {
      packetBuffer[len] = 0; // Null terminate
      // Serial.println("RX: " + String(packetBuffer));

      // Expected format: r3,r4,t3,t4
      char *m1 = sub_str(packetBuffer, 1);
      char *m2 = sub_str(packetBuffer, 2);
      char *m3 = sub_str(packetBuffer, 3);
      char *m4 = sub_str(packetBuffer, 4);
      
      if(m1 != NULL && m2 != NULL && m3 != NULL && m4 != NULL) {
          // Convert to the right value and max sure it stays within the correct bounds
          roll_setpoint[0]  = max(min(atoi(&m1[0]), ROLL_MAX), ROLL_MIN);
          roll_setpoint[1]  = max(min(atoi(&m2[0]), ROLL_MAX), ROLL_MIN);
          motor_throttle[0] = max(min(atoi(&m3[0]), MOTOR_SPEED_MAX), MOTOR_SPEED_MIN);
          motor_throttle[1] = max(min(atoi(&m4[0]), MOTOR_SPEED_MAX), MOTOR_SPEED_MIN);
          
          // Serial.println("Cmd: " + String(roll_setpoint[0]) + ", " + String(roll_setpoint[1]));
          
          // Send acknowledgement back to Master (optional)
          // Serial1.println("ACK");
      }
    }
  }

  // Get the latest IMU data
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

  // Loop through the motors
  for(int k = 0; k < NUMBER_ARMS; k++)
  {
    // Compute the roll output
    float motor_power = pid[k].get_output(roll_setpoint[k], global_roll[k]);
    // Serial.println("Motor " + String(k) + " power: " + String(motor_power));

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

