/*
   Controller using the encoder on the redbot to make it follow a straight line.
   Current status: Working Ok.
*/
// Uncomment if debug should be used
//#define DEBUG

#include <RedBot.h>
#include <AltSoftSerial.h>
#include <Wire.h> // Must include Wire library for I2C
#include <SparkFun_MMA8452Q.h> // Includes the SFE_MMA8452Q library

//AltSoftSerial ASSserial;
RedBotSoftwareSerial ASSserial;

// Variables used for incoming data
const byte maxDataLength = 20;
char BT_command[21] ;
boolean newCommand = false;
bool connected_to_gui = false;
bool data_read = true;

RedBotMotors motors;
MMA8452Q accel; // Accelerometer
float x_offset;
float y_offset;
float z_offset;

RedBotEncoder encoder = RedBotEncoder(A2, 10);  // initializes encoder on pins A2 and 10
String direction;

int motorSpeed = 200; // Motorspeed, 0-255 (low-high)
float rightSpeed = motorSpeed;
float leftSpeed = motorSpeed;

// Timer variables
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 2000; // Time between each consecutive transmission of data
const unsigned long timeout = 30000; // Timeout for when the connection is assumed lost
int average_constant = 10; // amount of samples used in the average

float lCount; // number of rotations for each wheel
float rCount;
// not used atm: int countsPerRev = 192; // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev

bool regulator = true;
float Kp = 1; // P-controller parameter

// Buzzer
const int buzzerPin = 9;

#ifdef DEBUG
#define DEBUG_PRINTLN(String) Serial.println(String)
#define DEBUG_PRINT(String) Serial.print(String)
#else
#define DEBUG_PRINT(String)
#define DEBUG_PRINTLN(String)
#endif

void setup(void)
{
  Serial.begin(9600);
  ASSserial.begin(9600);
  DEBUG_PRINTLN("Ready for command!");

  // INPUT_PULLUP defaults it to HIGH.
  pinMode(buzzerPin, OUTPUT);  // configures the buzzerPin as an OUTPUT
  startMillis = millis();

  accel.init(SCALE_2G);  

  calculateAccelOffset();
}

void loop(void)
{

  recvWithStartEndMarkers();
  if (newCommand) {
    DEBUG_PRINTLN("New command");
    processCommand();
  }

  if (regulator) {
    controller();
  }

  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)

  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    // Only send data if gui has sent confirmation of connection
    if (connected_to_gui && data_read) {
      DEBUG_PRINTLN("Start transmission");
      startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
      sendDataToGUI();
    }
  }
}

/*
   Processes and command that are received
*/
void processCommand() {
  DEBUG_PRINT("Received: ");
  DEBUG_PRINTLN(BT_command);

  // This will only be sent once, when the gui has sucessfully conencted => meaning data can be sent back
  if (strcmp(BT_command, "CONN_OK") == 0) {
    connected_to_gui = true;
    data_read = true;
    // Indicate with buzzer that gui is connected
    tone(buzzerPin, 1000);
    delay(500);
    noTone(buzzerPin);

    // Update timer
    startMillis = millis();
  } else if (strcmp(BT_command, "CONN_BREAK") == 0) {
    connected_to_gui = false;
  } else if (strcmp(BT_command, "DATA_OK") == 0) { // if transmitted data was received
    data_read = true;
  } else if (strcmp(BT_command, "W") == 0) {
    DEBUG_PRINTLN("W found");
    leftSpeed = -abs(motorSpeed);
    rightSpeed = abs(motorSpeed);

    encoder.clearEnc(BOTH);
    motors.leftMotor(leftSpeed);
    motors.rightMotor(rightSpeed);
    direction = "forward";
  } else if (strcmp(BT_command, "A") == 0) {
    leftSpeed = abs(motorSpeed);
    rightSpeed = abs(motorSpeed);

    encoder.clearEnc(BOTH);
    motors.leftMotor(leftSpeed);
    motors.rightMotor(rightSpeed);
    direction = "left";
  } else if (strcmp(BT_command, "D") == 0) {
    leftSpeed = -abs(motorSpeed);
    rightSpeed = -abs(motorSpeed);

    encoder.clearEnc(BOTH);
    motors.leftMotor(leftSpeed);
    motors.rightMotor(rightSpeed);
    direction = "right";
  } else if (strcmp(BT_command, "S") == 0) {
    leftSpeed = abs(motorSpeed);
    rightSpeed = -abs(motorSpeed);

    encoder.clearEnc(BOTH);
    motors.leftMotor(leftSpeed);
    motors.rightMotor(rightSpeed);
    direction = "backward";
  } else if (strcmp(BT_command, "STOP") == 0) {
    encoder.clearEnc(BOTH);
    motors.coast();
    direction = "still";
    leftSpeed = motorSpeed;
    rightSpeed = motorSpeed;
  } else if (strcmp(BT_command, "reg") == 0) {
    if (regulator) {
      regulator = false;
      DEBUG_PRINTLN("Regulator turned off");
    } else {
      regulator = true;
      DEBUG_PRINTLN("Regulator turned on");
    }
  } else if (strcmp(BT_command, "kp") == 0) {
    DEBUG_PRINT("Kp is now: ");
    DEBUG_PRINTLN(Kp);
    while (Serial.available() == 0) {
      delay(1);
    }
    Kp = Serial.parseFloat();
    DEBUG_PRINT("Kp is set to: ");
    DEBUG_PRINTLN(Kp);
  } else {
    DEBUG_PRINT("Command: ");
    DEBUG_PRINT(BT_command);
    DEBUG_PRINTLN(" not found!");
  }

  //  DEBUG_PRINTLN("Command processed");
  BT_command[0] = '\0';
  newCommand = false;

  return;
}

/*
    Regulator, if moving foward or backward make sure to follow the right angle around z-axis
*/
void controller() {
  if (direction == "forward" || direction == "backward") {
    // We want the robot move forwards => encoder value from left wheel = value from right wheel
    // i.e. reference value of 0
    lCount = encoder.getTicks(LEFT);    // read the left motor encoder
    rCount = encoder.getTicks(RIGHT);   // read the right motor encoder

    DEBUG_PRINT("lCount: ");
    DEBUG_PRINTLN(lCount);
    DEBUG_PRINT("rCount: ");
    DEBUG_PRINTLN(rCount);
    float error = lCount - rCount;

    DEBUG_PRINT("Error: ");
    DEBUG_PRINTLN(error);
    if (regulator) {

      if (error < 0) {
        // Turn right
        DEBUG_PRINT("leftSpeed: ");
        DEBUG_PRINTLN(leftSpeed + Kp * error);
        motors.leftMotor(leftSpeed + Kp * error);
        DEBUG_PRINT("rightSpeed: ");
        DEBUG_PRINTLN(rightSpeed + 1.2 *  Kp * error);
        motors.rightMotor(rightSpeed + Kp * error);
      } else if (error > 0) {
        // Turn left
        DEBUG_PRINT("leftSpeed: ");
        DEBUG_PRINTLN(leftSpeed + Kp * error);
        motors.leftMotor(leftSpeed + Kp * error);
        DEBUG_PRINT("rightSpeed: ");
        DEBUG_PRINTLN(rightSpeed + 1.2 * Kp * error);
        motors.rightMotor(rightSpeed + Kp * error);
      } else {
        motors.leftMotor(leftSpeed);
        motors.rightMotor(rightSpeed);
      }

      //delay(100);
    }
  }
}

// function recvWithStartEndMarkers by Robin2 of the Arduino forums
// See  http://forum.arduino.cc/index.php?topic=288234.0
/*
****************************************
  Function recvWithStartEndMarkers
  reads serial data and returns the content between a start marker and an end marker.

  passed:

  global:
        receivedChars[]
        newData

  Returns:

  Sets:
        newData
        receivedChars

*/
void recvWithStartEndMarkers()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '[';
  char endMarker = ']';
  char rc;

  if (ASSserial.available() > 0)
  {
    DEBUG_PRINTLN("Receiving data");
    rc = ASSserial.read();
    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        BT_command[ndx] = rc;
        ndx++;
        if (ndx > maxDataLength) {
          ndx = maxDataLength;
        }
      }
      else
      {
        BT_command[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newCommand = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

/*
   Sends measurement data to the GUI over bluetooth
 * */
void sendDataToGUI() {
  // Read raw data values from the accelerometer
  float x;
  float y;
  float z;

  // Update accelerometer values, average of predetermined amount of samples
  for (int i = 0; i < average_constant; i++) {
      accel.read();
      x += accel.cx;
      y += accel.cy;
      z += accel.cz;
      delay(1); // Small delay
  }

  x = x / average_constant - x_offset;
  y = y / average_constant - y_offset;
  z = z / average_constant - z_offset;

  DEBUG_PRINT("Sending data: ");
  //String data = String(x) + String(y) + String(z);
  String data = String(x) + " " + String(y) + " " + String(z);
  DEBUG_PRINTLN(data);
  char char_data[data.length()];
  // Convert to char array, + 1 size for line-end char '\0'
  data.toCharArray(char_data, data.length() + 1);
  for (int i = 0; i < data.length(); i++) {
    //DEBUG_PRINTLN(i);
    if (char_data[i] != 10 && char_data[i] != 13) {
      //DEBUG_PRINTLN(char_data[i]);
      ASSserial.write(char_data[i]);
    }
  }

  data_read = false;
}

void calculateAccelOffset(){
  int calc_constant = 1000;
  x_offset = 0;
  y_offset = 0;
  z_offset = 0;
  
  for(int i = 0; i < calc_constant; i++){
    accel.read();
    x_offset += accel.cx;
    y_offset += accel.cy;
    z_offset += accel.cz;
    delay(1);
  }

  x_offset = x_offset/calc_constant; // X is forward
  y_offset = y_offset/calc_constant; // Y is left
  z_offset = z_offset/calc_constant - 1; // Z is upward => 1 g 

  DEBUG_PRINT("OFFSET: ");
  DEBUG_PRINT(x_offset);
  DEBUG_PRINT(" ");
  DEBUG_PRINT(y_offset);
  DEBUG_PRINT(" ");
  DEBUG_PRINTLN(z_offset);
}

