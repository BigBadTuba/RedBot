/*
   Controller using the encoder on the redbot to make it follow a straight line.
   Current status: Working Ok.
*/

#include <RedBot.h>
#include <AltSoftSerial.h>

//AltSoftSerial ASSserial;
RedBotSoftwareSerial ASSserial;

// Variables used for incoming data
const byte maxDataLength = 20;
char BT_command[21] ;
boolean newCommand = false;

RedBotMotors motors;
RedBotAccel accelerometer;
RedBotEncoder encoder = RedBotEncoder(A2, 10);  // initializes encoder on pins A2 and 10
String direction;

int data;  // variable for holding incoming data from PC to Arduino
int motorSpeed = 200;
float rightSpeed = motorSpeed;
float leftSpeed = motorSpeed;

float lCount;
float rCount;
int countsPerRev = 192; // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev

bool regulator = true;
float Kp = 1;

// Comment in if debug should be used
//#define DEBUG

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

}

/*
   Processes and command that are received
*/
void processCommand() {
  DEBUG_PRINT("Received: ");
  DEBUG_PRINTLN(BT_command);

  if (strcmp(BT_command, "W") == 0) {
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
