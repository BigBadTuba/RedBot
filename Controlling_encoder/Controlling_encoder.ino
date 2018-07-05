/*
 * Controller using the encoder on the redbot to make it follow a straight line.
 * Current status: Working Ok. 
 */

#include <RedBot.h>
RedBotMotors motors;
RedBotAccel accelerometer;
RedBotEncoder encoder = RedBotEncoder(A2, 10);  // initializes encoder on pins A2 and 10
String command;
String direction;

int data;  // variable for holding incoming data from PC to Arduino
int motorSpeed = 100;
float rightSpeed = motorSpeed;
float leftSpeed = motorSpeed;

int lCount;
int rCount;
int countsPerRev = 192; // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev

bool regulator = false;
float Kp = 1;
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Ready for command!");
  Serial.setTimeout(100);
}

void loop(void)
{
  // If there is data available, read it
  if (Serial.available() > 0) {
    command = Serial.readString();
  }


  if (command != "") {
    Serial.print("Received: ");
    Serial.println(command);
    command.toLowerCase();

    if (command == "w") {
      leftSpeed = -abs(leftSpeed);
      rightSpeed = abs(rightSpeed);

      encoder.clearEnc(BOTH);
      motors.leftMotor(leftSpeed);
      motors.rightMotor(rightSpeed);
      direction = "forward";
    } else if (command == "a") {

      encoder.clearEnc(BOTH);
      motors.leftMotor(leftSpeed);
      motors.rightMotor(rightSpeed);
      direction = "left";
    } else if (command == "d") {

      encoder.clearEnc(BOTH);
      motors.leftMotor(-motorSpeed);
      motors.rightMotor(-motorSpeed);
      direction = "right";
    } else if (command == "s") {
      leftSpeed = abs(leftSpeed);
      rightSpeed = -abs(rightSpeed);

      encoder.clearEnc(BOTH);
      motors.leftMotor(leftSpeed);
      motors.rightMotor(rightSpeed);
      direction = "backward";
    } else if (command == "x") {
      encoder.clearEnc(BOTH);
      motors.brake();
      direction = "still";
      leftSpeed = motorSpeed;
      rightSpeed = motorSpeed;
    } else if (command == "reg") {
      if (regulator) {
        regulator = false;
        Serial.println("Regulator turned off");
      } else {
        regulator = true;
        Serial.println("Regulator turned on");
      }
    } else if (command == "kp") {
      Serial.print("Kp is now: ");
      Serial.println(Kp);
      while (Serial.available() == 0) {
        delay(1);
      }
      Kp = Serial.parseFloat();
      Serial.print("Kp is set to: ");
      Serial.println(Kp);
    } else {
      Serial.print("Command: ");
      Serial.print(command);
      Serial.println(" not found!");
    }

    command = "";
  }

  // Regulator, if moving foward or backward make sure to follow the right angle around z-axis
  if (direction == "forward" || direction == "backward") {
    // We want the robot move forwards => encoder value from left wheel = value from right wheel
    // i.e. reference value of 0
    lCount = encoder.getTicks(LEFT);    // read the left motor encoder
    rCount = encoder.getTicks(RIGHT);   // read the right motor encoder

    Serial.print("lCount: ");
    Serial.println(lCount);
    Serial.print("rCount: ");
    Serial.println(rCount);
    float error = lCount-rCount;

    Serial.print("Error: ");
    Serial.println(error);
    if (regulator) {

      if (error < 0) {
        // Turn right
        Serial.print("leftSpeed: ");
        Serial.println(leftSpeed + Kp * error);
        motors.leftMotor(leftSpeed + Kp * error);
        Serial.print("rightSpeed: ");
        Serial.println(rightSpeed + Kp * error);
        motors.rightMotor(rightSpeed + Kp * error);
      } else if (error > 0) {
        // Turn left
        Serial.print("leftSpeed: ");
        Serial.println(leftSpeed + Kp * error);
        motors.leftMotor(leftSpeed + Kp * error);
        Serial.print("rightSpeed: ");
        Serial.println(rightSpeed + Kp * error);
        motors.rightMotor(rightSpeed + Kp * error);
      } else {
        motors.leftMotor(leftSpeed);
        motors.rightMotor(rightSpeed);
      }

      delay(100);
    }
  }
}
