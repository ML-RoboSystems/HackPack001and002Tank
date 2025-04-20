#define IR_USE_AVR_TIMER2 // overrides conflicting pins in IRremote.hpp
#include "IRremote.hpp"
#include "SparkFun_TB6612.h"
#include <Arduino.h>
#include <Servo.h>
#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.
#include "IRremote.hpp"
#include "IRVersion.h"
#include "IRremoteInt.h"
#include "IRProtocol.h"
#include "digitalWriteFast.h"
#include "private/IRTimer.hpp"
#include "IRFeedbackLED.hpp"
#include "LongUnion.h"
#include "IRProtocol.hpp"
#include "IRReceive.hpp"
#include <stdlib.h>
#include <time.h>

// #define IR_LEFT 0xF708FF00
// #define IR_RIGHT 0xA55AFF00
// #define IR_FORWARD 0xE718FF00
// #define IR_HOLD 0
// #define IR_BACKWARD 0xAD52FF00
// #define IR_ONE 0xBA45FF00
// #define IR_TWO 0xB946FF00
// #define IR_OK 3810328320
// #define IR_SEVEN 4161273600
// #define IR_EIGHT 3927310080

#define DECODE_NEC  //defines the type of IR transmission to decode based on the remote. See IRremote library for examples on how to decode other types of remote

//defines the specific command code for each button on the remote
#define IR_LEFT 4144561920
#define IR_RIGHT 2774204160
#define IR_FORWARD 3877175040
#define IR_BACKWARD 2907897600
#define IR_OK 3810328320
#define IR_ONE 3125149440
#define IR_TWO 3108437760
#define IR_THREE 3091726080
#define IR_FOUR 3141861120
#define IR_FIVE 3208707840
#define IR_SIX 3158572800
#define IR_SEVEN 4161273600
#define IR_EIGHT 3927310080
#define IR_NINE 4127850240
#define IR_ZERO 3860463360
#define IR_STAR 3910598400
#define IR_HASHTAG 4061003520


#define PWMA 8  // Adjust as necessary
#define PWMB 2  // Adjust as necessary
#define AIN1 6  // Adjust as necessary
#define BIN1 4  // Adjust as necessary
#define AIN2 7  // Adjust as necessary
#define BIN2 3  // Adjust as necessary

#define OFFSET_A 1         // Adjust if necessary
#define OFFSET_B 1         // Adjust if necessary
#define STBY 5
#define TOP_SPEED 120       // Define top speed
#define SERVO_RIGHT       25                                //PWM value to make the servo move right
#define SERVO_LEFT        180                               //PWM value to make the servo move left.
#define SERVO_PIN         13                                //PWM pin for domino dispensing servo
#define SERVO_MIN_PULSE 650  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSE 2400 // Maximum pulse width in microseconds
#define DISPENSE_DISTANCE 5000 //Value combinedMotorSpeed needs to reach to drop a domino. Decreasing this puts dominoes closer together.
#define PULSE_INTERVAL 20000 // Interval between pulses in milliseconds
#define IR_Input 3
//////////////////////////////////////////////////
          //  PINS AND PARAMETERS  //
//////////////////////////////////////////////////
Servo yawServo; //names the servo responsible for YAW rotation, 360 spin around the base
Servo pitchServo; //names the servo responsible for PITCH rotation, up and down tilt
Servo rollServo; //names the servo responsible for ROLL rotation, spins the barrel to fire darts

int yawServoVal = 90; //initialize variables to store the current value of each servo
int pitchServoVal = 100;
int rollServoVal = 90;

int lastYawServoVal = 90; //initialize variables to store the last value of each servo
int lastPitchServoVal = 90; 
int lastRollServoVal = 90;

int pitchMoveSpeed = 7; //this variable is the angle added to the pitch servo to control how quickly the PITCH servo moves - try values between 3 and 10
int yawMoveSpeed = 50; //this variable is the speed controller for the continuous movement of the YAW servo motor. It is added or subtracted from the yawStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Try values between 10 and 90;
int yawStopSpeed = 90; //value to stop the yaw motor - keep this at 90
int rollMoveSpeed = 90; //this variable is the speed controller for the continuous movement of the ROLL servo motor. It is added or subtracted from the rollStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Keep this at 90 for best performance / highest torque from the roll motor when firing.
int rollStopSpeed = 90; //value to stop the roll motor - keep this at 90

int yawPrecision = 150; // this variable represents the time in milliseconds that the YAW motor will remain at it's set movement speed. Try values between 50 and 500 to start (500 milliseconds = 1/2 second)
int rollPrecision = 158; // this variable represents the time in milliseconds that the ROLL motor with remain at it's set movement speed. If this ROLL motor is spinning more or less than 1/6th of a rotation when firing a single dart (one call of the fire(); command) you can try adjusting this value down or up slightly, but it should remain around the stock value (270) for best results.

int pitchMax = 175; // this sets the maximum angle of the pitch servo to prevent it from crashing, it should remain below 180, and be greater than the pitchMin
int pitchMin = 10; // this sets the minimum angle of the pitch servo to prevent it from crashing, it should remain above 0, and be less than the pitchMax

bool servoRight = true;
bool lastServoRight = false;

// Initialize motors
Motor motor1 = Motor(AIN1, AIN2, PWMA, OFFSET_A, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, OFFSET_B, STBY);

unsigned long combinedmotorspeed;
int leftMotorSpeed, rightMotorSpeed;    //creating variables like this initializes all their values to 0.

Servo servoMotor;                                           //Create an instance of a servo motor object

bool raceMode = true;
unsigned long speedBoost = 0;

unsigned long lastValidCommand = 0; // This seems to be more reliable than checking if it is holding, which static interferes with. 

unsigned long lastCommandTime = 0;
unsigned long lastServoTime = 0;

const unsigned long commandTimeout = 250; // Timeout in milliseconds

unsigned long distanceSinceLastDrop = DISPENSE_DISTANCE;    //Starting with this as DISPENSE_DISTANCE makes the vehicle drop a domino immediately.
bool dominoDropped = false;

const int RECV = IR_Input;
//IRrecv irrecv(RECV);

//////////////////////////////////////////////////
                //  S E T U P  //
//////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  //irrecv.enableIRIn();
  yawServo.attach(10); //attach YAW servo to pin 3
  pitchServo.attach(11); //attach PITCH servo to pin 4
  rollServo.attach(12); //attach ROLL servo to pin 5

  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

  // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
  IrReceiver.begin(9, ENABLE_LED_FEEDBACK);

  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.println(F("at pin " STR(9)));
homeServos();
  // IrReceiver.begin(RECV, DISABLE_LED_FEEDBACK); //Version 4.x receiver initializer. 4.x uses IrReceiver instead of irrecv.
  IrReceiver.enableIRIn();

  digitalWrite(STBY, HIGH);  // Enable the motor driver
  pinMode(SERVO_PIN, OUTPUT);
  servoRight = true;

  // Send power down the blue wire, D2, to the IR sensor
  // pinMode(A5, OUTPUT);
  // digitalWrite(3, HIGH);

}

////////////////////////////////////////////////
                //  L O O P  //
////////////////////////////////////////////////

void loop() {
  delay(20); // Only write new values every so often so we can tell the placement more consistantly

  bool commandReceived = false;
  unsigned long currentMillis = millis();
  if (raceMode) {
    speedBoost = (255 - TOP_SPEED);
  } else {
    speedBoost = 0;
  }

  // Update servo if it's changed
  if (true) {

    // Serial.println(servoRight);

    // if (servoRight) {
    //   pulseServoRight();
    // } else {
    //   pulseServoLeft();
    // }

    // lastServoRight = servoRight;
  }

  bool holding = true;
  //if (irrecv.decode()) {
    if(IrReceiver.decode()) {
    commandReceived = true;
    // unsigned long irCode = irrecv.decodedIRData.decodedRawData;
    unsigned long irCode = IrReceiver.decodedIRData.decodedRawData;
    // holding = irCode == IR_HOLD;

    // // Serial.println(irrecv.decodedIRData.decodedRawData);
    // Serial.println(IrReceiver.decodedIRData.decodedRawData);
      
    // // If holding, use the last read command
    // if (holding) {
    //   irCode = lastValidCommand;
    // }

    // Update the last command time whenever a command is received
    lastCommandTime = millis();

    Serial.println(irCode);

    bool validCode = true; // true until otherwise specified

    switch (irCode) {
      case IR_FORWARD:
        // Handle up command
        upMove(1);
        break;
      case IR_LEFT:
        // Handle left command
        leftMove(1);
        break;
      case IR_RIGHT:
        // handle right command
        rightMove(1);
        break;
      case IR_BACKWARD:
        // Handle down command
        downMove(1);
        break;
      case IR_OK:
        // Handle fire command
        fire();
        Serial.println("FIRE");
        break;
      case IR_ONE:
        //Nothing happens
        break;
      case IR_TWO:
        driveForward();
        break;
      case IR_THREE:
        //Nothing happens
        break;
      case IR_FOUR:
        turnLeft();
        break;
      case IR_FIVE:
        //Nothing happens
        break;
      case IR_SIX:
        turnRight();
        break;
      case IR_SEVEN:
        //Nothing happens
        break;
      case IR_EIGHT:
        driveBackward();
        break;
      case IR_NINE:
        //Nothing Happens
        break;
      case IR_ZERO:
        //Nothing Happen
        break;
      case IR_STAR:
        //Nothing Happens
        break;
      case IR_HASHTAG:
        fireAll();
        break;
      default: // If none of the above case are run, this one is
        validCode = false;
        break;
    }

    // If we found a valid code, update the lastValidCommand 
    if (!holding && validCode) {
      lastValidCommand = validCode;
    }

    //irrecv.resume(); // Prepare for the next command
    IrReceiver.resume();
  }

  combinedmotorspeed = leftMotorSpeed + rightMotorSpeed;
  distanceSinceLastDrop += combinedmotorspeed;

  // Check for timeout to detect "no command received" condition
  if (holding && !commandReceived && millis() - lastCommandTime > commandTimeout) {
    // No command received within the timeout period
    stopCar();
    holding = false;
    commandReceived = true;
  }
  
  if ((distanceSinceLastDrop >= DISPENSE_DISTANCE) && !dominoDropped && !raceMode) {              //if we've travelled far enough and a domino hasn't been dropped yet
      servoRight = true;
      distanceSinceLastDrop = 0;
      dominoDropped = true;                        //domino has been dropped
    
  } else if ((distanceSinceLastDrop >= 0.7 * DISPENSE_DISTANCE) && dominoDropped && !raceMode) {  //if a domino was dropped and we're far enough away to close the domino gate without jamming
    servoRight = false;
    dominoDropped = false;                                                           //we haven't dropped the new domino, so now this is false
  }


}

void driveForward() {
  motor1.drive(TOP_SPEED + speedBoost);
  motor2.drive(TOP_SPEED + speedBoost);
  leftMotorSpeed = TOP_SPEED;
  rightMotorSpeed = TOP_SPEED;
}

void turnLeft() {
  motor1.drive(TOP_SPEED + speedBoost);
  motor2.drive(0 - speedBoost * 1.5);
  leftMotorSpeed = 0;
  rightMotorSpeed = TOP_SPEED;
}

void turnRight() {
  motor1.drive(0 - speedBoost * 1.5);
  motor2.drive(TOP_SPEED + speedBoost);
  leftMotorSpeed = TOP_SPEED;
  rightMotorSpeed = 0;
}

void stopCar() {
  motor1.drive(0);
  motor2.drive(0);
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;
}

void driveBackward() {
  motor1.drive(-TOP_SPEED - speedBoost);
  motor2.drive(-TOP_SPEED - speedBoost);
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;
}

// void pulseServoRight() {
//   // Serial.println("PulseServoRight");
//   unsigned long startTime;
//   unsigned long pulseWidth = SERVO_MIN_PULSE; // Duration of the pulse in microseconds
//   unsigned long period = PULSE_INTERVAL; // Total period duration in microseconds (20ms)

//   // Start the pulse
//   digitalWrite(SERVO_PIN, HIGH);
//   startTime = micros(); // Record the start time

//   // Wait for the pulse width duration
//   while (micros() - startTime < pulseWidth) {
//     // This loop waits until 2400 microseconds have passed since startTime
//   }

//   // End the pulse
//   digitalWrite(SERVO_PIN, LOW);
//   // Note: at this point, the time elapsed is roughly equal to pulseWidth

//   // Wait for the rest of the period to complete the 20ms cycle
//   while (micros() - startTime < period) {
//     // This loop waits until 20000 microseconds have passed since startTime
//   }

//   // The function will return here, right after the 20ms period has finished
// }

// void pulseServoLeft() {
//   // Serial.println("PulseServoLeft");
//   unsigned long startTime;
//   unsigned long pulseWidth = SERVO_MAX_PULSE; // Duration of the pulse in microseconds
//   unsigned long period = PULSE_INTERVAL; // Total period duration in microseconds (20ms)

//   // Start the pulse
//   digitalWrite(SERVO_PIN, HIGH);
//   startTime = micros(); // Record the start time

//   // Wait for the pulse width duration
//   while (micros() - startTime < pulseWidth) {
//     // This loop waits until 2400 microseconds have passed since startTime
//   }

//   // End the pulse
//   digitalWrite(SERVO_PIN, LOW);
//   // Note: at this point, the time elapsed is roughly equal to pulseWidth

//   // Wait for the rest of the period to complete the 20ms cycle
//   while (micros() - startTime < period) {
//     // This loop waits until 20000 microseconds have passed since startTime
//   }

//   // The function will return here, right after the 20ms period has finished
// }

void leftMove(int moves){
    for (int i = 0; i < moves; i++){
        yawServo.write(yawStopSpeed + yawMoveSpeed); // adding the servo speed = 180 (full counterclockwise rotation speed)
        delay(yawPrecision); // stay rotating for a certain number of milliseconds
        yawServo.write(yawStopSpeed); // stop rotating
        delay(5); //delay for smoothness
        Serial.println("LEFT");
  }

}

void rightMove(int moves){ // function to move right
  for (int i = 0; i < moves; i++){
      yawServo.write(yawStopSpeed - yawMoveSpeed); //subtracting the servo speed = 0 (full clockwise rotation speed)
      delay(yawPrecision);
      yawServo.write(yawStopSpeed);
      delay(5);
      Serial.println("RIGHT");
  }
}

void upMove(int moves){
  for (int i = 0; i < moves; i++){
      if(pitchServoVal > pitchMin){//make sure the servo is within rotation limits (greater than 10 degrees by default)
        pitchServoVal = pitchServoVal - pitchMoveSpeed; //decrement the current angle and update
        pitchServo.write(pitchServoVal);
        delay(50);
        Serial.println("UP");
      }
  }
}

void downMove (int moves){
  for (int i = 0; i < moves; i++){
        if(pitchServoVal < pitchMax){ //make sure the servo is within rotation limits (less than 175 degrees by default)
        pitchServoVal = pitchServoVal + pitchMoveSpeed;//increment the current angle and update
        pitchServo.write(pitchServoVal);
        delay(50);
        Serial.println("DOWN");
      }
  }
}

void homeServos(){
    yawServo.write(yawStopSpeed); //setup YAW servo to be STOPPED (90)
    delay(20);
    rollServo.write(rollStopSpeed); //setup ROLL servo to be STOPPED (90)
    delay(100);
    pitchServo.write(100); //set PITCH servo to 100 degree position
    delay(100);
    pitchServoVal = 100; // store the pitch servo value
}

void fire() { //function for firing a single dart
    rollServo.write(rollStopSpeed + rollMoveSpeed);//start rotating the servo
    delay(rollPrecision);//time for approximately 60 degrees of rotation
    rollServo.write(rollStopSpeed);//stop rotating the servo
    delay(5); //delay for smoothness
}

void fireAll() { //function to fire all 6 darts at once
    rollServo.write(rollStopSpeed + rollMoveSpeed);//start rotating the servo
    delay(rollPrecision * 6); //time for 360 degrees of rotation
    rollServo.write(rollStopSpeed);//stop rotating the servo
    delay(5); // delay for smoothness
}
