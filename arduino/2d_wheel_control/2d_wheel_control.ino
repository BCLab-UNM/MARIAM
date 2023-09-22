#include <Movement.h>
#include <BasicEncoder.h>
#include <TimerOne.h>
#define PIN_INPUT     A0
#define PIN_SETPOINT  A1
#define PIN_OUTPUT    9

//Movement (VNH5019 Motor Driver Carrier)
byte rightDirectionA = A3; //"clockwise" input
byte rightDirectionB = A2; //"counterclockwise" input
byte rightSpeedPin = 11; //PWM input
byte leftDirectionA = A5; //"clockwise" input
byte leftDirectionB = A4; //"counterclockwise" input
byte leftSpeedPin = 10; //PWM input


//Odometry (8400 CPR Encoder)
byte rightEncoderA = 7;
byte rightEncoderB = 8;
byte leftEncoderA = 0;
byte leftEncoderB = 1;


Movement move = Movement(rightSpeedPin, rightDirectionA, rightDirectionB, leftSpeedPin, leftDirectionA, leftDirectionB);
BasicEncoder leftEncoder(leftEncoderA,leftEncoderB);
BasicEncoder rightEncoder(rightEncoderA,rightEncoderB);

void timer_service() {
  leftEncoder.service();
  rightEncoder.service();
  Serial.print(leftEncoder.get_change());
  Serial.print(" ");
  Serial.println(rightEncoder.get_change());
 }

void setup(){
  Serial.begin(115200);
  while (!Serial) {} //wait for Serial to connect
  rightEncoder.set_reverse();
  Timer1.initialize(50);
  Timer1.attachInterrupt(timer_service);
} //end setup

void loop() {
  if (Serial.available()) {
    int speedL = Serial.parseInt();
    int speedR = Serial.parseInt();
    
    if (speedL == 0 && speedR == 0) { move.stop(); }
    if (speedL >= 0 && speedR >= 0) { move.forward(speedL, speedR); }
    else if (speedL <= 0 && speedR <= 0) { move.backward(speedL*-1, speedR*-1); }
    else if (speedL <= 0 && speedR >= 0) { move.rotateLeft(speedL*-1, speedR); }
    else { move.rotateRight(speedL, speedR*-1); }
  } //end Serial connected
  
} //end loop
