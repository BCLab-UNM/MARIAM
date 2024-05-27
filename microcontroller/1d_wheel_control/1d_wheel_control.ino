#include <Movement.h>
#include <BasicEncoder.h>
#include <TimerOne.h>

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
float wheelRatio = 1.0 ;//1.01; //ratio between left and right wheel circumference
BasicEncoder leftEncoder(leftEncoderA,leftEncoderB);
BasicEncoder rightEncoder(rightEncoderA,rightEncoderB);

void timer_service() {
  leftEncoder.service();
  rightEncoder.service();
  Serial.print(leftEncoder.get_count());
  Serial.print(" ");
  Serial.println(rightEncoder.get_count());
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
    int speed = Serial.parseInt();
    if (speed > 0)       { move.forward(speed, speed*wheelRatio); }
    else if (speed < 0 ) { move.backward(-speed, -speed*wheelRatio); } 
    else                 { move.stop(); }
  } //end Serial connected
  
} //end loop
