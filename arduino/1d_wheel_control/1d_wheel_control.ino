#include <Movement.h>

//Movement (VNH5019 Motor Driver Carrier)
byte rightDirectionA = A3; //"clockwise" input
byte rightDirectionB = A2; //"counterclockwise" input
byte rightSpeedPin = 11; //PWM input
byte leftDirectionA = A5; //"clockwise" input
byte leftDirectionB = A4; //"counterclockwise" input
byte leftSpeedPin = 10; //PWM input
Movement move = Movement(rightSpeedPin, rightDirectionA, rightDirectionB, leftSpeedPin, leftDirectionA, leftDirectionB);

void setup(){
  Serial.begin(115200);
  while (!Serial) {} //wait for Serial to connect
} //end setup

void loop() {
  if (Serial.available()) {
    int speed = Serial.parseInt();
    if (speed > 0)       { move.forward(speed, speed); }
    else if (speed < 0 ) { move.backward(-speed, -speed); } 
    else                 { move.stop(); }
  } //end Serial connected
} //end loop
