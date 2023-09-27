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
unsigned int serviced = 0;
unsigned int serviced_limit = 100;
int speedL, speedR;


const int bufferSize = 32; // expected max buffer "-100 -100\n" //so ~3x just incase
char buffer[bufferSize];
int bufferIndex = 0;

Movement move = Movement(rightSpeedPin, rightDirectionA, rightDirectionB, leftSpeedPin, leftDirectionA, leftDirectionB);
BasicEncoder leftEncoder(leftEncoderA,leftEncoderB);
BasicEncoder rightEncoder(rightEncoderA,rightEncoderB);

void timer_service() {
  leftEncoder.service();
  rightEncoder.service();
  serviced++;
  if (serviced > serviced_limit){
    serviced=0;
    Serial.print(leftEncoder.get_change());
    Serial.print(" ");
    Serial.println(rightEncoder.get_change());
 }}

void setup(){
  Serial.begin(115200);
  while (!Serial) {} //wait for Serial to connect
  rightEncoder.set_reverse();
  Timer1.initialize(50);
  Timer1.attachInterrupt(timer_service);
} //end setup

void loop() {
  while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') { // Check for newline character
            buffer[bufferIndex] = '\0';  // null-terminate the C string
            if (sscanf(buffer, "%d %d", &speedL, &speedR) == 2) {
              if (speedL == 0 && speedR == 0) { move.stop(); }
              if (speedL >= 0 && speedR >= 0) { move.forward(speedL, speedR); }
              else if (speedL <= 0 && speedR <= 0) { move.backward(speedL*-1, speedR*-1); }
              else if (speedL <= 0 && speedR >= 0) { move.rotateLeft(speedL*-1, speedR); }
              else { move.rotateRight(speedL, speedR*-1); }
            } //emd got 2 ints
            bufferIndex = 0;  // reset the buffer
        } else if (bufferIndex < bufferSize - 1) {  // Ensure we don't overflow the buffer
            buffer[bufferIndex++] = c;
        } //end fill buffer
    }//end Serial connected  
} //end loop
