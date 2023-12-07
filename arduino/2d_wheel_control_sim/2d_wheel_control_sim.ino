#include <Movement.h>
#include <BasicEncoder.h>
#include <TimerOne.h>

unsigned int serviced = 0;
unsigned int serviced_limit = 100;
int speedL, speedR;

const int bufferSize = 32; // expected max buffer "-100 -100\n" //so ~3x just incase
char buffer[bufferSize];
int bufferIndex = 0;

int leftEncoder = 0;
int rightEncoder = 0;
bool moving = false;
bool backward = false;

void timer_service() {
  serviced++;

  if(moving){
    leftEncoder += speedL/10;
    rightEncoder += speedR/10;
  }
  
  if (serviced > serviced_limit){
    serviced=0;
    Serial.print(leftEncoder);
    Serial.print(" ");
    Serial.println(rightEncoder);
    leftEncoder = 0;
    rightEncoder = 0;
 }}

void setup(){
  Serial.begin(115200);
  while (!Serial) {} //wait for Serial to connect
  Timer1.initialize(50);
  Timer1.attachInterrupt(timer_service);
} //end setup

void loop() {
  while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') { // Check for newline character
            buffer[bufferIndex] = '\0';  // null-terminate the C string
            if (sscanf(buffer, "%d %d", &speedL, &speedR) == 2) {
              if (speedL == 0 && speedR == 0) { moving = false; }
              else { moving = true; }
            } //emd got 2 ints
            bufferIndex = 0;  // reset the buffer
        } else if (bufferIndex < bufferSize - 1) {  // Ensure we don't overflow the buffer
            buffer[bufferIndex++] = c;
        } //end fill buffer
    }//end Serial connected  
} //end loop
