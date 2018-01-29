#include <Servo.h>
Servo s0;

void setup() {
  // put your setup code here, to run once:
s0.attach(4);
//s0.writeMicroseconds(1500);
//delay(500);
s0.writeMicroseconds(2000);
delay(2000);
s0.writeMicroseconds(1000);
delay(2000);
s0.writeMicroseconds(1500);
digitalWrite(13,1);
}

void loop() {
  // put your main code here, to run repeatedly:

}
