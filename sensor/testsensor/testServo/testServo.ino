#include <Servo.h>
Servo servo;
const int servoPin = A4;
void setup() {
  // put your setup code here, to run once:
  servo.attach(servoPin);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 30; i < 180; i++){
    servo.write(i); 
    delay(15);
  }

  for(int i = 180; i > 30; i--){
    servo.write(i); 
    delay(15);
  }  
}
