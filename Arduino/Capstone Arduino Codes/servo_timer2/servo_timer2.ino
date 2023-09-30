#include <Wire.h>
#include <ServoTimer2.h>
#define STEERING_CENTER 1490
#define DRIVING_CENTER 1500
ServoTimer2 steering_servo;
ServoTimer2 driving_servo;

void setup() {
  // put your setup code here, to run once:
   steering_servo.attach(5); 
   driving_servo.attach(6); 
   steering_servo.write(STEERING_CENTER); 
   driving_servo.write(DRIVING_CENTER);

}

void loop() {
  // put your main code here, to run repeatedly:

}
