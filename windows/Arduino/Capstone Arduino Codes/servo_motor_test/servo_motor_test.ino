#include <Servo.h>

// Define servo pins
int steeringPin = 5;
int drivingPin = 6;

// Create servo objects
Servo steeringServo;
Servo drivingServo;

void setup() {
  // Attach servo objects to their respective pins
  steeringServo.attach(steeringPin);
  drivingServo.attach(drivingPin);

  // Set servos to their idle position and speed
  steeringServo.writeMicroseconds(1490); // 1490 is middle position Mechanism limits 1150-2100 Greater number turns left
  drivingServo.writeMicroseconds(1500); // 1500 is idle LIMITS 700-2300, Lower Numbers go forward

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {

  }
