/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01
    by Dejan Nedelkovski, www.HowToMechatronics.com
*/

#define enA 9
#define in1 6
#define in2 7

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void loop() {
  // Drive the motor in one direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  // Set the desired speed (PWM value)
  int pwmOutput = 250; // Change this value to adjust the motor speed
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
}