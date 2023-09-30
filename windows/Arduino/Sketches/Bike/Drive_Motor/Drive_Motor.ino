// Define Motor Pins
#define BRAKE         8 
#define PWM           9
#define DIRECTION     4



void setup() {
  // put your setup code here, to run once:
  
  pinMode(PWM, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  digitalWrite(BRAKE, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(DIRECTION, HIGH);
  analogWrite(PWM,255); //-1 OFF, -255 MAX, 255 OFF, 0 MAX

}
