//Define Encoder Pin
#define ENCB 3 // WHITE
// Define Motor Pins
#define BRAKE         8 
#define PWM_pin           9
#define DIRECTION     4


//Encoder
volatile int pos = 0; // specify posi as volatile
int prevTime = 0; // variable to store the previous time
volatile int prevPos = 0; // variable to store the previous position
float velocity = 0; // variable to store the calculated velocity
int CPR = 100; //CPR is counts per revolution of encoder

double desired_vel = 1500; //145pwm //max speed is 2640 RPM
double pwm;
int max_vel = 2700; //This value will change depending on the battery charge
String run = "true"; //set to "false" if you want to run the program from python

//PID
double error = 0;
double dt = 0;
double kp = 3;
double kd = 2;
double ki = 0;
double integral = 0;
double previous_integral = 0;
double previous_error = 0;
double PID_Current_Timer = 0;
double PID_Previous_Timer = 0;
double RPM = 0;
double P = 0;
double I = 0;
double D = 0;
double input = 0;




void setup() {
  Serial.begin(9600);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder, RISING);

  //Motor Driver
  pinMode(PWM_pin, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  digitalWrite(BRAKE, HIGH); //Turn off brake
  analogWrite(PWM_pin,255);
  //delay(5000);
}

void loop() {

  while(run == "false"){  //Don't run the program until data is recived from python
    Serial.println("not running");
    if (Serial.available() > 0){
      run = Serial.readString();
      
    }
  
  }
  
  PID_Current_Timer = millis();
  // Calculate velocity
  int currentTime = millis()/1000; // get the current time sec
  int timeDiff = currentTime - prevTime; // calculate the time difference
  if (timeDiff > 0) {
    //velocity = ((pos - prevPos)/CPR) / (timeDiff/60000); // calculate the velocity in RPM
    velocity = ((pos - prevPos)/CPR) / timeDiff; // calculate the velocity
    prevPos = pos; // update the previous position
    prevTime = currentTime; // update the previous time
  }

  //PID Controller
  dt = (PID_Current_Timer - PID_Previous_Timer)/1000; //calculate time step dt
  RPM = velocity*60;
  error = (desired_vel - RPM);
  integral = previous_integral + error*dt;
  P = kp * error;
  I = ki * integral;
  D = kd * ((error - previous_error)/dt);
  input = P + I + D;
  if (input > max_vel){
    input = max_vel;
  }
  else if (input < -max_vel){
    input = -max_vel;
  }
  pwm = map(input,0,max_vel,-1,-255); //for some reasone the motor runs from -1 to -255 and 255 to 0 (where 0 is full speed and 255 is stopped)

  // Print values to serial monitor
  //Serial.print(String(millis()/1000));
  //Serial.print(",");
  Serial.print(round(desired_vel)); //desired_vel
  // Serial.print(",");
  // Serial.print(round(input));
  // Serial.print(",");
  // Serial.print(round(pwm));
  Serial.print(",");
  Serial.println(round(velocity*60));  //RPM  Max speed with load: 2640rpm (from experimentation)   Max speed without loat 4800rpm (from datasheed)   1380

  //Drive the motor
  //pwm = map(desired_vel,0,max_vel,-1,-255); //for some reasone the motor runs from -1 to -255 and 255 to 0 (where 0 is full speed and 255 is stopped)
  if (input > 0){
    digitalWrite(DIRECTION, HIGH); //HIGH = CW, LOW = CCW
  }
  else{
    digitalWrite(DIRECTION, LOW); //HIGH = CW, LOW = CCW
  }
  analogWrite(PWM_pin,pwm);
  //Serial.println(pwm);


  
previous_error = error;
previous_integral = integral;
  
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    pos++;
  }
  else {
    pos--;
  }
}
