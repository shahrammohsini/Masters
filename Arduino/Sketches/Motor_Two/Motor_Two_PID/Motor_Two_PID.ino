#include <util/atomic.h> // For the ATOMIC_BLOCK macro
//Motor driver pin decleration
#define enA 9
#define in1 6
#define in2 7
//Encoder pin Decleration
#define ENCA 3 // YELLOW
#define ENCB 2 // WHITE

volatile int posi = 0; // specify posi as volatile
volatile int pos = 0; // specify posi as volatile
int prevTime = 0; // variable to store the previous time
volatile int prevPos = 0; // variable to store the previous position
float velocity = 0; // variable to store the calculated velocity
int CPR = 11; //CPR is counts per revolution of encoder

double desired_vel = 3000; //max speed is 2640 RPM
double pwm;
int max_vel = 5340; //This value will change depending on the battery charge
String run = "true";
//Max voltage applied to motor two is 12 V. Must apply 15.5 V from the power source.

//PID
double error = 0;
double dt = 0;
double kp = 1.5; //2.8 //best = 1.5
double kd = 0.1; //1.5 //best = 0.1
double ki = 0.001; //0 //best = 0.0001   //Note: I added a condition on integral to decrease over time
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
double abs_input = 0;



void setup() {
  Serial.begin(9600);
  //Encoder Setup
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING); //check when encoder A is rising and un the readEncoder function

  //Motor driver Setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void loop() {
  while(run == "false"){  //Don't run the program until data is recived from python
      Serial.println("not running");
      if (Serial.available() > 0){
        run = Serial.readString();
      }
    }

//Encoder Run
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { //prevents the posi var from being changed by the interupt while its being read. Makes it more accurate
    pos = posi;
  }
  //Serial.println(pos);

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
  //I = (ki * integral);
  I = (ki * integral)/(millis()/10000); //I modified I a bit to decrease over time bc it makes the system unstable in the long run if you keep letting I increase
  D = kd * (-(error - previous_error)/dt);
  input = P + I + D;
  if (input > max_vel){
    input = max_vel;
  }
  else if (input < -max_vel){
    input = -max_vel;
  }

  abs_input = abs(input);
  pwm = map(abs_input,0,max_vel,0,255);


  //Print Data
  Serial.print(I);
  Serial.print(",");
  Serial.print(String(millis()/1000));
  Serial.print(",");
  Serial.print(round(desired_vel)); //desired_vel
  Serial.print(",");
  Serial.println(round(velocity*60));  //RPM  Max speed with load: 2640rpm (from experimentation)   Max speed without loat 4800rpm (from datasheed)   138


//Motor Run
// Drive the motor in one direction
if(input < 0){ //CW
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
else if ( input > 0){ //CCW
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
else{ //if the input is 0 the CW
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
  // Set the desired speed (PWM value)
  // Serial.println(pwm); //desired_vel
  analogWrite(enA, pwm); // Send PWM signal to L298N Enable pin



//set previous val to current val
previous_error = error;
previous_integral = integral;
  
}

void readEncoder(){ //check what ENCB is doing when A is rising. If Rising then add to pos if not sub from pos
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}