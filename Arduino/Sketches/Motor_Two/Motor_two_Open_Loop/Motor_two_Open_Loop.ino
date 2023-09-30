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
float prevTime = 0; // variable to store the previous time
float currentTime = 0;
volatile int prevPos = 0; // variable to store the previous position
float velocity = 0; // variable to store the calculated velocity
int CPR = 11; //CPR is counts per revolution of encoder

double desired_vel = 1350; //max speed is 2640 RPM
double pwm;
int max_vel = 2700; //This value will change depending on the battery charge
String run = "true"; //change this to "true" if you want to start the program from python. "false" to start it here
//Max voltage applied to motor two is 12 V. Must apply 15.5 V from the power source.




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
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { //prevents the posi var from being changed by the interupt while its being read. Makes it more accurate
    pos = posi;
  }
  //Serial.println(pos);
  delay(800); 
// Calculate velocity
  currentTime = millis(); // get the current time sec
  float timeDiff = currentTime - prevTime; // calculate the time difference
  if (timeDiff > 0) { //This insures we don't devide by zero but also inadvertantly ensures we only do one velocity calculation every sec as timeDiff variable will be in non decimal seconds so for TimeDiff to be >0 it has to be 1. 
    //velocity = ((pos - prevPos)/CPR) / (timeDiff/60000); // calculate the velocity in RPM
    velocity = ((pos - prevPos)/CPR) / (timeDiff/1000); // calculate the velocity
    prevPos = pos; // update the previous position
    prevTime = currentTime; // update the previous time
  }
  //Print Data
  // Serial.print(String(millis()));
  // Serial.print(",");
  // Serial.print(round(desired_vel)); //desired_vel
  // Serial.print(",");
  Serial.println(round(velocity*60));  //RPM  Max speed with load: 2640rpm (from experimentation)   Max speed without loat 4800rpm (from datasheed)   1380




//Motor Run
// Drive the motor in one direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  // Set the desired speed (PWM value)
  int pwmOutput = 255; // Change this value to adjust the motor speed
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin


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