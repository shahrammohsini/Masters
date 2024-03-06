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

double desired_vel = 1350; //max speed is 2640 RPM
double pwm;
int max_vel = 2700; //This value will change depending on the battery charge
String run = "false";


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
  // Calculate velocity
  int currentTime = millis()/1000; // get the current time sec
  int timeDiff = currentTime - prevTime; // calculate the time difference
  if (timeDiff > 0) {
    //velocity = ((pos - prevPos)/CPR) / (timeDiff/60000); // calculate the velocity in RPM
    velocity = ((pos - prevPos)/CPR) / timeDiff; // calculate the velocity
    prevPos = pos; // update the previous position
    prevTime = currentTime; // update the previous time
  }
  Serial.print(String(millis()/1000));
  Serial.print(",");
  Serial.print(round(desired_vel));
  Serial.print(",");
  Serial.println(round(velocity*60));  //RPM  Max speed with load: 2640rpm (from experimentation)   Max speed without loat 4800rpm (from datasheed)   1380

  //Drive the motor
  pwm = map(desired_vel,0,max_vel,-1,-255); //for some reasone the motor runs from -1 to -255 and 255 to 0 (where 0 is full speed and 255 is stopped)
  digitalWrite(DIRECTION, HIGH);
  analogWrite(PWM_pin,-212.5);
  //Serial.println(pwm);


  
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
