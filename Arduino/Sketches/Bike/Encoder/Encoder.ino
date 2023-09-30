#define ENCB 3 // WHITE

volatile int pos = 0; // specify posi as volatile
long prevTime = 0; // variable to store the previous time
volatile int prevPos = 0; // variable to store the previous position
float velocity = 0; // variable to store the calculated velocity
int CPR = 100; //CPR is counts per revolution

void setup() {
  Serial.begin(9600);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder, RISING);
}

void loop() {
  //Serial.println(pos);
  // Calculate velocity
  long currentTime = millis(); // get the current time
  float timeDiff = currentTime - prevTime; // calculate the time difference

  if (timeDiff > 0) {
    //velocity = ((pos - prevPos)/CPR) / (timeDiff/60000); // calculate the velocity in RPM
    velocity = (pos - prevPos) / timeDiff; // calculate the velocity
    prevPos = pos; // update the previous position
    prevTime = currentTime; // update the previous time
  }
 
  Serial.println(velocity);
  
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
