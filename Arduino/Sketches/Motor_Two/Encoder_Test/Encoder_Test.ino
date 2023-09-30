#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 3 // YELLOW
#define ENCB 2 // WHITE

volatile int posi = 0; // specify posi as volatile

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING); //check when encoder A is rising and un the readEncoder function
}

void loop() {

  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi; //prevents the posi var from being changed by the interupt while its being read. Makes it more accurate
  }
  
  Serial.println(pos);
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