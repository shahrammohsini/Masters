

#include <Wire.h>
#include <Arduino.h>

#define ENCB 3 // WHITE

volatile int pos = 0; // specify posi as volatile
int prevTime = 0; // variable to store the previous time
volatile int prevPos = 0; // variable to store the previous position
float velocity = 0; // variable to store the calculated velocity
int CPR = 45; //CPR is counts per revolution



// Define Motor Pins
#define BRAKE         8 
#define PWM           9
#define DIRECTION     4

// Define encoder variables
volatile long encoder_count = 0;
volatile bool enc_a_last_state = 0;
volatile bool enc_b_last_state = 0;

// Define timer variables
unsigned long last_time = 0;
const unsigned long sample_time = 4; // Sample time in milliseconds

// Motor PWM Frequency
const uint16_t PWM_FREQUENCY = 20000;                 // The motor driver can handle a PWM frequency up to 20kHz
const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2;  // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

// Define motor variables WE FOUND OUT THAT THE ENCODER CPR is approximately 400cpr
int pwm_s = 0;
byte dir;
int32_t motor_speed; 
uint32_t timer;

void setPWM(uint16_t dutyCycle) { // dutyCycle is a value between 0-ICR1
    OCR1A = dutyCycle;
}

void Motor_control(int pwm) {
  if (pwm <= 0) {
    digitalWrite(DIRECTION, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(DIRECTION, HIGH);
  }
  setPWM(map(pwm, 0, 255, PWMVALUE, 0));
}

void setup() {
  Serial.begin(9600);
  
  // Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/doc8272.pdf page 128-135 
  // Set up PWM, Phase and Frequency Correct on pin 9 (OC1A) & pin 10 (OC1B) with ICR1 as TOP using Timer1
  TCCR1B = (1 << WGM13) | (1 << CS10);  // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1 = PWMVALUE;                      // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz
  //Serial.print(ICR1)

  // Clear OC1A/OC1B on compare match when up-counting - Set OC1A/OC1B on compare match when downcounting
  TCCR1A = (1 << COM1A1) | (1 << COM1B1);
  setPWM(400); 
  
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder, RISING);
  
  pinMode(PWM, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  
  digitalWrite(BRAKE, HIGH);
  delay(1000);
}

void loop() {
  Motor_control(0); // HERE WE CONTROL THE MOTOR SPEED
  
  // Calculate velocity
  int currentTime = millis()/1000; // get the current time in sec
  int timeDiff = currentTime - prevTime; // calculate the time difference
  //Serial.println(currentTime);

  if (timeDiff > 0) {
    velocity = ((pos - prevPos)/CPR) / timeDiff; // calculate the velocity
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
