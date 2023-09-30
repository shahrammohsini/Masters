#include <Wire.h>
#include <Arduino.h>

// Define interrupt pins
#define ENC_A 2
#define ENC_B 3

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
const unsigned long sample_time = 250; // Sample time in milliseconds

// Motor PWM Frequency
const uint16_t PWM_FREQUENCY = 20000;                 // The motor driver can handle a PWM frequency up to 20kHz
const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2;  // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

// Define motor variables WE FOUND OUT THAT THE ENCODER CPR is approximately 400cpr
int pwm_s = 0;
byte dir;
float motor_speed = 0.0;
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
  Serial.begin(115200);
  
  // Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/doc8272.pdf page 128-135 
  // Set up PWM, Phase and Frequency Correct on pin 9 (OC1A) & pin 10 (OC1B) with ICR1 as TOP using Timer1
  TCCR1B = (1 << WGM13) | (1 << CS10);  // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1 = PWMVALUE;                      // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz

  // Clear OC1A/OC1B on compare match when up-counting - Set OC1A/OC1B on compare match when downcounting
  TCCR1A = (1 << COM1A1) | (1 << COM1B1);
  setPWM(400); 
  
  Serial.println("PWM: "); Serial.println(PWMVALUE); Serial.print("CPU_FREQ: "); Serial.println(F_CPU); 
  Serial.println("PWM,RPM");
  
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENC_A), isr_enc_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isr_enc_b, CHANGE);
  
  pinMode(PWM, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  
  digitalWrite(BRAKE, HIGH);
  delay(1000);
}

void loop() {
  for (int pwm = 0; pwm <= 255; pwm++) {
    //Control and settle time
    Motor_control(pwm);
    delay(200); // wait for settling time before taking the measurement

    // reset encoder count
    encoder_count = 0;

    // Collect sample
    unsigned long start_time = millis();
    // Wait for 250ms to measure the RPM
    delay(50);
    unsigned long elapsed_time = millis() - start_time;
    unsigned long current_count = encoder_count;

    // Calculate RPM based on encoder count and elapsed time
    motor_speed = (current_count * 60000L) / (400L * elapsed_time);

    // Output PWM and RPM to serial console in CSV format
    Serial.print(pwm);
    Serial.print(",");
    Serial.println(motor_speed);
    
  }
  delay(8000);
}



// Interrupt service routine for encoder signal A
void isr_enc_a() {
  bool enc_a_state = digitalRead(ENC_A);
  bool enc_b_state = digitalRead(ENC_B);

  if (enc_a_last_state == 1 && enc_a_state == 0) {
    if (enc_b_state == 0) {
      encoder_count--;
    } else {
      encoder_count++;
    }
  } else if (enc_a_last_state == 0 && enc_a_state == 1) {
    if (enc_b_state == 0) {
      encoder_count++;
    } else {
      encoder_count--;
    }
  }

  enc_a_last_state = enc_a_state;
}

// Interrupt service routine for encoder signal B
void isr_enc_b() {
  bool enc_b_state = digitalRead(ENC_B);
  bool enc_a_state = digitalRead(ENC_A);

  if (enc_b_last_state == 1 && enc_b_state == 0) {
    if (enc_a_state == 1) {
      encoder_count--;
    } else {
      encoder_count++;
    }
  } else if (enc_b_last_state == 0 && enc_b_state == 1) {
    if (enc_a_state == 1) {
      encoder_count++;
    } else {
      encoder_count--;
      
    }
  }

  enc_b_last_state = enc_b_state;
}
