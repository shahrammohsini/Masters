#include <Wire.h>

#include <ServoTimer2.h>          // Library uses timer 2 to derive servo pwm
#define STEERING_CENTER 1490      // Steering servo pulse width center position
#define DRIVING_CENTER 1500       // Driving servo pulse width static velocity
ServoTimer2 steering_servo;       // Define servo objects to run using timer 2 of nano
ServoTimer2 driving_servo;        // Define servo objects to run using timer 2 of nano

#define MPU6050       0x68        // IMU address
#define ACCEL_CONFIG  0x1C        // Accelerometer config address
#define GYRO_CONFIG   0x1B        // Gyro config address
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C

#define BRAKE         8           // Reaction wheel motor Brake Pin
#define PWM           9          // Reaction wheel motor Speed Pin
#define DIRECTION     4           // Reaction wheel motor Direction Pin



const uint16_t PWM_FREQUENCY = 20000;                 // Reaction wheel motor driver requires a 20kHz PWM frequency
const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2;  // Prescale for new pwm frequency

                      //Alternative Tunings
float P1 = 160;      //49.0  / 55.0 / defaults 75 //180/180/ 180
float I2 = 0.6;      //4.80  / 5.83 / defaults 5.25  //1/0.2/0.3
float D3 = 2.5;     //0.085 / 0.07 / defaults 0.04  //1/1.5/ 2.5
float previous_integral = 0;
float previous_error = 0;
float integral;
float dirivative;
float error = 0;
int16_t skip = 7;     //Deadband compensation
float loop_time = 10; // 100 Hz

int pwm_s = 0;        // PWM send
byte dir;             // Direction
int32_t motor_speed;  // Motor Speed
uint32_t timer;       // Timer

long currentT, previousT_1, previousT_2, vertical_time = 0;
long driving_on_delay_time =    3000; // 3 seconds
long steering_on_delay_time =   8000; // 8 seconds 
long steering_off_delay_time =  14500; // 13 seconds
long driving_off_delay_time =   19500; // 18 seconds 

int16_t AcX, AcY, AcZ, GyZ, gyroZ; // Initialize IMU

// Sensor output scaling

#define accSens 0             // Accelerometer Sensitivity 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // Gyro Sensitivity 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s
#define Gyro_amount 0.996     // percentage of gyro used in the complementary filter

// IMU offset values derived with suggested defaults and experimentally
int16_t  AcX_offset = -750;
int16_t  AcY_offset = 360;
int16_t  AcZ_offset = 0;
int16_t  GyZ_offset = -84;
int32_t  GyZ_offset_sum = 0;

// Gyro Filter parameters

float alpha = 0.40; 
float gyroZfilt;

float robot_angle;                // Calculated Robot angle
float Accel_angle;                // Accelerometer angle readout

bool vertical = false;            // True if within +-0.3deg of balanced position
bool driving_servo_set = true;   // Driving state for routine
bool steering_servo_set = true;  // Steering state for routine
bool vertical_check = false;      // Used to track instance of switching to vertical state
double dt;
unsigned long previousTime = 0;  // Previous time in milliseconds
unsigned long currentTime;      // Current time in milliseconds

uint8_t i2cData[14]; // I2C buffer

void setup() {
  Serial.begin(115200);

  // Set PWM frequency to 20kHz - Example derived from datasheet
  
  // Set up PWM, Phase and Frequency Correct on pin 9 (OC1A) & pin 10 (OC1B) with ICR1 as TOP using Timer1
  TCCR1B = (1 << WGM13) | (1 << CS10);  // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1 = PWMVALUE;                      // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz

  // Clear OC1A/OC1B on compare match when up-counting - Set OC1A/OC1B on compare match when downcounting
  TCCR1A = (1 << COM1A1) | (1 << COM1B1);
  setPWM(400); 
  previousTime = millis();  // Initialize the previous time

  // Reaction Wheel Pin Output Setup
  
  pinMode(PWM, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  delay(1000);
  angle_setup();

  // Servo pin and pulse width setup

   steering_servo.attach(5); 
   driving_servo.attach(6); 
   steering_servo.write(STEERING_CENTER); 
   driving_servo.write(DRIVING_CENTER);

}

void loop() {   // Main Loop
  //delay(5);//remove this later
  currentT = millis();
    // Calculate time step
  currentTime = millis();
  dt = (currentTime - previousTime) / 1000.0;  // Convert to seconds
  if (currentT - previousT_1 >= loop_time) {
   // Tuning();         // Run Tuning subroutine
    angle_calc();     // Run Angle Calculator subroutine
    
    if (vertical) {
      digitalWrite(BRAKE, HIGH);    // Disable Brake
      gyroZ = GyZ / 131.0;          // Convert to deg/s
      
      // Filter Gyroscope and compute PWM send Using PID
      // Serial.print(gyroZ);
      Serial.print(10);
      Serial.print(",");
      Serial.print(-0.7);
      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;  //This is a low pass filter
      Serial.print(",");
      // Serial.print(gyroZfilt);
      // Serial.print(",");
      //Serial.println(robot_angle);
      error = robot_angle + 0.75;
      Serial.println(error);
      //pwm_s = -constrain(P1 * robot_angle + I2 * gyroZfilt + D3 * -motor_speed, -255, 255); //This isn't a full implementation of PID. Integral and dirivative is not calculated anywhere.
      integral = previous_integral + error*dt;
      //integral += robot_angle * dt;
      dirivative = ((error - previous_error)/dt);
      pwm_s = -constrain(P1 * error + integral*I2 + dirivative*D3, -255, 255);
      // Serial.print(",");
      // Serial.println(integral*4);
      // Send computed pwm to motor control code
      Motor_control(pwm_s);
      motor_speed += pwm_s;

      // Routines
      // Drive forward
       if ((currentT - vertical_time >= driving_on_delay_time) && driving_servo_set == false) {
        //driving_servo.write(1000); // set driving_servo to a different value after 5 seconds
        previousT_2 = currentT;
        driving_servo_set = true;
      }
      //Steer
       if ((currentT - vertical_time >= steering_on_delay_time) && steering_servo_set == false) {
        //steering_servo.write(1800); // set driving_servo to a different value after 5 seconds
        previousT_2 = currentT;
        steering_servo_set = true;
      }
      //Stop Steering
       if ((currentT - vertical_time >= steering_off_delay_time) && steering_servo_set == true) {
        steering_servo.write(STEERING_CENTER); // set driving_servo to a different value after 5 seconds
        previousT_2 = currentT;
        steering_servo_set = false;
      }
      //Stop Driving
      if ((currentT - vertical_time >= driving_off_delay_time) && driving_servo_set == true) {
        driving_servo.write(DRIVING_CENTER); // set driving_servo to a different value after 5 seconds
        previousT_2 = currentT;
        driving_servo_set = false;
      }
      
    } else { // If bicycle is not actively balancing
      
      Motor_control(0);                       // Disable Reaction Wheel
      digitalWrite(BRAKE, LOW);               // Engage Brake Mode
      motor_speed = 0;                        // Set speed to 0
      driving_servo_set = false;              // Reset Driving Servo Tracker
      steering_servo_set = false;             // Reset Steering Servo Tracker
    }
    previous_error = error;
    previousT_1 = currentT;                   // Store time for tracking next iteration
    previousTime = currentTime;
    previous_integral = integral;
  }
}
