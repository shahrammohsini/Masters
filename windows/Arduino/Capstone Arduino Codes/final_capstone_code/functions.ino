void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void angle_setup() {  // Setup IMU Routine
  Wire.begin();
  delay(100);   // Allow 100ms compute time
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Scale Accelerometer based on selected sensitivity
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Scale Gyro based on selected sensitivity
  delay(100);   // Allow 100ms compute time

  for (int i = 0; i < 1024; i++) {    // Polling
    angle_calc();                     // Run Angle Calculator
    GyZ_offset_sum += GyZ;            // Integrate offset
    delay(5);                         // Loop at 200Hz
  }
  GyZ_offset = GyZ_offset_sum >> 10;  // Send calculated Offset
  Serial.print("GyZ offset: "); Serial.println(GyZ_offset);   // Display the calculated Offset
}

void angle_calc() {   // Angle Calculator Routine
  Wire.beginTransmission(MPU6050);
  
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 4, true);
  
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(MPU6050);

  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);
  GyZ = Wire.read() << 8 | Wire.read();

  AcX += AcX_offset;
  AcY += AcY_offset;  
  AcZ += AcZ_offset;
  GyZ -= GyZ_offset;

  robot_angle += GyZ * loop_time / 1000 / 65.536;                                 // Unfused robot angle
  Accel_angle = atan2(AcY, -AcX) * 57.2958;                                       // angle from accel after unit conversion
  robot_angle = robot_angle * Gyro_amount + Accel_angle * (1.0 - Gyro_amount);    // Fused Robot Angle
  if (abs(robot_angle) > 9) vertical = false;                                     // Safety limit (+- 9 deg)
  vertical_check = vertical;                                                      // Vertical Tracking Variable
  
  if (abs(robot_angle) < 0.3) vertical = true;                                    // Starting limit (+- 0.3 deg)
  
  if (vertical == true && vertical_check == false){

   vertical_time = millis();      // Baseline time for routine timer
   //robot_angle= robot_angle - 2.5;
  }

  //Serial.print("Angle: "); Serial.println(robot_angle);
}

void setPWM(uint16_t dutyCycle) { // Set Reaction Wheel Duty Cycle derived from nano arduino nano Timer1
    OCR1A = dutyCycle;
}

void Motor_control(int pwm) { // four quadrant style motor control
  // Direction Sensing
  if (pwm <= 0) {
    digitalWrite(DIRECTION, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(DIRECTION, HIGH);
  }

  // Deadzone Compensation
  if (pwm == 0){
  }
  else{
    pwm = map(pwm, 0, 255, skip, 255);    // Scaling to values outside of deadzone
  }
   setPWM(map(pwm, 0, 255, PWMVALUE, 0));
}

int Tuning() {
  if (!Serial.available())  return 0;       // Check for serial monitor input
  delay(2);
  char param = Serial.read();               // get parameter byte
  if (!Serial.available()) return 0;        // Check for serial monitor input
  char cmd = Serial.read();                 // get command byte
  Serial.flush();
  
  switch (param) {  //This lets the user adjust the P I and D values using the serial monitor. The code is incomplete. They pulled it from somewhere lol
    case 'p':                               //Proportional Tuner
      if (cmd == '+')    P1 += 1;
      if (cmd == '-')    P1 -= 1;
      printValues();
      break;
      
    case 'i':                               //Integral Tuner
      if (cmd == '+')    I2 += 0.01;
      if (cmd == '-')    I2 -= 0.01;
      printValues();
      break;
      
     case 'd':                               //Derivative Tuner
      if (cmd == '+')    D3 += 0.005;
      if (cmd == '-')    D3 -= 0.005;
      printValues();
      break;  
      
    case 's':                               //Deadband Compensation Range Tuner
      if (cmd == '+')    skip += 1;
      if (cmd == '-')    skip -= 1;
      printValues();
      break;
  }
}

void printValues() {                        // Print Current Tuning Values
  Serial.print("P: "); Serial.print(P1);
  Serial.print(" I: "); Serial.print(I2);
  Serial.print(" D: "); Serial.print(D3, 3);
  Serial.print(" skip: "); Serial.println(skip);
}
