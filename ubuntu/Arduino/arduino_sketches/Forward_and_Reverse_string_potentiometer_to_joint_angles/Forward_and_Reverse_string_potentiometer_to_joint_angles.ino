// the setup routine runs once when you press reset:

// Constants
// joint 3
double b3 = 1.2; //cm 1, 1.4
double a3 = 1.3;
// joint 2
double b2 = 1.45;
double a2 = 1.5;
// joint 1
double b1 = 1.8;
double a1 = 1.8;

double string_length = 0;
double cm_travled = 100; //how many potentiometer values equal to 1 cm of string pulled
double offset = 250; // value of potentiometer when all joint angles are zero
double j3_angle = 0;
double j2_angle = 0;
double j1_angle = 0;
double max_j3_length = 1.51; //cm 1.43
double max_j2_length = 3.79; // cm
double max_j1_length = 6.72; //cm
double max_j3_angle = 68; //deg
double max_j2_angle = 99; //deg
double max_j1_angle = 128; //deg

// Reverse
double j1_j2_length = 5.2; //When j3 stops moving //This is because in reverse j1 and j2 are already closed while in forward j3 and j2 close first
double j1_length = 2.52; //when j2 stops moving


double full_string_length = 0;
double previous_full_string_length = full_string_length;

unsigned long current_time = 0;
unsigned long prev_time = 0;
unsigned long start_time;





void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);


  // // Wait for a signal from the Python script
  // while (!Serial.available()) {
  //   // Do nothing until data is available
  // }
  // Serial.read(); // Read the signal (assume it's a single byte)
  // delay(220);
  // start_time = millis();


}

// the loop routine runs over and over again forever:
void loop() {
  float dt = 0.03;
  // float total_time = 1; //sec
  // while (millis() - start_time < total_time * 1000) {
    
  int sensorValue = analogRead(A1);
  // read the input on analog pin 0:
  sensorValue = sensorValue - offset;
  if (sensorValue < 0){
    sensorValue = 0;
  }



  // print out the value you read:
  // Serial.print(sensorValue);

  string_length = get_length(sensorValue);
  full_string_length = get_length(sensorValue);


  
  // // Forward
  // if(previous_full_string_length < full_string_length){
    // Serial.print(" forward ");
  if(full_string_length >= 0 && full_string_length <= max_j3_length){
    // joint 3 moves
    
    j3_angle = get_joint_angle(a3, b3, string_length);
    j2_angle = 0;
    j1_angle = 0;
  }
  else if (full_string_length >= max_j3_length && full_string_length <= max_j2_length){
    string_length = string_length - max_j3_length;
    j3_angle = max_j3_angle;
    j2_angle = get_joint_angle(a2, b2, string_length);
    j1_angle = 0;
  }

  else if (full_string_length >= max_j2_length && full_string_length <= max_j1_length){
    string_length = string_length - max_j2_length;
    j3_angle = max_j3_angle;
    j2_angle = max_j2_angle;
    j1_angle = get_joint_angle(a1, b1, string_length);
  }
  // }

  //Reverse Mid to forward***************************************************************************************************************** 
  
  
  
  // // Reverse
  // else if(previous_full_string_length > full_string_length){
  //   // Serial.print(" reverse ");
  //   if(full_string_length >= j1_j2_length){
  //   // joint 3 moves
  //     string_length = string_length - j1_j2_length;
  //     j3_angle = get_joint_angle(a3, b3, string_length);
  //     j2_angle = max_j2_angle;
  //     j1_angle = max_j1_angle;
  //   } //joint 2 moves
  //   else if(full_string_length < j1_j2_length && full_string_length >= j1_length){
  //     string_length = string_length - j1_length;
  //     j3_angle = 0;
  //     j2_angle = get_joint_angle(a2, b2, string_length);
  //     j1_angle = max_j1_angle;
  //   } // Joint 1 moves
  //   else if (full_string_length < j1_length){
  //   // string_length = string_length - max_j2_length;
  //   j3_angle = 0;
  //   j2_angle = 0;
  //   j1_angle = get_joint_angle(a1, b1, string_length);
  // }
  // }


  // // else if(string_length >= max_j3_length && <= max_j2_length){

  // // }
  Serial.print(" | full_string_length: ");
  Serial.print(full_string_length);
  Serial.print(" | length: ");
  Serial.print(string_length);
  Serial.print(" | joint_3: ");
  Serial.print(j3_angle);
  Serial.print(",");
  Serial.print(" | joint_2: ");
  Serial.print(j2_angle);
  Serial.print(",");
  Serial.print(" | joint_1: ");
  Serial.print(j1_angle);

  Serial.print(",");
  current_time = millis() - start_time;
  Serial.println(current_time/1000.0);

  previous_full_string_length = full_string_length;

  delay(dt * 1000);  // Ensure delay matches dt
  // delay(10);  // delay in between reads for stability
// }
 
}

// convert deg to rad
double rad(double deg){
  double rad = deg*(3.14/180);
  return rad;
}
// convert rad to deg
double deg(double rad){
  double deg = rad*(180/3.14);
  return deg;
}

// convert potentiometer value to cm
double get_length(double pot_val){
  double length = pot_val/cm_travled;
  return length; //in cm
}
// use cos law to get joint angle
// a and b are distances from joint to string. C is length of string
double get_joint_angle(double a, double b, double c){
  // Calculate the value inside acos
  double cos_angle = (sq(c) - sq(a) - sq(b))/(-2*a*b);
  // Serial.print(" a: ");
  // Serial.print(a);
  // Serial.print("| b: ");
  // Serial.print(a);
  // Serial.print("| c: ");
  // Serial.print(c);
  
  // Clamp the value within the valid range for acos
  // Serial.print("| cos angle: ");
  // Serial.print(cos_angle);
  if (cos_angle > 1.0) {
    cos_angle = 1.0;}
  
  double angle = acos(cos_angle); // angle in radians
  // Serial.print(" | cos: ");
  // Serial.print((cos_angle));
  // Serial.print(" | deg: ");
  // Serial.println(deg(angle));

  // if(isnan(angle)){ // when c = 0 acos returns error so angle become NaN.
  //   angle = 0;
  // }
  angle = deg(angle); //angle in deg
  return angle;
  }


  

