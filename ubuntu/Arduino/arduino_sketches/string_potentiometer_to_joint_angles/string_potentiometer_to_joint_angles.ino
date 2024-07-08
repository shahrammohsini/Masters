// the setup routine runs once when you press reset:

// Constants
// joint 3
double b3 = 0.90; //cm 1, 1.4
double a3 = 0.92;
// joint 2
double b2 = 1.25;
double a2 = 1.3;
// joint 1
double b1 = 1.8;
double a1 = 1.8;

double string_length = 0;
double cm_travled = 143; //how many potentiometer values equal to 1 cm of string pulled
double offset = 114; // value of potentiometer when all joint angles are zero
double j3_angle = 0;
double j2_angle = 0;
double j1_angle = 0;
double max_j3_length = 1.09; //cm 1.51
double max_j2_length = 3.04; // cm 3.79
double max_j1_length = 5.1; //cm
double max_j3_angle = 72; //deg -D
double max_j2_angle = 89; //deg -P
double max_j1_angle = 67; //deg -M

double full_string_length = 0;

unsigned long current_time = 0;
unsigned long prev_time = 0;
unsigned long start_time;





void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);


  // Wait for a signal from the Python script
  while (!Serial.available()) {
    // Do nothing until data is available
  }
  Serial.read(); // Read the signal (assume it's a single byte)
  // delay(350);
  // delay(250);
  // delay(220);
  // delay(190);
  // delay(160);
  delay(130);

  start_time = millis();


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


  

  if(full_string_length >= 0 && full_string_length <= max_j3_length){
    // joint 3 moves
    if(full_string_length < max_j3_length/2){ // The first half requires a greater change than the second half. Slightly different models for first and second half for better accuracy
    j3_angle = get_joint_angle((a3-0.09), (b3-0.09), string_length);
    j2_angle = 0;
    j1_angle = 0;
    }
    else{
      j3_angle = get_joint_angle(a3, b3, string_length);
      j2_angle = 0;
      j1_angle = 0;
    }
    if(j3_angle > max_j3_angle){ //To prevent miss reading a greater angle than max angle
      j3_angle = max_j3_angle;
    }
  }
  else if (full_string_length >= max_j3_length && full_string_length <= max_j2_length){
    string_length = string_length - max_j3_length;
    j3_angle = max_j3_angle;
    j2_angle = get_joint_angle(a2, b2, string_length);
    if(j2_angle > max_j2_angle){
      j2_angle = max_j2_angle;
    }
    j1_angle = 0;
  }

  else if (full_string_length >= max_j2_length && full_string_length <= max_j1_length){
    string_length = string_length - max_j2_length;
    j3_angle = max_j3_angle;
    j2_angle = max_j2_angle;
    j1_angle = get_joint_angle(a1, b1, string_length);
  }



  // // else if(string_length >= max_j3_length && <= max_j2_length){

  // // }
  // Serial.print(" | full_string_length: ");
  // Serial.print(full_string_length);
  // Serial.print(" | length: ");
  // Serial.print(string_length);
  // Serial.print(" | joint_3: ");
  Serial.print(j3_angle);
  Serial.print(",");
  // Serial.print(" | joint_2: ");
  Serial.print(j2_angle);
  Serial.print(",");
  // Serial.print(" | joint_1: ");
  Serial.print(j1_angle);

  Serial.print(",");
  current_time = millis() - start_time;
  Serial.println(current_time/1000.0);


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
  
  // Clamp the value within the valid range for acos
  if (cos_angle > 1.0) {
    cos_angle = 1.0;}
  
  double angle = acos(cos_angle); // angle in radians
  // Serial.print(" | cos: ");
  // Serial.print((cos_angle));
  // Serial.print(" | deg: ");
  // Serial.print(deg(angle));

  // if(isnan(angle)){ // when c = 0 acos returns error so angle become NaN.
  //   angle = 0;
  // }
  angle = deg(angle); //angle in deg
  return angle;
  }


  

