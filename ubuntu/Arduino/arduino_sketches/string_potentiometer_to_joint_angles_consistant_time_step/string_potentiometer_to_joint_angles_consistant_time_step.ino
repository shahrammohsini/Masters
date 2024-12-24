// Constants
// joint 3
double b3 = 1.2; // cm
double a3 = 1.3;
// joint 2
double b2 = 1.45;
double a2 = 1.5;
// joint 1
double b1 = 1.8;
double a1 = 1.8;

double string_length = 0;
double cm_travled = 100; // how many potentiometer values equal to 1 cm of string pulled
double offset = 250; // value of potentiometer when all joint angles are zero
double j3_angle = 0;
double j2_angle = 0;
double j1_angle = 0;
double max_j3_length = 1.14; // cm
double max_j2_length = 3.57; // cm
double max_j1_length = 5.72; // cm
double max_j3_angle = 54; // deg
double max_j2_angle = 110; // deg
double max_j1_angle = 80; // deg

double full_string_length = 0;

unsigned long current_time = 0;
unsigned long prev_time = 0;
unsigned long start_time;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

//  // Wait for a signal from the Python script
//  while (!Serial.available()) {
//    // Do nothing until data is available
//  }
//  Serial.read(); // Read the signal (assume it's a single byte)
//  delay(200);

  start_time = millis();
  prev_time = start_time;
}

void loop() {
  unsigned long current_millis = millis();
  if (current_millis - prev_time >= 30) { // 30 ms interval
    prev_time = current_millis;
    
    int sensorValue = analogRead(A1);
    
    Serial.print(sensorValue);
    Serial.print(",");
    
    sensorValue = sensorValue - offset;
    if (sensorValue < 0) {
      sensorValue = 0;
    }

    string_length = get_length(sensorValue);
    full_string_length = get_length(sensorValue);

    if (full_string_length >= 0 && full_string_length <= max_j3_length) {
      // joint 3 moves
      j3_angle = get_joint_angle(a3, b3, string_length);
      j2_angle = 0;
      j1_angle = 0;
    } else if (full_string_length > max_j3_length && full_string_length <= max_j2_length) {
      string_length = string_length - max_j3_length;
      j3_angle = max_j3_angle;
      j2_angle = get_joint_angle(a2, b2, string_length);
      j1_angle = 0;
    } else if (full_string_length > max_j2_length && full_string_length <= max_j1_length) {
      string_length = string_length - max_j2_length;
      j3_angle = max_j3_angle;
      j2_angle = max_j2_angle;
      j1_angle = get_joint_angle(a1, b1, string_length);
    }

    Serial.print(j3_angle);
    Serial.print(",");
    Serial.print(j2_angle);
    Serial.print(",");
    Serial.print(j1_angle);
    Serial.print(",");
    current_time = millis() - start_time;
    Serial.println(current_time / 1000.0);
  }
}

// convert deg to rad
double rad(double deg) {
  return deg * (3.14 / 180);
}

// convert rad to deg
double deg(double rad) {
  return rad * (180 / 3.14);
}

// convert potentiometer value to cm
double get_length(double pot_val) {
  return pot_val / cm_travled; // in cm
}

// use cos law to get joint angle
// a and b are distances from joint to string. C is length of string
double get_joint_angle(double a, double b, double c) {
  double cos_angle = (sq(c) - sq(a) - sq(b)) / (-2 * a * b);

  // Clamp the value within the valid range for acos
  if (cos_angle > 1.0) {
    cos_angle = 1.0;
  }

  double angle = acos(cos_angle); // angle in radians
  return deg(angle); // angle in degrees
}
