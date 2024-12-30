// Must establish connection with ROS through serial port:
// rosrun rosserial_python serial_node.py /dev/ttyUSB0

#define nullptr NULL
#include <ros.h>
#include <std_msgs/Int32.h>  // For integer message type (if needed)
#include "bionic_hand/FingerPos.h"  // Updated consolidated message
#include <ros/time.h>
#include "FingerConfig.h"

// FingerConfig and JointParametr are both defined in FingerConfig.h. This ensures all functions have access to the structures

// Initialize parameters for the index finger
FingerConfig indexFingerConfig = {
    // JointParameters. 
    {
        1.1,  // a1_index
        1.1,  // b1_index
        1.8,  // a2_index
        1.9,  // b2_index
        0.7,  // a3_index
        0.8   // b3_index
    },
    145.0,   // cm_travled_index (Potentiometer counts per cm.)
    22.0,    //offset: initial potentiometer value before string is drawn
    1.24,    // max_j3_length_index
    3.0,    // max_j2_length_index
    5.15,     // max_j1_length_index
    102.0,    // max_j3_angle_index
    56.0,    // max_j2_angle_index
    141.0     // max_j1_angle_index
};

// Initialize parameters for the middle finger
FingerConfig middleFingerConfig = {
    // JointParameters
    {
        1.80,  // a1_middle
        1.80,  // b1_middle
        1.30,  // a2_middle
        1.25,  // b2_middle
        0.92,  // a3_middle
        0.90   // b3_middle
    },
    150.0,   // cm_travled_middle (adjust if different)
    67.0,    //offset: initial potentiometer value before string is drawn
    1.15,    // max_j3_length_middle
    3.15,    // max_j2_length_middle
    5.71,     // max_j1_length_middle
    80.0,    // max_j3_angle_middle
    103.0,    // max_j2_angle_middle
    90.0     // max_j1_angle_middle
};

// Initialize parameters for the thumb
FingerConfig thumbConfig = {
    // JointParameters
    {
        1.4,  // a1_thumb
        1.4,  // b1_thumb
        1.5,  // a2_thumb
        1.5,  // b2_thumb
        0.92,  // a3_thumb //not required for thumb
        0.90   // b3_thumb //not required for thumb
    },
    150.0,   // cm_travled_middle (adjust if different)
    623,    //offset: initial potentiometer value before string is drawn
    1.45,    // max_j3_length_thumb
    2.35,    // max_j2_length_thumb
    5.71,     // max_j1_length_thumb //not required for thumb
    107.0,    // max_j3_angle_thumb
    33,       // max_j2_angle_thumb
    90.0     // max_j1_angle_thumb //not required for thumb
};

// Timing & ROS
unsigned long current_time = 0.0;
unsigned long prev_time = 0.0;
unsigned long start_time = 0.0;
ros::NodeHandle nh;

// Message for all fingers
bionic_hand::FingerPos finger_positions;

// Publisher for all fingers
ros::Publisher chatter("updated_Finger_Positions", &finger_positions);

// Function prototypes
double rad(double deg_val);
double deg(double rad_val);
double get_length(double pot_val, double cm_traveled);
double get_joint_angle(double a, double b, double c);
void computeFingerJoints(double full_string_length,
                         FingerConfig config,
                         double &j1_angle, double &j2_angle, double &j3_angle);

void setup() {
  nh.initNode();
  nh.advertise(chatter);

  Serial.begin(57600);
  // If needed, wait for a start signal
  // while (!Serial.available()) { }
  // Serial.read();
  // delay(300);
  start_time = millis();
}

void loop() {
  float dt = 0.03; // Control loop time step

  // ---------------- Index Finger Computation ----------------
  int sensorValue_index = analogRead(A0);
  // Serial.print("sensorValue_index: ");
  // Serial.print(sensorValue_index);
  
  sensorValue_index -= indexFingerConfig.offset; // offset_index
  if (sensorValue_index < 0) { // Remove negative values
    sensorValue_index = 0;
  }

  double full_string_length_index = get_length(sensorValue_index, indexFingerConfig.cm_travled); // Full string length is the current length of string drawn in cm
  // Serial.print(" full_length_index: ");
  // Serial.print(full_string_length_index);
  
  double j3_angle_index = 0.0, j2_angle_index = 0.0, j1_angle_index = 0.0;
  // Compute joint angles (position) for the index finger
  computeFingerJoints(full_string_length_index, indexFingerConfig, j1_angle_index, j2_angle_index, j3_angle_index);
  
  // Print joint angles for the index finger
  // Serial.print("   Index Finger - J3: ");
  // Serial.print(j3_angle_index);
  // Serial.print(" J2: ");
  // Serial.print(j2_angle_index);
  // Serial.print(" J1: ");
  // Serial.print(j1_angle_index);

  // ---------------- Middle Finger Computation ----------------
  int sensorValue_middle = analogRead(A1);
  // Serial.print("------sensorValue_middle: ");
  // Serial.print(sensorValue_middle);
  
  sensorValue_middle -= middleFingerConfig.offset; // offset_middle
  if (sensorValue_middle < 0) { // Remove negative values
    sensorValue_middle = 0;
  }

  double full_string_length_middle = get_length(sensorValue_middle, middleFingerConfig.cm_travled);
  // Serial.print(" full_length_middle: ");
  // Serial.print(full_string_length_middle);
  // Serial.print(" ");
  
  double j3_angle_middle = 0.0, j2_angle_middle = 0.0, j1_angle_middle = 0.0;
  // Compute joint angles for the middle finger
  computeFingerJoints(full_string_length_middle, middleFingerConfig, j1_angle_middle, j2_angle_middle, j3_angle_middle);
  
  // Print joint angles for the middle finger
  // Serial.print("   Middle Finger - J3: ");
  // Serial.print(j3_angle_middle);
  // Serial.print(" J2: ");
  // Serial.print(j2_angle_middle);
  // Serial.print(" J1: ");
  // Serial.println(j1_angle_middle);

  // ---------------- Thumb Computation ----------------
  int sensorValue_thumb = analogRead(A2);
//  Serial.print("------sensorValue_thumb: ");
//  Serial.print(sensorValue_thumb);
  
  sensorValue_thumb -= thumbConfig.offset; // offset_thumb
  if (sensorValue_thumb < 0) { // Remove negative values
    sensorValue_thumb = 0;
  }

  double full_string_length_thumb = get_length(sensorValue_thumb, thumbConfig.cm_travled);
//  Serial.print(" full_length_thumb: ");
//  Serial.print(full_string_length_thumb);
//  Serial.print(" ");
  
  double j3_angle_thumb = 0.0, j2_angle_thumb = 0.0, j1_angle_thumb = 0.0;
  // Compute joint angles for the thumb
  computeFingerJoints(full_string_length_thumb, thumbConfig, j1_angle_thumb, j2_angle_thumb, j3_angle_thumb);
  
  // Print joint angles for the thumb
//  Serial.print("   Thumb - J3: ");
//  Serial.print(j3_angle_thumb);
//  Serial.print(" J2: ");
//  Serial.print(j2_angle_thumb);
//  Serial.print(" J1: ");
//  Serial.println(j1_angle_thumb);


//Open loop data collection
// uncomment whichever finger open loop data is being collected for


//  Serial.print(j3_angle_index);
//  Serial.print(",");
//  Serial.print(j2_angle_index);
//  Serial.print(",");
//  Serial.print(j1_angle_index);

  Serial.print(j3_angle_thumb);
  Serial.print(",");
  Serial.print(j2_angle_thumb);
  Serial.print(",");
  Serial.print(j1_angle_thumb);


  Serial.print(",");
  current_time = millis() - start_time;
  Serial.println(current_time/1000.0);


  // ---------------- Populate FingerPos Message ----------------
  finger_positions.header.stamp = nh.now(); // Set the timestamp to current time

  // Populate index finger data
  finger_positions.index.theta_M = j1_angle_index;
  finger_positions.index.theta_P = j2_angle_index;
  finger_positions.index.theta_D = j3_angle_index;

  // Populate middle finger data
  finger_positions.middle.theta_M = j1_angle_middle;
  finger_positions.middle.theta_P = j2_angle_middle;
  finger_positions.middle.theta_D = j3_angle_middle;

  // Populate thumb data
  finger_positions.thumb.theta_M = j1_angle_thumb;
  finger_positions.thumb.theta_P = j2_angle_thumb;
  finger_positions.thumb.theta_D = j3_angle_thumb;

  // Publish the consolidated message
  chatter.publish(&finger_positions);

  nh.spinOnce();
  delay(dt * 1000); // Delay to slow down the loop
}

// ---------------- Helper Functions ----------------

double rad(double deg_val) {
  return deg_val * (3.14159265358979323846 / 180.0);
}

double deg(double rad_val) {
  return rad_val * (180.0 / 3.14159265358979323846);
}

double get_length(double pot_val, double cm_traveled) {
  return pot_val / cm_traveled; // Convert potentiometer value to cm
}

double get_joint_angle(double a, double b, double c) {
  double cos_angle = (sq(c) - sq(a) - sq(b)) / (-2.0 * a * b);
  cos_angle = constrain(cos_angle, -1.0, 1.0); // Clamp value within [-1, 1]
  return deg(acos(cos_angle)); // Convert to degrees
}

void computeFingerJoints(double full_string_length, FingerConfig config, double &j1_angle, double &j2_angle, double &j3_angle) {
  double string_length = full_string_length;

  // Joint 3 calculation
  if (string_length <= config.max_j3_length) {
    j3_angle = get_joint_angle(config.joints.a3, config.joints.b3, string_length);
    j3_angle = min(j3_angle, config.max_j3_angle);
    j2_angle = 0.0;
    j1_angle = 0.0;
  }
  // Joint 2 calculation
  else if (string_length <= config.max_j2_length) {
    string_length -= config.max_j3_length;
    j3_angle = config.max_j3_angle;
    j2_angle = get_joint_angle(config.joints.a2, config.joints.b2, string_length);
    j2_angle = min(j2_angle, config.max_j2_angle);
    j1_angle = 0.0;
  }
  // Joint 1 calculation
  else if (string_length <= config.max_j1_length) {
    string_length -= config.max_j2_length;
    j3_angle = config.max_j3_angle;
    j2_angle = config.max_j2_angle;
    j1_angle = get_joint_angle(config.joints.a1, config.joints.b1, string_length);
    j1_angle = min(j1_angle, config.max_j1_angle);
  } else {
    // Outside expected range, set all to 0 or handle appropriately
    j1_angle = j2_angle = j3_angle = 0.0;
  }
}

