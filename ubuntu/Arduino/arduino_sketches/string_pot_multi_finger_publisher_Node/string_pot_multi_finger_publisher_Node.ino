// Must establish connection with ROS through serial port:
// rosrun rosserial_python serial_node.py /dev/ttyUSB0
#define nullptr NULL
#include <ros.h>
#include <std_msgs/Int32.h>  // For integer message type
#include "bionic_hand/FingerPos.h"
#include <ros/time.h>

// ---------- Constants (shared for both fingers unless otherwise needed) ----------
// joint 3
double b3 = 0.90; // For both fingers, adjust if needed
double a3 = 0.92;
// joint 2
double b2 = 1.25;
double a2 = 1.3;
// joint 1
double b1 = 1.8;
double a1 = 1.8;

double cm_travled = 143; // potentiometer counts per cm
double offset_index = 114;  // offset for the index finger’s sensor
double offset_middle = 114; // offset for the middle finger’s sensor (adjust if needed)

// Max lengths and angles for a single finger
double max_j3_length = 1.09; 
double max_j2_length = 3.04; 
double max_j1_length = 5.1; 

double max_j3_angle = 72; //deg 
double max_j2_angle = 89; //deg 
double max_j1_angle = 67; //deg 

// Timing & ROS
unsigned long current_time = 0;
unsigned long prev_time = 0;
ros::NodeHandle nh;

// Messages for two fingers
bionic_hand::FingerPos finger_pos_index; 
bionic_hand::FingerPos finger_pos_middle; 

// Publishers for two fingers
ros::Publisher chatter_index("Updated_Finger_Position_Index", &finger_pos_index);
ros::Publisher chatter_middle("Updated_Finger_Position_Middle", &finger_pos_middle);

void setup() {
  nh.initNode();
  nh.advertise(chatter_index);
  nh.advertise(chatter_middle);

  Serial.begin(57600);
  // If needed, wait for a start signal
  // while (!Serial.available()) { }
  // Serial.read();
  // start_time = millis();
}

void loop() {
  float dt = 0.03; // Control loop time step

  // ---------------- Index Finger Computation ----------------
  int sensorValue_index = analogRead(A0);
  sensorValue_index = sensorValue_index - offset_index;
  if (sensorValue_index < 0) {
    sensorValue_index = 0;
  }

  double full_string_length_index = get_length(sensorValue_index);
  double j3_angle_index = 0;
  double j2_angle_index = 0;
  double j1_angle_index = 0;

  // Compute joint angles for the index finger
  computeFingerJoints(full_string_length_index, 
                      a3, b3, a2, b2, a1, b1,
                      max_j3_length, max_j2_length, max_j1_length,
                      max_j3_angle, max_j2_angle, max_j1_angle,
                      j1_angle_index, j2_angle_index, j3_angle_index);

  // Set message for index finger
  finger_pos_index.header.stamp = nh.now();
  finger_pos_index.theta_M = j1_angle_index;
  finger_pos_index.theta_P = j2_angle_index;
  finger_pos_index.theta_D = j3_angle_index;


  // ---------------- Middle Finger Computation ----------------
  int sensorValue_middle = analogRead(A1);
  sensorValue_middle = sensorValue_middle - offset_middle;
  if (sensorValue_middle < 0) {
    sensorValue_middle = 0;
  }

  double full_string_length_middle = get_length(sensorValue_middle);
  double j3_angle_middle = 0;
  double j2_angle_middle = 0;
  double j1_angle_middle = 0;

  // Compute joint angles for the middle finger (same parameters, could be changed if needed)
  computeFingerJoints(full_string_length_middle, 
                      a3, b3, a2, b2, a1, b1,
                      max_j3_length, max_j2_length, max_j1_length,
                      max_j3_angle, max_j2_angle, max_j1_angle,
                      j1_angle_middle, j2_angle_middle, j3_angle_middle);

  // Set message for middle finger
  finger_pos_middle.header.stamp = nh.now();
  finger_pos_middle.theta_M = j1_angle_middle;
  finger_pos_middle.theta_P = j2_angle_middle;
  finger_pos_middle.theta_D = j3_angle_middle;

  // Publish both messages
  chatter_index.publish(&finger_pos_index);
  chatter_middle.publish(&finger_pos_middle);

  nh.spinOnce();
  delay(dt * 1000);
}

// ---------------- Helper Functions ----------------

double rad(double deg) {
  return deg * (3.14 / 180);
}

double deg(double rad) {
  return rad * (180 / 3.14);
}

// convert potentiometer value to cm
double get_length(double pot_val) {
  double length = pot_val / cm_travled;
  return length; // in cm
}

// use cosine law to get joint angle
double get_joint_angle(double a, double b, double c) {
  double cos_angle = (sq(c) - sq(a) - sq(b)) / (-2 * a * b);

  // Clamp value within [-1, 1]
  if (cos_angle > 1.0) {
    cos_angle = 1.0;
  } else if (cos_angle < -1.0) {
    cos_angle = -1.0;
  }

  double angle = acos(cos_angle); // radians
  angle = deg(angle); // convert to degrees
  return angle;
}

// Compute the joint angles for one finger
void computeFingerJoints(double full_string_length,
                         double a3, double b3, double a2, double b2, double a1, double b1,
                         double max_j3_length, double max_j2_length, double max_j1_length,
                         double max_j3_angle, double max_j2_angle, double max_j1_angle,
                         double &j1_angle, double &j2_angle, double &j3_angle) 
{
  double string_length = full_string_length;
  
  // Joint 3 calculation
  if (full_string_length >= 0 && full_string_length <= max_j3_length) {
    if (full_string_length < max_j3_length / 2) {
      j3_angle = get_joint_angle((a3 - 0.09), (b3 - 0.09), string_length);
    } else {
      j3_angle = get_joint_angle(a3, b3, string_length);
    }
    if (j3_angle > max_j3_angle) {
      j3_angle = max_j3_angle;
    }
    j2_angle = 0;
    j1_angle = 0;
  }
  // Joint 2 calculation
  else if (full_string_length >= max_j3_length && full_string_length <= max_j2_length) {
    string_length = string_length - max_j3_length;
    j3_angle = max_j3_angle;
    j2_angle = get_joint_angle(a2, b2, string_length);
    if (j2_angle > max_j2_angle) {
      j2_angle = max_j2_angle;
    }
    j1_angle = 0;
  }
  // Joint 1 calculation
  else if (full_string_length >= max_j2_length && full_string_length <= max_j1_length) {
    string_length = string_length - max_j2_length;
    j3_angle = max_j3_angle;
    j2_angle = max_j2_angle;
    j1_angle = get_joint_angle(a1, b1, string_length);
    if (j1_angle > max_j1_angle) {
      j1_angle = max_j1_angle;
    }
  } else {
    // Outside expected range, set all to 0 or handle appropriately
    j1_angle = 0;
    j2_angle = 0;
    j3_angle = 0;
  }
}

