

#include <ros.h>
#include <std_msgs/Int32.h>  // Include header for integer message type
#include "bionic_hand/FingerPos.h"



// // Structure to hold the message data
// struct FingerPos {
//     double theta_M;
//     double theta_P;
//     double theta_D;
// };


ros::NodeHandle nh;

FingerPos finger_pos;  // Declare your custom message object
ros::Publisher chatter("finger_pos_topic", &finger_pos);  // Publisher for custom message type

void setup() {
  nh.initNode();  // Initialize the ROS node
  nh.advertise(chatter);  // Advertise your topic
}

void loop() {
  int sensorValue = analogRead(A0);  // Read the sensor value
  // Map the sensor value to your desired range
  // float mappedValue = map(sensorValue, 0, 1023, 1800, 3400);

  // Assign mapped values to your custom message fields
  finger_pos.theta_M = 1;
  finger_pos.theta_P = 2 + 1;  // Example calculation
  finger_pos.theta_D = 3 + 2;  // Example calculation

  chatter.publish(&finger_pos);  // Publish the message
  nh.spinOnce();  // Handle ROS events
  delay(400);  // Delay to slow down the loop
}
