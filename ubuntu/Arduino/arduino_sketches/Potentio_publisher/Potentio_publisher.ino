
#include <ros.h>
#include <std_msgs/Int32.h>  // Include header for integer message type

ros::NodeHandle  nh;

std_msgs::Int32 int_msg;  // Use std_msgs::Int32 for integer message
ros::Publisher chatter("int_topic", &int_msg);  // Correctly initialize the publisher

int msg = 0;  // This will be our message

void setup()
{
  nh.initNode();
  nh.advertise(chatter);

}

void loop()
{
  int sensorValue = analogRead(A0);
  msg = sensorValue;
  msg = map(msg,0, 1023, 1800, 3400 );
  int_msg.data = msg;  // Set the message data
  chatter.publish(&int_msg);  // Publish the message
  nh.spinOnce();
  delay(400);  // Delay to slow down the loop, if needed
}                      