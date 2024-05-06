#include <ros/ros.h>
#include <bionic_hand/SetPWM.h> // Import SetPosition.msg file
#include <iostream>
#include "finger.h"
using namespace std;
#include "std_msgs/Int32.h"

std_msgs::Int32 int_msg;

int global_int_value = 0;

void intCallback(const std_msgs::Int32::ConstPtr& msg)
{
    global_int_value = msg->data;
    // ROS_INFO("Received: [%d]", global_int_value);
    // cout << "new msg " << msg << endl;
    // ROS_INFO("Received: [%d]", msg->data);
    // middle_finger.setPosition(global_int_value);

}



int main(int argc, char** argv) {
    ros::init(argc, argv, "move_motor"); // Initialize ROS node
    ros::NodeHandle nh; // Create a NodeHandle
    // Create a finger object
    Finger middle_finger(1); // Set motor ID. Middle finger's motor has id 1.
    
    // Create the publisher with queue size 10. **NOTE: This line should later be changed when new fingers are added. It should be made so that all the 
    //fingers use the same publisher and publish a new pos for all the fingers at the same time. The set position function will just look for the pos of the specific finger. Should also look at the option of moving the publisher decleration to finger.cpp 
    middle_finger.pub = nh.advertise<bionic_hand::SetPWM>("/set_pwm", 10);

    
    // ros::init(argc, argv, "int_subscriber");
    ros::Subscriber sub = nh.subscribe("/int_topic", 1000, intCallback);



    int input;
    
    while (ros::ok()) {
        ros::spinOnce();
        // cout << "Enter New Pos: " << global_int_value <<endl;
        cout << "Enter New Pos: " <<endl;

        cin >> input;
        middle_finger.setPWM(input);

        // middle_finger.setPosition(input);

        ros::Rate loop_rate(1);
    }

    // ros::spin();
    return 0;
}
