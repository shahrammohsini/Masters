#include <ros/ros.h>
#include <bionic_hand/SetPWM.h> // Import SetPosition.msg file
#include <iostream>
#include "finger.h"
using namespace std;
#include "std_msgs/Int32.h"
#include "controller.h"
#include <chrono>
#include <thread>

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

    float max_M_joint_angle;
    float max_P_joint_angle;
    float max_D_joint_angle;


    // sleep for 1 sec
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout <<"begin------------------------------------------------------------------------"<< std::endl;


    // ros::init(argc, argv, "move_motor"); // Initialize ROS node
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh; // Create a NodeHandle
    // Create a finger object
    Finger middle_finger(1, "middle_finger"); // Set motor ID. Middle finger's motor has id 1.
    // middle_finger.name = "middle_finger";

    // controller object for middle finger
    Controller control_middle_finger(middle_finger.name); //create a middle finger object for Controller class

    
    // Create the publisher with queue size 10. **NOTE: This line should later be changed when new fingers are added. It should be made so that all the 
    //fingers use the same publisher and publish a new pos for all the fingers at the same time. The set position function will just look for the pos of the specific finger. Should also look at the option of moving the publisher decleration to finger.cpp 
    middle_finger.pub = nh.advertise<bionic_hand::SetPWM>("/set_pwm", 10);

    
    // ros::init(argc, argv, "int_subscriber");
    ros::Subscriber sub = nh.subscribe("/int_topic", 1000, intCallback);

    
    //Load max joint angles form parameter yaml file
    nh.getParam("/middle_finger/joint_limits/max_M_joint_angle", max_M_joint_angle);
    nh.getParam("/middle_finger/joint_limits/max_P_joint_angle", max_P_joint_angle);
    nh.getParam("/middle_finger/joint_limits/max_D_joint_angle", max_D_joint_angle);


    float setpoint_M = 15;
    float setpoint_P = max_P_joint_angle;
    float setpoint_D = max_D_joint_angle;

    control_middle_finger.run(setpoint_M, setpoint_P, setpoint_D);
    int input;
    

    return 0;
}
