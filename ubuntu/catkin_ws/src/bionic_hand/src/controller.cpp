#include "bionic_hand/ControlCommands.h"
#include <ros/ros.h>

#include "controller.h"

ControllerPublisher::ControllerPublisher(const std::string& finger_name) {
    pub_ = nh_.advertise<bionic_hand::ControlCommands>("Control_Command", 1000);
}

//Create publishData method to publish data
void ControllerPublisher::publishData(double PWM)
{
   
  bionic_hand::ControlCommands msg;
    msg.PWM = PWM;

    pub_.publish(msg);
    ROS_INFO("Published PWM command");

}

//run method holding main control loop
void ControllerPublisher::run() {
    ros::Rate rate(10); // 10 Hz


    while (ros::ok()) {
        // You might want to move or remove this loop, depending on your use case.
        // For instance, publish data could be triggered by a timer or external event instead.
    
        
        publishData(4); //run publishData method
        ros::spinOnce();
        rate.sleep();
    }
}