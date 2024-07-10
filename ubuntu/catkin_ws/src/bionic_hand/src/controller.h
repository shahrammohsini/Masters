#ifndef CONTROLLER_PUBLISHER_H
#define CONTROLLER_PUBLISHER_H

#include "ros/ros.h"
#include "bionic_hand/ControlCommands.h"

class ControllerPublisher {
public:
    ControllerPublisher(const std::string& finger_name);
    void publishData(double PWM);
    void run();  // Handles the ros::spin or ros::spinOnce inside

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};

#endif // CONTROLLER_PUBLISHER_H