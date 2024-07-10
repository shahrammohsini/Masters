#include "bionic_hand/ControlCommands.h"
#include <ros/ros.h>
#include <iostream>
#include "bionic_hand/FingerPos.h"
#include "controller.h"


Controller::Controller(const std::string& finger_name) {
    pub_ = nh_.advertise<bionic_hand::ControlCommands>("Control_Command", 1000); //initialize publisher node
    sub_ = nh_.subscribe("Updated_Finger_Position", 1000, &Controller::fingerPositionCallback, this);  //initialize subscriber node
}

//Create publishData method to publish data
void Controller::publishData(double PWM)
{
   
  bionic_hand::ControlCommands msg;
    msg.PWM = PWM;

    pub_.publish(msg);
    // ROS_INFO("Published PWM command");

}

//create a subscriber node to get the latest pos info
void Controller::fingerPositionCallback(const bionic_hand::FingerPos& msg){
    position = msg;
    theta_M = position.theta_M;
    theta_P = position.theta_P;
    theta_D = position.theta_D;

    // std::cout <<"position" << position.theta_D << std::endl;
}


double Controller::updatePID(double setpoint, double measured_position, double dt){
    // std::cout <<"measured pos " << measured_position << std::endl;

    error = (setpoint - measured_position);
    // std::cout <<"error " << error << std::endl;

    integral += error * dt;
    derivative = (error - previous_error)/dt;
    previous_error = error;
    control_effort = kp * error + ki * integral + kd * derivative;
    // std::cout <<"control effort " << control_effort << std::endl;
    if (control_effort > 800){control_effort = 800;}
    return control_effort;
}

//run method holding main control loop
void Controller::run() {
    ros::Rate rate(10); // 10 Hz

    //Load PID values form parameter yaml file
    nh_.getParam("middle_finger/D_joint/kp", kp);
    nh_.getParam("middle_finger/D_joint/ki", ki);
    nh_.getParam("middle_finger/D_joint/kd", kd);



    while (ros::ok()) {


        control_effort = updatePID(25, theta_D, 0.01);


        publishData(control_effort); //run publishData method
        ros::spinOnce();
        rate.sleep();
    }
}