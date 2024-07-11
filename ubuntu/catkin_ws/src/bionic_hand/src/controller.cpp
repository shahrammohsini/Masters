#include "bionic_hand/ControlCommands.h"
#include <ros/ros.h>
#include <iostream>
#include "bionic_hand/FingerPos.h"
#include "controller.h"


Controller::Controller(const std::string& finger_name) {
    pub_ = nh.advertise<bionic_hand::ControlCommands>("Control_Command", 1000); //initialize publisher node
    sub_ = nh.subscribe("Updated_Finger_Position", 1000, &Controller::fingerPositionCallback, this);  //initialize subscriber node
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


double Controller::PID_Control(double setpoint, double measured_position, double kp, double ki, double kd, double dt){

    std::cout <<"setpoint: " << setpoint << std::endl;
    std::cout <<"measured_pos: " << measured_position << std::endl;

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
void Controller::run(float setpoint_M, float setpoint_P, float setpoint_D) {
    ros::Rate rate(10); // 10 Hz

    //Load PID values form parameter yaml file
    nh.getParam("middle_finger/M_joint/kp", kp_M);
    nh.getParam("middle_finger/M_joint/ki", ki_M);
    nh.getParam("middle_finger/M_joint/kd", kd_M);
    nh.getParam("middle_finger/P_joint/kp", kp_P);
    nh.getParam("middle_finger/P_joint/ki", ki_P);
    nh.getParam("middle_finger/P_joint/kd", kd_P);
    nh.getParam("middle_finger/D_joint/kp", kp_D);
    nh.getParam("middle_finger/D_joint/ki", ki_D);
    nh.getParam("middle_finger/D_joint/kd", kd_D);
    ///Load max joint angles form parameter yaml file
    nh.getParam("/middle_finger/joint_limits/max_M_joint_angle", max_M_joint_angle);
    nh.getParam("/middle_finger/joint_limits/max_P_joint_angle", max_P_joint_angle);
    nh.getParam("/middle_finger/joint_limits/max_D_joint_angle", max_D_joint_angle);




    while (ros::ok()) {

        // select the controlled joint. Note: only one joint can move at a time. J_D and J_P must be at max to move J_M
        if(setpoint_M == 0){
            if(setpoint_P == 0){
                //control D joint
                measured_pos = theta_D + theta_P + theta_M;
                setpoint = setpoint_D;
                control_effort = PID_Control(setpoint, measured_pos, kp_D, ki_D, kd_D, 0.01);
            }
            else{
                //control P joint
                measured_pos = theta_D + theta_P + theta_M;
                setpoint = max_D_joint_angle + setpoint_P;
                control_effort = PID_Control(setpoint, measured_pos, kp_P, ki_P, kd_P, 0.01);
            }
        }
        else{
            //control M joint
            measured_pos = theta_D + theta_P + theta_M;
            setpoint = max_D_joint_angle + max_P_joint_angle + setpoint_M;
            control_effort = PID_Control(setpoint, measured_pos, kp_M, ki_M, kd_M, 0.01);
        }



        publishData(control_effort); //run publishData method
        ros::spinOnce();
        rate.sleep();
    }
}