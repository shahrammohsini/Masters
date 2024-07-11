#ifndef CONTROLLER_PUBLISHER_H
#define CONTROLLER_PUBLISHER_H

#include "ros/ros.h"
#include "bionic_hand/ControlCommands.h"
#include "bionic_hand/FingerPos.h"

class Controller{
public:
    Controller(const std::string& finger_name);
    void publishData(double PWM);
    void fingerPositionCallback(const bionic_hand::FingerPos& msg);
    void run(float setpoint_M, float setpoint , float setpoint_D);  // Handles the ros::spin or ros::spinOnce inside
    double PID_Control(double setpoint, double measured_position, double kp, double ki, double kd, double dt);


    //joint angles
    double theta_M = 0;
    double theta_P = 0;
    double theta_D = 0;



private:
    ros::NodeHandle nh;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    bionic_hand::FingerPos position;
    // Dynamic states for PID
    double integral = 0.0;
    double previous_error = 0.0;
    double error;
    double derivative;
    double control_effort = 0;
    double setpoint;
    double measured_pos;
    // PID coefficients
    double kp_M;  // Proportional gain
    double ki_M; // Integral gain
    double kd_M; // Derivative gain
    double kp_P;  // Proportional gain
    double ki_P; // Integral gain
    double kd_P; // Derivative gain
    double kp_D;  // Proportional gain
    double ki_D; // Integral gain
    double kd_D; // Derivative gain
    //Finger Parameters
    float max_M_joint_angle;
    float max_P_joint_angle;
    float max_D_joint_angle;
};

#endif // CONTROLLER_PUBLISHER_H