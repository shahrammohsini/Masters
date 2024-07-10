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
    void run();  // Handles the ros::spin or ros::spinOnce inside
    double updatePID(double setpoint, double measured_position, double dt);

    //joint angles
    double theta_M = 0;
    double theta_P = 0;
    double theta_D = 0;



private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    bionic_hand::FingerPos position;
    // Dynamic states for PID
    double integral = 0.0;
    double previous_error = 0.0;
    double error;
    double derivative;
    double control_effort = 0;
    // PID coefficients
    double kp;  // Proportional gain
    double ki; // Integral gain
    double kd; // Derivative gain
};

#endif // CONTROLLER_PUBLISHER_H