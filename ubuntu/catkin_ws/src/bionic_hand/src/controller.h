#ifndef CONTROLLER_PUBLISHER_H
#define CONTROLLER_PUBLISHER_H

#include "ros/ros.h"
#include "bionic_hand/ControlCommands.h"
#include "bionic_hand/FingerPos.h"
#include <eigen3/Eigen/Dense>//for matrix calculations 


class Controller{
public:
    Controller(const std::string& finger_name);
    void publishData(double PWM);
    void fingerPositionCallback(const bionic_hand::FingerPos& msg);
    void run(float setpoint_M, float setpoint , float setpoint_D);  // Handles the ros::spin or ros::spinOnce inside
    double PID_Control(double setpoint, double measured_position, double kp, double ki, double kd, double dt);
    Eigen::MatrixXd generate_Dynamic_Matrix(int prediction_horizon, int control_horizon);
    double MPC_Control(Eigen::MatrixXd setpoint, Eigen::MatrixXd measured_position, int N, int nu);


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
    //MPC coefficients
    //mpc parameters
    int N;
    int nu;
    Eigen::MatrixXd A;
    Eigen::MatrixXd u;
    Eigen::MatrixXd PHI;
    Eigen::MatrixXd I_Matrix;
    Eigen::MatrixXd measured_posi;
    Eigen::MatrixXd errors;
    Eigen::MatrixXd delta_u;
    Eigen::MatrixXd u_prev;
    Eigen::MatrixXd delta_y;
    int LAMBDA;
    Eigen::MatrixXd y_hat;
    //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) calculate first part of delta_u outside the loop for effecincy
    Eigen::MatrixXd A_T;
    Eigen::MatrixXd LambdaI;
    Eigen::MatrixXd ATA;
    Eigen::MatrixXd ATA_LambdaI;
    Eigen::MatrixXd ATA_LambdaI_Inv;
    Eigen::MatrixXd du;
    int max_pwm;
    int min_pwm;
    double setpoint_val;

};

#endif // CONTROLLER_PUBLISHER_H