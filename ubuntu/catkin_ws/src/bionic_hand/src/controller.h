#ifndef CONTROLLER_PUBLISHER_H
#define CONTROLLER_PUBLISHER_H

#include "ros/ros.h"
#include "bionic_hand/ControlCommands.h"
#include "bionic_hand/FingerPos.h"
#include <eigen3/Eigen/Dense>//for matrix calculations 
#include <eigen3/Eigen/Dense> // For matrix calculations
#include <tuple> // For returning multiple matrices



class Controller{
public:
    Controller(int ID, const std::string& finger_name);
    void publishData(double PWM);
    void fingerPositionCallback(const bionic_hand::FingerPos& msg);
    void run(float setpoint_M, float setpoint , float setpoint_D);  // Handles the ros::spin or ros::spinOnce inside
    double PID_Control(double setpoint, double measured_position, double kp, double ki, double kd, double dt);
    // Eigen::MatrixXd generate_Dynamic_Matrix(int prediction_horizon, int control_horizon);
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> generate_Dynamic_Matrix(int prediction_horizon_D, int control_horizon_D, int prediction_horizon_P, int control_horizon_P, int prediction_horizon_M, int control_horizon_M);
    //std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> generate_Dynamic_Matrix_rev(int prediction_horizon_D_rev, int control_horizon_D_rev, int prediction_horizon_P_rev, int control_horizon_P_rev, int prediction_horizon_M_rev, int control_horizon_M_rev);
    Eigen::MatrixXd generate_Dynamic_Matrix_rev(int prediction_horizon_D_rev, int control_horizon_D_rev, int prediction_horizon_P_rev, int control_horizon_P_rev, int prediction_horizon_M_rev, int control_horizon_M_rev);
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> generate_du_Matrix(int prediction_horizon_D, int control_horizon_D,int prediction_horizon_P, int control_horizon_P,int prediction_horizon_M, int control_horizon_M, int prediction_horizon_P_rev, int control_horizon_P_rev, Eigen::MatrixXd A_D, Eigen::MatrixXd A_P, Eigen::MatrixXd A_M, Eigen::MatrixXd A_P_rev, float LAMBDA_D, float LAMBDA_P, float LAMBDA_M, float LAMBDA_M_rev);

    double MPC_Control_D(Eigen::MatrixXd setpoint, double measured_position_D, int N_D, int nu_D, Eigen::MatrixXd A_D);
    double MPC_Control_P(Eigen::MatrixXd setpoint, double measured_position_P, int N_P, int nu_P, Eigen::MatrixXd A_P);
    double MPC_Control_M(Eigen::MatrixXd setpoint, Eigen::MatrixXd measured_position, int N_M, int nu_M, Eigen::MatrixXd A_M);
    double MPC_Control_P_Reverse(Eigen::MatrixXd setpoint, double measured_position_P_rev, int N_P_rev, int nu_P_rev, Eigen::MatrixXd A_P_rev);
    double convert_Voltage_to_PWM(double voltage);
   Eigen::MatrixXd addDeadTime(const Eigen::MatrixXd& dynamicMatrix, int deadTimeSteps);
   Eigen::MatrixXd addDeadTime_rev(const Eigen::MatrixXd& dynamicMatrix, int deadTimeSteps);



    //joint angles
    double theta_M = 0;
    double theta_P = 0;
    double theta_D = 0;



private:
    ros::NodeHandle nh;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    bionic_hand::FingerPos position;
    int finger_ID;
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
        //forward
    float dt;
    int N_D;
    int nu_D;
    int deadTimeSteps_D;
    float LAMBDA_D;
    int N_P;
    int nu_P;
    int deadTimeSteps_P;
    float LAMBDA_P;
    int N_M;
    int nu_M;
    int deadTimeSteps_M;
    float LAMBDA_M;
        //reverse
    int N_P_rev;
    int nu_P_rev;
    int deadTimeSteps_P_rev;
    float LAMBDA_P_rev;
        //I matrix forward
    Eigen::MatrixXd I_Matrix_D;
    Eigen::MatrixXd I_Matrix_P;
    Eigen::MatrixXd I_Matrix_M;
        //I matrix reverse
    Eigen::MatrixXd I_Matrix_P_rev;

    //Forward
    //D_joint controller parameters
    Eigen::MatrixXd A_D;
    Eigen::MatrixXd u_D;
    double PHI_D;
    Eigen::MatrixXd errors_D;
    Eigen::MatrixXd delta_u_D;
    Eigen::MatrixXd u_prev_D;
    Eigen::MatrixXd delta_y_D;
    Eigen::MatrixXd y_hat_D;
    //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) calculate first part of delta_u outside the loop for effecincy
    Eigen::MatrixXd A_T_D;
    Eigen::MatrixXd LambdaI_D;
    Eigen::MatrixXd ATA_D;
    Eigen::MatrixXd ATA_LambdaI_D;
    Eigen::MatrixXd ATA_LambdaI_Inv_D;
    Eigen::MatrixXd du_D;

    //P_joint controller parameters
    Eigen::MatrixXd A_P;
    Eigen::MatrixXd u_P;
    double PHI_P;
    Eigen::MatrixXd errors_P;
    Eigen::MatrixXd delta_u_P;
    Eigen::MatrixXd u_prev_P;
    Eigen::MatrixXd delta_y_P;
    Eigen::MatrixXd y_hat_P;
    //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) calculate first part of delta_u outside the loop for effecincy
    Eigen::MatrixXd A_T_P;
    Eigen::MatrixXd LambdaI_P;
    Eigen::MatrixXd ATA_P;
    Eigen::MatrixXd ATA_LambdaI_P;
    Eigen::MatrixXd ATA_LambdaI_Inv_P;
    Eigen::MatrixXd du_P;

    //M_joint controller parameters
    Eigen::MatrixXd A_M;
    Eigen::MatrixXd u_M;
    Eigen::MatrixXd PHI_M;
    Eigen::MatrixXd measured_posi_M;
    Eigen::MatrixXd errors_M;
    Eigen::MatrixXd delta_u_M;
    Eigen::MatrixXd u_prev_M;
    Eigen::MatrixXd delta_y_M;
    Eigen::MatrixXd y_hat_M;
    //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) calculate first part of delta_u outside the loop for effecincy
    Eigen::MatrixXd A_T_M;
    Eigen::MatrixXd LambdaI_M;
    Eigen::MatrixXd ATA_M;
    Eigen::MatrixXd ATA_LambdaI_M;
    Eigen::MatrixXd ATA_LambdaI_Inv_M;
    Eigen::MatrixXd du_M;

    //Reverse
    //P_joint controller parameters reverse
    Eigen::MatrixXd A_P_rev;
    Eigen::MatrixXd u_P_rev;
    double PHI_P_rev;
    Eigen::MatrixXd errors_P_rev;
    Eigen::MatrixXd delta_u_P_rev;
    Eigen::MatrixXd u_prev_P_rev;
    Eigen::MatrixXd delta_y_P_rev;
    Eigen::MatrixXd y_hat_P_rev;
    //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) calculate first part of delta_u outside the loop for effecincy
    Eigen::MatrixXd A_T_P_rev;
    Eigen::MatrixXd LambdaI_P_rev;
    Eigen::MatrixXd ATA_P_rev;
    Eigen::MatrixXd ATA_LambdaI_P_rev;
    Eigen::MatrixXd ATA_LambdaI_Inv_P_rev;
    Eigen::MatrixXd du_P_rev;



    int max_pwm;
    int min_pwm;
    double max_Voltage = 5;
    double min_Voltage = -5;
    double setpoint_val;

};

#endif // CONTROLLER_PUBLISHER_H