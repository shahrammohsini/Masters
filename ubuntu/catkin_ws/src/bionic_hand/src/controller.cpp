#include "bionic_hand/ControlCommands.h"
#include <ros/ros.h>
#include <iostream>
#include "bionic_hand/FingerPos.h"
#include "controller.h"
//libraries for reading csv files
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>//for matrix calculations 


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

// Define a custom data type to hold open loop data in csv file
struct openLoop_data{
    double time;
    double theta_m;
    double theta_D_joint;
    double theta_P_joint;
    double theta_M_joint;
    int pwm;
};

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

/*Generate and return a dynamic matrix of given size (prediction horizon, control horizon) */
Eigen::MatrixXd Controller::generate_Dynamic_Matrix(int prediction_horizon, int control_horizon){
    std::cout <<"test" << std::endl;

    //open file
    std:: string filePath = "/home/shahram/Documents/GitHub/Masters/ubuntu/python/Finger_Multi_Model_Approach/Data/Finger_Validation_Sim_data.csv";
    std::ifstream file(filePath);

    //check if file is open
    if(!file.is_open()){
        std::cerr <<"Error opening file: " <<filePath <<std::endl;
        return Eigen::MatrixXd(); //return empty eigen vecotr to indicate error
    }


    std::vector<openLoop_data> dataset; //create a vecotor of type openLoop_data to hold data called dataset
    std::string line;

    //skip the header
    std::getline(file, line);

    //collect the data
    while (std::getline(file,line)){
        std::istringstream s(line);
        std::string field;
        openLoop_data data;

        //seperate the data
        std::getline(s, field, ',');
        data.time = std::stod(field);
        std::getline(s, field, ',');
        data.theta_m = std::stod(field);
        std::getline(s, field, ',');
        data.theta_D_joint = std::stod(field);
        std::getline(s, field, ',');
        data.theta_P_joint = std::stod(field);
        std::getline(s, field, ',');
        data.theta_M_joint = std::stod(field);
        std::getline(s, field, ',');
        data.pwm = std::stoi(field);

        //add the data to the vecotr
        dataset.push_back(data);
    }

    file.close();


    Eigen::MatrixXd DM(prediction_horizon,control_horizon); //dynamic matrix vector
    DM.setZero(); //populate with zeros
    //populate dynamic matrix
     for (int i =0; i < control_horizon; i++) { //create columns
        for(int j=0; j < prediction_horizon - i; j++){ //create rows
            const auto& d = dataset[j]; //d holds the current row
            DM(i + j, i) = d.theta_D_joint; //fill current row for current column
        }
        
     }
    std::cout << "Matrix: \n" << DM <<std::endl;

    return DM;
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





    // while (ros::ok()) {

    //     // select the controlled joint. Note: only one joint can move at a time. J_D and J_P must be at max to move J_M
    //     // if(setpoint_M == 0){
    //     //     if(setpoint_P == 0){
    //     //         //control D joint
    //     //         measured_pos = theta_D + theta_P + theta_M;
    //     //         setpoint = setpoint_D;
    //     //         control_effort = PID_Control(setpoint, measured_pos, kp_D, ki_D, kd_D, 0.01);
    //     //     }
    //     //     else{
    //     //         //control P joint
    //     //         measured_pos = theta_D + theta_P + theta_M;
    //     //         setpoint = max_D_joint_angle + setpoint_P;
    //     //         control_effort = PID_Control(setpoint, measured_pos, kp_P, ki_P, kd_P, 0.01);
    //     //     }
    //     // }
    //     // else{
    //     //     //control M joint
    //     //     measured_pos = theta_D + theta_P + theta_M;
    //     //     setpoint = max_D_joint_angle + max_P_joint_angle + setpoint_M;
    //     //     control_effort = PID_Control(setpoint, measured_pos, kp_M, ki_M, kd_M, 0.01);
    //     // }



       

    //     publishData(control_effort); //run publishData method
    //     ros::spinOnce();
    //     rate.sleep();
    // }


    //MPC testing
    int N = 7; //prediction horizon
    int nu = 3; //control horizon
    //generate dynamic matrix A
    Eigen::MatrixXd A;
    A = generate_Dynamic_Matrix(N, nu);
    std::cout << "Matrix: \n" << A <<std::endl;

    //mpc parameters
    Eigen::MatrixXd u = Eigen::MatrixXd::Zero(nu, 1); // control input -  1 dimentional matrix filled with zeros
    Eigen::MatrixXd PHI = Eigen::MatrixXd::Zero(N, 1); // model adjustment (correciton) matrix
    Eigen::MatrixXd I_Matrix = Eigen::MatrixXd::Identity(nu, nu); // model adjustment (correciton) matrix
    Eigen::MatrixXd measured_pos = Eigen::MatrixXd::Zero(nu, 1); // model adjustment (correciton) matrix
    Eigen::MatrixXd error = Eigen::MatrixXd::Zero(N, 1); // error
    Eigen::MatrixXd delta_u = Eigen::MatrixXd::Zero(nu, 1); // delta_u
    Eigen::MatrixXd u_prev = Eigen::MatrixXd::Zero(nu, 1); // previous input
    Eigen::MatrixXd delta_y = Eigen::MatrixXd::Zero(N, 1); // change in predicted output
    int LAMBDA = 1.01; //penalty factor
    Eigen::MatrixXd y_hat = Eigen::MatrixXd::Zero(N, 1); // pridected output matrix
    double setpoint_Val = 50; //desired setpoint



    //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) calculate first part of delta_u outside the loop for effecincy
    Eigen::MatrixXd A_T = A.transpose();//transpose the A matrix
    std::cout << "transpose Matrix: \n" << A_T <<std::endl; 
    Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N,1,setpoint_Val); //matrix of setpoint values
    Eigen::MatrixXd LambdaI = LAMBDA*I_Matrix;
    std::cout << "lambda*I: \n" << LambdaI <<std::endl; 
    Eigen::MatrixXd ATA = A_T*A;
    std::cout << "ATA: \n" << ATA <<std::endl; 
    Eigen::MatrixXd ATA_LambdaI = ATA - LambdaI;
    std::cout << "ATA_LambdaI: \n" << ATA_LambdaI <<std::endl; 
    Eigen::MatrixXd ATA_LambdaI_Inv = ATA_LambdaI.inverse();
    std::cout << "ATA_LambdaI_Inv: \n" << ATA_LambdaI_Inv <<std::endl; 
    Eigen::MatrixXd du = ATA_LambdaI_Inv*A_T; //(ATA+λI)^−1)AT this part of the formula is calculated offline for effeciency. now all that's left is to multiply by error
    std::cout << "du: \n" << du <<std::endl; 

    int max_pwm = 800;
    int min_pwm = -800;

    while (ros::ok()) {

        measured_pos = Eigen::MatrixXd::Constant(N, 1, theta_D); //set all values of measured pos to theta_D
        // std::cout << "measured_pos: \n" << measured_pos <<std::endl; 
        
        PHI = measured_pos - y_hat; //difference between predicted pos (y_hat) and measured pos to obtain model inaccuracy.
        // std::cout << "PHI: \n" << PHI <<std::endl; 
        y_hat = y_hat + PHI; //correct prediciton inaccuracy
        // std::cout << "y_hat: \n" << y_hat <<std::endl; 

        error = setpoint - y_hat; //error
        // std::cout << "error: \n" << error <<std::endl; 

        delta_u = du*error; //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) // delta_u is the control input step
        std::cout << "delta_u: \n" << delta_u <<std::endl; 
        u = u_prev + delta_u; //calculate control move vector

        //add control input limits here
        for(int i; i < nu; i++){
            if (u(i,0) > max_pwm){
                u(i,0) = max_pwm;
            }
            else if (u(i,0) < min_pwm){
                u(i,0) = min_pwm;
            }
        }

        delta_u = u - u_prev; //revaluate delta_u to account for the limits you've introduced to u


        //send u(0) to plant
        control_effort = u(0);
        publishData(control_effort); //run publishData method
        ros::spinOnce();
        rate.sleep();



        delta_y = A*delta_u; //caluclate change in predicted output: y_1 - y_0 = (a_1)(Delta u_0)
        std::cout << "delta_y: \n" << delta_y <<std::endl; 
        y_hat = y_hat + delta_y; // calculate new prediction for output
        std::cout << "y_hat: \n" << y_hat <<std::endl; 


        // advance prediction horizon one step ahead (Shift all elements to the left). y_hat[i] = y_hat[i+1]
        double y_hat_last = y_hat(0);
        for (int i = 1; i < y_hat.size(); ++i) {
            y_hat(i - 1) = y_hat(i);
        }
        //ensure the last element becomes the first ([a,b,c,d] ----> [b,c,d,a])
        y_hat(y_hat.size() - 1) = y_hat_last;  // Setting the last element to first element
        std::cout << "y_hat: \n" << y_hat <<std::endl; 

        //update control move
        u_prev = u;



        



    }



    


    


    






}