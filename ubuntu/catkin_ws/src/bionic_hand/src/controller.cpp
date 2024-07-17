#include "bionic_hand/ControlCommands.h"
#include <ros/ros.h>
#include <iostream>
#include "bionic_hand/FingerPos.h"
#include "controller.h"
#include <unistd.h> //needed for sleep funciton
//libraries for reading csv files
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>//for matrix calculations 
//allow for returning multiple matracies
#include <utility>  // For std::pair
#include <tuple>    // For std::tuple


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
    std::cout <<"measured_position: " << measured_position << std::endl;

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
// Function signature returning a tuple of three matrices
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> Controller::generate_Dynamic_Matrix(int prediction_horizon, int control_horizon){
   
    Eigen::MatrixXd DM_j_D(prediction_horizon, control_horizon);
    Eigen::MatrixXd DM_j_P(prediction_horizon, control_horizon);
    Eigen::MatrixXd DM_j_M(prediction_horizon, control_horizon);

    std::cout <<"test" << std::endl;

    //open file
    std:: string filePath = "/home/shahram/Documents/GitHub/Masters/ubuntu/python/Finger_Multi_Model_Approach/Data/Finger_Validation_Sim_data.csv";
    std::ifstream file(filePath);

    //check if file is open
    if(!file.is_open()){
        std::cerr <<"Error opening file: " <<filePath <<std::endl;
        return std::make_tuple(Eigen::MatrixXd(), Eigen::MatrixXd(), Eigen::MatrixXd()); //return empty eigen vecotrs to indicate error
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

    DM_j_D.setZero(); //populate with zeros
    //populate dynamic matrix
     for (int i =0; i < control_horizon; i++) { //create columns
        for(int j=0; j < prediction_horizon - i; j++){ //create rows
            const auto& d = dataset[j]; //d holds the current row
            DM_j_D(i + j, i) = (d.theta_D_joint/max_D_joint_angle); //fill current row for current column. Also, normalize the angle
        }
     }
    std::cout << "Matrix_D: \n" << DM_j_D <<std::endl;

    DM_j_P.setZero(); //populate with zeros
    //populate dynamic matrix
     for (int i =0; i < control_horizon; i++) { //create columns
        for(int j=0; j < prediction_horizon - i; j++){ //create rows
            const auto& d = dataset[j+6]; //d holds the current row
            DM_j_P(i + j, i) = (d.theta_P_joint/max_P_joint_angle); //fill current row for current column. Also, normalize the angle
        }
     }
    std::cout << "Matrix_P: \n" << DM_j_P <<std::endl;

    DM_j_M.setZero(); //populate with zeros
    //populate dynamic matrix
     for (int i =0; i < control_horizon; i++) { //create columns
        for(int j=0; j < prediction_horizon - i; j++){ //create rows
            const auto& d = dataset[j+6 + 6]; //d holds the current row
            DM_j_M(i + j, i) = (d.theta_M_joint/max_M_joint_angle); //fill current row for current column. Also, normalize the angle
        }
     }
    std::cout << "Matrix_M: \n" << DM_j_M <<std::endl;

    return std::make_tuple(DM_j_D, DM_j_P, DM_j_M);
}


//**convert voltage to pwm. Input: voltae, output: pwm **/
double Controller::convert_Voltage_to_PWM(double voltage){
    double PWM = (voltage/max_Voltage)*max_pwm;
    return PWM;

}

double Controller::MPC_Control_D(Eigen::MatrixXd setpoint,Eigen::MatrixXd measured_position, int N, int nu, Eigen::MatrixXd A_D){
    // std::cout << "setpoint: \n" << setpoint <<std::endl; 
    std::cout << "measured pos_D: \n" << measured_position <<std::endl; 
    std::cout << "y_hat_D: \n" << y_hat_D <<std::endl; 
    
    PHI_D = measured_position - y_hat_D; //difference between predicted pos (y_hat) and measured pos to obtain model inaccuracy.
    // std::cout << "PHI: \n" << PHI <<std::endl; 
    y_hat_D = y_hat_D + PHI_D; //correct prediciton inaccuracy
    // std::cout << "y_hat: \n" << y_hat <<std::endl; 

    errors_D = setpoint - y_hat_D; //error
    // std::cout << "errors: \n" << errors <<std::endl; 

    // std::cout << "du: \n" << du <<std::endl; 
    std::cout << "test: \n" <<std::endl;
    std::cout << "du_D: \n" << du_D <<std::endl; 

    delta_u_D = du_D*errors_D; //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) // delta_u is the control input step

    u_D = u_prev_D + delta_u_D; //calculate control move vector
    // std::cout << "u: \n" << u <<std::endl; 
    //add control input limits here
    std::cout << "test2: \n" <<std::endl;

    
    for(int i = 0; i < nu; i++){
        if (u_D(i) > max_Voltage){
            u_D(i) = max_Voltage;
        }
        else if (u_D(i) < min_Voltage){
            u_D(i) = min_Voltage;
        }
    }

    std::cout << "test2: \n" <<std::endl;


    delta_u_D = u_D - u_prev_D; //revaluate delta_u to account for the limits you've introduced to u
    std::cout << "A_D: \n"<< A_D <<std::endl;

    delta_y_D = A_D*delta_u_D; //caluclate change in predicted output: y_1 - y_0 = (a_1)(Delta u_0)
    // std::cout << "delta_y: \n" << delta_y <<std::endl; 
   
    y_hat_D = y_hat_D + delta_y_D; // calculate new prediction for output
    // std::cout << "y_hat: \n" << y_hat <<std::endl; 
    std::cout << "test3: \n" <<std::endl;

    
    // advance prediction horizon one step ahead (Shift all elements to the left). y_hat[i] = y_hat[i+1]
    double y_hat_last_D = y_hat_D(0);
    for (int i = 1; i < y_hat_D.size(); ++i) {
        y_hat_D(i - 1) = y_hat_D(i);
    }
    //ensure the last element becomes the first ([a,b,c,d] ----> [b,c,d,a])
    y_hat_D(y_hat_D.size() - 1) = y_hat_last_D;  // Setting the last element to first element
    // std::cout << "y_hat: \n" << y_hat <<std::endl; 

    //update control move
    u_prev_D = u_D;
    // std::cout << "u_prev: \n" << u_prev <<std::endl; 


    return u_D(0);
}

double Controller::MPC_Control_P(Eigen::MatrixXd setpoint,Eigen::MatrixXd measured_position, int N, int nu, Eigen::MatrixXd A_P){
    // std::cout << "setpoint: \n" << setpoint <<std::endl; 
    // std::cout << "measured pos: \n" << measured_position <<std::endl; 
    std::cout<<" y_hat_P: "<< y_hat_P<<std::endl;
    std::cout<<" measured_position: "<< measured_position<<std::endl;

    
    PHI_P = measured_position - y_hat_P; //difference between predicted pos (y_hat) and measured pos to obtain model inaccuracy.
    // std::cout << "PHI: \n" << PHI <<std::endl; 
    std::cout<<"test 5: "<<std::endl;
    
    y_hat_P = y_hat_P + PHI_P; //correct prediciton inaccuracy
    // std::cout << "y_hat: \n" << y_hat <<std::endl; 

    errors_P = setpoint - y_hat_P; //error
    // std::cout << "errors: \n" << errors <<std::endl; 
    std::cout<<"test 5: "<<std::endl;

    // std::cout << "du: \n" << du <<std::endl; 


    delta_u_P = du_P*errors_P; //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) // delta_u is the control input step
    // std::cout << "delta_u: \n" << delta_u <<std::endl; 
    u_P = u_prev_P + delta_u_P; //calculate control move vector
    // std::cout << "u: \n" << u <<std::endl; 
    //add control input limits here
    for(int i = 0; i < nu; i++){
        if (u_P(i) > max_Voltage){
            u_P(i) = max_Voltage;
        }
        else if (u_P(i) < min_Voltage){
            u_P(i) = min_Voltage;
        }
    }
    std::cout<<"test 5: "<<std::endl;

    delta_u_P = u_P - u_prev_P; //revaluate delta_u to account for the limits you've introduced to u

    delta_y_P = A_P*delta_u_P; //caluclate change in predicted output: y_1 - y_0 = (a_1)(Delta u_0)
    // std::cout << "delta_y: \n" << delta_y <<std::endl; 
    y_hat_P = y_hat_P + delta_y_P; // calculate new prediction for output
    // std::cout << "y_hat: \n" << y_hat <<std::endl; 


    // advance prediction horizon one step ahead (Shift all elements to the left). y_hat[i] = y_hat[i+1]
    double y_hat_last_P = y_hat_P(0);
    for (int i = 1; i < y_hat_P.size(); ++i) {
        y_hat_P(i - 1) = y_hat_P(i);
    }
    //ensure the last element becomes the first ([a,b,c,d] ----> [b,c,d,a])
    y_hat_P(y_hat_P.size() - 1) = y_hat_last_P;  // Setting the last element to first element
    // std::cout << "y_hat: \n" << y_hat <<std::endl; 

    //update control move
    u_prev_P = u_P;
    // std::cout << "u_prev: \n" << u_prev <<std::endl; 


    return u_P(0);

}

double Controller::MPC_Control_M(Eigen::MatrixXd setpoint,Eigen::MatrixXd measured_position, int N, int nu, Eigen::MatrixXd A_M){
    // std::cout << "setpoint: \n" << setpoint <<std::endl; 
    // std::cout << "measured pos: \n" << measured_position <<std::endl; 
    
    PHI_M = measured_position - y_hat_M; //difference between predicted pos (y_hat) and measured pos to obtain model inaccuracy.
    // std::cout << "PHI: \n" << PHI <<std::endl; 
    y_hat_M = y_hat_M + PHI_M; //correct prediciton inaccuracy
    // std::cout << "y_hat: \n" << y_hat <<std::endl; 

    errors_M = setpoint - y_hat_M; //error
    // std::cout << "errors: \n" << errors <<std::endl; 

    // std::cout << "du: \n" << du <<std::endl; 


    delta_u_M = du_M*errors_M; //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) // delta_u is the control input step
    // std::cout << "delta_u: \n" << delta_u <<std::endl; 
    u_M = u_prev_M + delta_u_M; //calculate control move vector
    // std::cout << "u: \n" << u <<std::endl; 
    //add control input limits here
    for(int i = 0; i < nu; i++){
        if (u_M(i) > max_Voltage){
            u_M(i) = max_Voltage;
        }
        else if (u_M(i) < min_Voltage){
            u_M(i) = min_Voltage;
        }
    }

    delta_u_M = u_M - u_prev_M; //revaluate delta_u to account for the limits you've introduced to u

    delta_y_M = A_M*delta_u_M; //caluclate change in predicted output: y_1 - y_0 = (a_1)(Delta u_0)
    // std::cout << "delta_y: \n" << delta_y <<std::endl; 
    y_hat_M = y_hat_M + delta_y_M; // calculate new prediction for output
    // std::cout << "y_hat: \n" << y_hat <<std::endl; 


    // advance prediction horizon one step ahead (Shift all elements to the left). y_hat[i] = y_hat[i+1]
    double y_hat_last_M = y_hat_M(0);
    for (int i = 1; i < y_hat_M.size(); ++i) {
        y_hat_M(i - 1) = y_hat_M(i);
    }
    //ensure the last element becomes the first ([a,b,c,d] ----> [b,c,d,a])
    y_hat_M(y_hat_M.size() - 1) = y_hat_last_M;  // Setting the last element to first element
    // std::cout << "y_hat: \n" << y_hat <<std::endl; 

    //update control move
    u_prev_M = u_M;
    // std::cout << "u_prev: \n" << u_prev <<std::endl; 


    return u_M(0);

}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> Controller:: generate_du_Matrix(int prediction_horizon, int control_horizon, Eigen::MatrixXd A_D, Eigen::MatrixXd A_P, Eigen::MatrixXd A_M, float LAMBDA_D, float LAMBDA_P, float LAMBDA_M){
    //j_D
    //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) calculate first part of delta_u outside the loop for effecincy
    A_T_D = A_D.transpose();//transpose the A matrix
    std::cout << "A_D__: \n" << A_D <<std::endl; 
    LambdaI_D = LAMBDA_D*I_Matrix;
    ATA_D = A_T_D*A_D;
    ATA_LambdaI_D = ATA_D - LambdaI_D;
    ATA_LambdaI_Inv_D = ATA_LambdaI_D.inverse();
    du_D = ATA_LambdaI_Inv_D*A_T_D; //(ATA+λI)^−1)AT this part of the formula is calculated offline for effeciency. now all that's left is to multiply by error
    std::cout << "du: \n" << du_D <<std::endl; 

    //j_P
    A_T_P = A_P.transpose();//transpose the A matrix
    LambdaI_P = LAMBDA_P*I_Matrix;
    ATA_P = A_T_P*A_P;
    ATA_LambdaI_P = ATA_P - LambdaI_P;
    ATA_LambdaI_Inv_P = ATA_LambdaI_P.inverse();
    du_P = ATA_LambdaI_Inv_P*A_T_P; //(ATA+λI)^−1)AT this part of the formula is calculated offline for effeciency. now all that's left is to multiply by error

    //j_M
    A_T_M = A_M.transpose();//transpose the A matrix
    LambdaI_M = LAMBDA_M*I_Matrix;
    ATA_M = A_T_M*A_M;
    ATA_LambdaI_M = ATA_M - LambdaI_M;
    ATA_LambdaI_Inv_M = ATA_LambdaI_M.inverse();
    du_M = ATA_LambdaI_Inv_M*A_T_M; //(ATA+λI)^−1)AT this part of the formula is calculated offline for effeciency. now all that's left is to multiply by error

    return std::make_tuple(du_D, du_P, du_M);

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



    std::cout << "test:" <<std::endl;

    //MPC
    N = 10;
    nu = 3;
    I_Matrix = Eigen::MatrixXd::Identity(nu, nu); // model adjustment (correciton) matrix
    
    //j_D parameters
    u_D = Eigen::MatrixXd::Zero(nu, 1); // control input -  1 dimentional matrix filled with zeros
    PHI_D = Eigen::MatrixXd::Zero(N, 1); // model adjustment (correciton) matrix
    measured_posi_D = Eigen::MatrixXd::Zero(N, 1); // model adjustment (correciton) matrix
    
    errors_D = Eigen::MatrixXd::Zero(N, 1); // error
    delta_u_D = Eigen::MatrixXd::Zero(nu, 1); // delta_u
    u_prev_D = Eigen::MatrixXd::Zero(nu, 1); // previous input
    delta_y_D = Eigen::MatrixXd::Zero(N, 1); // change in predicted output
    float LAMBDA_D = 1.083; //penalty factor //decrasing this to 1.08 seems to slow down the rise time but also reduces overshoot
    y_hat_D = Eigen::MatrixXd::Zero(N, 1); // pridected output matrix

    //j_P parameters
    u_P = Eigen::MatrixXd::Zero(nu, 1); // control input -  1 dimentional matrix filled with zeros
    PHI_P = Eigen::MatrixXd::Zero(N, 1); // model adjustment (correciton) matrix
    measured_posi_P = Eigen::MatrixXd::Zero(N, 1); // model adjustment (correciton) matrix
    
    errors_P = Eigen::MatrixXd::Zero(N, 1); // error
    delta_u_P = Eigen::MatrixXd::Zero(nu, 1); // delta_u
    u_prev_P = Eigen::MatrixXd::Zero(nu, 1); // previous input
    delta_y_P = Eigen::MatrixXd::Zero(N, 1); // change in predicted output
    float LAMBDA_P = 1.083; //penalty factor //decrasing this to 1.08 seems to slow down the rise time but also reduces overshoot
    y_hat_P = Eigen::MatrixXd::Zero(N, 1); // pridected output matrix

    //j_M parameters
    u_M = Eigen::MatrixXd::Zero(nu, 1); // control input -  1 dimentional matrix filled with zeros
    PHI_M = Eigen::MatrixXd::Zero(N, 1); // model adjustment (correciton) matrix
    measured_posi_M = Eigen::MatrixXd::Zero(N, 1); // model adjustment (correciton) matrix
    
    errors_M = Eigen::MatrixXd::Zero(N, 1); // error
    delta_u_M = Eigen::MatrixXd::Zero(nu, 1); // delta_u
    u_prev_M = Eigen::MatrixXd::Zero(nu, 1); // previous input
    delta_y_M = Eigen::MatrixXd::Zero(N, 1); // change in predicted output
    float LAMBDA_M = 1.083; //penalty factor //decrasing this to 1.08 seems to slow down the rise time but also reduces overshoot
    y_hat_M = Eigen::MatrixXd::Zero(N, 1); // pridected output matrix



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


    //MPC coeffecients
    
    Eigen::MatrixXd A_D = Eigen::MatrixXd::Zero(N,nu);
    Eigen::MatrixXd A_P = Eigen::MatrixXd::Zero(N,nu);

    // double setpoint_Val = 45; //desired setpoint
    // Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N,1,setpoint_Val); //matrix of setpoint values
    //generate dynamic matrix A
    Eigen::MatrixXd DM_j_D, DM_j_P, DM_j_M;

    std::tie(DM_j_D, DM_j_P, DM_j_M) = generate_Dynamic_Matrix(N, nu);
    A_D = DM_j_D;
    A_P = DM_j_P;
    A_M = DM_j_M;
    std::cout<<"A_D: " << A_D <<std::endl;
    std::cout<<"A_P: " << A_P <<std::endl;
    std::cout<<"A_M: " << A_M <<std::endl;

    std::tie(du_D, du_P, du_M) = generate_du_Matrix(N, nu, A_D, A_P, A_M, LAMBDA_D, LAMBDA_P, LAMBDA_M);

    std::cout<<"du_D: " << du_D <<std::endl;
    std::cout<<"du_P: " << du_P <<std::endl;
    std::cout<<"du_M: " << du_M <<std::endl;

    
    


    // Eigen::MatrixXd A_P = DM_j_P;
    // std::cout<<"A_P: " << *A_P <<std::endl;



    // // Compute the Singular Value Decomposition
    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);

    // std::cout << "Condition number of A is: " << cond << std::endl;




    max_pwm = 800;
    min_pwm = -800;


    
    
//**************REAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAADDDDDDDDD*******************/
/*Made the switch to using multiple models and multiple controllers (Hybrid controller)
While j_D works well the other two do not
I'm still using the same prediction horizon and control horizon for both. Should update so each joint have their own N and nu*/

    float dt = 0.04;
    while (ros::ok()) {


         sleep(dt);

        // select the controlled joint. Note: only one joint can move at a time. J_D and J_P must be at max to move J_M
        if(setpoint_M == 0){
            if(setpoint_P == 0){
                //control D joint
                measured_posi_D = Eigen::MatrixXd::Constant(N, 1, theta_D + theta_P + theta_M); //set all values of measured pos to theta_D
                setpoint_val = setpoint_D;
                Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N,1,setpoint_val); //matrix of setpoint values
                
                control_effort = MPC_Control_D(setpoint, measured_posi_D, N, nu, A_D);


            }
            else{
                //control P joint
                measured_posi_P = Eigen::MatrixXd::Constant(N, 1, theta_D + theta_P + theta_M); //set all values of measured pos to theta_D
                setpoint_val = max_D_joint_angle + setpoint_P;
                Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N,1,setpoint_val); //matrix of setpoint values                
                control_effort = MPC_Control_P(setpoint, measured_posi_P, N, nu, A_P);


            }
        }
        else{
            //control M joint
            measured_posi_M = Eigen::MatrixXd::Constant(N, 1, theta_D + theta_P + theta_M); //set all values of measured pos to theta_D
            setpoint_val = max_D_joint_angle + max_P_joint_angle + setpoint_M;
            Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N,1,setpoint_val); //matrix of setpoint values
            control_effort = MPC_Control_M(setpoint, measured_posi_M, N, nu, A_M);

        }


        //send u(0) to plant
        // std::cout <<"control_effor voltage: " << control_effort <<std::endl;
        control_effort = convert_Voltage_to_PWM(control_effort); //convert from voltage to PWM
        std::cout <<"control_effor pwm: " << control_effort <<std::endl;

        publishData(control_effort); //run publishData method
        ros::spinOnce();
        rate.sleep();

    }



}



    


    

