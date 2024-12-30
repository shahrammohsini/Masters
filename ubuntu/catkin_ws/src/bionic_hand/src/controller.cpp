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


Controller::Controller(int ID, const std::string& finger_name) {
    this-> finger_ID = ID;
    pub_ = nh.advertise<bionic_hand::ControlCommands>("Control_Command", 1000); //initialize publisher node
    sub_ = nh.subscribe("updated_Finger_Positions", 1000, &Controller::fingerPositionCallback, this);  //initialize subscriber node
}


//Create publishData method to publish data
void Controller::publishData(double PWM)
{
   
    bionic_hand::ControlCommands msg;
    msg.PWM = PWM;
    msg.ID = this->finger_ID;


    pub_.publish(msg);
    // ROS_INFO("Published PWM command");

}

//create a subscriber node to get the latest pos info
void Controller::fingerPositionCallback(const bionic_hand::FingerPos& msg){

    //Note: Consider adding a designated thread for data collection.

    ros::Time current_time = ros::Time::now();
    ros::Duration time_diff = current_time - msg.header.stamp;
    
    // Check if the message is too old
    float acceptable_delay = 0.03;
    if (time_diff.toSec() > acceptable_delay) {
        ROS_WARN("Received stale finger position data. Delay: %f seconds", time_diff.toSec());
        return;
    }
    else{ //ensure older data is not used
        position = msg;

        switch (finger_ID)
        {
        case 3: //index finger
            theta_M = position.index.theta_M;
            theta_P = position.index.theta_P;
            theta_D = position.index.theta_D;
            break;
        case 2: //middle finger
            theta_M = position.middle.theta_M;
            theta_P = position.middle.theta_P;
            theta_D = position.middle.theta_D;
            break;
        case 4: //thumb finger
            theta_M = position.thumb.theta_M;
            theta_P = position.thumb.theta_P;
            theta_D = position.thumb.theta_D;
            break;
        }
    }

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

    std::cout <<"setpoint_PID: " << setpoint << std::endl;
    std::cout <<"measured_position_PID: " << measured_position << std::endl;

    error = (setpoint - measured_position);
    std::cout <<"error " << error << std::endl;

    integral += error * dt;
    derivative = (error - previous_error)/dt;
    previous_error = error;
    control_effort = kp * error + ki * integral + kd * derivative;
    std::cout <<"control effort " << control_effort << std::endl;
    if (control_effort > 12){control_effort = 12;}

    if (control_effort < -12){control_effort = -12;}
    
    std::cout<<"kp "<< kp <<std::endl;
    std::cout<<"ki "<< ki <<std::endl;

    std::cout<<"control effort from PID"<< control_effort <<std::endl;
    return control_effort;
}

/*Generate and return a dynamic matrix of given size (prediction horizon, control horizon) */
// Function signature returning a tuple of three matrices
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> Controller::generate_Dynamic_Matrix(int prediction_horizon_D, int control_horizon_D, int prediction_horizon_P, int control_horizon_P, int prediction_horizon_M, int control_horizon_M){

    Eigen::MatrixXd DM_j_D(prediction_horizon_D, control_horizon_D);
    Eigen::MatrixXd DM_j_P(prediction_horizon_P, control_horizon_P);
    Eigen::MatrixXd DM_j_M(prediction_horizon_M, control_horizon_M);

    //open file
    std:: string filePath;
    switch (finger_ID)
        {
        case 3: //index finger
            filePath = "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/Data/open_loop_data_index.csv";
            break;
        case 2: //middle finger
            filePath = "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/Data/open_loop_data_middle.csv";
            break;
        case 4: //thumb finger
            filePath = "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/Data/open_loop_data_thumb.csv";
            break;
        }
    std::ifstream file(filePath);

    //check if file is open
    if(!file.is_open()){
        std::cerr <<"ERROR OPPENING FILE:    " <<filePath <<std::endl;
        std::exit(EXIT_FAILURE);
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
        data.theta_D_joint = std::stod(field);
        std::getline(s, field, ',');
        data.theta_P_joint = std::stod(field);
        std::getline(s, field, ',');
        data.theta_M_joint = std::stod(field);
        std::getline(s, field, ',');
        data.time = std::stod(field);

        //add the data to the vecotr
        dataset.push_back(data);
    }

    file.close();


    //Dynamic Matrix D
    DM_j_D.setZero(); //populate with zeros
    // float bias = 4.2; //If motor has a bias value (min voltage to move motor)
    int normalize_val = 1;
    // DM_j_D.setConstant(bias); // populate with min voltage to move. This is the bias
    //populate dynamic matrix D
     for (int i =0; i < control_horizon_D; i++) { //create columns
        for(int j=0; j < prediction_horizon_D - i; j++){ //create rows
            const auto& d = dataset[j]; //d holds the current row
            DM_j_D(i + j, i) = ((d.theta_D_joint)/normalize_val); //fill current row for current column. Also, normalize the angle
            // if(DM_j_D(i + j, i) < bias) {DM_j_D(i + j, i) = bias;} //The motor does not move bellow 4 volts so to normalize we have to add 4 volts to values bellow 4 volts
        }
     }
    std::cout << "Matrix_D: \n" << DM_j_D <<std::endl;

    
    //Dynamic Matrix P
    switch (finger_ID)
        {
        case 3: //index finger
            DM_j_P.setZero(); //populate with zeros
            // DM_j_P.setConstant(bias);
            //populate dynamic matrix P
            for (int i =0; i < control_horizon_P; i++) { //create columns
                for(int j=0; j < prediction_horizon_P - i; j++){ //create rows
                    const auto& d = dataset[j+47]; //d holds the current row adding +6 to start at the point where M joint moves
                    DM_j_P(i + j, i) = ((d.theta_P_joint)/normalize_val); //fill current row for current column. Also, normalize the angle
                    // if(DM_j_P(i + j, i) < bias) {DM_j_P(i + j, i) = bias;} //The motor does not move bellow 4 volts so to normalize we have to add 4 volts to values bellow 4 volts

                }
            }
            break;
        case 2: //middle finger
            DM_j_P.setZero(); //populate with zeros
            // DM_j_P.setConstant(bias);
            //populate dynamic matrix P
            for (int i =0; i < control_horizon_P; i++) { //create columns
                for(int j=0; j < prediction_horizon_P - i; j++){ //create rows
                    const auto& d = dataset[j+54]; //d holds the current row adding +6 to start at the point where M joint moves
                    DM_j_P(i + j, i) = ((d.theta_P_joint)/normalize_val); //fill current row for current column. Also, normalize the angle
                    // if(DM_j_P(i + j, i) < bias) {DM_j_P(i + j, i) = bias;} //The motor does not move bellow 4 volts so to normalize we have to add 4 volts to values bellow 4 volts

                }
            }
            break;

        case 4: //thumb finger
            DM_j_P.setZero(); //populate with zeros
            // DM_j_P.setConstant(bias);
            //populate dynamic matrix P
            for (int i =0; i < control_horizon_P; i++) { //create columns
                for(int j=0; j < prediction_horizon_P - i; j++){ //create rows
                    const auto& d = dataset[j+92]; //d holds the current row adding +6 to start at the point where M joint moves
                    DM_j_P(i + j, i) = ((d.theta_P_joint)/normalize_val); //fill current row for current column. Also, normalize the angle
                    // if(DM_j_P(i + j, i) < bias) {DM_j_P(i + j, i) = bias;} //The motor does not move bellow 4 volts so to normalize we have to add 4 volts to values bellow 4 volts

                }
            }
            break;
        }

    std::cout << "Matrix_P: \n" << DM_j_P <<std::endl;




    //Dynamic Matrix M
    switch (finger_ID)
        {
        case 3: //index finger
            DM_j_M.setZero(); //populate with zeros
            // DM_j_M.setConstant(bias);
            //populate dynamic matrix M
            for (int i =0; i < control_horizon_M; i++) { //create columns
                for(int j=0; j < prediction_horizon_M - i; j++){ //create rows
                    const auto& d = dataset[j + 115]; //d holds the current row.
                    DM_j_M(i + j, i) = ((d.theta_M_joint)/normalize_val); //fill current row for current column. Also, normalize the angle
                    // if(DM_j_M(i + j, i) < bias) {DM_j_M(i + j, i) = bias;} //The motor does not move bellow 4 volts so to normalize we have to add 4 volts to values bellow 4 volts

                }
            }
            break;
        case 2: //middle finger
            DM_j_M.setZero(); //populate with zeros
            // DM_j_M.setConstant(bias);
            //populate dynamic matrix M
            for (int i =0; i < control_horizon_M; i++) { //create columns
                for(int j=0; j < prediction_horizon_M - i; j++){ //create rows
                    const auto& d = dataset[j + 158]; //d holds the current row.
                    DM_j_M(i + j, i) = ((d.theta_M_joint)/normalize_val); //fill current row for current column. Also, normalize the angle
                    // if(DM_j_M(i + j, i) < bias) {DM_j_M(i + j, i) = bias;} //The motor does not move bellow 4 volts so to normalize we have to add 4 volts to values bellow 4 volts

                }
            }
            break;

        case 4: //thumb finger
            DM_j_P.setZero(); //populate with zeros
            // DM_j_P.setConstant(bias);
            //populate dynamic matrix P
            for (int i =0; i < control_horizon_P; i++) { //create columns
                for(int j=0; j < prediction_horizon_P - i; j++){ //create rows
                    const auto& d = dataset[j+5]; //d holds the current row adding +6 to start at the point where M joint moves
                    DM_j_P(i + j, i) = ((d.theta_P_joint)/normalize_val); //fill current row for current column. Also, normalize the angle
                    // if(DM_j_P(i + j, i) < bias) {DM_j_P(i + j, i) = bias;} //The motor does not move bellow 4 volts so to normalize we have to add 4 volts to values bellow 4 volts

                }
            }
            break;
        }
    
    std::cout << "Matrix_M: \n" << DM_j_M <<std::endl;

    return std::make_tuple(DM_j_D, DM_j_P, DM_j_M);
}



//reverse
/*Generate and return a dynamic matrix of given size (prediction horizon, control horizon) */
// Function signature returning a tuple of three matrices
Eigen::MatrixXd Controller::generate_Dynamic_Matrix_rev(int prediction_horizon_D_rev, int control_horizon_D_rev, int prediction_horizon_P_rev, int control_horizon_P_rev, int prediction_horizon_M_rev, int control_horizon_M_rev){
   
    // Eigen::MatrixXd DM_j_D(prediction_horizon_D, control_horizon_D);
    Eigen::MatrixXd DM_j_P_rev(prediction_horizon_P_rev, control_horizon_P_rev);
    // Eigen::MatrixXd DM_j_M(prediction_horizon_M, control_horizon_M);

    //open file
    std:: string filePath = "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/Data/open_loop_data_middle.csv";
    std::ifstream file(filePath);

    //check if file is open
    if(!file.is_open()){
        std::cerr <<"ERROR OPPENING FILE:    " <<filePath <<std::endl;
        //return std::make_tuple(Eigen::MatrixXd(), Eigen::MatrixXd(), Eigen::MatrixXd()); //return empty eigen vecotrs to indicate error
        std::exit(EXIT_FAILURE);
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
        data.theta_D_joint = std::stod(field);
        std::getline(s, field, ',');
        data.theta_P_joint = std::stod(field);
        std::getline(s, field, ',');
        data.theta_M_joint = std::stod(field);

        //add the data to the vecotr
        dataset.push_back(data);
    }

    file.close();



    // //flip the data
    // // Create an Eigen matrix to store the reversed data
    // int numRows = dataset.size();
    // Eigen::MatrixXd reverse_dataset(numRows, 4);  // Assuming 4 columns: time, theta_D_joint, theta_P_joint, theta_M_joint

    // // Fill the matrix in reverse order
    // for (int i = 0; i < numRows; ++i) {
    //     const auto& data = dataset[numRows - 1 - i];
    //     reverse_dataset(i, 0) = data.time;
    //     reverse_dataset(i, 1) = data.theta_D_joint;
    //     reverse_dataset(i, 2) = data.theta_P_joint;
    //     reverse_dataset(i, 3) = data.theta_M_joint;
    // }

    // DM_j_D.setZero(); //populate with zeros
    // //populate dynamic matrix
    //  for (int i =0; i < control_horizon_D; i++) { //create columns
    //     for(int j=0; j < prediction_horizon_D - i; j++){ //create rows
    //         const auto& d = dataset[j]; //d holds the current row
    //         DM_j_D(i + j, i) = (d.theta_D_joint/1); //fill current row for current column. Also, normalize the angle
    //     }
    //  }
    // std::cout << "reverse_matrix: \n" << reverse_dataset <<std::endl;
    //
    
    float bias = 0;
    int normalize_val = 1;
    // DM_j_P_rev = Eigen::MatrixXd::Constant(N_P_rev, nu_P_rev, bias); //populate with 0 or bias
    DM_j_P_rev = Eigen::MatrixXd::Constant(N_P_rev, nu_P_rev, max_P_joint_angle); //populate with 0 or bias
    //populate dynamic matrix
     for (int i =0; i < control_horizon_P_rev; i++) { //create columns
        for(int j=0; j < prediction_horizon_P_rev - i; j++){ //create rows
            
            const auto& d = dataset[j + 18]; //d holds the current row.
            DM_j_P_rev(i + j, i) = ((d.theta_P_joint)/normalize_val);

           // DM_j_P_rev(i + j, i) = (reverse_dataset(14 + j ,2)/normalize_val); //fill current row for current column. Also, normalize the angle
            
            // if(DM_j_P_rev(i + j, i) < bias){DM_j_P_rev(i + j, i) = bias;}
        }
     }


// DM_j_M(i + j, i) = ((d.theta_M_joint)/normalize_val); //fill current row for current column. Also, normalize the angle
//             if(DM_j_M(i + j, i) < bias) {DM_j_M(i + j, i) = bias;} //The motor does not move bellow 4 volts so to normalize we have to add 4 volts to values bellow 4 volts



    std::cout << "DM_j_P_rev: \n" << DM_j_P_rev <<std::endl;

    // DM_j_M.setZero(); //populate with zeros
    // //populate dynamic matrix
    //  for (int i =0; i < control_horizon_M; i++) { //create columns
    //     for(int j=0; j < prediction_horizon_M - i; j++){ //create rows
    //         const auto& d = dataset[j + 17]; //d holds the current row.
    //         DM_j_M(i + j, i) = (d.theta_M_joint/1); //fill current row for current column. Also, normalize the angle
    //     }
    //  }
    // std::cout << "Matrix_M: \n" << DM_j_M <<std::endl;


    // return std::make_tuple(DM_j_D, DM_j_P, DM_j_M);
    return DM_j_P_rev;

}


//**convert voltage to pwm. Input: voltae, output: pwm **/
double Controller::convert_Voltage_to_PWM(double voltage){
    double PWM = (voltage/max_Voltage)*max_pwm;
    return PWM;

}


//add dead time to a dynamic matrix
//input: dynamic matrix, numbuer of deadtime steps, output: dynamic matrix with dea time
Eigen::MatrixXd Controller::addDeadTime(const Eigen::MatrixXd& dynamicMatrix, int deadTimeSteps) {
    int rows = dynamicMatrix.rows();
    int cols = dynamicMatrix.cols();
    Eigen::MatrixXd dead_Time_Matrix = Eigen::MatrixXd::Zero(rows, cols);

    for (int i = 0; i < cols; ++i) {
        for (int j = deadTimeSteps; j < rows; ++j) {
            dead_Time_Matrix(j, i) = dynamicMatrix(j - deadTimeSteps, i);
        }
    }
    return dead_Time_Matrix;
}

//add dead time to a reverse dynamic matrix
//input: dynamic matrix, numbuer of deadtime steps, output: dynamic matrix with dea time
Eigen::MatrixXd Controller::addDeadTime_rev(const Eigen::MatrixXd& dynamicMatrix, int deadTimeSteps) {
    int rows = dynamicMatrix.rows();
    int cols = dynamicMatrix.cols();
    Eigen::MatrixXd dead_Time_Matrix = Eigen::MatrixXd::Zero(rows, cols);
    dead_Time_Matrix.setConstant(89);

    for (int i = 0; i < cols; ++i) {
        for (int j = deadTimeSteps; j < rows; ++j) {
            dead_Time_Matrix(j, i) = dynamicMatrix(j - deadTimeSteps, i);
        }
    }
    return dead_Time_Matrix;
}

double Controller::MPC_Control_D(Eigen::MatrixXd setpoint,double measured_position_D, int N_D, int nu_D, Eigen::MatrixXd A_D){
    
    PHI_D = measured_position_D - y_hat_D(0); //PHI is the difference between actual model and predicted model
    y_hat_D = y_hat_D + Eigen::MatrixXd::Constant(y_hat_D.rows(), y_hat_D.cols(), PHI_D); //add the constant PHI to each value of y_hat

    errors_D = setpoint - y_hat_D; //error
    delta_u_D = du_D*errors_D; //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) // delta_u is the control input step

    u_D = u_prev_D + delta_u_D; //calculate control move vector

    //add control input limits here
    for(int i = 0; i < nu_D; i++){
        if (u_D(i) > max_Voltage){
            u_D(i) = max_Voltage;

        }
        else if (u_D(i) < min_Voltage){
            u_D(i) = min_Voltage;
        }
        
        delta_u_D(i) = u_D(i) - u_prev_D(i); //revaluate delta_u to account for the limits you've introduced to u
    }

    delta_y_D = A_D*delta_u_D; //caluclate change in predicted output: y_1 - y_0 = (a_1)(Delta u_0). Multiply first col of A_D with first val of delta_u

    y_hat_D = y_hat_D + delta_y_D; // calculate new prediction for output

   
    // advance prediction horizon one step ahead (Shift all elements to the left). y_hat[i] = y_hat[i+1]
    // Shift the prediction horizon
    double y_hat_last_D = y_hat_D(0);
    for (int i = 1; i < y_hat_D.size(); ++i) {
        y_hat_D(i - 1) = y_hat_D(i);
    }
    y_hat_D(y_hat_D.size() - 1) = y_hat_last_D;  // Setting the last element to first element
    
    //update control move
    u_prev_D = u_D;

    return u_D(0);
}

double Controller::MPC_Control_P(Eigen::MatrixXd setpoint,double measured_position_P, int N_P, int nu_P, Eigen::MatrixXd A_P){

    PHI_P = measured_position_P - y_hat_P(0); //PHI is the difference between actual model and predicted model

    y_hat_P = y_hat_P + Eigen::MatrixXd::Constant(y_hat_P.rows(), y_hat_P.cols(), PHI_P); //add the constant PHI to each value of y_hat

    errors_P = setpoint - y_hat_P; //error
  
    delta_u_P = du_P*errors_P; //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) // delta_u is the control input step

    u_P = u_prev_P + delta_u_P; //calculate control move vector

    //add control input limits here
    for(int i = 0; i < nu_P; i++){
        if (u_P(i) > max_Voltage){
            u_P(i) = max_Voltage;
        }
        else if (u_P(i) < min_Voltage){
            u_P(i) = min_Voltage;
        }
        delta_u_P(i) = u_P(i) - u_prev_P(i); //revaluate delta_u to account for the limits you've introduced to u
    }

    delta_y_P = A_P*delta_u_P; //caluclate change in predicted output: y_1 - y_0 = (a_1)(Delta u_0). Multiply first col of A_D with first val of delta_u
   
    y_hat_P = y_hat_P + delta_y_P; // calculate new prediction for output
    // std::cout << "y_hat_P_post: " << y_hat_P <<std::endl; 

    // Shift the prediction horizon
    double y_hat_last_P = y_hat_P(0);
    for (int i = 1; i < y_hat_P.size(); ++i) {
        y_hat_P(i - 1) = y_hat_P(i);
    }
    y_hat_P(y_hat_P.size() - 1) = y_hat_last_P;  // Setting the last element to first element

    //update control move
    u_prev_P = u_P;
    // std::cout << "control effort: " << u_P(0) <<std::endl; 

    return u_P(0);
}

double Controller::MPC_Control_M(Eigen::MatrixXd setpoint,double measured_position_M, int N_M, int nu_M, Eigen::MatrixXd A_M){
    // std::cout << "setpoint: \n" << setpoint <<std::endl; 
    // std::cout << "measured pos: \n" << measured_position <<std::endl; 
    
    PHI_M = measured_position_M - y_hat_M(0); //difference between predicted pos (y_hat) and measured pos to obtain model inaccuracy.
 
    y_hat_M = y_hat_M + Eigen::MatrixXd::Constant(y_hat_M.rows(), y_hat_M.cols(), PHI_M); //add the constant PHI to each value of y_hat

    errors_M = setpoint - y_hat_M; //error

    delta_u_M = du_M*errors_M; //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) // delta_u is the control input step
    // std::cout << "delta_u: \n" << delta_u <<std::endl; 
    u_M = u_prev_M + delta_u_M; //calculate control move vector
    // std::cout << "u: \n" << u <<std::endl; 
    //add control input limits here
    for(int i = 0; i < nu_M; i++){
        if (u_M(i) > max_Voltage){
            u_M(i) = max_Voltage;
        }
        else if (u_M(i) < min_Voltage){
            u_M(i) = min_Voltage;
        }
        delta_u_M(i) = u_M(i) - u_prev_M(i); //revaluate delta_u to account for the limits you've introduced to u

    }


    delta_y_M = A_M*delta_u_M; //caluclate change in predicted output: y_1 - y_0 = (a_1)(Delta u_0). Multiply first col of A_D with first val of delta_u
    
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


double Controller::MPC_Control_P_Reverse(Eigen::MatrixXd setpoint,double measured_position_P_rev, int N_P_rev, int nu_P_rev, Eigen::MatrixXd A_P_rev){
    // std::cout << "setpoint: \n" << setpoint <<std::endl; 
    // std::cout << "measured pos: \n" << measured_position_P_rev <<std::endl; 
    // std::cout<<" y_hat_P: "<< y_hat_P<<std::endl;
    // std::cout<<" measured_position: "<< measured_position<<std::endl;

    PHI_P_rev = measured_position_P_rev - y_hat_P_rev(0); //PHI is the difference between actual model and predicted model

    y_hat_P_rev = y_hat_P_rev + Eigen::MatrixXd::Constant(y_hat_P_rev.rows(), y_hat_P_rev.cols(), PHI_P_rev); //add the constant PHI to each value of y_hat

    errors_P_rev = setpoint - y_hat_P_rev; //error

    // std::cout << "errors_p_rev: \n" << errors_P_rev <<std::endl; 

    // std::cout << "du: \n" << du <<std::endl; 
    // std::cout << "du_D: \n" << du_D <<std::endl; 

    delta_u_P_rev = du_P_rev*errors_P_rev; //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) // delta_u is the control input step

    u_P_rev = u_prev_P_rev + delta_u_P_rev; //calculate control move vector
    // std::cout << "u: \n" << u <<std::endl; 
    
    
    //add control input limits here
    for(int i = 0; i < nu_P_rev; i++){
        if (u_P_rev(i) > max_Voltage){
            u_P_rev(i) = max_Voltage;
        delta_u_P_rev(i) = u_P_rev(i) - u_prev_P_rev(i); //revaluate delta_u to account for the limits you've introduced to u

        }
        else if (u_P_rev(i) < min_Voltage){
            u_P_rev(i) = min_Voltage;
            delta_u_P_rev(i) = u_P_rev(i) - u_prev_P_rev(i); //revaluate delta_u to account for the limits you've introduced to u

        }
    }


    // delta_y_P_rev = A_P_rev*delta_u_P_rev; //caluclate change in predicted output: y_1 - y_0 = (a_1)(Delta u_0)
    // delta_y_P_rev = A_P_rev.col(0)*delta_u_P_rev(0,0); //caluclate change in predicted output: y_1 - y_0 = (a_1)(Delta u_0). Multiply first col of A_D with first val of delta_u
    delta_y_P_rev = A_P_rev*delta_u_P_rev;
    
    // std::cout << "delta_y: \n" << delta_y <<std::endl; 
   
    y_hat_P_rev = y_hat_P_rev + delta_y_P_rev; // calculate new prediction for output
   
    // std::cout << "y_hat_P_preshift: \n" << y_hat_P <<std::endl; 
    // advance prediction horizon one step ahead (Shift all elements to the left). y_hat[i] = y_hat[i+1]
    // Shift the prediction horizon
    double y_hat_last_P_rev = y_hat_P_rev(0);
    for (int i = 1; i < y_hat_P_rev.size(); ++i) {
        y_hat_P_rev(i - 1) = y_hat_P_rev(i);
    }
    y_hat_P_rev(y_hat_P_rev.size() - 1) = y_hat_last_P_rev;  // Setting the last element to first element
    
    //update control move
    u_prev_P_rev = u_P_rev;
    // std::cout << "u_prev: \n" << u_prev <<std::endl; 

    return u_P_rev(0);
}




std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> Controller:: generate_du_Matrix(int prediction_horizon_D, int control_horizon_D,int prediction_horizon_P, int control_horizon_P,int prediction_horizon_M, int control_horizon_M, int prediction_horizon_P_rev, int control_horizon_P_rev, Eigen::MatrixXd A_D, Eigen::MatrixXd A_P, Eigen::MatrixXd A_M,Eigen::MatrixXd A_P_rev, float LAMBDA_D, float LAMBDA_P, float LAMBDA_M, float LAMBDA_P_rev){
    //j_D
    //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) calculate first part of delta_u outside the loop for effecincy
    A_T_D = A_D.transpose();//transpose the A matrix
    // LambdaI_D = LAMBDA_D*I_Matrix_D;
    ATA_D = A_T_D*A_D;
    // ATA_LambdaI_D = ATA_D + LambdaI_D; //Add penalty factor (lambda) to diagonal values
    for (int i = 0; i < ATA_D.rows(); ++i) { //multiply the diagonal by penalty LAMBDA
        ATA_D(i, i) = ATA_D(i, i)*LAMBDA_D;}
    // ATA_LambdaI_D = ATA_D;
    ATA_LambdaI_Inv_D = ATA_D.inverse();
    du_D = ATA_LambdaI_Inv_D*A_T_D; //(ATA+λI)^−1)AT this part of the formula is calculated offline for effeciency. now all that's left is to multiply by error


    //j_P
    A_T_P = A_P.transpose();//transpose the A matrix
    // LambdaI_P = LAMBDA_P*I_Matrix_P;
    ATA_P = A_T_P*A_P;
    // ATA_LambdaI_P = ATA_P + LambdaI_P;
    for (int i = 0; i < ATA_P.rows(); ++i) { //multiply the diagonal by penalty LAMBDA
        ATA_P(i, i) = ATA_P(i, i)*LAMBDA_P;}
    // ATA_LambdaI_P = ATA_P;
    ATA_LambdaI_Inv_P = ATA_P.inverse();
    du_P = ATA_LambdaI_Inv_P*A_T_P; //(ATA+λI)^−1)AT this part of the formula is calculated offline for effeciency. now all that's left is to multiply by error
    
    
    //j_M
    A_T_M = A_M.transpose();//transpose the A matrix
    // LambdaI_M = LAMBDA_M*I_Matrix_M;
    ATA_M = A_T_M*A_M;
    // ATA_LambdaI_M = ATA_M + LambdaI_M;
    for (int i = 0; i < ATA_M.rows(); ++i) { //multiply the diagonal by penalty LAMBDA
        ATA_M(i, i) = ATA_M(i, i)*LAMBDA_M;}
    // ATA_LambdaI_M = ATA_M;
    ATA_LambdaI_Inv_M = ATA_M.inverse();
    du_M = ATA_LambdaI_Inv_M*A_T_M; //(ATA+λI)^−1)AT this part of the formula is calculated offline for effeciency. now all that's left is to multiply by error


    //reverse
      //j_P_rev
    A_T_P_rev = A_P_rev.transpose();//transpose the A matrix
    LambdaI_P_rev = LAMBDA_P_rev*I_Matrix_P_rev;
    ATA_P_rev = A_T_P_rev*A_P_rev;
    // std::count <<"ATA_P_rev" << ATA_P_rev <<std::endl;
    std::cout << "ATA_P_rev: "<< ATA_P_rev <<std::endl;

    //instead of adding lambda (penalty factor) we multiply it to the diagonal values of the matrix
    for (int i = 0; i < ATA_P_rev.rows(); ++i) {
        ATA_P_rev(i, i) = ATA_P_rev(i, i)*LAMBDA_P_rev;}

        std::cout << "ATA_P_rev_lambda: "<< ATA_P_rev <<std::endl;

    
    ATA_LambdaI_Inv_P_rev = ATA_P_rev.inverse();
    du_P_rev = ATA_LambdaI_Inv_P_rev*A_T_P_rev; //(ATA+λI)^−1)AT this part of the formula is calculated offline for effeciency. now all that's left is to multiply by error
   


    return std::make_tuple(du_D, du_P, du_M, du_P_rev);

}


void Controller::loadParameters(int finger_ID) {
    ///load parameters from catkin_ws/src/bionic_hand/config
    std::string base_param_path;

    switch (finger_ID)
        {
        case 3: //index finger
            base_param_path = "index_finger";

            break;
        case 2: //middle finger
            base_param_path = "middle_finger";

            break;
        case 4: //thumb finger
            base_param_path = "thumb_finger";
            break;
        }
    // Load PID values from parameter server
    nh.getParam(base_param_path + "/M_joint/kp", kp_M);
    nh.getParam(base_param_path + "/M_joint/ki", ki_M);
    nh.getParam(base_param_path + "/M_joint/kd", kd_M);
    nh.getParam(base_param_path + "/P_joint/kp", kp_P);
    nh.getParam(base_param_path + "/P_joint/ki", ki_P);
    nh.getParam(base_param_path + "/P_joint/kd", kd_P);
    nh.getParam(base_param_path + "/D_joint/kp", kp_D);
    nh.getParam(base_param_path + "/D_joint/ki", ki_D);
    nh.getParam(base_param_path + "/D_joint/kd", kd_D);

    // Load maximum joint angles
    nh.getParam(base_param_path + "/joint_limits/max_M_joint_angle", max_M_joint_angle);
    nh.getParam(base_param_path + "/joint_limits/max_P_joint_angle", max_P_joint_angle);
    nh.getParam(base_param_path + "/joint_limits/max_D_joint_angle", max_D_joint_angle);

    // Load MPC parameters
    nh.getParam(base_param_path + "/forward/D_joint/N_D", N_D);
    nh.getParam(base_param_path + "/forward/D_joint/nu_D", nu_D);
    nh.getParam(base_param_path + "/forward/D_joint/deadTimeSteps_D", deadTimeSteps_D);
    nh.getParam(base_param_path + "/forward/D_joint/LAMBDA_D", LAMBDA_D);
    nh.getParam(base_param_path + "/forward/P_joint/N_P", N_P);
    nh.getParam(base_param_path + "/forward/P_joint/nu_P", nu_P);
    nh.getParam(base_param_path + "/forward/P_joint/deadTimeSteps_P", deadTimeSteps_P);
    nh.getParam(base_param_path + "/forward/P_joint/LAMBDA_P", LAMBDA_P);
    nh.getParam(base_param_path + "/forward/M_joint/N_M", N_M);
    nh.getParam(base_param_path + "/forward/M_joint/nu_M", nu_M);
    nh.getParam(base_param_path + "/forward/M_joint/deadTimeSteps_M", deadTimeSteps_M);
    nh.getParam(base_param_path + "/forward/M_joint/LAMBDA_M", LAMBDA_M);

    // Load reverse MPC parameters (if applicable)
    nh.getParam(base_param_path + "/reverse/P_joint/N_P_rev", N_P_rev);
    nh.getParam(base_param_path + "/reverse/P_joint/nu_P_rev", nu_P_rev);
    nh.getParam(base_param_path + "/reverse/P_joint/deadTimeSteps_P_rev", deadTimeSteps_P_rev);
    nh.getParam(base_param_path + "/reverse/P_joint/LAMBDA_P_rev", LAMBDA_P_rev);

    // Load other parameters
    nh.getParam(base_param_path + "/dt", dt);
}


void Controller::initializeMPCMatrices() {
    // Initialize identity matrices for the model adjustment
    I_Matrix_D = Eigen::MatrixXd::Identity(nu_D, nu_D);
    I_Matrix_P = Eigen::MatrixXd::Identity(nu_P, nu_P);
    I_Matrix_M = Eigen::MatrixXd::Identity(nu_M, nu_M);
    I_Matrix_P_rev = Eigen::MatrixXd::Identity(nu_P_rev, nu_P_rev);

    // Initialize control vectors and matrices for all joints
    u_D = Eigen::MatrixXd::Zero(nu_D, 1);
    errors_D = Eigen::MatrixXd::Zero(N_D, 1);
    delta_u_D = Eigen::MatrixXd::Zero(nu_D, 1);
    u_prev_D = Eigen::MatrixXd::Zero(nu_D, 1);
    delta_y_D = Eigen::MatrixXd::Zero(N_D, 1);
    y_hat_D = Eigen::MatrixXd::Zero(N_D, 1);

    u_P = Eigen::MatrixXd::Zero(nu_P, 1);
    errors_P = Eigen::MatrixXd::Zero(N_P, 1);
    delta_u_P = Eigen::MatrixXd::Zero(nu_P, 1);
    u_prev_P = Eigen::MatrixXd::Zero(nu_P, 1);
    delta_y_P = Eigen::MatrixXd::Zero(N_P, 1);
    y_hat_P = Eigen::MatrixXd::Zero(N_P, 1);

    u_M = Eigen::MatrixXd::Zero(nu_M, 1);
    errors_M = Eigen::MatrixXd::Zero(N_M, 1);
    delta_u_M = Eigen::MatrixXd::Zero(nu_M, 1);
    u_prev_M = Eigen::MatrixXd::Zero(nu_M, 1);
    delta_y_M = Eigen::MatrixXd::Zero(N_M, 1);
    y_hat_M = Eigen::MatrixXd::Zero(N_M, 1);

    u_P_rev = Eigen::MatrixXd::Zero(nu_P_rev, 1);
    errors_P_rev = Eigen::MatrixXd::Zero(N_P_rev, 1);
    delta_u_P_rev = Eigen::MatrixXd::Zero(nu_P_rev, 1);
    u_prev_P_rev = Eigen::MatrixXd::Zero(nu_P_rev, 1);
    delta_y_P_rev = Eigen::MatrixXd::Zero(N_P_rev, 1);
    y_hat_P_rev = Eigen::MatrixXd::Zero(N_P_rev, 1);

}


//run method holding main control loop
void Controller::run(float setpoint_M, float setpoint_P, float setpoint_D) {

    float dt_pid;
    std::cout <<"id -----------------------: " << finger_ID <<std::endl;

    loadParameters(finger_ID); //load finger parameters based on finger ID

    std::cout << "max_D_joint_angle----------------------------------------- "<< max_D_joint_angle <<std::endl;
    initializeMPCMatrices(); // Initialize all MPC related matrices and vectors


    //gernerate the forward dynamic matracies
    std::tie(DM_j_D, DM_j_P, DM_j_M) = generate_Dynamic_Matrix(N_D, nu_D, N_P, nu_P, N_M, nu_M);
    A_D = DM_j_D;
    A_P = DM_j_P;
    A_M = DM_j_M;

    //reverse
    // std::tie(DM_j_D, DM_j_P, DM_j_M) = generate_Dynamic_Matrix(N_D, nu_D, N_P, nu_P, N_M, nu_M);
    DM_j_P_rev = generate_Dynamic_Matrix_rev(1, 1, N_P_rev, nu_P_rev, 1, 1);
    A_P_rev = DM_j_P_rev;

    //add dead time to dynamic matrix
    //forward
    A_D = addDeadTime(A_D,  deadTimeSteps_D);
    A_P = addDeadTime(A_P,  deadTimeSteps_P);
    A_M = addDeadTime(A_M,  deadTimeSteps_M);
    A_P_rev = addDeadTime_rev(A_P_rev,  deadTimeSteps_P_rev);

    //gernerate du matrix (offline matrix calculations)
    std::tie(du_D, du_P, du_M, du_P_rev) = generate_du_Matrix(N_D, nu_D,N_P, nu_P,N_M, nu_M, N_P_rev, nu_P_rev, A_D, A_P, A_M, A_P_rev, LAMBDA_D, LAMBDA_P, LAMBDA_M, LAMBDA_P_rev);

    max_pwm = 368.75; //use 5v max
    min_pwm = -368.75; //use 5v max

    //initialize trajectory vectors
    Eigen::MatrixXd y_pdt_D = Eigen::MatrixXd::Zero(N_D, 1);
    Eigen::MatrixXd y_pdt_P = Eigen::MatrixXd::Zero(N_P, 1);
    Eigen::MatrixXd y_pdt_P_rev = Eigen::MatrixXd::Zero(N_P_rev, 1);
    Eigen::MatrixXd y_pdt_M = Eigen::MatrixXd::Zero(N_M, 1);
    
    double alpha_D = 0; //trajectory smoothness parameter between 0 and 1
    double alpha_P = 0; //trajectory smoothness parameter
    double alpha_M = 0; //trajectory smoothness parameter

    // sleep(1);
    
    while (ros::ok()) {


        //  sleep(dt);
        ros::Rate rate(1.0 / dt); // rate = 10 Hz if dt = 0.1


        // select the controlled joint. Note: only one joint can move at a time. J_D and J_P must be at max to move J_M
        if(theta_M == 0){
            if(theta_P == 0){
                //control D joint
                // measured_posi_D = Eigen::MatrixXd::Constant(N_D, 1, theta_D); //set all values of measured pos to theta_D
                setpoint_val = setpoint_D;
                y_pdt_D(0) = theta_D; // Initialize with the current position
                // Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N_D,1,setpoint_val); //matrix of setpoint values
                


                for (int j = 1; j < 3; ++j) { //create a setpoint trajectory for a smoother rise
                y_pdt_D(j) = alpha_D * y_pdt_D(j - 1) + (1 - alpha_D) * setpoint_val;
                }
                Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N_D,1,y_pdt_D(1)); //only use first value from the setpoint trajectory
                
                // std::cout<<"setpoint: " << setpoint<<std::endl;
                
                control_effort = MPC_Control_D(setpoint, theta_D, N_D, nu_D, A_D);
                // std::cout<<"Controlling D_Joint:  " <<control_effort <<std::endl;

                // control_effort = PID_Control(setpoint_val, theta_D, kp_D, ki_D, kd_D, dt_pid);

                // control_effort = MPC_Control_D(setpoint, theta_D, N_D, nu_D, A_D);
                // std::cout<<"Controlling D_Joint: " << control_effort<<std::endl;



            }
            else{
                //control P joint
                // measured_posi_P = Eigen::MatrixXd::Constant(N_P, 1, theta_P); //set all values of measured pos to theta_D
                setpoint_val = setpoint_P;
                y_pdt_P(0) = theta_P; // Initialize with the current position
                // Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N_D,1,setpoint_val); //matrix of setpoint values
                for (int j = 1; j < 3; ++j) { //create a setpoint trajectory for a smoother rise
                y_pdt_P(j) = alpha_P * y_pdt_P(j - 1) + (1 - alpha_P) * setpoint_val;}
               
                Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N_P,1,y_pdt_P(1)); //only use first value from the setpoint trajectory
                // /Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N_P,1,setpoint_val); //matrix of setpoint values                
                
                //reverse
                Eigen::MatrixXd setpoint_rev = Eigen::MatrixXd::Constant(N_P_rev,1,y_pdt_P(1)); //matrix of setpoint values                
                
                control_effort = MPC_Control_P(setpoint, theta_P, N_P, nu_P, A_P);
                // control_effort = PID_Control(setpoint_val, theta_P, kp_P, ki_P, kd_P, dt_pid);

                // std::cout<<"Controlling P_Joint:  " <<convert_Voltage_to_PWM(control_effort) <<std::endl;

                // std::cout<<"Controlling P_Joint_Forward:  " <<control_effort <<std::endl;
                // if(control_effort < 0){ //Reverse motion
                // // control_effort = -1;
                // control_effort = 1*(MPC_Control_P_Reverse(setpoint_rev, theta_P, N_P_rev, nu_P_rev, A_P_rev));
                // // std::cout<<"Controlling P_Joint_Reverse:  " <<control_effort <<std::endl;

                // }


            }
        }
        else{
            //control M joint
            setpoint_val = setpoint_M;
            y_pdt_M(0) = theta_M; // Initialize with the current position
            // Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N_D,1,setpoint_val); //matrix of setpoint values
            for (int j = 1; j < 3; ++j) { //create a setpoint trajectory for a smoother rise
            y_pdt_M(j) = alpha_M * y_pdt_M(j - 1) + (1 - alpha_M) * setpoint_val;}

            Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N_M,1,y_pdt_M(1)); //only use first value from the setpoint trajectory
            // std::cout<<"theta M_Joint: "<< theta_M<<std::endl;
            // std::cout<<"setpoint M_Joint: "<< setpoint<<std::endl;
            // Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N_M,1,setpoint_val); //matrix of setpoint values
            
            control_effort = MPC_Control_M(setpoint, theta_M, N_M, nu_M, A_M);
            // control_effort = PID_Control(setpoint_val, theta_M, kp_M, ki_M, kd_M, dt_pid);

            // std::cout<<"Controlling M_Joint "<<std::endl;


        }


        //send u(0) to plant
        // std::cout <<"control_effor voltage: " << control_effort <<std::endl;
        if (finger_ID == 4){ //thumb closes when motor truns CCW. Other finger CW
            control_effort = convert_Voltage_to_PWM(control_effort); //convert from voltage to PWM

        }
        else{
            control_effort = convert_Voltage_to_PWM(-control_effort); //convert from voltage to PWM
        }

        // std::cout <<"control_effor pwm: " << -control_effort <<std::endl;
        // control_effort = 50;
        publishData(control_effort); //run publishData method
        ros::spinOnce();
        rate.sleep();
    }



}


    


    

