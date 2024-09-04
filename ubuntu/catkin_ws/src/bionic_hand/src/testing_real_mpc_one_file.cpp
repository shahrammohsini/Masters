#include <iostream>
#include <fstream>
#include <unistd.h> // Needed for sleep function
#include <vector>
#include <eigen3/Eigen/Dense>//for matrix calculations 
#include <random>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include "bionic_hand/FingerPos.h"
#include "controller.h"

//global variables
//mpc parameters
float N_D = 29;
float nu_D = 2;
float LAMBDA_D = 1.5;
float time_Step = 0.03;

Eigen::MatrixXd DM_j_D;
Eigen::MatrixXd A_D = Eigen::MatrixXd::Zero(N_D,nu_D);
Eigen::MatrixXd du_D;

Eigen::MatrixXd u_D = Eigen::MatrixXd::Zero(nu_D, 1); // control input -  1 dimentional matrix filled with zeros;
double PHI_D;
Eigen::MatrixXd errors_D = Eigen::MatrixXd::Zero(N_D, 1); // error;
Eigen::MatrixXd delta_u_D = Eigen::MatrixXd::Zero(nu_D, 1); // delta_u;
Eigen::MatrixXd y_hat_D = Eigen::MatrixXd::Zero(N_D, 1);
Eigen::MatrixXd u_prev_D = Eigen::MatrixXd::Zero(nu_D, 1); // previous input;
Eigen::MatrixXd delta_y_D = Eigen::MatrixXd::Zero(N_D, 1); // change in predicted output;


double max_Voltage = 12;
double min_Voltage = -12;



// Dynamixel settings
#define PROTOCOL_VERSION 2.0
#define DXL_ID 1
#define BAUDRATE 57600
#define DEVICENAME "/dev/ttyUSB0"
#define ADDR_PRO_GOAL_PWM 100
#define ADDR_PRO_TORQUE_ENABLE 64
#define ADDR_PRO_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_OPERATING_MODE 11
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define MAX_PWM 885
#define POSITION_MODE 3
#define PWM_MODE 16


// Initialize Dynamixel
dynamixel::PortHandler* initializeDynamixel(const char* devicename, int baudrate) {
    auto portHandler = dynamixel::PortHandler::getPortHandler(devicename);
    if (!portHandler->openPort()) {
        throw std::runtime_error("Failed to open the port");
    }
    if (!portHandler->setBaudRate(baudrate)) {
        throw std::runtime_error("Failed to change the baudrate");
    }
    return portHandler;
}

float read_position(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler, int dxl_ID, int addr_presnt_position){
    uint32_t dxl_present_position;
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_ID, addr_presnt_position, &dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        packetHandler->getTxRxResult(dxl_comm_result);
        return -1.0; //indicate an error
    } else if (dxl_error != 0) {
        packetHandler->getRxPacketError(dxl_error);
        return -1.0; //indicate an error
    }
    // Convert the position reading if necessary (depends on Dynamixel model and firmware)
    if (dxl_present_position > 2147483647){  // 2147483647 is 2^31 - 1, half of the range of 32-bit signed integer
        dxl_present_position -= 4294967296;  // Subtract 2^32 to get the correct negative value
    }
    //  Convert position from raw value to degrees if necessary (depends on Dynamixel model)
    dxl_present_position = dxl_present_position * 0.088;  // Example conversion factor to degrees (may vary)
    
    
    return dxl_present_position; //return current position
}



void set_goal_position(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler, int dxl_id,  float position){
    int dxl_comm_result; // Result of communication
    uint8_t dxl_error = 0; // Dynamixel error

    // Assuming goal position is a 4-byte value; change to write2ByteTxRx if it's a 2-byte value
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_PRO_GOAL_POSITION, position, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Failed to set goal position: " << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        throw std::runtime_error("Failed to set goal position");
    }
    if (dxl_error != 0) {
        std::cerr << "Dynamixel error: " << packetHandler->getRxPacketError(dxl_error) << std::endl;
    } else {
        std::cout << "Moving robot to starting position: " << position << std::endl;
    }
}



//noise function
double generateNoise(double mean, double stddev) {
    // Create a random number generator with a normal distribution
    static std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, stddev);

    // Generate and return the noise
    return distribution(generator);
}


// PID Controller Function
float PIDController(float target_theta, float current_theta, float& integral, float& prev_error, float time_Step) {
    // PID controller gains
    float Kp = 1;  // Proportional gain
    float Ki = 3;  // Integral gain
    float Kd = 0; // Derivative gain

    // Calculate the error
    float error = target_theta - current_theta;

    // Proportional term
    float P_out = Kp * error;

    // Integral term
    integral += error * time_Step;
    float I_out = Ki * integral;

    // Derivative term
    float derivative = (error - prev_error) / time_Step;
    float D_out = Kd * derivative;

    // Update previous error
    prev_error = error;

    // Compute the output voltage
    float output = P_out + I_out + D_out;
    return output;
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


Eigen::MatrixXd generate_Dynamic_Matrix(int prediction_horizon_D, int control_horizon_D){
    Eigen::MatrixXd DM_j_D(prediction_horizon_D, control_horizon_D);
    
    //open file
    std:: string filePath = "//home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/open_loop_data.csv";
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


    DM_j_D.setZero(); //populate with zeros
    // float bias = 0.5; //If motor has a bias value (min voltage to move motor)
    int normalize_val = 1;
    // DM_j_D.setConstant(bias); // populate with min voltage to move. This is the bias
    //populate dynamic matrix
     for (int i =0; i < control_horizon_D; i++) { //create columns
        for(int j=0; j < prediction_horizon_D - i; j++){ //create rows
            const auto& d = dataset[j]; //d holds the current row
            DM_j_D(i + j, i) = ((d.theta_D_joint)/normalize_val); //fill current row for current column. Also, normalize the angle
            // if(DM_j_D(i + j, i) < bias) {DM_j_D(i + j, i) = bias;} //The motor does not move bellow 4 volts so to normalize we have to add 4 volts to values bellow 4 volts
        }
     }

    return DM_j_D;

}

Eigen::MatrixXd generate_du_Matrix(int prediction_horizon_D, int control_horizon_D, Eigen::MatrixXd A_D, float LAMBDA_D){
    Eigen::MatrixXd A_T_D;
    Eigen::MatrixXd LambdaI_D;
    Eigen::MatrixXd ATA_D;
    Eigen::MatrixXd ATA_LambdaI_D;
    Eigen::MatrixXd ATA_LambdaI_Inv_D;
    Eigen::MatrixXd du_D;

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

    return du_D;
}

//**convert voltage to pwm. Input: voltae, output: pwm **/
double convert_Voltage_to_PWM(double voltage){
    double max_pwm = 885;
    double PWM = (voltage/max_Voltage)*max_pwm;
    return PWM;

}

double MPC_Control_D(Eigen::MatrixXd setpoint,double measured_position_D, int N_D, int nu_D, Eigen::MatrixXd A_D){
    
    PHI_D = measured_position_D - y_hat_D(0); //PHI is the difference between actual model and predicted model

    y_hat_D = y_hat_D + Eigen::MatrixXd::Constant(y_hat_D.rows(), y_hat_D.cols(), PHI_D); //add the constant PHI to each value of y_hat

    errors_D = setpoint - y_hat_D; //error
    // std::cout << "errors: \n" << errors_D(0,0) <<std::endl; 

    // std::cout << "du: \n" << du <<std::endl; 
    // std::cout << "du_D: \n" << du_D <<std::endl; 

    delta_u_D = du_D*errors_D; //Δu=((ATA+λI)^−1)AT(setpoint-y_hat) // delta_u is the control input step
    std::cout << "u_prev 2: \n" << u_prev_D <<std::endl; 
    u_D = u_prev_D + delta_u_D; //calculate control move vector
    // std::cout << "u: \n" << u <<std::endl; 
    
    
    //add control input limits here
    for(int i = 0; i < nu_D; i++){
        if (u_D(i) > max_Voltage){
            u_D(i) = max_Voltage;
        delta_u_D(i) = u_D(i) - u_prev_D(i); //revaluate delta_u to account for the limits you've introduced to u

        }
        else if (u_D(i) < min_Voltage){
            u_D(i) = min_Voltage;
            delta_u_D(i) = u_D(i) - u_prev_D(i); //revaluate delta_u to account for the limits you've introduced to u

        }
    }


    
    delta_y_D = A_D.col(0)*delta_u_D(0,0); //caluclate change in predicted output: y_1 - y_0 = (a_1)(Delta u_0). Multiply first col of A_D with first val of delta_u

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
    std::cout << "u_prev 1: \n" << u_prev_D <<std::endl; 


    return u_D(0);

}


Controller::Controller(const std::string& finger_name) {
    sub_ = nh.subscribe("Updated_Finger_Position", 1000, &Controller::fingerPositionCallback, this);  //initialize subscriber node
}

//create a subscriber node to get the latest pos info
void Controller::fingerPositionCallback(const bionic_hand::FingerPos& msg){
    ros::Time current_time = ros::Time::now();
    ros::Duration time_diff = current_time - msg.header.stamp;
    
    // Check if the message is too old
    float acceptable_delay = 0.01;
    if (time_diff.toSec() > acceptable_delay) {
        ROS_WARN("Received stale finger position data. Delay: %f seconds", time_diff.toSec());
        return;
    }

    
    
    position = msg;
    theta_M = position.theta_M;
    theta_P = position.theta_P;
    theta_D = position.theta_D;
    // ros::Rate rate(1.0 / dt); // rate = 10 Hz if dt = 0.1

    // ROS_INFO("theta_P: %f", theta_P);


    // std::cout <<"position" << position.theta_D << std::endl;
}


// Define a custom data type to hold open loop data in csv file
// struct openLoop_data{
//     double time;
//     double theta_m;
//     double theta_D_joint;
//     double theta_P_joint;
//     double theta_M_joint;
//     int pwm;
// };






int main() {
    //sim parameters
    float currentTime = 0.0;
    float theta_D_joint = 0;
    float prev_theta_D = 0;
    float voltage = 0; // Voltage starts at 0
    float max_D_joint_angle = 72;
    float target_theta = 25; // Target position in degrees
    double setpoint_val = 35;
    Eigen::MatrixXd setpoint = Eigen::MatrixXd::Constant(N_D,1,setpoint_val); //matrix of setpoint values

    

    auto portHandler = initializeDynamixel(DEVICENAME, BAUDRATE);
    auto packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

 // # Set operating mode to Position Control Mode
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, POSITION_MODE);
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
    float starting_point = 180;
    float final_point = 350;
    float goal_position = int(starting_point / final_point * 4095);  // Convert 180 degrees to the corresponding value (assuming 12-bit resolution)
    std::cout<<"Moving robot to starting position: " << goal_position<< std::endl;
    set_goal_position(packetHandler, portHandler, DXL_ID, goal_position);
    sleep(1);
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE); //disable torequ
    std::cout<<"Begin Control "<< std::endl;

    // Set operating mode to PWM
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, PWM_MODE);

    // Enable torque
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);


    // // PID controller parameters
    // float integral = 0.0;
    // float prev_error = 0.0;

    //generate dynamic matrix:
    DM_j_D = generate_Dynamic_Matrix(N_D,nu_D);
    A_D = DM_j_D;
    std::cout<<"A_D: " << A_D <<std::endl;

    du_D = generate_du_Matrix(N_D, nu_D, A_D, LAMBDA_D);
    std::cout<<"du_D: " << du_D <<std::endl;




    // Open a file to store the data
    std::ofstream dataFile("data.dat");
    double pwm = 0;
    

    while(currentTime < 6){
        // Calculate the control input using PID controller
        // voltage = PIDController(target_theta, theta_D_joint, integral, prev_error, time_Step);

        // // Limit the voltage to a maximum value (e.g., the range of the motor)
        // if (voltage > 12) voltage = 12;
        // if (voltage < -12) voltage = -12;
        



        
        //mpc controller
        voltage = MPC_Control_D(setpoint, theta_D_joint, N_D, nu_D, A_D);
        pwm = convert_Voltage_to_PWM(voltage);
        int dxl_present_position = read_position(packetHandler, portHandler, DXL_ID, ADDR_PRESENT_POSITION);

        // std::cout << "Current Position: " << dxl_present_position << " -- PWM: " << g_PWM << std::endl;
        if (dxl_present_position < 175 || dxl_present_position > 340) {
            packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, 0);
            std::cout<<"Motor limit reached. Stopping at: "<< dxl_present_position<< std::endl;
            break;

        }
        else{
            packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, pwm);
        }
        // ros::Rate rate(1.0 / 0.005); // rate = 10 Hz if dt = 0.1


        ros::spinOnce();
        
        // std::cout << "theta_D_joint: " << theta_D_joint << ", voltage: " << voltage << std::endl;
        // theta_D_joint = prev_theta_D + time_Step * ((-5.712) * prev_theta_D + 104.5 * voltage);
        
        //add noise to the simulation
        // double noise = generateNoise(0.0, 0.5); // Mean = 0, Stddev = 0.5 (adjust these values as needed)
        // theta_D_joint += noise;

        // Write the time and theta_D_joint values to the file
        // dataFile << currentTime << " " << theta_D << std::endl;

        // currentTime += time_Step;
        // prev_theta_D = theta_D_joint;

        // usleep(time_Step * 1e6); // Sleep for time_Step seconds
    }

    dataFile.close();

    // Use GNUPlot to plot the data
    FILE *gnuplotPipe = popen("gnuplot -persistent", "w");
    fprintf(gnuplotPipe, "set title 'Theta_D_joint over Time'\n");
    fprintf(gnuplotPipe, "set xlabel 'Time (s)'\n");
    fprintf(gnuplotPipe, "set ylabel 'Theta_D_joint'\n");
    fprintf(gnuplotPipe, "plot 'data.dat' with lines\n");
    pclose(gnuplotPipe);

    return 0;
}
