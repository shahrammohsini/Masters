#include <ros/ros.h>
#include <bionic_hand/SetPWM.h> // Import SetPosition.msg file
#include <iostream>
#include "finger.h"
using namespace std;
#include "std_msgs/Int32.h"
#include "controller.h"
#include <chrono>
#include <thread>

std_msgs::Int32 int_msg;

int global_int_value = 0;

void intCallback(const std_msgs::Int32::ConstPtr& msg)
{
    global_int_value = msg->data;
}



int main(int argc, char** argv) {

    float max_M_joint_angle;
    float max_P_joint_angle;
    float max_D_joint_angle;


    // sleep for 1 sec
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout <<"begin------------------------------------------------------------------------"<< std::endl;


    // ros::init(argc, argv, "move_motor"); // Initialize ROS node
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh; // Create a NodeHandle

    // Set up AsyncSpinner with 2 threads. This ensures data collection is done in a seperate thread
    ros::AsyncSpinner spinner(3); // Number of threads
    spinner.start();

    // Create a finger object
    Finger middle_finger(2, "middle_finger"); // Set motor ID. Middle finger's motor has id 2.
    Finger index_finger(3, "index_finger"); // Set motor ID. Middle finger's motor has id 3.
    Finger thumb_finger(4, "thumb_finger"); // Set motor ID. thumb finger's motor has id 4.


    // controller object for middle finger
    Controller control_middle_finger(middle_finger.id, middle_finger.name); //create a middle finger object for Controller class
    Controller control_index_finger(index_finger.id, index_finger.name); //create a middle finger object for Controller class
    Controller control_thumb_finger(thumb_finger.id, thumb_finger.name); //create a thumb finger object for Controller class


    
    // Create the publisher with queue size 10. **NOTE: This line should later be changed when new fingers are added. It should be made so that all the 
    //fingers use the same publisher and publish a new pos for all the fingers at the same time. The set position function will just look for the pos of the specific finger. Should also look at the option of moving the publisher decleration to finger.cpp 
    middle_finger.pub = nh.advertise<bionic_hand::SetPWM>("/set_pwm", 10);
    index_finger.pub = nh.advertise<bionic_hand::SetPWM>("/set_pwm", 10);
    thumb_finger.pub = nh.advertise<bionic_hand::SetPWM>("/set_pwm", 10);


    
    // ros::init(argc, argv, "int_subscriber");
    ros::Subscriber sub = nh.subscribe("/int_topic", 1000, intCallback);

    //dersired finger setpoints
    float setpoint_M_mid = 0;
    float setpoint_P_mid = 90;
    float setpoint_D_mid =  85;

    float setpoint_M_ind = 0;
    float setpoint_P_ind = 45;
    float setpoint_D_ind =  110;

    float setpoint_M_thumb = 0;
    float setpoint_P_thumb = 0;
    float setpoint_D_thumb =  45;


    //Run each finger's controller in a seperate thread to control all fingers symultaneously
   std::thread middle_thread([&](){
        
        control_middle_finger.run(setpoint_M_mid, setpoint_P_mid, setpoint_D_mid);
    });

    std::thread index_thread([&](){
        // This lambda runs in another separate thread
        control_index_finger.run(setpoint_M_ind, setpoint_P_ind, setpoint_D_ind);
    });

    std::thread thumb_thread([&](){
        // This lambda runs in another separate thread
        control_thumb_finger.run(setpoint_M_thumb, setpoint_P_thumb, setpoint_D_thumb);
    });

  

    //wait for my command to stop running
    ros::waitForShutdown();

    // Join the threads before exiting (if run() is blocking and will eventually end)
    if(middle_thread.joinable())
        middle_thread.join();
    if(index_thread.joinable())
        index_thread.join();
    if(thumb_thread.joinable())
        thumb_thread.join();

    
    return 0;
}
