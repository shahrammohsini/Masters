#include <ros/ros.h>

#include <bionic_hand/SetPosition.h> 

int main(int argc, char ** argv)
{

    ros::init(argc, argv, "move_motor"); //Initialize a ros node
    ROS_INFO("Node successfuly started");

    // Create a NodeHandle
    ros::NodeHandle nh;

    // Create a publisher
    //ros::Publisher pub = nh.advertise<your_package_name::YourMessageType>("topic_name", 10);
    ros::Publisher pub = nh.advertise<bionic_hand::SetPosition>("/set_position", 10);

    ros::Rate loop_rate(2);


    while (ros::ok())
    {
        bionic_hand::SetPosition msg;

        
        msg.id = 1;
        msg.position = 1000;
        // Publish the message
        pub.publish(msg);
    

        // Sleep to maintain loop rate
        loop_rate.sleep();


    }









// #include "dynamixel_sdk_examples/SetPosition.h"


// using namespace dynamixel;













}