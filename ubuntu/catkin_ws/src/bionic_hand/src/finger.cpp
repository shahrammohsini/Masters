#include "finger.h"
#include <ros/ros.h>

// Define the constructor using the scope resolution operator
Finger::Finger(uint8_t motor_id) : id(motor_id), message_published(false) {
    // Initialize other member variables or perform initial setup here if necessary
    
}

// // Define the setPosition method using the scope resolution operator
// void Finger::setPosition(int32_t position) {
//     // ROS_INFO("Node successfully set motor position.");
//     this->msg.id = this->id;  // using 'this' pointer for clarity
//     this->msg.position = position;
//     this->pub.publish(this->msg);

//     ros::spinOnce(); // Handle ROS callbacks
//     ros::Rate loop_rate(2); // Control the rate of loop execution
//     loop_rate.sleep(); // Sleep to maintain the loop rate
// }


// Define the setPWM method using the scope resolution operator
void Finger::setPWM(int32_t PWM) {
    // ROS_INFO("Node successfully set motor position.");
    this->msg.id = this->id;  // using 'this' pointer for clarity
    this->msg.pwm = PWM;
    this->pub.publish(this->msg);

    ros::spinOnce(); // Handle ROS callbacks
    ros::Rate loop_rate(2); // Control the rate of loop execution
    loop_rate.sleep(); // Sleep to maintain the loop rate
}
