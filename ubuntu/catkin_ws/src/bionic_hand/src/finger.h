#ifndef FINGER_H
#define FINGER_H

#include <ros/ros.h>
#include <bionic_hand/SetPosition.h> // Import SetPosition.msg file
#include <iostream>

class Finger {
public:
    Finger(uint8_t motor_id);
    void setPosition(int32_t position);
    ros::Publisher pub; // Declare the publisher as a member variable
    bionic_hand::SetPosition msg; // declare msg as custom data taype SetPosition (the file is in bionic_hand/msg)

private:
    uint8_t id; // Motor ID, private access
    bool message_published; // Tracks status of message publishing
};

#endif // FINGER_H
