#ifndef FINGERCONFIG_H
#define FINGERCONFIG_H

// Define a structure to hold joint parameters for a finger
struct JointParameters { //a and b are the distances from the rotation point of each joint to the edge of the joint where the string makes contact. a1 = m joint, a2 = p joint a3 = d jont.
    double a1;
    double b1;
    double a2;
    double b2;
    double a3;
    double b3;
};

// Define a structure to hold all configuration parameters for a finger
struct FingerConfig {
    JointParameters joints;
    double cm_travled;      // Potentiometer counts per cm.
    double offset;          //initial potentiometer value before string is drawn 
    double max_j3_length;  // length when j3 (Joint D) is fully bent
    double max_j2_length;
    double max_j1_length;
    double max_j3_angle;    // degrees
    double max_j2_angle;    // degrees
    double max_j1_angle;    // degrees
};


#endif



