// #include <ros/ros.h>
// #include <bionic_hand/SetPosition.h> // Import SetPosition.msg file
// #include <iostream>
// using namespace std;



// class Motor{
//     public:
//         Motor(uint8_t motor_id) : id(motor_id) {} // Contructor method to initialize id for each motor. This type of constructor is called initialization list. Its a more effecient way to initialize with constructors
//         ros::Publisher pub; // Declare the publisher as a member variable
//         bionic_hand::SetPosition msg;



//     void setPosition(int32_t position){

//         ROS_INFO("Node successfuly ");
//         msg.id = id;
//         msg.position = position;
//         // Publish the message
//         pub.publish(msg);
    
//         // Allow some time for ROS to publish the message
//         ros::spinOnce(); // Handle callbacks
//         ros::Rate loop_rate(2); // Adjust the loop rate as necessary
//         loop_rate.sleep(); // Sleep for a short duration

//     }

//     private: 
//         uint8_t id; // Make id private
//         bool message_published;



// };


// int main(int argc, char** argv) {
//     ros::init(argc, argv, "move_motor"); // Initialize ROS node
//     // Create a NodeHandle
//     ros::NodeHandle nh;

//     // Create a Motor object
//     Motor motor_1(1); // Example: Motor with id 1
        

//     // // Create the publisher
//     motor_1.pub = nh.advertise<bionic_hand::SetPosition>("/set_position", 10);

//     // Example: Set position for motor
//     int input;
    
//     while (1) {

//         cout << "Enter New Pos: ";
//         cin >> input;
//         motor_1.setPosition(input);
//     }


//     return 0;
// }
