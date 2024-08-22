#include <iostream>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <ros/ros.h>
#include <thread>
#include <bionic_hand/ControlCommands.h>  //ROS message header

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

// Global variable for PWM value
int g_PWM = 0;

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

// ROS callback function
void controlCommandCallback(const bionic_hand::ControlCommands::ConstPtr& msg) {
    g_PWM = msg->PWM;
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

// Main function
int main(int argc, char **argv) {
    std::cout << "Prepared to move motor" << std::endl;

    
    ros::init(argc, argv, "move_motor_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("Control_Command", 1000, controlCommandCallback);

    auto portHandler = initializeDynamixel(DEVICENAME, BAUDRATE);
    auto packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Set operating mode to PWM
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, PWM_MODE);

    // Enable torque
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

    // Main loop
    while (ros::ok()) {
        // std::cout << "driving motor" << std::endl;
        
        int dxl_present_position = read_position(packetHandler, portHandler, DXL_ID, ADDR_PRESENT_POSITION);

        std::cout << "Current Position: " << dxl_present_position << std::endl;
        if (dxl_present_position < 175 || dxl_present_position > 250) {
            packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, 0);
            // break;
        }
        packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, g_PWM);
    
        ros::spinOnce();
    }

    // Disable torque and close port
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
    portHandler->closePort();

    return 0;
}
