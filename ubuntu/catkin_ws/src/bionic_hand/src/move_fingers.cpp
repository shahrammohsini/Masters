#include <ros/ros.h>
#include "bionic_hand/ControlCommands.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table addresses
#define ADDR_TORQUE_ENABLE     64
#define ADDR_OPERATING_MODE    11
#define ADDR_GOAL_PWM          100
#define ADDR_PRESENT_POSITION  132

// Protocol version
#define PROTOCOL_VERSION       2.0

// Default settings
// #define DXL1_ID                1  // Motor 1 ID
#define DXL2_ID                2  // Motor 2 ID middle finger
#define DXL3_ID                3  // Motor 3 ID index finger
#define DXL4_ID                4  // Motor 4 ID thumb finger

#define BAUDRATE               57600
#define DEVICE_NAME            "/dev/ttyUSB1"

// Operating mode values
#define OPERATING_MODE_PWM     16

PortHandler *portHandler;
PacketHandler *packetHandler;
GroupBulkWrite *groupBulkWrite;

// Function declarations
void setPWMCallback(const bionic_hand::ControlCommands::ConstPtr &msg);
void shutdownMotors();

// Function to enable torque
void enableTorque(uint8_t dxl_id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to enable torque for ID:%d: %s", dxl_id, packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        ROS_WARN("Dynamixel error while enabling torque for ID:%d: %s", dxl_id, packetHandler->getRxPacketError(dxl_error));
    } else {
        ROS_INFO("Torque enabled for ID:%d", dxl_id);
    }
}

// Function to disable torque
void disableTorque(uint8_t dxl_id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to disable torque for ID:%d: %s", dxl_id, packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        ROS_WARN("Dynamixel error while disabling torque for ID:%d: %s", dxl_id, packetHandler->getRxPacketError(dxl_error));
    } else {
        ROS_INFO("Torque disabled for ID:%d", dxl_id);
    }
}

// Clean up function for safe shutdown
void shutdownMotors() {
    ROS_INFO("Shutting down motors...");
    // disableTorque(DXL1_ID);
    disableTorque(DXL2_ID);
    disableTorque(DXL3_ID);
    disableTorque(DXL4_ID);



    // Close the port and delete objects
    if (portHandler) {
        portHandler->closePort();
        delete portHandler;
    }
    if (groupBulkWrite) {
        delete groupBulkWrite;
    }
    ROS_INFO("Motors safely stopped and port closed.");
}

// Function to set operating mode
void setOperatingMode(uint8_t dxl_id, uint8_t mode) {
    disableTorque(dxl_id);

    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to set operating mode for ID:%d: %s", dxl_id, packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        ROS_WARN("Dynamixel error while setting operating mode for ID:%d: %s", dxl_id, packetHandler->getRxPacketError(dxl_error));
    } else {
        ROS_INFO("Operating mode set to %d for ID:%d", mode, dxl_id);
    }

    enableTorque(dxl_id);
}

// Callback to set PWM for motors
void setPWMCallback(const bionic_hand::ControlCommands::ConstPtr &msg) {
    uint8_t param_goal_pwm[2];
    param_goal_pwm[0] = DXL_LOBYTE((uint16_t)msg->PWM); // Extract low byte of PWM
    param_goal_pwm[1] = DXL_HIBYTE((uint16_t)msg->PWM); // Extract high byte of PWM

    // Add parameter to group bulk write
    if (!groupBulkWrite->addParam(msg->ID, ADDR_GOAL_PWM, 2, param_goal_pwm)) {
        ROS_ERROR("Failed to add parameter for Motor ID:%d", msg->ID);
        return;
    }

    // Send the bulk write packet
    int dxl_comm_result = groupBulkWrite->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Bulk write failed: %s", packetHandler->getTxRxResult(dxl_comm_result));
    }
    // else {
    //     ROS_INFO("PWM set to %f for Motor ID:%d", msg->PWM, msg->ID);
    // }

    // Clear parameters for the next operation
    groupBulkWrite->clearParam();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_finger_node");
    ros::NodeHandle nh;

    // Initialize port handler and packet handler
    portHandler = PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    groupBulkWrite = new GroupBulkWrite(portHandler, packetHandler);

    // Open port
    if (!portHandler->openPort()) {
        ROS_ERROR("Failed to open the port!");
        return -1;
    }
    ROS_INFO("Succeeded in opening the port!");

    // Set baudrate
    if (!portHandler->setBaudRate(BAUDRATE)) {
        ROS_ERROR("Failed to set the baudrate!");
        return -1;
    }
    ROS_INFO("Succeeded in setting the baudrate!");

    // Set operating mode to PWM for both motors
    // setOperatingMode(DXL1_ID, OPERATING_MODE_PWM);
    setOperatingMode(DXL2_ID, OPERATING_MODE_PWM);
    setOperatingMode(DXL3_ID, OPERATING_MODE_PWM);
    setOperatingMode(DXL4_ID, OPERATING_MODE_PWM);



    // ROS subscriber for setting PWM
    ros::Subscriber set_pwm_sub = nh.subscribe("/Control_Command", 10, setPWMCallback);

    // Main loop to check for ROS shutdown
    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    // Clean up before exiting
    shutdownMotors();

    return 0;
}
