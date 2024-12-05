#include <ros/ros.h>
#include "bionic_hand/SetPWM.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table addresses
#define ADDR_TORQUE_ENABLE     64
#define ADDR_OPERATING_MODE    11
#define ADDR_GOAL_PWM          100

// Protocol version
#define PROTOCOL_VERSION       2.0

// Default settings
#define DXL1_ID                1  // Motor 1 ID
#define DXL2_ID                2  // Motor 2 ID
#define BAUDRATE               57600
#define DEVICE_NAME            "/dev/ttyUSB0"

// Operating mode values
#define OPERATING_MODE_PWM     16

PortHandler *portHandler;
PacketHandler *packetHandler;
GroupBulkWrite *groupBulkWrite;

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

// Function to set operating mode
void setOperatingMode(uint8_t dxl_id, uint8_t mode) {
    // Disable torque before changing the operating mode
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

    // Re-enable torque
    enableTorque(dxl_id);
}

// Callback to set PWM for motors
void setPWMCallback(const bionic_hand::SetPWM::ConstPtr &msg) {
    uint8_t param_goal_pwm[2];
    param_goal_pwm[0] = DXL_LOBYTE((uint16_t)msg->pwm);
    param_goal_pwm[1] = DXL_HIBYTE((uint16_t)msg->pwm);

    // Add motor 1 to bulk write
    if (!groupBulkWrite->addParam(DXL1_ID, ADDR_GOAL_PWM, 2, param_goal_pwm)) {
        ROS_ERROR("Failed to add parameter for Motor 1");
        return;
    }

    // Add motor 2 to bulk write
    if (!groupBulkWrite->addParam(DXL2_ID, ADDR_GOAL_PWM, 2, param_goal_pwm)) {
        ROS_ERROR("Failed to add parameter for Motor 2");
        return;
    }

    // Send bulk write packet
    int dxl_comm_result = groupBulkWrite->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Bulk write failed: %s", packetHandler->getTxRxResult(dxl_comm_result));
    } else {
        ROS_INFO("PWM set to %d for both motors", msg->pwm);
    }

    // Clear parameters for next write
    groupBulkWrite->clearParam();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pwm_control_node");
    ros::NodeHandle nh;
    // ROS subscriber for setting PWM
    ros::Subscriber set_pwm_sub = nh.subscribe("/set_pwm", 10, setPWMCallback);

    // Initialize port handler and packet handler
    portHandler = PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Initialize GroupBulkWrite
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
    setOperatingMode(DXL1_ID, OPERATING_MODE_PWM);
    setOperatingMode(DXL2_ID, OPERATING_MODE_PWM);

    

    // Spin to process ROS callbacks
    ros::spin();

    // Clean up
    portHandler->closePort();
    delete groupBulkWrite;

    ROS_INFO("Closed the port. Shutting down.");
    return 0;
}
