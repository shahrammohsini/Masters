#include <ros/ros.h>
#include "std_msgs/String.h"
#include "bionic_hand/GetPosition.h"
#include "bionic_hand/SetPWM.h"  // Updated to use SetPWM
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_PWM         100
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION      2.0

// Default setting
#define DXL_ID                1
#define BAUDRATE              57600
#define DEVICE_NAME           "/dev/ttyUSB0"

PortHandler *portHandler;
PacketHandler *packetHandler;

void setPWMCallback(const bionic_hand::SetPWM::ConstPtr & msg) {
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  uint32_t pwm_value = (uint32_t)msg->pwm;

  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, (uint8_t)msg->id, ADDR_GOAL_PWM, pwm_value, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPWM : [ID:%d] [PWM:%d]", msg->id, msg->pwm);
  } else {
    ROS_ERROR("Failed to set PWM! Result: %d", dxl_comm_result);
  }
}

int main(int argc, char **argv) {
  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);


  uint8_t dxl_error = 0;  // This line declares the dxl_error variable
  
  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  if (packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error) != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL_ID);
    return -1;
  }

  ros::init(argc, argv, "pwm_control_node");
  ros::NodeHandle nh;
  // ros::ServiceServer get_position_srv = nh.advertiseService("/get_position", getPresentPositionCallback);
  ros::Subscriber set_pwm_sub = nh.subscribe("/set_pwm", 10, setPWMCallback);  // Updated topic name to /set_pwm
  ros::spin();

  portHandler->closePort();
  return 0;
}
