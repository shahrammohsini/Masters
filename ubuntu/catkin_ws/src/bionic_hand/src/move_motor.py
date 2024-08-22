import serial
import csv
import os
import dynamixel_sdk as dxl
import time
import queue
# from scipy import signal
import numpy as np
from threading import Thread
import rospy

# Protocol version and Dynamixel settings
PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'
ADDR_PRO_GOAL_PWM = 100
ADDR_PRO_TORQUE_ENABLE = 64
ADDR_PRO_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11  # Address for the operating mode
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
MAX_VOLTAGE = 12.2
MAX_PWM = 885
step_input = 12  # input voltage
POSITION_MODE = 3
PWM_MODE = 16
step_magnitude = 5 #volts
from bionic_hand.msg import ControlCommands


# Function to initialize Dynamixel
def initialize_dynamixel(devicename, baudrate, protocol_version):
    portHandler = dxl.PortHandler(devicename)
    packetHandler = dxl.PacketHandler(protocol_version)
    if not portHandler.openPort():
        raise IOError("Failed to open the port")
    if not portHandler.setBaudRate(baudrate):
        raise IOError("Failed to change the baudrate")
    return portHandler, packetHandler

# Function to set operating mode
def set_operating_mode(packet_handler, port_handler, mode):
    disable_torque(packet_handler, port_handler, DXL_ID)
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_OPERATING_MODE, mode)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        raise Exception("Failed to set operating mode")
    if dxl_error != 0:
        print("Dynamixel error: %s" % packet_handler.getRxPacketError(dxl_error))


# Function to enable torque
def enable_torque(packetHandler, portHandler, dxl_id, torque_enable):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, torque_enable)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        raise RuntimeError(f"Torque enable failed: {packetHandler.getTxRxResult(dxl_comm_result)}")

# Function to disable torque
def disable_torque(packetHandler, portHandler, dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)


# Function to close port
def close_port(portHandler):
    portHandler.closePort()

def read_position(packet_handler, port_handler, dxl_id, addr_present_position):
    dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, dxl_id, addr_present_position)
    if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error != 0:
        print("Failed to read position")
        return None
    
    # Convert the position reading if necessary (depends on Dynamixel model and firmware)
    if dxl_present_position > 2147483647:  # 2147483647 is 2^31 - 1, half of the range of 32-bit signed integer
        dxl_present_position -= 4294967296  # Subtract 2^32 to get the correct negative value

    # Convert position from raw value to degrees if necessary (depends on Dynamixel model)
    dxl_present_position = dxl_present_position * 0.088  # Example conversion factor to degrees (may vary)
    
    return dxl_present_position



# Function to set goal position
def set_goal_position(packet_handler, port_handler, position):
    dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, DXL_ID, ADDR_PRO_GOAL_POSITION, position)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        raise Exception("Failed to set goal position")
    if dxl_error != 0:
        print("Dynamixel error: %s" % packet_handler.getRxPacketError(dxl_error))


# Function to set PWM
def set_pwm(packet_handler, port_handler, dxl_id, addr_pro_goal_pwm, pwm):
    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, dxl_id, addr_pro_goal_pwm, pwm)
    if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error != 0:
        print("Failed to set PWM: %s, %s" % (packet_handler.getTxRxResult(dxl_comm_result), packet_handler.getRxPacketError(dxl_error)))
        return False
    print(f"Set PWM to {pwm}")
    return True


class FingerControlCommandSubscriber:
    def __init__(self):
        # Initialize the ROS node
        # rospy.init_node('control_command_subscriber', anonymous=True)

        # Variable to hold the position

        self.PWM = 0

        # Set up the subscriber
        # The callback function is called every time a message is received
        self.subscriber = rospy.Subscriber("Control_Command", ControlCommands, self.update_control_command)
        self.thread = Thread(target=self.spin) #create a thread for the spin method so it runs independantly of the rest of the code
        self.thread.daemon = True #thread will exit when main program is terminated
        self.thread.start() # start thread

    def update_control_command(self, msg):
        """Callback function to handle incoming messages."""
        # self.finger_position = msg
        self.PWM = msg.PWM

        # rospy.loginfo(f"Updated finger position: {msg}")

    def spin(self):
        """Keep the node running so it can keep receiving messages."""
        rospy.spin()
    
    def get_latest_control_command(self):
        # print(self.PWM)

        return self.PWM
    

def main():
    rospy.init_node('move_motor_node', anonymous=True)
    fccs = FingerControlCommandSubscriber()

    portHandler, packetHandler = None, None
    print("Motor 1 ready to move")

    try:

        # Initialize PortHandler and PacketHandler
        portHandler, packetHandler = initialize_dynamixel(DEVICENAME, BAUDRATE, PROTOCOL_VERSION)

        # Switch back to PWM mode
        set_operating_mode(packetHandler, portHandler, PWM_MODE)
        enable_torque(packetHandler, portHandler, DXL_ID, TORQUE_ENABLE)
        pwm = int(fccs.get_latest_control_command())
        # print("pwm: ", pwm)

        # Main loop to set PWM values
        while True:
            dxl_present_position = read_position(packetHandler, portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            if dxl_present_position < 175 or dxl_present_position > 250: #if the motor reaches max safe pos stop. CREATE A SEPERATE FILE/FUNCTION FOR THIS
                print("Stopping pos: ", dxl_present_position)
                set_pwm(packetHandler, portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, pwm = 0) #stop 
                break
            if not set_pwm(packetHandler, portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, pwm): #set pwm to whatever is on the pwms list
                continue
            # time.sleep(dt)





    except Exception as e:
        print(e)

    finally:
        if packetHandler and portHandler:
            disable_torque(packetHandler, portHandler, DXL_ID)
            close_port(portHandler)

if __name__ == "__main__":
    main()
