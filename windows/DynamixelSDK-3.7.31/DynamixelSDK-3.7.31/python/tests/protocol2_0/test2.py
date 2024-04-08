#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 2.0
# This example is designed for using a Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
# To use another Dynamixel model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below variables yourself.
# Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
#

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132
ADDR_PRO_VELOCITY          = 128               # Address of velocity from the manual on Dynamixel website
ADDR_PRO_TORQUE            = 126
ADDR_PRO_GOAL_VELOCITY     = 104
ADDR_PRO_OPERATING_MODE    = 11                # Control table address for operating mode


# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM10'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 3           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1800            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold
VELOCITY_CONTROL_MODE       = 1                 # Value for velocity control mode. 1 is for velocity
POSITION_CONTROL_MODE       = 3
index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
goal_velocity = 20  # desired velocity if we're driving by velocity

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()




# Set Dynamixel operating mode.  By default its position cotrol mode
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print("Failed to set operating mode: %s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("Error setting operating mode: %s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode successfully")

# Ensure to wait a bit after changing operating mode, as it may require a reboot of the Dynamixel
time.sleep(0.5)




# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")







while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break
    # Write goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        # time.sleep(0.5)
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Write goal velocity
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_VELOCITY, goal_velocity)
    if dxl_comm_result != COMM_SUCCESS:
        print("Failed to write goal velocity: %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("Error writing goal velocity: %s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Goal velocity set successfully")



    while 1:
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        # Read velocity
        dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_VELOCITY)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            # Convert from unsigned to signed integer if the value is negative
            if dxl_present_velocity & (1 << 31):  # If the sign bit (32nd bit) is set
                dxl_present_velocity_signed = dxl_present_velocity - (1 << 32)
            else:
                dxl_present_velocity_signed = dxl_present_velocity

        # Read torque
        dxl_present_torque, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE)
        # dxl_present_torque, dxl_comm_result, dxl_error = packetHandler.read2ByteTx
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # convert from unsigned to signed integer if the value is negative
        if dxl_present_torque & (1 << 15):  # If the sign bit (16th bit) is set
            dxl_present_load_signed = dxl_present_torque - (1 << 16)
        else:
            dxl_present_load_signed = dxl_present_torque

        # print position
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position[index], dxl_present_position))
        #print velocity
        print("present velocity: ", dxl_present_velocity_signed)
        #print torque
        # print("present torque: ", dxl_present_load_signed)

        if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            print("flip")
            goal_velocity = -goal_velocity 
            break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0


# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()


