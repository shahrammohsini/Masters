import os
import dynamixel_sdk as dxl
import time

# Protocol version
PROTOCOL_VERSION = 2.0

# Default settings
DXL_ID = 1  # Dynamixel ID
BAUDRATE = 57600  # Dynamixel default baudrate
DEVICENAME = '/dev/ttyUSB0'  
ADDR_PRO_GOAL_PWM = 100  # Address of Goal PWM (depends on your motor model)
ADDR_PRO_TORQUE_ENABLE = 64  # Address of Torque Enable
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MINIMUM_PWM_VALUE = 0  # Dynamixel will accept
DXL_MAXIMUM_PWM_VALUE = 885  # Dynamixel will accept, adjust if needed
ADDR_PRO_PRESENT_VOLTAGE = 144  # Address of Present Input Voltage
ADDR_PRESENT_POSITION = 132


def correct_negative_position(position):
    if position > 2147483648:
        position -= 4294967296
    return position



# Initialize PortHandler instance
portHandler = dxl.PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != dxl.COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

# control the motor using pwm
exit = False
while not exit:
    # Write goal PWM
    goal_pwm = int(input("enter pwm: "))
    if (goal_pwm == 1000):
        exit = True
        goal_pwm = 0

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, goal_pwm)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Goal PWM has been set")




    # Read present position
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    dxl_present_position = correct_negative_position(dxl_present_position)
    print("present pos: ", dxl_present_position)



# Close port
portHandler.closePort()
