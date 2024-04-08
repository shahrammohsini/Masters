from dynamixel_sdk import *                    # Import the Dynamixel SDK library

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
DXL_ID = 1                                    # Dynamixel ID: 1
BAUDRATE = 57600                            # Baud Rate: 1 Mbps
DEVICENAME = 'COM10'                   # Check your port name (Linux: '/dev/ttyUSB0', Windows: 'COMx')

# Control table addresses
ADDR_PRO_TORQUE_ENABLE = 64                   # Control table address for torque enable
ADDR_PRO_GOAL_POSITION = 116                  # Control table address for goal position
ADDR_PRO_PRESENT_POSITION = 132               # Control table address for present position

# Data Byte Length
LEN_PRO_GOAL_POSITION = 4                     # Data length for goal position
LEN_PRO_PRESENT_POSITION = 4                  # Data length for present position

# Protocol version
PROTOCOL_VERSION = 2.0

# Initialize PortHandler and PacketHandler instances
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    exit()

# Set port baud rate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    exit()

# Enable Dynamixel Motor Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 1)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

# Write goal position
goal_position = 20  # Example goal position
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, goal_position)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print(f"Goal position set to {goal_position}")

# Close port
portHandler.closePort()
