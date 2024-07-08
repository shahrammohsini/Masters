import dynamixel_sdk as dxl
import time

# Protocol version and Dynamixel settings
PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = 'COM10'
ADDR_PRO_GOAL_POSITION = 116
ADDR_PRO_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11  # Address for the operating mode
POSITION_MODE = 3  # Value for Position Control Mode
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

def initialize_port():
    """Initialize the PortHandler and PacketHandler."""
    port_handler = dxl.PortHandler(DEVICENAME)
    packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)
    if not port_handler.openPort():
        raise Exception("Failed to open the port")
    if not port_handler.setBaudRate(BAUDRATE):
        raise Exception("Failed to change the baudrate")
    return port_handler, packet_handler

def set_operating_mode(packet_handler, port_handler, mode):
    """Set the operating mode."""
    disable_torque(packet_handler, port_handler)
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_OPERATING_MODE, mode)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        raise Exception("Failed to set operating mode")
    if dxl_error != 0:
        print("Dynamixel error: %s" % packet_handler.getRxPacketError(dxl_error))

def enable_torque(packet_handler, port_handler):
    """Enable Dynamixel torque."""
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        raise Exception("Failed to enable torque")
    if dxl_error != 0:
        print("Dynamixel error: %s" % packet_handler.getRxPacketError(dxl_error))

def disable_torque(packet_handler, port_handler):
    """Disable Dynamixel torque."""
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        raise Exception("Failed to disable torque")
    if dxl_error != 0:
        print("Dynamixel error: %s" % packet_handler.getRxPacketError(dxl_error))

def set_goal_position(packet_handler, port_handler, position):
    """Set the goal position."""
    dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, DXL_ID, ADDR_PRO_GOAL_POSITION, position)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        raise Exception("Failed to set goal position")
    if dxl_error != 0:
        print("Dynamixel error: %s" % packet_handler.getRxPacketError(dxl_error))

def main():
    """Main function to run the Dynamixel motor control."""
    try:
        # Initialize PortHandler and PacketHandler
        port_handler, packet_handler = initialize_port()
        
        # Set operating mode to Position Control Mode
        set_operating_mode(packet_handler, port_handler, POSITION_MODE)
        
        # Enable Dynamixel Torque
        enable_torque(packet_handler, port_handler)
        
        # Set the goal position to 180 degrees
        goal_position = int(180 / 360.0 * 4095)  # Convert 180 degrees to the corresponding value (assuming 12-bit resolution)
        set_goal_position(packet_handler, port_handler, goal_position)
        
        # Allow time for the motor to reach the position
        time.sleep(3)
        
        # Read and print the current position
        dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, DXL_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print("Failed to read position: %s" % packet_handler.getTxRxResult(dxl_comm_result))
        else:
            print("Current position: ", dxl_present_position)
    
    except Exception as e:
        print(str(e))
    finally:
        # Disable torque and close port
        disable_torque(packet_handler, port_handler)
        port_handler.closePort()

if __name__ == "__main__":
    main()
