import serial
import csv
import time
import threading
import queue
import numpy as np
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Protocol version and Dynamixel settings
PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'  # Adjust for your system
ADDR_PRO_GOAL_PWM = 100
ADDR_PRO_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRO_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
PWM_MODE = 16
POSITION_MODE = 3
DXL_ID = 2  # Motor ID
MAX_VOLTAGE = 12
MAX_PWM = 885
step_magnitude = -5  # Volts. -volts means cw rotation

# Initialize Bulk Write
port_handler = PortHandler(DEVICENAME)
packet_handler = PacketHandler(PROTOCOL_VERSION)
group_bulk_write = GroupBulkWrite(port_handler, packet_handler)

# Function to generate step input
def generate_step_input(total_time, dt, step_magnitude, max_pwm, max_voltage):
    num_steps = int(total_time / dt)
    pwm_value = int((step_magnitude / max_voltage) * max_pwm)
    pwms = np.full(num_steps, pwm_value, dtype=int)
    voltages = np.full(num_steps, step_magnitude)
    times = np.arange(0, total_time, dt)
    return times, voltages, pwms

def initialize_dynamixel():
    if not port_handler.openPort():
        raise IOError("Failed to open the port")
    if not port_handler.setBaudRate(BAUDRATE):
        raise IOError("Failed to set baudrate")
    print("Port opened and baudrate set.")

def set_operating_mode(dxl_id, mode):
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_OPERATING_MODE, mode)
    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError(f"Failed to set operating mode: {packet_handler.getTxRxResult(dxl_comm_result)}")
    if dxl_error != 0:
        raise RuntimeError(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")

def enable_torque(dxl_id):
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError(f"Torque enable failed: {packet_handler.getTxRxResult(dxl_comm_result)}")
    if dxl_error != 0:
        raise RuntimeError(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")

def disable_torque(dxl_id):
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to disable torque: {packet_handler.getTxRxResult(dxl_comm_result)}")
    if dxl_error != 0:
        print(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")

def bulk_write_goal_position(dxl_id, position):
    param_goal_position = [
        DXL_LOBYTE(DXL_LOWORD(position)),
        DXL_HIBYTE(DXL_LOWORD(position)),
        DXL_LOBYTE(DXL_HIWORD(position)),
        DXL_HIBYTE(DXL_HIWORD(position)),
    ]
    add_param_result = group_bulk_write.addParam(dxl_id, ADDR_PRO_GOAL_POSITION, 4, param_goal_position)
    if not add_param_result:
        raise RuntimeError(f"Failed to add goal position to bulk write for ID {dxl_id}")
    dxl_comm_result = group_bulk_write.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError(f"Bulk write failed: {packet_handler.getTxRxResult(dxl_comm_result)}")
    group_bulk_write.clearParam()

def bulk_write_pwm(dxl_id, pwm_value):
    param_goal_pwm = [
        DXL_LOBYTE(pwm_value & 0xFFFF),
        DXL_HIBYTE(pwm_value & 0xFFFF),
    ]
    add_param_result = group_bulk_write.addParam(dxl_id, ADDR_PRO_GOAL_PWM, 2, param_goal_pwm)
    if not add_param_result:
        raise RuntimeError(f"Failed to add PWM to bulk write for ID {dxl_id}")
    dxl_comm_result = group_bulk_write.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError(f"Bulk write failed: {packet_handler.getTxRxResult(dxl_comm_result)}")
    group_bulk_write.clearParam()

def read_current_position(dxl_id):
    """
    Reads the current position of the motor in degrees.
    - dxl_id: ID of the motor.
    Returns the current position in degrees.
    """
    dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, dxl_id, ADDR_PRESENT_POSITION)
    
    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError(f"Failed to read position: {packet_handler.getTxRxResult(dxl_comm_result)}")
    if dxl_error != 0:
        raise RuntimeError(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")

    # Convert position from raw units to degrees
    if dxl_present_position > 2147483647:  # Handle signed 32-bit overflow
        dxl_present_position -= 4294967296
    
    current_position_degrees = dxl_present_position * 360.0 / 4095.0
    return current_position_degrees

def convert_raw_pos_to_deg(position):
    position_in_deg = position * 360.0 / 4095.0 #convert to degrees
    return position_in_deg

def convert_deg_pos_to_raw(position):
    raw_position = position * 4095.0 / 360.0 #convert deg to raw pos
    return raw_position



def move_to_position_with_pwm(target_position, dxl_id, max_pwm, tolerance=1.0):
    """
    Moves the motor to the target position using PWM mode until it reaches the desired position.
    - target_position: Target position in degrees.
    - dxl_id: ID of the motor.
    - max_pwm: PWM value to apply (absolute).
    - tolerance: Position tolerance in degrees.
    """
    
    print(f"Target Position: {target_position} degrees")

    while True:
        # Read the current position
        current_position = read_current_position(dxl_id)
        current_position = current_position * 360.0 / 4095.0 #convert to degrees

        print(f"Target Position: {target_position} degrees")

        print(f"Current Position: {current_position:.2f} degrees")

        # Calculate the position error
        error = target_position - current_position

        # Check if the motor is within the tolerance
        if abs(error) <= tolerance:
            print(f"Reached Target Position: {current_position:.2f} degrees")
            bulk_write_pwm(dxl_id, 0)  # Stop the motor
            break

        # Determine the PWM value to apply (negative for CCW movement)
        if(error > 0):
            pwm_value = max_pwm
        else:
            pwm_value = -max_pwm
        bulk_write_pwm(dxl_id, pwm_value)
        print(f"Applied PWM: {pwm_value}")

        time.sleep(0.05)  # Small delay to allow motor movement


def collect_data(ser, data_queue, stop_event):
    start_time = time.time()
    while not stop_event.is_set():
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(f"Raw line: {line}")

                    real_time = time.time() - start_time
                    line_data = line.split(',')
                    # data_queue.put([real_time] + line_data)
                    data_queue.put(line_data)
            except Exception as e:
                print(f"Error reading from Arduino: {e}")

def save_data_to_csv(data, filename):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Joint_3', 'Joint_2', 'Joint_1','time'])
        writer.writerows(data)
    print(f"Data collection complete. Data saved to '{filename}'.")

def main():
    try:
        initialize_dynamixel()

        # # Switch to Position Control Mode and move to starting position
        # print("Switching to Position Control Mode")
        # set_operating_mode(DXL_ID, POSITION_MODE)
        # time.sleep(0.1)
        # enable_torque(DXL_ID)
        # starting_point = 65  # Degrees
        # goal_position = int(starting_point / 360.0 * 4095)
        # print(f"Moving to starting position: {starting_point} degrees ({goal_position})")
        # bulk_write_goal_position(DXL_ID, goal_position)
        # time.sleep(2)




        # Switch to PWM Mode
        print("Switching to PWM Mode")
        disable_torque(DXL_ID)
        set_operating_mode(DXL_ID, PWM_MODE)
        time.sleep(0.2)
        enable_torque(DXL_ID)

        # Move to 65 degrees(starting pos) using PWM mode
        target_position = 32  #desired pos in deg                                                                                                                                                                                                                                                                                                                                                                                                                                                               # Desired position in degrees
        move_to_position_with_pwm(target_position, DXL_ID, max_pwm=300, tolerance=1)

        # Stop motor
        bulk_write_pwm(DXL_ID, 0)
        print("Motor stopped.")

    except Exception as e:
        print(e)
    finally:
        disable_torque(DXL_ID)
        port_handler.closePort()

if __name__ == "__main__":
    main()
