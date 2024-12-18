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
DXL_ID = 1  # ID of the motor
MAX_VOLTAGE = 12
MAX_PWM = 885
step_magnitude = 5  # Volts

# Initialize the port handler and packet handler
port_handler = PortHandler(DEVICENAME)
packet_handler = PacketHandler(PROTOCOL_VERSION)
group_bulk_write = GroupBulkWrite(port_handler, packet_handler)
group_bulk_read = GroupBulkRead(port_handler, packet_handler)

# Generate step input
def generate_step_input(total_time, dt, step_magnitude, max_pwm, max_voltage):
    num_steps = int(total_time / dt)
    pwm_value = int((step_magnitude / max_voltage) * max_pwm)
    pwms = np.full(num_steps, pwm_value, dtype=int)
    zeros_prepend = np.zeros(5, dtype=int)  # Create zeros array for prepending
    pwms = np.concatenate([zeros_prepend, pwms])  # Add zeros for smooth transition
    voltages = np.concatenate([zeros_prepend, np.full(num_steps, step_magnitude)])
    times = np.arange(0, total_time + 5 * dt, dt)
    return times, voltages, pwms

# Initialize Dynamixel
def initialize_dynamixel():
    if not port_handler.openPort():
        raise IOError("Failed to open the port")
    if not port_handler.setBaudRate(BAUDRATE):
        raise IOError("Failed to set baudrate")
    print("Port opened and baudrate set.")

# Enable torque
def enable_torque(dxl_id):
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError(f"Torque enable failed: {packet_handler.getTxRxResult(dxl_comm_result)}")
    if dxl_error != 0:
        raise RuntimeError(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")

# Disable torque
def disable_torque(dxl_id):
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to disable torque: {packet_handler.getTxRxResult(dxl_comm_result)}")
    if dxl_error != 0:
        print(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")

# Set operating mode
def set_operating_mode(dxl_id, mode):
    disable_torque(dxl_id)
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_OPERATING_MODE, mode)
    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError(f"Failed to set operating mode: {packet_handler.getTxRxResult(dxl_comm_result)}")
    if dxl_error != 0:
        raise RuntimeError(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")

# Set PWM
def set_pwm(dxl_id, pwm_value):
    if abs(pwm_value) > MAX_PWM:
        raise ValueError(f"PWM value must be between {-MAX_PWM} and {MAX_PWM}")
    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, dxl_id, ADDR_PRO_GOAL_PWM, pwm_value)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to set PWM: {packet_handler.getTxRxResult(dxl_comm_result)}")
    if dxl_error != 0:
        print(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")

# Read position
def read_position(dxl_id):
    dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, dxl_id, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to read position: {packet_handler.getTxRxResult(dxl_comm_result)}")
        return None
    if dxl_error != 0:
        print(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")
        return None

    # Convert raw position to signed 32-bit value if necessary
    if dxl_present_position > 2147483647:  # Handle 32-bit overflow
        dxl_present_position -= 4294967296

    # Convert raw position to degrees (optional, depending on your motor's resolution)
    dxl_present_position = dxl_present_position * 0.088  # Example conversion factor
    return dxl_present_position

# Collect data from Arduino
def collect_data(ser, data_queue, stop_event):
    start_time = time.time()
    while not stop_event.is_set():
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    real_time = time.time() - start_time
                    data_queue.put([real_time] + line.split(','))
            except Exception as e:
                print(f"Error reading from Arduino: {e}")

# Save data to CSV
def save_data_to_csv(data, filename):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Time', 'Joint_3', 'Joint_2', 'Joint_1'])
        writer.writerows(data)
    print(f"Data saved to {filename}")


# Set goal position
# Set goal position
def set_goal_position(dxl_id, position):
    dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, dxl_id, ADDR_PRO_GOAL_POSITION, position)
    if dxl_comm_result != COMM_SUCCESS:
        raise RuntimeError(f"Failed to set goal position: {packet_handler.getTxRxResult(dxl_comm_result)}")
    if dxl_error != 0:
        raise RuntimeError(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")


# Main function
def main():
    try:
        initialize_dynamixel()

        # Setup Dynamixel in Position Control Mode
        set_operating_mode(DXL_ID, POSITION_MODE)
        enable_torque(DXL_ID)

        # Move to 180 degrees (starting position)
        starting_point = 0
        goal_position = int(starting_point / 360.0 * 4095)  # Convert to 12-bit value
        set_goal_position(DXL_ID, goal_position)
        time.sleep(2)  # Allow time to reach position

        # Switch to PWM Mode
        set_operating_mode(DXL_ID, PWM_MODE)
        enable_torque(DXL_ID)

        # Prepare step input
        total_time = 0.3
        dt = 0.01
        times, voltages, pwms = generate_step_input(total_time, dt, step_magnitude, MAX_PWM, MAX_VOLTAGE)

        # Initialize Arduino serial communication
        ser = serial.Serial('/dev/ttyACM0', 9600)
        time.sleep(2)
        ser.write(b'start')

        # Start Arduino data collection thread
        data_queue = queue.Queue()
        stop_event = threading.Event()
        data_thread = threading.Thread(target=collect_data, args=(ser, data_queue, stop_event))
        data_thread.start()

        # Main control loop
        for pwm in pwms:
            set_pwm(DXL_ID, pwm)

            # Read motor position
            position = read_position(DXL_ID)
            if position is not None:
                print(f"PWM: {pwm}, Position: {position:.2f} degrees")
            time.sleep(dt)

        # Stop motor
        print("Stopping motor")
        set_pwm(DXL_ID, 0)

        # Stop data collection
        stop_event.set()
        data_thread.join()

        # Save collected data
        data = []
        while not data_queue.empty():
            data.append(data_queue.get())
        save_data_to_csv(data, 'Finger_Multi_Model_Approach/Data/Finger_Pos_Data.csv')

    except Exception as e:
        print(e)
    finally:
        disable_torque(DXL_ID)
        port_handler.closePort()

if __name__ == "__main__":
    main()
