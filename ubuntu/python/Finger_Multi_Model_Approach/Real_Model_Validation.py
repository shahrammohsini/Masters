import serial
import csv
import os
import dynamixel_sdk as dxl
import time
import threading
import queue
from scipy import signal
import numpy as np

# Protocol version and Dynamixel settings
PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = 'COM10'
ADDR_PRO_GOAL_PWM = 100
ADDR_PRO_TORQUE_ENABLE = 64
ADDR_PRO_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11  # Address for the operating mode
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
MAX_VOLTAGE = 12.2
MAX_PWM = 850
step_input = 10  # input voltage
POSITION_MODE = 3
PWM_MODE = 16
step_magnitude = 6

# Function to generate sinusoidal input
def generate_sinusoidal_input(total_time, dt, amplitude, frequency, max_pwm, max_voltage):
    times = np.arange(0, total_time, dt)
    voltages = amplitude * np.sin(2 * np.pi * frequency * times)
    pwms = np.round(voltages / max_voltage * max_pwm).astype(int)
    return times, voltages, pwms

def generate_step_input(total_time, dt, step_magnitude, max_pwm, max_voltage):
    num_steps = int(total_time / dt)
    pwm_value = int((step_magnitude / max_voltage) * max_pwm)
    pwms = np.full(num_steps, pwm_value, dtype=int)
    voltages = np.full(num_steps, step_magnitude)
    times = np.arange(0, total_time, dt)
    # print("pwms: ", pwms)
    return times, voltages, pwms

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
    
# Function to collect data from Arduino
def collect_data(ser, data_queue, stop_event):
    start_time = time.time()
    while not stop_event.is_set():
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    real_time = time.time() - start_time
                    line_data = line.split(',')
                    # line_data.append(real_time)
                    data_queue.put(line_data)
            except Exception as e:
                print(f"Error reading from Arduino: {e}")

def save_data_to_csv(data, filename):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Joint_3', 'Joint_2', 'Joint_1', 'time'])
        writer.writerows(data)
    print(f"Data collection complete. Data saved to '{filename}'.")

def main():
    portHandler, packetHandler = None, None
    ser = None
    stop_event = threading.Event()
    data_queue = queue.Queue()
    data = []

    try:
        # Initialize PortHandler and PacketHandler
        portHandler, packetHandler = initialize_dynamixel(DEVICENAME, BAUDRATE, PROTOCOL_VERSION)

        # Set operating mode to Position Control Mode
        set_operating_mode(packetHandler, portHandler, POSITION_MODE)
        enable_torque(packetHandler, portHandler, DXL_ID, TORQUE_ENABLE)

        # Set the goal position to 180 degrees (starting pos)
        starting_point = 340 #180 #340
        goal_position = int(starting_point / 360.0 * 4095)  # Convert 180 degrees to the corresponding value (assuming 12-bit resolution)
        set_goal_position(packetHandler, portHandler, goal_position)
        time.sleep(1)

        # Switch back to PWM mode
        set_operating_mode(packetHandler, portHandler, PWM_MODE)
        enable_torque(packetHandler, portHandler, DXL_ID, TORQUE_ENABLE)

        # Total time and time step for the step input
        # total_time = 0.4 #for sin input
        total_time = 0.07 #for step input
        dt = 0.01
        # times, voltages, pwms = generate_sinusoidal_input(total_time, dt = 0.01, amplitude=12, frequency=3.3, max_pwm=MAX_PWM, max_voltage=MAX_VOLTAGE)
        times, voltages, pwms = generate_step_input(total_time, dt, step_magnitude, max_pwm = MAX_PWM, max_voltage = (MAX_VOLTAGE))
       
        print("voltages: ", voltages)
        # Open the serial connection
        ser = serial.Serial('COM3', 9600)  # Replace 'COM3' with your Arduino's serial port
        time.sleep(2)  # Give some time to establish the connection
        ser.write(b'start')  # Send start signal to Arduino

        # Start the data collection thread
        data_thread = threading.Thread(target=collect_data, args=(ser, data_queue, stop_event))
        data_thread.start()

        # Main loop to set PWM values
        for pwm in pwms:
            #reverse
            pwm = -pwm
            dxl_present_position = read_position(packetHandler, portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            if dxl_present_position > 350 or dxl_present_position < 120: #if the motor reaches max safe pos stop
                print("Stopping pos: ", dxl_present_position)
                set_pwm(packetHandler, portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, pwm = 0) #stop 
                break
            if not set_pwm(packetHandler, portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, pwm):
                continue
            time.sleep(dt)
        
        set_pwm(packetHandler, portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, pwm = 0) #stop 
        
        
        time.sleep(0.4) # Allow for all the data to pass from arduino before we stop collection.

        # Stop data collection
        stop_event.set()
        data_thread.join()

        # Collect all data from the queue
        while not data_queue.empty():
            data.append(data_queue.get())

    except Exception as e:
        print(e)

    finally:
        if packetHandler and portHandler:
            disable_torque(packetHandler, portHandler, DXL_ID)
            close_port(portHandler)
        if ser:
            ser.close()
        save_data_to_csv(data, 'Finger_Multi_Model_Approach/Data/Finger_Validation_Real_data.csv')

if __name__ == "__main__":
    main()
