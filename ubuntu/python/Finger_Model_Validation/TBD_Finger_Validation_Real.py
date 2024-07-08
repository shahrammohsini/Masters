import serial
import csv
import os
import dynamixel_sdk as dxl
import time
import csv
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
MAX_PWM = 885
step_input = 12  # input voltage
POSITION_MODE = 3
PWM_MODE = 16

def generate_sinusoidal_input(total_time, dt, amplitude, frequency, max_pwm, max_voltage):
    times = np.arange(0, total_time, dt)
    voltages = amplitude * np.sin(2 * np.pi * frequency * times)
    pwms = np.round(voltages / max_voltage * max_pwm).astype(int)
    return times, voltages, pwms

def initialize_dynamixel(devicename, baudrate, protocol_version):
    portHandler = dxl.PortHandler(devicename)
    packetHandler = dxl.PacketHandler(protocol_version)
    if not portHandler.openPort():
        raise IOError("Failed to open the port")
    if not portHandler.setBaudRate(baudrate):
        raise IOError("Failed to change the baudrate")
    return portHandler, packetHandler

def set_operating_mode(packet_handler, port_handler, mode):
    """Set the operating mode."""
    disable_torque(packet_handler, port_handler, DXL_ID)
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_OPERATING_MODE, mode)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        raise Exception("Failed to set operating mode")
    if dxl_error != 0:
        print("Dynamixel error: %s" % packet_handler.getRxPacketError(dxl_error))

def enable_torque(packetHandler, portHandler, dxl_id, torque_enable):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, torque_enable)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        raise RuntimeError(f"Torque enable failed: {packetHandler.getTxRxResult(dxl_comm_result)}")

def disable_torque(packetHandler, portHandler, dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)

def close_port(portHandler):
    portHandler.closePort()


def set_goal_position(packet_handler, port_handler, position):
    """Set the goal position."""
    dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, DXL_ID, ADDR_PRO_GOAL_POSITION, position)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        raise Exception("Failed to set goal position")
    if dxl_error != 0:
        print("Dynamixel error: %s" % packet_handler.getRxPacketError(dxl_error))

def set_pwm(packet_handler, port_handler, dxl_id, addr_pro_goal_pwm, pwm):
    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, dxl_id, addr_pro_goal_pwm, pwm)
    if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error != 0:
        print("Failed to set PWM: %s, %s" % (packet_handler.getTxRxResult(dxl_comm_result), packet_handler.getRxPacketError(dxl_error)))
        return False
    print(f"Set PWM to {pwm}")
    return True

def collect_data(dt, packet_handler, port_handler, scaled_voltage, scaled_pwm, addr_pro_goal_pwm, addr_present_position):
    data = []
    ser = serial.Serial('COM3', 9600) # Replace 'COM3' with your Arduino's serial port
    time.sleep(2) # Give some time to establish the connection

    # Send start signal to Arduino
    ser.write(b'start')
    start_time = time.time()

    try:
        for pwm in scaled_pwm:
            if not set_pwm(packet_handler, port_handler, DXL_ID, addr_pro_goal_pwm, pwm):
                continue
            real_time = time.time() - start_time
            # Collect data from Arduino
            # while time.time() - start_time < dt:
            time.sleep(0.01)
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if line:  # Check if the line is not empty
                        print("Received from Arduino: ", line)
                        # data.append(line.split(','))
                        line_data = line.split(',')
                        line_data.append(real_time)
                        # print("line_data: ", line_data)
                        data.append(line_data)
                    else:
                        print("Received empty line from Arduino.")
                except Exception as e:
                    print(f"Error reading from Arduino: {e}")
            else:
                print("No data received from Arduino.")
    finally:            
        ser.close()
        return data

def save_data_to_csv(data, filename):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Joint_3', 'Joint_2', 'Joint_1', 'time'])
        writer.writerows(data)
    print(f"Data collection complete. Data saved to '{filename}'.")

def main():
    portHandler, packetHandler = None, None
    try:
        starting_point = 180
        # Initialize PortHandler and PacketHandler
        portHandler, packetHandler = initialize_dynamixel(DEVICENAME, BAUDRATE, PROTOCOL_VERSION)

        # Set operating mode to Position Control Mode
        set_operating_mode(packetHandler, portHandler, POSITION_MODE)

        enable_torque(packetHandler, portHandler, DXL_ID, TORQUE_ENABLE)


        # Set the goal position to 180 degrees. (Starting pos)
        goal_position = int(starting_point / 360.0 * 4095)  # Convert 180 degrees to the corresponding value (assuming 12-bit resolution)
        set_goal_position(packetHandler, portHandler, goal_position)
        time.sleep(1)

        # Switch back to pwm mode
        set_operating_mode(packetHandler, portHandler, PWM_MODE)
        enable_torque(packetHandler, portHandler, DXL_ID, TORQUE_ENABLE)



        total_time = 1  # Total time for the step input
        dt = 0.01  # Time step
        step_magnitude = MAX_VOLTAGE  # Step magnitude

        times, voltages, pwms = generate_sinusoidal_input(total_time = 1, dt = dt, amplitude = 12, frequency = 2, max_pwm=MAX_PWM, max_voltage=MAX_VOLTAGE)

    
        data = collect_data(dt, packetHandler, portHandler, voltages, pwms, ADDR_PRO_GOAL_PWM, ADDR_PRESENT_POSITION)
    
    except Exception as e:
        print(e)

    finally:
        if packetHandler and portHandler:
            disable_torque(packetHandler, portHandler, DXL_ID)
            close_port(portHandler)
            save_data_to_csv(data, './Finger_Model_Validation/Validation_Data/finger_Validation_Real_Data.csv')

if __name__ == "__main__":
    main()
