import serial
import csv

import os
import dynamixel_sdk as dxl
import time
import csv
from scipy import signal
import numpy as np

PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = 'COM10'
ADDR_PRO_GOAL_PWM = 100
ADDR_PRO_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
MAX_VOLTAGE = 12.2
MAX_PWM = 885



# # Open the CSV file for writing
# with open('./Finger_Model_Validation/Validation_Data/finger_Validation_Real_Data.csv', 'w', newline='') as csvfile:
#     csv_writer = csv.writer(csvfile)
#     csv_writer.writerow(['Joint_3', 'Joint_2', 'Joint_1']) # Write CSV header
    
#     try:
#         while True:
#             if ser.in_waiting > 0:
#                 line = ser.readline().decode('utf-8').strip()
#                 print(line) # Print the line for debugging
#                 csv_writer.writerow(line.split(','))
#     except KeyboardInterrupt:
#         print("Data logging stopped.")
#     finally:
#         ser.close()



def generate_sinusoidal_input(total_time, dt, amplitude, frequency, max_pwm, max_voltage):
    times = np.arange(0, total_time, dt)
    voltages = amplitude * np.sin(2 * np.pi * frequency * times)
    pwms = np.round(voltages / max_voltage * max_pwm).astype(int)
    print("times: ", times)
    return times, voltages, pwms


def initialize_port(devicename, baudrate):
    port_handler = dxl.PortHandler(devicename)
    packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)
    
    if not port_handler.openPort():
        print("Failed to open the port")
        quit()
    if not port_handler.setBaudRate(baudrate):
        print("Failed to change the baudrate")
        quit()
    return port_handler, packet_handler

def enable_torque(packet_handler, port_handler, dxl_id, addr_pro_torque_enable):
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, addr_pro_torque_enable, TORQUE_ENABLE)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print("%s" % packet_handler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packet_handler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel has been successfully connected")

def set_pwm(packet_handler, port_handler, dxl_id, addr_pro_goal_pwm, pwm):
    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, dxl_id, addr_pro_goal_pwm, pwm)
    if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error != 0:
        print("Failed to set PWM")
        return False
    return True

# def read_position(packet_handler, port_handler, dxl_id, addr_present_position):
#     dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, dxl_id, addr_present_position)
#     if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error != 0:
#         print("Failed to read position")
#         return None
    
#     # Convert the position reading if necessary (depends on Dynamixel model and firmware)
#     if dxl_present_position > 2147483647:  # 2147483647 is 2^31 - 1, half of the range of 32-bit signed integer
#         dxl_present_position -= 4294967296  # Subtract 2^32 to get the correct negative value
    
#     return dxl_present_position

def collect_data(dt, packet_handler, port_handler, scaled_voltage, scaled_pwm, addr_pro_goal_pwm, addr_present_position):
    data = []
    # Configure the serial port and baud rate
    ser = serial.Serial('COM3', 9600) # Replace 'COM3' with your Arduino's serial port
    # Give some time to establish the connection
    time.sleep(2)

    try:
        for pwm in scaled_pwm:
            if not set_pwm(packet_handler, port_handler, DXL_ID, addr_pro_goal_pwm, pwm):
                continue
            
            #Collect data from arduino
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                print(line) # Print the line for debugging
                data.append(line.split(','))
            time.sleep(dt)
    finally:
        return data

def save_data_to_csv(data, filename):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Joint_3', 'Joint_2', 'Joint_1'])
        writer.writerows(data)
    print(f"Data collection complete. Data saved to '{filename}'.")

def main():
    total_time = 1  # Total time for the step input
    dt = 0.01  # Time step
    step_magnitude = MAX_VOLTAGE  # Step magnitude


    times, voltages, pwms = generate_sinusoidal_input(total_time, dt, amplitude = 12, frequency = 3.8, max_pwm=MAX_PWM, max_voltage=MAX_VOLTAGE)
    port_handler, packet_handler = initialize_port(DEVICENAME, BAUDRATE)

    
    try:
        enable_torque(packet_handler, port_handler, DXL_ID, ADDR_PRO_TORQUE_ENABLE)
        data = collect_data(dt, packet_handler, port_handler, voltages, pwms, ADDR_PRO_GOAL_PWM, ADDR_PRESENT_POSITION)
    finally:
        port_handler.closePort()
        save_data_to_csv(data, './Finger_Model_Validation/Validation_Data/finger_Validation_Real_Data.csv')

if __name__ == "__main__":
    main()
