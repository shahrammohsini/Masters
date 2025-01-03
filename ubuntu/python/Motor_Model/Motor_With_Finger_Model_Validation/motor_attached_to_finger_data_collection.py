import os
import dynamixel_sdk as dxl
import time
import csv
from scipy import signal
import numpy as np

# Constants
PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = 'COM10'
ADDR_PRO_GOAL_PWM = 100
ADDR_PRO_TORQUE_ENABLE = 64
ADDR_PRESENT_VELOCITY = 128
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
MAX_VOLTAGE = 12.2
MAX_PWM = 885

# PRBS Parameters
degree = 7
sequence_length = 2 ** degree - 1
repeat_factor = 1

def generate_prbs_signal(degree, repeat_factor, max_voltage, max_pwm):
    prbs = signal.max_len_seq(degree)[0] * 2 - 1
    prbs_repeated = np.tile(prbs, repeat_factor)
    scaled_voltage = (prbs_repeated + 1) / 2 * max_voltage
    scaled_pwm = np.round(scaled_voltage / max_voltage * max_pwm).astype(int)
    return scaled_voltage, scaled_pwm

def generate_step_input(total_time, dt, step_magnitude, max_pwm, max_voltage):
    num_steps = int(total_time / dt)
    pwm_value = int((step_magnitude / max_voltage) * max_pwm)
    pwms = np.full(num_steps, pwm_value, dtype=int)
    voltages = np.full(num_steps, step_magnitude)
    times = np.arange(0, total_time, dt)
    print("pwms: ", pwms)
    return times, voltages, pwms

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

def read_velocity(packet_handler, port_handler, dxl_id, addr_present_velocity):
    dxl_present_velocity, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, dxl_id, addr_present_velocity)
    if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error != 0:
        print("Failed to read velocity")
        return None
    
    # Convert the velocity reading if necessary (depends on Dynamixel model and firmware)
    if dxl_present_velocity > 2147483647:  # 2147483647 is 2^31 - 1, half of the range of 32-bit signed integer
        dxl_present_velocity -= 4294967296  # Subtract 2^32 to get the correct negative value
    # print("current velocity: ", dxl_present_velocity)
    return dxl_present_velocity

def collect_data(dt, packet_handler, port_handler, scaled_voltage, scaled_pwm, addr_pro_goal_pwm, addr_present_velocity):
    data = []
    try:
        for voltage, pwm in zip(scaled_voltage, scaled_pwm):
            if not set_pwm(packet_handler, port_handler, DXL_ID, addr_pro_goal_pwm, pwm):
                continue
            dxl_present_velocity = read_velocity(packet_handler, port_handler, DXL_ID, addr_present_velocity)
            if dxl_present_velocity is None:
                continue
            
            dxl_present_velocity = dxl_present_velocity*(360*0.229/(60))# convert velocity from 0.229[rev/min] to deg/sec


            data.append([voltage, dxl_present_velocity])
            print(f"Voltage: {voltage} Velocity: {dxl_present_velocity}")
            time.sleep(dt)
    finally:
        return data

def save_data_to_csv(data, filename):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Voltage", "Velocity"])
        writer.writerows(data)
    print(f"Data collection complete. Data saved to '{filename}'.")

def main():
    total_time = 1  # Total time for the step input
    dt = 0.01  # Time step
    step_magnitude = MAX_VOLTAGE  # Step magnitude


    # scaled_voltage, scaled_pwm = generate_prbs_signal(degree, repeat_factor, MAX_VOLTAGE, MAX_PWM)
    # times, voltages, pwms = generate_step_input(total_time, dt, step_magnitude, MAX_PWM, MAX_VOLTAGE)

    times, voltages, pwms = generate_sinusoidal_input(total_time, dt, amplitude = 12, frequency = 3.5, max_pwm=MAX_PWM, max_voltage=MAX_VOLTAGE)
    port_handler, packet_handler = initialize_port(DEVICENAME, BAUDRATE)

    
    try:
        enable_torque(packet_handler, port_handler, DXL_ID, ADDR_PRO_TORQUE_ENABLE)
        data = collect_data(dt, packet_handler, port_handler, voltages, pwms, ADDR_PRO_GOAL_PWM, ADDR_PRESENT_VELOCITY)
    finally:
        port_handler.closePort()
        save_data_to_csv(data, './Motor_Model/Motor_With_Finger_Model_Validation/motor_data_v2.csv')

if __name__ == "__main__":
    main()
