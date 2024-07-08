import matplotlib.pyplot as plt
import os
import dynamixel_sdk as dxl
import time
import csv
import numpy as np

# Protocol version and Dynamixel settings
PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = 'COM10'
ADDR_PRO_GOAL_PWM = 100
ADDR_PRO_TORQUE_ENABLE = 64
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
MAX_VOLTAGE = 12.2
MAX_PWM = 885
step_input = 12  # input voltage

# Experiment settings
total_time = 1  # Total duration of the experiment
dt = 0.01  # Time step for simulation
step_time = 0  # Time at which the voltage steps up


def initialize_dynamixel(devicename, baudrate, protocol_version):
    portHandler = dxl.PortHandler(devicename)
    packetHandler = dxl.PacketHandler(protocol_version)
    if not portHandler.openPort():
        raise IOError("Failed to open the port")
    if not portHandler.setBaudRate(baudrate):
        raise IOError("Failed to change the baudrate")
    return portHandler, packetHandler

def enable_torque(packetHandler, portHandler, dxl_id, torque_enable):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, torque_enable)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        raise RuntimeError(f"Torque enable failed: {packetHandler.getTxRxResult(dxl_comm_result)}")

def disable_torque(packetHandler, portHandler, dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)

def close_port(portHandler):
    portHandler.closePort()

def generate_step_input(total_time, dt, step_time, step_input, max_voltage, max_pwm):
    times = np.arange(0, total_time, dt)
    voltages = np.zeros_like(times)
    voltages[times >= step_time] = step_input
    pwms = (voltages / max_voltage * max_pwm).astype(int)
    return times, voltages, pwms


def generate_sinusoidal_input(total_time, dt, amplitude, frequency, max_pwm, max_voltage):
    times = np.arange(0, total_time, dt)
    voltages = amplitude * np.sin(2 * np.pi * frequency * times)
    pwms = np.round(voltages / max_voltage * max_pwm).astype(int)
    print("pwms: ", pwms)
    return times, voltages, pwms


def read_velocity(packetHandler, portHandler, dxl_id):
    dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_VELOCITY)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        raise RuntimeError("Failed to read velocity")
    if dxl_present_velocity > 4294967295 // 2:
        dxl_present_velocity -= 4294967296


    dxl_present_velocity = dxl_present_velocity*(360*0.229/(60))# convert velocity from 0.229[rev/min] to deg/sec
    # print(dxl_present_velocity)

    return dxl_present_velocity

def set_pwm(packetHandler, portHandler, dxl_id, pwm):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_PRO_GOAL_PWM, pwm)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        raise RuntimeError("Failed to set PWM")

def calculate_velocity(previous_velocity, voltage, dt):
    # return previous_velocity + dt * (-22.37 * previous_velocity + 506.7 * voltage)
    # return previous_velocity + dt*(-37.770*previous_velocity + 746.300*voltage)
    # return previous_velocity + dt*(-38.770*previous_velocity + 800.300*voltage)
    # return previous_velocity + dt*(-491*previous_velocity + 13500*voltage)
    return previous_velocity + dt*(-169.8*previous_velocity + 4657*voltage)


    # return previous_velocity + dt*(-36.49*previous_velocity + 343*voltage)



def run_experiment(times, voltages, pwms, packetHandler, portHandler, dxl_id):
    sim_time = []
    sim_velocity = []
    actual_velocity = []
    data = []

    previous_velocity = 0

    try:
        for time_point, voltage, pwm in zip(times, voltages, pwms):
            set_pwm(packetHandler, portHandler, dxl_id, pwm)
            dxl_present_velocity = read_velocity(packetHandler, portHandler, dxl_id)

            current_velocity = calculate_velocity(previous_velocity, voltage, dt)
            previous_velocity = current_velocity

            actual_velocity.append(dxl_present_velocity)
            sim_time.append(time_point)
            sim_velocity.append(current_velocity)
            data.append([time_point, voltage, dxl_present_velocity, current_velocity])

            time.sleep(dt)  # Sampling delay
    finally:
        set_pwm(packetHandler, portHandler, dxl_id, 0)  # stop the motor
    return sim_time, sim_velocity, actual_velocity, data

def plot_results(sim_time, sim_velocity, actual_velocity):
    plt.plot(sim_time, sim_velocity, label='Simulated Velocity')
    plt.plot(sim_time, actual_velocity, label='Actual Velocity')
    plt.title('Simulated vs Actual Velocity')
    plt.legend()
    plt.grid(True)
    plt.show()

def save_to_csv(data, filename='motor_model_validation_step_input.csv'):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Time", "Voltage", "Actual Velocity", "Simulated Velocity"])
        writer.writerows(data)

def main():
    portHandler, packetHandler = None, None
    try:
        portHandler, packetHandler = initialize_dynamixel(DEVICENAME, BAUDRATE, PROTOCOL_VERSION)
        enable_torque(packetHandler, portHandler, DXL_ID, TORQUE_ENABLE)

        # times, voltages, pwms = generate_step_input(total_time, dt, step_time, step_input, MAX_VOLTAGE, MAX_PWM)
        times, voltages, pwms = generate_sinusoidal_input(total_time, dt, amplitude = 12, frequency = 3.5, max_pwm=MAX_PWM, max_voltage=MAX_VOLTAGE)
        
        sim_time, sim_velocity, actual_velocity, data = run_experiment(times, voltages, pwms, packetHandler, portHandler, DXL_ID)

        

        plot_results(sim_time, sim_velocity, actual_velocity)
        save_to_csv(data)
    except Exception as e:
        print(e)
    finally:
        if packetHandler and portHandler:
            disable_torque(packetHandler, portHandler, DXL_ID)
            close_port(portHandler)

if __name__ == "__main__":
    main()
