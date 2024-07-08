import matplotlib.pyplot as plt
import os
import dynamixel_sdk as dxl
import time
import csv
from scipy import signal
import numpy as np
from scipy.integrate import solve_ivp

# Default settings
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

# Generate PRBS signal for Voltage
degree = 7
sequence_length = 2 ** degree - 1
repeat_factor = 1
prbs = signal.max_len_seq(degree)[0] * 2 - 1
prbs_repeated = np.tile(prbs, repeat_factor)
scaled_voltage = (prbs_repeated + 1) / 2 * MAX_VOLTAGE
scaled_pwm = np.round(scaled_voltage / MAX_VOLTAGE * MAX_PWM).astype(int)

# Initialize communication
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
if not portHandler.openPort():
    print("Failed to open the port")
    quit()
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to change the baudrate")
    quit()
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != dxl.COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    portHandler.closePort()
    quit()

# ODE Solver setup
def ode_system(t, y, v):
    return 506.7 * v - 22.37 * y

# Loop to simulate and collect data
prev_time = time.time()
t = 0
y = [0]  # initial condition
data = []

# Initialization for storing results
t_list = []
sim_velocity = []
actual_velocity = []

try:
    for voltage, pwm in zip(scaled_voltage, scaled_pwm):
        # Set PWM
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, pwm)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print("Failed to set PWM")
            continue

        # Read velocity
        dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print("Failed to read velocity")
            continue

        # Simulation step
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
        t_end = t + dt
        sol = solve_ivp(ode_system, [t, t_end], y, args=(voltage,), dense_output=True)
        # print("y", sol.y)
        y = sol.y[:, -1].tolist()  # Update the current state
        t = t_end  # Update time
        print("y: ", y)
        print("actual: ", dxl_present_velocity)
        # Collect data for plotting
        t_list.append(t)
        sim_velocity.append(y[0])
        actual_velocity.append(dxl_present_velocity)

finally:
    # Close port
    portHandler.closePort()

# Plotting outside the loop
plt.figure(figsize=(10, 5))
plt.plot(t_list, sim_velocity, label='Simulated Velocity')
plt.plot(t_list, actual_velocity, label='Actual Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity')
plt.title('Simulated vs Actual Velocity')
plt.legend()
plt.grid(True)
plt.show()




with open('motor_model_validation_ODE_Data.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["sim_time", "Voltage", "actual_Velocity", "sim_velocity"])
        writer.writerows(data)