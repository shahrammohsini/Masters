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
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
MAX_VOLTAGE = 12.2 #12.2
MAX_PWM = 885
step_input = 12 #input voltage

# Initialize timing and velocity variables
t = 0
previous_velocity = 0
current_velocity = 0
# previous_acceleration = 0
# current_acceleration = 0
# previous_voltage = 0
step_time = 0  # Time at which the voltage steps up

# Create time array and step voltage array
total_time = 1  # Total duration of the experiment
dt = 0.01  # Time step for simulation
times = np.arange(0, total_time, dt)
voltages = np.zeros_like(times)
print("voltages: ", voltages)
voltages[times >= step_time] = step_input
print("voltages: ", voltages)
pwms = (voltages / MAX_VOLTAGE * MAX_PWM).astype(int)
print("pwms: ", pwms)

# Initialize PortHandler and PacketHandler
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)

# Open port and set baudrate
if not portHandler.openPort():
    print("Failed to open the port")
    quit()
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to change the baudrate")
    quit()

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != dxl.COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    portHandler.closePort()
    quit()

# Data collection
sim_time = []
sim_velocity = []
actual_velocity = []
data = []

prev_time = time.time()
try:
    for time_point, voltage, pwm in zip(times, voltages, pwms):
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, pwm)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print("Failed to set PWM")
            continue
        
        dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print("Failed to read velocity")
            continue

        # Handle potential overflow in velocity readings (assuming unsigned 32-bit integer)
        if dxl_present_velocity > 4294967295 // 2:
            dxl_present_velocity -= 4294967296
        
        

        #Model calculations
        # voltage_dot = (voltage - previous_voltage)/dt
        # current_velocity = previous_velocity + dt*previous_acceleration
        # current_acceleration = previous_acceleration + dt*(-135.5*previous_acceleration - 10890*previous_velocity - 128.5*voltage_dot + 175800*voltage)
        # current_velocity = previous_velocity + dt*(-37770*previous_velocity + 746300*voltage)
        current_velocity = previous_velocity + dt*(-22.37*previous_velocity + 506.7*voltage)

        previous_velocity = current_velocity
        # previous_voltage = voltage
        # previous_acceleration = current_acceleration


        actual_velocity.append(dxl_present_velocity)
        sim_time.append(time_point)
        data.append([time_point, voltage, dxl_present_velocity, current_velocity])
        sim_velocity.append(current_velocity)
        # current_time = time.time()
        # dt = current_time - prev_time
        # prev_time = current_time


        time.sleep(0.01)  # Sampling delay
finally:
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, 0) # stop the motor
    portHandler.closePort()

# Plot results
plt.plot(sim_time, sim_velocity, label='Simulated Velocity')
plt.plot(sim_time, actual_velocity, label='Actual Velocity')
print("actual velocity: ", actual_velocity)
print("simulated velocity: ", sim_velocity)

plt.title('Simulated vs Actual Velocity')
plt.legend()
plt.grid(True)
plt.show()

# Save data to CSV
with open('motor_model_validation_step_input.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Voltage", "Actual Velocity", "Simulated Velocity"])
    writer.writerows(data)
