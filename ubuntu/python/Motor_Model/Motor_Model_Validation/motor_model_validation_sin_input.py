import matplotlib.pyplot as plt
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
MAX_VOLTAGE = 12.2
MAX_PWM = 885


def correct_negative_velocity(velocity):
    """ Convert 32-bit unsigned integer to signed integer if necessary """
    if velocity > 2147483647:  # This is 2^31 - 1, the max value for a 32-bit signed integer
        velocity -= 4294967296  # This is 2^32, subtracting this converts to the correct negative value
    return velocity


# Initialize timing and velocity variables
t = 0
previous_velocity = 0
current_velocity = 0

# Sine wave parameters
frequency = 3  # Frequency of the sine wave in Hz
omega = 2 * np.pi * frequency  # Angular frequency

# Time settings
total_time = 3  # Total duration of the experiment
dt = 0.01  # Time step for simulation
times = np.arange(0, total_time, dt)

# Create sinusoidal voltage array
voltages = MAX_VOLTAGE * np.sin(omega * times)
# print("voltages:", voltages)
pwms = np.round((voltages / MAX_VOLTAGE * MAX_PWM)).astype(int)  # Scale and shift to ensure PWM is always positive

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
        print("voltage:", voltage)
        print("pwm: ", pwm)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, pwm)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print("Failed to set PWM")
            continue
        
        dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print("Failed to read velocity")
            continue
        

        dxl_present_velocity = correct_negative_velocity(dxl_present_velocity) #correct the negative velocity overflow
        actual_velocity.append(dxl_present_velocity)
        sim_time.append(time_point)
        data.append([time_point, voltage, dxl_present_velocity, current_velocity])
        sim_velocity.append(current_velocity)
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
        current_velocity = previous_velocity + dt*(-22.37*previous_velocity + 506.7 * voltage)
        print("current velocity: ", dxl_present_velocity)
        previous_velocity = current_velocity
        
        time.sleep(0.01)  # Sampling delay
finally:
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, 0) #stop the motor

    portHandler.closePort()

# Plot results
plt.plot(sim_time, sim_velocity, label='Simulated Velocity')
plt.plot(sim_time, actual_velocity, label='Actual Velocity')
plt.title('Simulated vs Actual Velocity')
plt.legend()
plt.grid(True)
plt.show()

# Save data to CSV
with open('motor_model_Validation_sin_input.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Voltage", "Actual Velocity", "Simulated Velocity"])
    writer.writerows(data)
