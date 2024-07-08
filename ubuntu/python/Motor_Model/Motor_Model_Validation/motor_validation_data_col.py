import os
import dynamixel_sdk as dxl
import time
import csv
from scipy import signal
import numpy as np

# Protocol version
PROTOCOL_VERSION = 2.0

# Default settings
DXL_ID = 1  # Dynamixel ID
BAUDRATE = 57600  # Dynamixel default baudrate
DEVICENAME = 'COM10'  # Adjust your COM port here
ADDR_PRO_GOAL_PWM = 100  # Address of Goal PWM (depends on your motor model)
ADDR_PRO_TORQUE_ENABLE = 64  # Address of Torque Enable
ADDR_PRESENT_VELOCITY = 128  # Address of Present Velocity (Check your motor model manual)
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
MAX_VOLTAGE = 12.2  # Maximum voltage at maximum PWM
MAX_PWM = 885  # Maximum PWM value



def correct_negative_velocity(velocity):
    """ Convert 32-bit unsigned integer to signed integer if necessary """
    if velocity > 2147483647:  # This is 2^31 - 1, the max value for a 32-bit signed integer
        velocity -= 4294967296  # This is 2^32, subtracting this converts to the correct negative value
    return velocity



# Time parameters
t_end = 50
dt = 0.1
t = np.arange(0, t_end, dt)

# Generate mixed signal for Voltage
# Step signal
step_signal = np.ones_like(t) * MAX_VOLTAGE * (t >= 5) * (t < 15) - MAX_VOLTAGE * (t >= 15) * (t < 25)

# Sinusoidal signal
freq = 0.5  # frequency in Hz
sin_signal = MAX_VOLTAGE * np.sin(2 * np.pi * freq * t) * (t >= 25) * (t < 40)

# Generate PRBS signal for the last segment of the time period
degree = 7  # Degree of the shift register for PRBS
prbs_length = int((t_end - 40) / dt)  # Length of PRBS sequence needed
prbs_signal_raw = signal.max_len_seq(degree, length=prbs_length)[0]  # Generate PRBS
prbs_signal = (prbs_signal_raw * 2 - 1) * MAX_VOLTAGE  # Scale PRBS to voltage range
# Extend the PRBS signal to align with the main time array
full_prbs_signal = np.zeros_like(t)  # Initialize a full-length PRBS signal array with zeros
full_prbs_signal[int(40 / dt):] = prbs_signal  # Place the PRBS signal starting at t=40s

# Composite voltage signal
voltage_input = step_signal + sin_signal + full_prbs_signal  

# Convert voltage to PWM
pwm_input = np.round((voltage_input / MAX_VOLTAGE) * MAX_PWM).astype(int)

# Initialize PortHandler instance
portHandler = dxl.PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != dxl.COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")



# Collect and save data
data = []
try:
    for pwm, velocity in zip(pwm_input, voltage_input):
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, pwm)
        if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error != 0:
            print("Failed to set PWM")
            continue

        # Read present velocity
        dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error != 0:
            print("Failed to read velocity")
            continue

        # Correct potential negative velocity wrap-around due to unsigned to signed conversion
        dxl_present_velocity = correct_negative_velocity(dxl_present_velocity)

        # Save the data
        data.append([velocity, dxl_present_velocity])
        print(f"Velocity: {dxl_present_velocity} at Voltage: {velocity}")

        # Delay for sampling time
        time.sleep(0.1)

finally:
    # Close port
    portHandler.closePort()

    # Save data to CSV file
    with open('model_validation_velocity_motor_data.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Voltage", "Velocity"])
        writer.writerows(data)

    print("Data collection complete. Data saved to 'model_validation_velocity_motor_data.csv'.")