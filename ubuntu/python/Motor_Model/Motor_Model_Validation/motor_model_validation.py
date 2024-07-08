import matplotlib.pyplot as plt
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




t = 0
# dt = 0.01 # If this step size is too large the model becomes very innacurate
voltage = 12
previous_velocity = 0
current_Velocity = 0
MAX_VOLTAGE = 12.2

sim_time = []
sim_velocity = []
actual_velocity = []
data = []



# Generate PRBS signal for Voltage
degree = 7  # Degree of the shift register for PRBS
sequence_length = 2 ** degree - 1  # Length of one sequence
repeat_factor = 1  # Repeat the sequence to cover more time
prbs = signal.max_len_seq(degree)[0] * 2 - 1  # Generates a PRBS signal (-1, 1)
prbs_repeated = np.tile(prbs, repeat_factor)  # Repeat the PRBS sequence
scaled_voltage = (prbs_repeated + 1) / 2 * MAX_VOLTAGE  # Scale to voltage range
scaled_pwm = np.round(scaled_voltage / MAX_VOLTAGE * MAX_PWM).astype(int)  # Convert to PWM


# print(scaled_voltage)
# for voltage in scaled_voltage:
#     sim_time.append(t)
#     velocity.append(current_Velocity)
#     print(current_Velocity)

#     current_Velocity = previous_velocity + dt*(-22.37*previous_velocity + 506.7*voltage)

#     previous_velocity = current_Velocity

#     t = t + dt





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

prev_time = time.time()
try:
    for voltage, pwm in zip(scaled_voltage, scaled_pwm):
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, pwm)
        if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error != 0:
            print("Failed to set PWM")
            continue

        # Read present velocity
        dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error != 0:
            print("Failed to read velocity")
            continue

        # Save the data
        actual_velocity.append(dxl_present_velocity)
        print(f"Velocity: {dxl_present_velocity} at Voltage: {voltage}")



        #sim
        sim_time.append(t)

        data.append([t, voltage, dxl_present_velocity, current_Velocity])

        sim_velocity.append(current_Velocity)
        print(current_Velocity)

        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
        
        current_Velocity = previous_velocity + dt*(-22.37*previous_velocity + 506.7*voltage)

        previous_velocity = current_Velocity

        t = t + dt





        # Delay for sampling time
        time.sleep(0.01)

finally:
    # Close port
    portHandler.closePort()

print("sim_vel:", sim_velocity)
print("actual_vel: ", actual_velocity)
# plt.plot(sim_time, sim_velocity, actual_velocity)
plt.plot(sim_time, sim_velocity, label = 'sim velocity')
plt.plot(sim_time, actual_velocity, label = 'actual velocity')
plt.title('Simulated vs. Actual Velocity')
plt.legend()
plt.grid(True)

plt.show()



with open('motor_model validation_PRBS.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["sim_time", "Voltage", "actual_Velocity", "sim_velocity"])
        writer.writerows(data)