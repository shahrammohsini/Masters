import os
import dynamixel_sdk as dxl
import time
import csv
from scipy import signal
import numpy as np
import math

# Constants
PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = 'COM10'
ADDR_PRO_GOAL_PWM = 100
ADDR_PRO_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11  # Address for the operating mode
ADDR_PRO_GOAL_POSITION = 116
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
MAX_VOLTAGE = 12.2
MAX_PWM = 885
POSITION_MODE = 3
PWM_MODE = 16
step_magnitude = 12 #volts


# PRBS Parameters
degree = 7
sequence_length = 2 ** degree - 1
repeat_factor = 1


# finger Paramters
rm = 1.5 # radius of motor pulley
p_phalanx_midF_Length = 3.9 # proximal phalanx length in cm
I_phalanx_midF_Length = 2.5 # intermediate phalanx length
d_phalanx_midF_length = 1.4 # Distal phalanx
r1 = 0.9 # radius of rotation of joint 1 (Metacropophalagenal joint)
r2 = 0.7 # radius of rotation of joint 2 (Proximal joint)
r3 = 0.95 # radius of rotation of join 3 (Distal joint)
max_D_joint_angle = 76 #68
max_P_joint_angle = 83
max_M_joint_angle = 110
max_D_joint_angle_motor = (r3/rm)*(max_D_joint_angle) #72.85
max_P_joint_angle_motor = (r2/rm)*(max_P_joint_angle) #45
max_M_joint_angle_motor = (r1/rm)*(max_M_joint_angle)
# upby = 125

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
    print("pwms: ", pwms)
    return times, voltages, pwms

def generate_sinusoidal_input(total_time, dt, amplitude, frequency, max_pwm, max_voltage):
    times = np.arange(0, total_time, dt)
    voltages = amplitude * np.sin(2 * np.pi * frequency * times)
    pwms = np.round(voltages / max_voltage * max_pwm).astype(int)
    print("times: ", times)
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


def set_pwm(packet_handler, port_handler, dxl_id, addr_pro_goal_pwm, pwm):
    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, dxl_id, addr_pro_goal_pwm, pwm)
    if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error != 0:
        print("Failed to set PWM")
        return False
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

def collect_data(dt, packet_handler, port_handler, scaled_voltage, scaled_pwm, addr_pro_goal_pwm, addr_present_position):
    data = []
    try:
        for voltage, pwm in zip(scaled_voltage, scaled_pwm):
            if not set_pwm(packet_handler, port_handler, DXL_ID, addr_pro_goal_pwm, pwm):
                continue
            dxl_present_position = read_position(packet_handler, port_handler, DXL_ID, addr_present_position)
            if dxl_present_position is None:
                continue
            
            theta_M_joint, theta_P_joint, theta_D_joint = motor_pos_to_finger_pos(theta_m=math.radians(dxl_present_position - 180))
            
            data.append([voltage, dxl_present_position, theta_M_joint, theta_P_joint, theta_D_joint])
            print(f"Voltage: {voltage} Motor Position: {dxl_present_position - 180}, theta_M_joint: {theta_M_joint}, theta_P_joint: {theta_P_joint}, theta_D_joint: {theta_D_joint}")
            #stop the motor before finger reaches max pos
            if (theta_M_joint >= 90 ):
                set_pwm(packet_handler, port_handler, DXL_ID, addr_pro_goal_pwm, pwm = 0) #stop the motor
                print("max theta d reached")
                break
            time.sleep(dt)

        set_pwm(packet_handler, port_handler, DXL_ID, addr_pro_goal_pwm, pwm = 0) #stop the motor
        
    finally:
        return data

def save_data_to_csv(data, filename):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Voltage", "theta_joint"])
        writer.writerows(data)
    print(f"Data collection complete. Data saved to '{filename}'.")

def motor_pos_to_finger_pos(theta_m):
     # radius of rotation for the joints (belt pulley system)
    # joint 1
    # if joint 2 and 3 are at max move joint 1
    print(math.degrees(theta_m))
    if (theta_m <= math.radians(max_D_joint_angle_motor + max_P_joint_angle_motor)):
        theta_M_joint = math.radians(0) 
    else: # Move joint 1
        theta_M_joint= (theta_m - math.radians(max_D_joint_angle_motor + max_P_joint_angle_motor))*(rm/r1) #angle of joint 1 (metacarpophalangeal). Only rotates once joint 2 and 3 have stopped rotating so subtract offset theta_m first
        print("max ", (max_D_joint_angle_motor + max_P_joint_angle_motor))
        
        print("theta_M_joint: ", math.degrees(theta_M_joint))
        #if joint 1 has reached max angle stop joint 1
        if(theta_M_joint) >= math.radians(max_M_joint_angle):
            theta_M_joint = math.radians(max_M_joint_angle)
            # print("theta_M_joint", math.degrees(theta_M_joint))
        #if joint 1 tries to go bellow minimum(0 deg). stop joint 1
        if(theta_M_joint) <= math.radians(0):
            theta_M_joint = math.radians(0)


    #Joint 2
    # if joint 1 is at max move joint 2
    if (theta_m <= math.radians(max_D_joint_angle_motor)):
        theta_P_joint = 0
    else: #Move joint 2
        theta_P_joint = (theta_m - math.radians(max_D_joint_angle_motor))*(rm/r2) # angle of joint 2 (proximal joint) only rotates when joint 3 stops
        # print("theta_p_joint", math.degrees(theta_P_joint))
        #if joint 2 has reached max angle stop joint 2
        if(theta_P_joint) >= math.radians(max_P_joint_angle):
            theta_P_joint = math.radians(max_P_joint_angle)
            # print("theta_p_joint", math.degrees(theta_P_joint))
        #if joint 2 tries to go bellow minimum(0 deg). stop joint 2
        if(theta_P_joint) <= math.radians(0):
            theta_P_joint = math.radians(0)
    
    #Joint 3
    #move joint 3
    theta_D_joint = theta_m*(rm/r3) # angle of joint 3 (distal joint)
    # print("theta_D_joint", math.degrees(theta_D_joint))

    # if joint 3 has reached max angle stop joint 3
    if(theta_D_joint) >= math.radians(max_D_joint_angle):
            theta_D_joint = math.radians(max_D_joint_angle)
            # print("theta_D_joint", math.degrees(theta_D_joint))
    # if joint 3 tries to go below minmum (0 deg). Stop joint 3       
    if(theta_D_joint) <= math.radians(0):
            theta_D_joint = math.radians(0)

    return math.degrees(theta_M_joint), math.degrees(theta_P_joint), math.degrees(theta_D_joint)

def main():
    try:
        # Initialize PortHandler and PacketHandler
        portHandler, packetHandler = initialize_dynamixel(DEVICENAME, BAUDRATE, PROTOCOL_VERSION)

        # Set operating mode to Position Control Mode
        set_operating_mode(packetHandler, portHandler, POSITION_MODE)
        enable_torque(packetHandler, portHandler, DXL_ID, TORQUE_ENABLE)

        # Set the goal position to 180 degrees (starting pos)
        starting_point = 180
        goal_position = int(starting_point / 360.0 * 4095)  # Convert 180 degrees to the corresponding value (assuming 12-bit resolution)
        set_goal_position(packetHandler, portHandler, goal_position)
        time.sleep(5)
        # starting_point = 180 + upby
        # goal_position = int(starting_point / 360.0 * 4095)
        # set_goal_position(packetHandler, portHandler, goal_position)
        # time.sleep(0.5)
        # dxl_present_position = read_position(packetHandler, portHandler, DXL_ID, ADDR_PRESENT_POSITION)

        # theta_M_joint, theta_P_joint, theta_D_joint = motor_pos_to_finger_pos(theta_m=math.radians(dxl_present_position - 180))
            
        # print(f"Voltage: {1} Motor Position: {dxl_present_position - 180}, theta_M_joint: {theta_M_joint}, theta_P_joint: {theta_P_joint}, theta_D_joint: {theta_D_joint}")
        # time.sleep(50)


        # Switch back to PWM mode
        set_operating_mode(packetHandler, portHandler, PWM_MODE)
        enable_torque(packetHandler, portHandler, DXL_ID, TORQUE_ENABLE)

        # Total time and time step for the step input
        total_time = 3
        dt = 0.01


        # times, voltages, pwms = generate_sinusoidal_input(total_time, dt, amplitude = 12, frequency = 3.8, max_pwm=MAX_PWM, max_voltage=MAX_VOLTAGE)
        times, voltages, pwms = generate_step_input(total_time, dt, step_magnitude, max_pwm = MAX_PWM, max_voltage = MAX_VOLTAGE)
        data = collect_data(dt, packetHandler, portHandler, voltages, pwms, ADDR_PRO_GOAL_PWM, ADDR_PRESENT_POSITION)


    except Exception as e:
        print(e)

    finally:
        portHandler.closePort()
        data = np.array(data) #convert to numpy array to do more advanced operations easily
        mjoint_Data = data[:,[0,2]]
        save_data_to_csv(mjoint_Data, './Finger_Multi_Model_Approach/Data/Finger_Pos_Data_MJoint.csv')
        pjoint_Data = data[:,[0,3]]
        save_data_to_csv(pjoint_Data, './Finger_Multi_Model_Approach/Data/Finger_Pos_Data_PJoint.csv')
        djoint_Data = data[:,[0,4]]
        save_data_to_csv(djoint_Data, './Finger_Multi_Model_Approach/Data/Finger_Pos_Data_DJoint.csv')


if __name__ == "__main__":
    main()
