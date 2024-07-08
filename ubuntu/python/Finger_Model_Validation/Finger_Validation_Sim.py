import matplotlib.pyplot as plt
import asyncio
import time
import csv
import numpy as np
from vpython import *
import math
import matplotlib.pyplot as plt


MAX_VOLTAGE = 12.2
MAX_PWM = 850
#Paramters
rm = 1.5 # radius of motor pulley
p_phalanx_midF_Length = 3.9 # proximal phalanx length in cm
I_phalanx_midF_Length = 2.5 # intermediate phalanx length
d_phalanx_midF_length = 1.4 # Distal phalanx
r1 = 3.3 # radius of rotation of joint 1 (Metacropophalagenal joint)
r2 = 3.3 # radius of rotation of joint 2 (Proximal joint)
r3 = d_phalanx_midF_length # radius of rotation of join 3 (Distal joint)
max_D_joint_angle = 68 #68
max_P_joint_angle = 99
max_M_joint_angle = 128
max_D_joint_angle_motor = (rm/r3)*(max_D_joint_angle)
max_P_joint_angle_motor = (rm/r2)*(max_P_joint_angle)
max_M_joint_angle_motor = (rm/r1)*(max_M_joint_angle)

dt = 0.01  # should be small to ensure my model is accurate. At max 0.01
# prev_thetaDot_m = 0 # prev velocity is global so it can be updated from anywhere
# prev_theta_m = 0
theta_m = 0
global prev_theta_m


#sim functions
def create_phalanx(position, axis, radius):
    """Creates a single link as a cylinder."""
    link = cylinder(pos=position, axis=axis, radius=radius, color=color.white)
    return link

def create_joint(position, radius):
    """Creates a joint as a sphere."""
    return sphere(pos=position, radius=radius, color=color.gray(0.8))


def finger_pos_update(theta_m):
    # radius of rotation for the joints (belt pulley system)

    # joint 1
    # if joint 2 and 3 are at max move joint 1
    if (theta_m >= math.radians(0) and theta_m <= math.radians(max_D_joint_angle_motor + max_P_joint_angle_motor)):
        theta_M_joint = math.radians(0) 
    else: # Move joint 1
        theta_M_joint= ((theta_m - math.radians(max_D_joint_angle_motor + max_P_joint_angle_motor)))*(r1/rm) #angle of joint 1 (metacarpophalangeal). Only rotates once joint 2 and 3 have stopped rotating so subtract offset theta_m first
        # print("theta_M_joint: ", math.degrees(theta_M_joint))
        #if joint 1 has reached max angle stop joint 1
        if(theta_M_joint) >= math.radians(max_M_joint_angle):
            theta_M_joint = math.radians(max_M_joint_angle)
            # print("theta_M_joint", math.degrees(theta_M_joint))
        #if joint 1 tries to go bellow minimum(0 deg). stop joint 1
        if(theta_M_joint) <= math.radians(0):
            theta_M_joint = math.radians(0)


    #Joint 2
    # if joint 1 is at max move joint 2
    if (theta_m >= math.radians(0) and theta_m <= math.radians(max_D_joint_angle_motor)):
        theta_P_joint = 0
    else: #Move joint 2
        theta_P_joint = ((theta_m - math.radians(max_D_joint_angle_motor)))*(r2/rm) # angle of joint 2 (proximal joint) only rotates when joint 3 stops
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
    theta_D_joint = theta_m*(r3/rm) # angle of joint 3 (distal joint)
    # print("theta_D_joint", math.degrees(theta_D_joint))

    # if joint 3 has reached max angle stop joint 3
    if(theta_D_joint) >= math.radians(max_D_joint_angle):
            theta_D_joint = math.radians(max_D_joint_angle)
            # print("theta_D_joint", math.degrees(theta_D_joint))
    # if joint 3 tries to go below minmum (0 deg). Stop joint 3       
    if(theta_D_joint) <= math.radians(0):
            theta_D_joint = math.radians(0)



    # calculate joint positions
    pos_M_joint_Y = 0
    pos_M_joint_X = 0

    pos_P_joint_Y= p_phalanx_midF_Length*math.cos(theta_M_joint)
    pos_P_joint_X = p_phalanx_midF_Length*math.sin(theta_M_joint)

    Pos_D_joint_Y = p_phalanx_midF_Length*math.cos(theta_M_joint) + I_phalanx_midF_Length*math.cos(theta_M_joint + theta_P_joint)
    Pos_D_joint_X = p_phalanx_midF_Length*math.sin(theta_M_joint) + I_phalanx_midF_Length*math.sin(theta_M_joint + theta_P_joint)

    pos_tip_Y = p_phalanx_midF_Length*math.cos(theta_M_joint) + I_phalanx_midF_Length*math.cos(theta_M_joint + theta_P_joint) + d_phalanx_midF_length*math.cos(theta_M_joint + theta_P_joint + theta_D_joint)
    pos_tip_X = p_phalanx_midF_Length*math.sin(theta_M_joint) + I_phalanx_midF_Length*math.sin(theta_M_joint + theta_P_joint) + d_phalanx_midF_length*math.sin(theta_M_joint + theta_P_joint + theta_D_joint)



    return pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip_X, pos_tip_Y, theta_M_joint, theta_P_joint, theta_D_joint



    
def create_visual_model(pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip_X, pos_tip_Y):
    meta_joint_mid = create_joint(position=vector(0, 0, 0), radius=1) #Joint 1
    p_joint_mid = create_joint(position= vector(pos_P_joint_X, pos_P_joint_Y, 0), radius=0.8) #joint 2

    p_phalanx_mid = create_phalanx(position=meta_joint_mid.pos , axis = p_joint_mid.pos - meta_joint_mid.pos, radius=meta_joint_mid.radius) #Phalanx 1

    D_joint_mid = create_joint(position= vector(Pos_D_joint_X, Pos_D_joint_Y, 0), radius=0.6)


    I_phalanx_mid = create_phalanx(position=p_joint_mid.pos, axis = D_joint_mid.pos - p_joint_mid.pos, radius= p_joint_mid.radius)
    
    finger_tip_mid = sphere(pos= vector(pos_tip_X, pos_tip_Y, 0), radius=0.6, color=color.white)


    D_phalanx_mid = create_phalanx(position= D_joint_mid.pos, axis = finger_tip_mid.pos - D_joint_mid.pos, radius= D_joint_mid.radius)

    return meta_joint_mid, p_joint_mid, p_phalanx_mid, D_joint_mid, I_phalanx_mid, finger_tip_mid, D_phalanx_mid


def update_visual_model(p_joint_mid_pos, D_joint_mid_pos, finger_tip_mid_pos):
    p_joint_mid.pos = p_joint_mid_pos

    p_phalanx_mid.pos =   meta_joint_mid.pos
    p_phalanx_mid.axis = p_joint_mid.pos - meta_joint_mid.pos

    D_joint_mid.pos = D_joint_mid_pos


    I_phalanx_mid.pos = p_joint_mid.pos
    I_phalanx_mid.axis = D_joint_mid.pos - p_joint_mid.pos
    
    finger_tip_mid.pos = finger_tip_mid_pos


    D_phalanx_mid.pos = D_joint_mid.pos
    D_phalanx_mid.axis = finger_tip_mid.pos - D_joint_mid.pos


def plot_joint_angles(motor_angles, theta_M_joints, theta_P_joints, theta_D_joints):
    # Plotting the angles
    plt.figure(figsize=(10, 6))
    plt.plot(motor_angles, theta_M_joints, label='Metacarpophalangeal Joint (Theta_M)')
    plt.plot(motor_angles, theta_P_joints, label='Proximal Joint (Theta_P)')
    plt.plot(motor_angles, theta_D_joints, label='Distal Joint (Theta_D)')
    plt.xlabel('Motor Angle (degrees)')
    plt.ylabel('Joint Angle (degrees)')
    plt.title('Joint Angles as a Function of Motor Angle')
    plt.legend()
    plt.grid(True)
    plt.show()


def plot_joint_pos(motor_angles, pos_P_joint_Xs, Pos_D_joint_Xs, pos_tip_Xs, pos_P_joint_Ys, Pos_D_joint_Ys, pos_tip_Ys):
    # Plotting the positions
    plt.figure(figsize=(10, 6))
    plt.subplot(1, 2, 1)
    plt.plot(motor_angles, pos_P_joint_Xs, label='Proximal Joint X')
    plt.plot(motor_angles, Pos_D_joint_Xs, label='Distal Joint X')
    plt.plot(motor_angles, pos_tip_Xs, label='Tip X')
    plt.xlabel('Motor Angle (degrees)')
    plt.ylabel('Position X (cm)')
    plt.title('X Positions of Joints as a Function of Motor Angle')
    plt.legend()
    plt.grid(True)

    plt.subplot(1, 2, 2)
    plt.plot(motor_angles, pos_P_joint_Ys, label='Proximal Joint Y')
    plt.plot(motor_angles, Pos_D_joint_Ys, label='Distal Joint Y')
    plt.plot(motor_angles, pos_tip_Ys, label='Tip Y')
    plt.xlabel('Motor Angle (degrees)')
    plt.ylabel('Position Y (cm)')
    plt.title('Y Positions of Joints as a Function of Motor Angle')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()
 
# def get_thetaDot_m(voltage):
#     global prev_thetaDot_m  # Declare that we use the global variable
#     # print("prev_thetaDot_m: ", prev_thetaDot_m)
#     # thetaDot_m = (prev_thetaDot_m + dt*(-22.37*prev_thetaDot_m + 506.7*voltage))   #thetaDot_m in 0.229(rev/min)
#     thetaDot_m = (prev_thetaDot_m + dt*(-169.8*prev_thetaDot_m + 4657*voltage))   #thetaDot_m in deg/sec
#     # -169.8*previous_velocity + 4657*voltage
#     # print("thetaDot_m: ", thetaDot_m) 
#     # thetaDot_m = thetaDot_m * ((360*0.229 / (60)))  # convert to deg/sec
#     # print("voltage: ", voltage)
#     # print("deg/sec: ", thetaDot_m)


# def get_theta_m(voltage, dt):
#     global prev_theta_m # Declare that we use the global variable
  
#     theta_m = prev_theta_m + dt*(-0.4273*prev_theta_m + 127.8*voltage)
#     # print("theta_m: ", theta_m)
#     prev_theta_m = theta_m
#     return theta_m


def calculate_theta_m(prev_theta_m, voltage, dt):
    # print("prev: ", prev_theta_m)
    
    # return previous_velocity + dt*(-169.8*previous_velocity + 4657*voltage)
    # theta_m = (prev_theta_m + dt*(-0.4273*prev_theta_m + 127.8*voltage))
    theta_m = (prev_theta_m + dt*(-0.40*prev_theta_m + 105*voltage))
    print("theta_m: ", theta_m)
    # prev_theta_m = theta_m
    return theta_m


def create_sine_wave(frequency, total_time, dt, max_voltage, max_pwm):
    """Create a sine wave for the specified parameters."""
    omega = 2 * np.pi * frequency  # Angular frequency
    # print("Angular frequency (omega):", omega)
    times = np.arange(0, total_time, dt)
    amplitude = max_voltage / 2
    voltages = amplitude * np.sin(omega * times)   # Shift the sine wave up
    # print("Voltages:", voltages)
    pwms = np.round((voltages / max_voltage * max_pwm)).astype(int)  # Scale and shift to ensure PWM is always positive
    # print("PWM values:", pwms)
    return times, pwms


def generate_sinusoidal_input(total_time, dt, amplitude, frequency, max_pwm, max_voltage):
    times = np.arange(0, total_time, dt)
    voltages = amplitude * np.sin(2 * np.pi * frequency * times)
    pwms = np.round(voltages / max_voltage * max_pwm).astype(int)
    return times, voltages, pwms


def create_step_input(total_time, dt, step_magnitude, max_pwm):
    """
    Create a step input for the specified parameters.

    Parameters:
    - total_time: Total duration of the step input in seconds
    - dt: Time step for the discrete time array
    - step_magnitude: Magnitude of the step in volts
    - max_pwm: Maximum PWM value corresponding to maximum voltage

    Returns:
    - pwms: Array of PWM values corresponding to the step input
    """
    num_steps = int(total_time / dt)
    # Normalize step magnitude to the range of PWM values
    pwm_value = int((step_magnitude / MAX_VOLTAGE) * max_pwm)
    pwms = np.full(num_steps, pwm_value, dtype=int)  # Create an array filled with the step PWM value
    times = np.arange(0, total_time, dt)
    return times, pwms


def save_data(times,theta_ms,theta_M_joints, theta_P_joints, theta_D_joints, pwms):
    data = []
    for time,theta_m,theta_M_joint, theta_P_joint, theta_D_joint, pwm in zip(times,theta_ms,theta_M_joints, theta_P_joints, theta_D_joints, pwms):
        data.append([time,theta_m, theta_D_joint, theta_P_joint, theta_M_joint, pwm])

    with open('./Finger_Model_Validation/Validation_Data/finger_validation_sim_data.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["time","theta_m", "theta_D_joint", "theta_P_joint" ,"theta_M_joint", "pwm"])
        writer.writerows(data)
    print("data saved at ./Finger_Model_Validation/Validation_Data/finger_validation_sim_data.csv")



async def main():
    try:
        

        #save run data in csv file
        # save_data(positions, times, pwms)

        # Create sinusoidal voltage array
        times, voltages, pwms = generate_sinusoidal_input(total_time = 1, dt = 0.01, amplitude = 12, frequency = 2, max_pwm=MAX_PWM, max_voltage=MAX_VOLTAGE)
        sim_times = []
        print("voltages: ", voltages)
        #simulate finger
        prev_theta_m = 0 #250 - 180  #starting ponit
        start_time = time.time()
        for voltage,pwm in zip(voltages,pwms):
            sim_time = time.time() - start_time
            sim_times.append(round(sim_time, 6))
            await asyncio.sleep(0.01)
            time.sleep(0.01)
            # rate(100)
            
            theta_m = math.radians(calculate_theta_m(prev_theta_m,voltage = voltage,dt = dt)) 
            prev_theta_m = math.degrees(theta_m)

            pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip_X, pos_tip_Y, theta_M_joint, theta_P_joint, theta_D_joint = finger_pos_update(theta_m)

            update_visual_model(p_joint_mid_pos = vector(pos_P_joint_X,pos_P_joint_Y, 0), D_joint_mid_pos = vector(Pos_D_joint_X, Pos_D_joint_Y, 0), finger_tip_mid_pos = vector(pos_tip_X, pos_tip_Y, 0))


            # Store the data for plotting
            motor_positions.append(math.degrees(theta_m))
            theta_M_joints.append(math.degrees(theta_M_joint))
            theta_P_joints.append(math.degrees(theta_P_joint))
            theta_D_joints.append(math.degrees(theta_D_joint))

            pos_P_joint_Xs.append(pos_P_joint_X)
            pos_P_joint_Ys.append(pos_P_joint_Y)
            Pos_D_joint_Xs.append(Pos_D_joint_X)
            Pos_D_joint_Ys.append(Pos_D_joint_Y)
            pos_tip_Xs.append(pos_tip_X)
            pos_tip_Ys.append(pos_tip_Y)



        #Plot joint angles
        # plot_joint_angles(motor_angles, theta_M_joints, theta_P_joints, theta_D_joints)

        # plot_joint_pos(motor_angles, pos_P_joint_Xs, Pos_D_joint_Xs, pos_tip_Xs, pos_P_joint_Ys, Pos_D_joint_Ys, pos_tip_Ys)

        #Save data
        save_data(sim_times,motor_positions,theta_M_joints, theta_P_joints, theta_D_joints, pwms)
        

    except Exception as e:
        print(str(e))

if __name__ == "__main__":
    #Initialize lists to store data
    motor_positions = []
    theta_M_joints = []
    theta_P_joints = []
    theta_D_joints = []

    pos_P_joint_Xs = []
    pos_P_joint_Ys = []
    Pos_D_joint_Xs = []
    Pos_D_joint_Ys = []
    pos_tip_Xs = []
    pos_tip_Ys = []


    pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip_X, pos_tip_Y, theta_M_joint, theta_P_joint, theta_D_joint = finger_pos_update(theta_m = 0)
    meta_joint_mid, p_joint_mid, p_phalanx_mid, D_joint_mid, I_phalanx_mid, finger_tip_mid, D_phalanx_mid = create_visual_model(pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip_X, pos_tip_Y)

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program interrupted")
    except Exception as e:
        print(f"Unexpected error: {e}")