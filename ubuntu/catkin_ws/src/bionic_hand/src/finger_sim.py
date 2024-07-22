#!/usr/bin/env python3

import matplotlib.pyplot as plt
import asyncio
import time
import csv
import numpy as np
from vpython import *
import math
import matplotlib.pyplot as plt
from collections import deque
import rospy
from bionic_hand.msg import FingerPos
from bionic_hand.msg import ControlCommands
from threading import Thread
MAX_VOLTAGE = 12.2
MAX_PWM = 850
#Paramters
# rm = 3 # radius of motor pulley
p_phalanx_midF_Length = 3.9 # proximal phalanx length in cm
I_phalanx_midF_Length = 2.5 # intermediate phalanx length
d_phalanx_midF_length = 1.4 # Distal phalanx
# r1 = 3.3 # radius of rotation of joint 1 (Metacropophalagenal joint)
# r2 = 3.3 # radius of rotation of joint 2 (Proximal joint)
# r3 = d_phalanx_midF_length # radius of rotation of join 3 (Distal joint)
max_D_joint_angle = 72 #68
max_P_joint_angle = 89
max_M_joint_angle = 67
# max_D_joint_angle_motor = (r3/rm)*(max_D_joint_angle)
# max_P_joint_angle_motor = (r2/rm)*(max_P_joint_angle)
# max_M_joint_angle_motor = (r1/rm)*(max_M_joint_angle)

step_magnitude = 6
dt = 0.03  # should be small to ensure my model is accurate. At max 0.01
theta_m = 0
global prev_theta_m

# Dead time parameters
dead_time = 0.235  # 0.25 seconds
buffer_length = int(dead_time / 0.03)
voltage_buffer = deque([0] * buffer_length, maxlen=buffer_length)


#sim functions
def create_phalanx(position, axis, radius):
    """Creates a single link as a cylinder."""
    link = cylinder(pos=position, axis=axis, radius=radius, color=color.white)
    return link

def create_joint(position, radius):
    """Creates a joint as a sphere."""
    return sphere(pos=position, radius=radius, color=color.gray(0.8))


def finger_pos_update(voltage, prev_theta_D, prev_theta_P, prev_theta_M, time_Step, acumalating_time):
    
    #convert to deg
    prev_theta_D = math.degrees(prev_theta_D)
    prev_theta_P = math.degrees(prev_theta_P)
    prev_theta_M = math.degrees(prev_theta_M)

    theta_D_joint = prev_theta_D
    theta_P_joint = prev_theta_P
    theta_M_joint = prev_theta_M

    #forward motion:
    if voltage >= 0:
        #Joint 3 (D)
        #move joint D
        if theta_D_joint < max_D_joint_angle:
            theta_D_joint = prev_theta_D + time_Step*((-5.712)*prev_theta_D + 104.5*voltage)
            if(theta_D_joint > max_D_joint_angle):
                theta_D_joint = max_D_joint_angle
            theta_P_joint = 0
            theta_M_joint = 0
            # print("moving J_D", theta_D_joint)
        elif theta_D_joint >= max_D_joint_angle and theta_P_joint < max_P_joint_angle: # Move joint P
            # print("Moving J_P", theta_P_joint)
            theta_D_joint = max_D_joint_angle
            # theta_P_joint = prev_theta_P + time_Step*(-(1.165)*prev_theta_P + 53.13*voltage)
            theta_P_joint = prev_theta_P + time_Step*(-(7.19e-12)*prev_theta_P + 76.06*voltage)
            if(theta_P_joint > max_P_joint_angle):
                theta_P_joint = max_P_joint_angle

            theta_M_joint = 0
        elif theta_D_joint >= max_D_joint_angle and theta_P_joint >= max_P_joint_angle and theta_M_joint < max_M_joint_angle: # Move joint M
            # print("Moving J_M,", theta_M_joint)
            theta_D_joint = max_D_joint_angle
            theta_P_joint = max_P_joint_angle
            theta_M_joint = prev_theta_M + time_Step*(-(1.36e-08)*prev_theta_M + 51.44*voltage)
            if(theta_M_joint > max_M_joint_angle):
                theta_M_joint = max_M_joint_angle
            # print("JM: ", theta_M_joint)
    else: #reverse motion    These models need to be replaced with reverse motion data models
        if theta_M_joint > 0: #Move joint M
            theta_D_joint = max_D_joint_angle
            theta_P_joint = max_P_joint_angle
            theta_M_joint = prev_theta_M + time_Step*(-(0.002942)*prev_theta_M + (50.526*voltage))
            if(theta_M_joint < 0):
                theta_M_joint = 0
            # print("M joint reverse: ", theta_M_joint)

        elif theta_M_joint <= 0 and theta_P_joint > 0: #Move joint P
            theta_D_joint = max_D_joint_angle
            theta_P_joint = prev_theta_P + time_Step*(-(5.557)*prev_theta_P + 36.1*voltage)
            if(theta_P_joint < 0):
                theta_P_joint = 0
            theta_M_joint = 0
            # print("P joint reverse: ", theta_P_joint)

        elif theta_M_joint <= 0 and theta_P_joint <= 0 and theta_D_joint >= 0: #Move joint D
            theta_D_joint = prev_theta_D + time_Step*(-0.0006918*prev_theta_D + 49.12*voltage)
            if(theta_D_joint < 0):
                theta_D_joint = 0
            theta_P_joint = 0
            theta_M_joint = 0
            # print("D joint reverse: ", theta_D_joint)

    # convert to radians
    theta_D_joint = math.radians(theta_D_joint)
    theta_P_joint = math.radians(theta_P_joint)
    theta_M_joint = math.radians(theta_M_joint)
    
    acumalating_time = acumalating_time + time_Step

    
    # calculate joint positions
    pos_M_joint_Y = 0
    pos_M_joint_X = 0

    pos_P_joint_Y= p_phalanx_midF_Length*math.cos(theta_M_joint)
    pos_P_joint_X = p_phalanx_midF_Length*math.sin(theta_M_joint)

    Pos_D_joint_Y = p_phalanx_midF_Length*math.cos(theta_M_joint) + I_phalanx_midF_Length*math.cos(theta_M_joint + theta_P_joint)
    Pos_D_joint_X = p_phalanx_midF_Length*math.sin(theta_M_joint) + I_phalanx_midF_Length*math.sin(theta_M_joint + theta_P_joint)

    pos_tip_Y = p_phalanx_midF_Length*math.cos(theta_M_joint) + I_phalanx_midF_Length*math.cos(theta_M_joint + theta_P_joint) + d_phalanx_midF_length*math.cos(theta_M_joint + theta_P_joint + theta_D_joint)
    pos_tip_X = p_phalanx_midF_Length*math.sin(theta_M_joint) + I_phalanx_midF_Length*math.sin(theta_M_joint + theta_P_joint) + d_phalanx_midF_length*math.sin(theta_M_joint + theta_P_joint + theta_D_joint)


    return pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip_X, pos_tip_Y, degrees(theta_M_joint), degrees(theta_P_joint), degrees(theta_D_joint), acumalating_time



    
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
 

def calculate_theta_m(prev_theta_m, voltage, dt):
    # print("prev: ", prev_theta_m)
    
    theta_m = (prev_theta_m + dt*(-0.40*prev_theta_m + 105*voltage))
    # theta_m = (prev_theta_m + dt*(-2*prev_theta_m + 105*voltage))

    # theta_m = (prev_theta_m + dt*(-3*prev_theta_m + 350*voltage)) # decreased rise time

    # print("theta_m: ", theta_m)
    # prev_theta_m = theta_m
    return theta_m

def generate_step_input(total_time, dt, step_magnitude, max_pwm, max_voltage):
    num_steps = int(total_time / dt)
    pwm_value = int((step_magnitude / max_voltage) * max_pwm)
    pwms = np.full(num_steps, pwm_value, dtype=int)
    voltages = np.full(num_steps, step_magnitude)
    times = np.arange(0, total_time, dt)
    print("pwms: ", pwms)
    return times, voltages, pwms

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

    with open('Finger_Multi_Model_Approach/Data/Finger_Validation_Sim_data.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["time","theta_m", "theta_D_joint", "theta_P_joint" ,"theta_M_joint", "pwm"])
        writer.writerows(data)
    print("data saved at Finger_Multi_Model_Approach/Data/Finger_Validation_Sim_data.csv")


def convert_pwm_to_voltage(pwm):
    voltage = (pwm/MAX_PWM)*MAX_VOLTAGE
    return voltage


class FingerControlCommandSubscriber:
    def __init__(self):
        # Initialize the ROS node
        # rospy.init_node('control_command_subscriber', anonymous=True)

        # Variable to hold the position

        self.PWM = 0

        # Set up the subscriber
        # The callback function is called every time a message is received
        self.subscriber = rospy.Subscriber("Control_Command", ControlCommands, self.update_control_command)
        self.thread = Thread(target=self.spin) #create a thread for the spin method so it runs independantly of the rest of the code
        self.thread.daemon = True #thread will exit when main program is terminated
        self.thread.start() # start thread

    def update_control_command(self, msg):
        """Callback function to handle incoming messages."""
        # self.finger_position = msg
        self.PWM = msg.PWM

        # rospy.loginfo(f"Updated finger position: {msg}")

    def spin(self):
        """Keep the node running so it can keep receiving messages."""
        rospy.spin()
    
    def get_latest_control_command(self):
        # print(self.PWM)

        return self.PWM

class FingerPositionPublisher:
    def __init__(self):
        # Initialize the ROS node
        # rospy.init_node('finger_position_publisher', anonymous=True)
        # Set up the publisher
        self.publisher = rospy.Publisher('Updated_Finger_Position', FingerPos, queue_size=10)

    def publish_position(self, theta_M, theta_P, theta_D):
        """Publishes the updated position."""
        msg = FingerPos()
        msg.theta_M = theta_M
        msg.theta_P = theta_P
        msg.theta_D = theta_D
        self.publisher.publish(msg)
        # rospy.loginfo("Published updated position data ", msg.theta_M )


async def main():
    rospy.init_node('main_node', anonymous=True)

    #Initial position 
    prev_theta_D = 0
    prev_theta_P = 0
    prev_theta_M = 0

    fccs = FingerControlCommandSubscriber()
    fpp = FingerPositionPublisher()

    try:
        PWM = 0
        voltage = 0

        while not rospy.is_shutdown():

            PWM = fccs.get_latest_control_command()
            # print("PWM: ", PWM)
            voltage = convert_pwm_to_voltage(PWM)
            # print("voltage: ", voltage)
            #simulate new position
            pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip_X, pos_tip_Y, theta_M_joint, theta_P_joint, theta_D_joint, next_acumalating_time = finger_pos_update(voltage, math.radians(prev_theta_D), math.radians(prev_theta_P), math.radians(prev_theta_M), time_Step = 0.03, acumalating_time = 123)
            
            #publish latest position data
            fpp.publish_position(theta_M_joint, theta_P_joint, theta_D_joint)

            prev_theta_M = theta_M_joint
            prev_theta_P = theta_P_joint
            prev_theta_D = theta_D_joint
            update_visual_model(p_joint_mid_pos = vector(pos_P_joint_X,pos_P_joint_Y, 0), D_joint_mid_pos = vector(Pos_D_joint_X, Pos_D_joint_Y, 0), finger_tip_mid_pos = vector(pos_tip_X, pos_tip_Y, 0))

            await asyncio.sleep(0.1)  # Non-blocking sleep

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in main loop: {e}")

            


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

    acumalating_time = 0
    #initial position setup
    pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip_X, pos_tip_Y, theta_M_joint, theta_P_joint, theta_D_joint, acumalating_time = finger_pos_update(voltage = 0, prev_theta_D = 0, prev_theta_P = 0, prev_theta_M = 0, time_Step= 0.03, acumalating_time = acumalating_time)
    meta_joint_mid, p_joint_mid, p_phalanx_mid, D_joint_mid, I_phalanx_mid, finger_tip_mid, D_phalanx_mid = create_visual_model(pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip_X, pos_tip_Y)

    
    asyncio.run(main())

