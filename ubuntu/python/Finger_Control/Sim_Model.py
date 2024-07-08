import matplotlib.pyplot as plt
import asyncio
import time
import csv
import numpy as np
from vpython import *
import math
import matplotlib.pyplot as plt
from collections import deque

#Paramters
p_phalanx_midF_Length = 3.9 # proximal phalanx length in cm
I_phalanx_midF_Length = 2.5 # intermediate phalanx length
d_phalanx_midF_length = 1.4 # Distal phalanx
max_D_joint_angle = 54 #68
max_P_joint_angle = 110
max_M_joint_angle = 80

def finger_pos_update(voltage, prev_theta_D, prev_theta_P, prev_theta_M, time_Step):
    
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
            theta_D_joint = prev_theta_D + time_Step*(-6.358*prev_theta_D + 66.32*voltage)
            if(theta_D_joint > max_D_joint_angle):
                theta_D_joint = max_D_joint_angle
            theta_P_joint = 0
            theta_M_joint = 0
            print("moving J_D: ", theta_D_joint)
        elif theta_D_joint >= max_D_joint_angle and theta_P_joint < max_P_joint_angle: # Move joint P
            print("Moving J_P")
            theta_D_joint = max_D_joint_angle
            # theta_P_joint = prev_theta_P + time_Step*(-(1.165)*prev_theta_P + 53.13*voltage)
            theta_P_joint = prev_theta_P + time_Step*(-(4.758)*prev_theta_P + 85.43*voltage)
            if(theta_P_joint > max_P_joint_angle):
                theta_P_joint = max_P_joint_angle

            theta_M_joint = 0
        elif theta_D_joint >= max_D_joint_angle and theta_P_joint >= max_P_joint_angle and theta_M_joint < max_M_joint_angle: # Move joint M
            print("Moving J_M")
            theta_D_joint = max_D_joint_angle
            theta_P_joint = max_P_joint_angle
            theta_M_joint = prev_theta_M + time_Step*(-(6.262)*prev_theta_M + 69.76*voltage)
    else: #reverse motion    These models need to be replaced with reverse motion data models
        if theta_M_joint > 0: #Move joint M
            print("M joint reverse")
            theta_D_joint = max_D_joint_angle
            theta_P_joint = max_P_joint_angle
            theta_M_joint = prev_theta_M + time_Step*(-(6.262)*prev_theta_M + 69.76*voltage)
            if(theta_M_joint < 0):
                theta_M_joint = 0
        elif theta_M_joint <= 0 and theta_P_joint >= 0: #Move joint P
            print("P joint reverse")
            theta_D_joint = max_D_joint_angle
            theta_P_joint = prev_theta_P + time_Step*(-(4.758)*prev_theta_P + 85.43*voltage)
            if(theta_P_joint < 0):
                theta_P_joint = 0
            theta_M_joint = 0
        elif theta_M_joint <= 0 and theta_P_joint <= 0 and theta_D_joint >= 0: #Move joint D
            theta_D_joint = prev_theta_D + time_Step*(-6.358*prev_theta_D + 66.32*voltage)
            if(theta_D_joint < 0):
                theta_D_joint = 0
            theta_P_joint = 0
            theta_M_joint = 0
            print("D joint reverse")

    # convert to radians
    theta_D_joint = math.radians(theta_D_joint)
    theta_P_joint = math.radians(theta_P_joint)
    theta_M_joint = math.radians(theta_M_joint)
    
    # acumalating_time = acumalating_time + time_Step

    
    # calculate joint positions
    pos_M_joint_Y = 0
    pos_M_joint_X = 0

    # pos_P_joint_Y= p_phalanx_midF_Length*math.cos(theta_M_joint)
    # pos_P_joint_X = p_phalanx_midF_Length*math.sin(theta_M_joint)

    # Pos_D_joint_Y = p_phalanx_midF_Length*math.cos(theta_M_joint) + I_phalanx_midF_Length*math.cos(theta_M_joint + theta_P_joint)
    # Pos_D_joint_X = p_phalanx_midF_Length*math.sin(theta_M_joint) + I_phalanx_midF_Length*math.sin(theta_M_joint + theta_P_joint)

    # pos_tip_Y = p_phalanx_midF_Length*math.cos(theta_M_joint) + I_phalanx_midF_Length*math.cos(theta_M_joint + theta_P_joint) + d_phalanx_midF_length*math.cos(theta_M_joint + theta_P_joint + theta_D_joint)
    # pos_tip_X = p_phalanx_midF_Length*math.sin(theta_M_joint) + I_phalanx_midF_Length*math.sin(theta_M_joint + theta_P_joint) + d_phalanx_midF_length*math.sin(theta_M_joint + theta_P_joint + theta_D_joint)


    # return pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip_X, pos_tip_Y, theta_M_joint, theta_P_joint, theta_D_joint, acumalating_time
    return theta_M_joint, theta_P_joint, theta_D_joint
