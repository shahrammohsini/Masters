import math


theta_m = math.radians(45)
rm = 1.5 # radius of motor pulley
p_phalanx_midF_Length = 3.9 # proximal phalanx length in cm
I_phalanx_midF_Length = 2.5 # intermediate phalanx length
d_phalanx_midF_length = 1.4 # Distal phalanx
r1 = p_phalanx_midF_Length + I_phalanx_midF_Length + d_phalanx_midF_length # radius of rotation of joint 1 (Metacropophalagenal joint)
r2 = p_phalanx_midF_Length + I_phalanx_midF_Length # radius of rotation of joint 2 (Proximal joint)
r3 = p_phalanx_midF_Length # radius of rotation of join 3 (Distal joint)
max_P_joint_angle = 45 # Needs to be measured
max_D_joint_angle = 45





def finger_pos_update(theta_m):
# radius of rotation for the joints (belt pulley system)
    if (theta_m >=0 and theta_m <=90):
        theta_M_joint = 0
    else: theta_M_joint= theta_m*(r1/rm) #angle of joint 1 (metacarpophalangeal). Only rotates once joint 2 and 3 have stopped rotating
    
    if (theta_m >= 0 and theta_m <= 45):
        theta_P_joint = 0
    else: theta_P_joint = theta_m*(r2/rm) # angle of joint 2 (proximal joint) only rotates when joint 3 stops
    
    
    theta_D_joint = theta_m*(r1/rm) # angle of joint 3 (distal joint)

    pos_M_joint_X = 0
    pos_M_joint_Y = 0

    pos_P_joint_X = p_phalanx_midF_Length*math.cos(theta_M_joint)
    pos_P_joint_Y = p_phalanx_midF_Length*math.sin(theta_M_joint)

    Pos_D_joint_X = p_phalanx_midF_Length*math.cos(theta_M_joint) + I_phalanx_midF_Length*math.cos(theta_M_joint + theta_P_joint)
    Pos_D_joint_Y = p_phalanx_midF_Length*math.sin(theta_M_joint) + I_phalanx_midF_Length*math.sin(theta_M_joint + theta_P_joint)

    pos_tip__X = p_phalanx_midF_Length*math.cos(theta_M_joint) + I_phalanx_midF_Length*math.cos(theta_M_joint + theta_P_joint) + d_phalanx_midF_length*math.cos(theta_M_joint + theta_P_joint + theta_D_joint)
    pos_tip__Y = p_phalanx_midF_Length*math.sin(theta_M_joint) + I_phalanx_midF_Length*math.sin(theta_M_joint + theta_P_joint) + d_phalanx_midF_length*math.sin(theta_M_joint + theta_P_joint + theta_D_joint)



    print("pos_p: ", pos_P_joint_X, pos_P_joint_Y )
    print("pos_d: ", Pos_D_joint_X, Pos_D_joint_Y)
    print("pos_tip:", pos_tip__X, pos_tip__Y)

    return pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip__X, pos_tip__Y



if __name__ == "__main__":

    angles = range(0,90)
    for angle in angles:
        theta_m = angle
        finger_pos_update(theta_m)