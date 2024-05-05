from vpython import *
import math


#Paramters

rm = 1.5 # radius of motor pulley
p_phalanx_midF_Length = 3.9 # proximal phalanx length in cm
I_phalanx_midF_Length = 2.5 # intermediate phalanx length
d_phalanx_midF_length = 1.4 # Distal phalanx
r1 = 3.3 # radius of rotation of joint 1 (Metacropophalagenal joint)
r2 = 3.3 # radius of rotation of joint 2 (Proximal joint)
r3 = p_phalanx_midF_Length # radius of rotation of join 3 (Distal joint)
max_P_joint_angle = 45 # Needs to be measured
max_D_joint_angle = 45
max_M_joint_angle = 45
max_D_joint_angle_motor = (r3/rm)*(max_D_joint_angle)
max_P_joint_angle_motor = (r2/rm)*(max_P_joint_angle)
max_M_joint_angle_motor = (r1/rm)*(max_M_joint_angle)




def create_phalanx(position, axis, radius):
    """Creates a single link as a cylinder."""
    link = cylinder(pos=position, axis=axis, radius=radius, color=color.white)
    return link

def create_joint(position, radius):
    """Creates a joint as a sphere."""
    return sphere(pos=position, radius=radius, color=color.red)




def finger_pos_update(theta_m):
    # radius of rotation for the joints (belt pulley system)
    if (theta_m >= math.radians(0) and theta_m <= math.radians(max_D_joint_angle_motor + max_P_joint_angle_motor)):
        theta_M_joint = math.radians(0) 
    else:
        theta_M_joint= ((theta_m - math.radians(max_D_joint_angle_motor + max_P_joint_angle_motor))*(rm/r1)) #angle of joint 1 (metacarpophalangeal). Only rotates once joint 2 and 3 have stopped rotating
        print("theta_M_joint: ", math.degrees(theta_M_joint))
        if(theta_M_joint) >= math.radians(max_M_joint_angle):
            theta_P_joint = math.radians(max_M_joint_angle)
            print("theta_M_joint", math.degrees(theta_M_joint))

    if (theta_m >= math.radians(0) and theta_m <= math.radians(max_D_joint_angle_motor)):
        theta_P_joint = 0
    else:
        theta_P_joint = ((theta_m - math.radians(max_D_joint_angle_motor))*(rm/r2)) # angle of joint 2 (proximal joint) only rotates when joint 3 stops
        print("theta_p_joint", math.degrees(theta_P_joint))

        if(theta_P_joint) >= math.radians(max_P_joint_angle):
            theta_P_joint = math.radians(max_P_joint_angle)
            print("theta_p_joint", math.degrees(theta_P_joint))
    
    theta_D_joint = theta_m*(rm/r3) # angle of joint 3 (distal joint)
    print("theta_D_joint", math.degrees(theta_D_joint))

    if(theta_D_joint) >= math.radians(max_D_joint_angle):
            theta_D_joint = math.radians(max_D_joint_angle)
            print("theta_D_joint", math.degrees(theta_D_joint))

    pos_M_joint_Y = 0
    pos_M_joint_X = 0

    pos_P_joint_Y= p_phalanx_midF_Length*math.cos(theta_M_joint)
    pos_P_joint_X = p_phalanx_midF_Length*math.sin(theta_M_joint)

    Pos_D_joint_Y = p_phalanx_midF_Length*math.cos(theta_M_joint) + I_phalanx_midF_Length*math.cos(theta_M_joint + theta_P_joint)
    Pos_D_joint_X = p_phalanx_midF_Length*math.sin(theta_M_joint) + I_phalanx_midF_Length*math.sin(theta_M_joint + theta_P_joint)

    pos_tip__Y = p_phalanx_midF_Length*math.cos(theta_M_joint) + I_phalanx_midF_Length*math.cos(theta_M_joint + theta_P_joint) + d_phalanx_midF_length*math.cos(theta_M_joint + theta_P_joint + theta_D_joint)
    pos_tip__X = p_phalanx_midF_Length*math.sin(theta_M_joint) + I_phalanx_midF_Length*math.sin(theta_M_joint + theta_P_joint) + d_phalanx_midF_length*math.sin(theta_M_joint + theta_P_joint + theta_D_joint)



    # print("pos_p: ", pos_P_joint_X, pos_P_joint_Y )
    # print("pos_d: ", Pos_D_joint_X, Pos_D_joint_Y)
    # print("pos_tip:", pos_tip__X, pos_tip__Y)

    return pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip__X, pos_tip__Y



    
def create_visual_model(pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip__X, pos_tip__Y):
    meta_joint_mid = create_joint(position=vector(0, 0, 0), radius=1.6) #Joint 1
    p_joint_mid = create_joint(position= vector(pos_P_joint_X, pos_P_joint_Y, 0), radius=1.4) #joint 2

    p_phalanx_mid = create_phalanx(position=meta_joint_mid.pos , axis = p_joint_mid.pos - meta_joint_mid.pos, radius=meta_joint_mid.radius) #Phalanx 1

    D_joint_mid = create_joint(position= vector(Pos_D_joint_X, Pos_D_joint_Y, 0), radius=1.3)


    I_phalanx_mid = create_phalanx(position=p_joint_mid.pos, axis = D_joint_mid.pos - p_joint_mid.pos, radius= p_joint_mid.radius)
    
    finger_tip_mid = sphere(pos= vector(pos_tip__X, pos_tip__Y, 0), radius=1.3, color=color.white)


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



if __name__ == "__main__":

    pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip__X, pos_tip__Y = finger_pos_update(theta_m = 0)
    meta_joint_mid, p_joint_mid, p_phalanx_mid, D_joint_mid, I_phalanx_mid, finger_tip_mid, D_phalanx_mid = create_visual_model(pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip__X, pos_tip__Y)



    angles = range(0, 365)
    for angle in angles:
        rate(100)

        theta_m = math.radians(angle)
        pos_P_joint_X, pos_P_joint_Y, Pos_D_joint_X, Pos_D_joint_Y, pos_tip__X, pos_tip__Y = finger_pos_update(theta_m)

        update_visual_model(p_joint_mid_pos = vector(pos_P_joint_X,pos_P_joint_Y, 0), D_joint_mid_pos = vector(Pos_D_joint_X, Pos_D_joint_Y, 0), finger_tip_mid_pos = vector(pos_tip__X, pos_tip__Y, 0))
        

        

    # # Model position of middle finger
    # finger_tip_mid_X =  

    

   

   