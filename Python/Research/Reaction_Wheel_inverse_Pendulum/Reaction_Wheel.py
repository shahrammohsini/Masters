from vpython import *
import math

#Parameters
radius_f = 0.05
g = 9.8
L = 0.5 # Length of Rod
thetaR = 20 # angle of rod
thetaR = -math.radians(thetaR)
Xf = L*sin(thetaR) # x pos of flywheel
Yf = L*cos(thetaR) # y pos of flywheel
mR = 1000 # mass of rod 0.1
mF = 8000 # mass of flywheels  0.8
M = 0.5*mR + mF
thetaF = 0 #sping angle of flywheel
thetaF = -math.radians(thetaF)
Torque_M = 0 # input torque of motor
t = 0
dt = 0.01
thetadotR = 0
thetadotF = 0
IR = (1/3)*mR*L*L  #Moment of interti of rod
IF = 0.5*mF*radius_f*radius_f  #Moment of intertia of flywheel
IL = IR + mF*L*L #For conviniance let IL = IR + mF*L*L

#Visual modeling
ground = box(color = color.white, pos = vector(0,0,0), size = vector(2.5,0.02,1))
hingePlatform = box(color = color.white, pos =vector(0,0.125,0.5) , size = vector(0.25,0.25,0.1))
hinge = sphere(color = color.red, radius = 0.01, pos = hingePlatform.pos + vector(0,0,0.07) )
fly_Wheel = ring(color = color.red, radius = radius_f, thickness = 0.009, pos = vector(Xf,Yf,hinge.pos.z), axis = vector(0,0,1))
fan_a = box(color = color.red, pos = fly_Wheel.pos, size = vector(0.09,0.01,0.001), axis = vector(1,0,0))
fan_b = box(color = color.red, pos = fly_Wheel.pos, size = vector(0.09,0.01,0.001), axis = vector(0,1,0))
rod = cylinder(color = color.blue, radius = 0.009, pos = hinge.pos - vector(0,0,0.01), axis = fly_Wheel.pos - hinge.pos)




while t < 25:
    
    rate(100)

    #thetaddotR = ((M*g*L*thetaR) - Torque_M) / IL  #Linear equation
    thetaddotR = ((M*g*L*sin(thetaR)) - Torque_M) / IL

    thetadotR = thetadotR + thetaddotR*dt
    thetaR = thetaR + thetadotR*dt
    Xf = L*sin(thetaR) # x pos of flywheel
    Yf = L*cos(thetaR) # y pos of flywheel
    fly_Wheel.pos = vector(Xf,Yf,hinge.pos.z)
    rod.pos = hinge.pos - vector(0,0,0.01)
    rod.axis = fly_Wheel.pos - hinge.pos
    fan_a.pos = fly_Wheel.pos
    fan_b.pos = fly_Wheel.pos

    thetaddotF = Torque_M/IF
    thetadotF = thetadotF + thetaddotF*dt
    thetaF = thetaF + thetadotF*dt
    fly_Wheel.rotate(angle = radians(thetaF), axis=vector(0, 0, 1))
    fan_a.rotate(angle =radians(thetaF), axis = vector(0,0,1)) 
    fan_b.rotate(angle =radians(thetaF), axis = vector(0,0,1))
    
    t = t + dt








'''
while True:
    rate(30)  # Set the frame rate
    
    # Rotate the circle
    reaction_Wheel.rotate(angle=radians(3), axis=vector(0, 0, 1))
    fan_a.rotate(angle =radians(3), axis = vector(0,0,1)) 
    fan_b.rotate(angle =radians(3), axis = vector(0,0,1)) 
'''