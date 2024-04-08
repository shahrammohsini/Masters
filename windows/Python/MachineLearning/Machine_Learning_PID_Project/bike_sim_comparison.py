from vpython import *
import math

#Parameters
desired_angle = 0.1  #desired control angle
desired_angle2 = 0.1  #desired control angle

thetaR = 7# angle of rod  (Initial rod angle)
thetaR2 = 7# angle of rod  (Initial rod angle)

thetaR = -math.radians(thetaR)
thetaR2 = -math.radians(thetaR2)

print(thetaR)
radius_f1 = 0.05
radius_f2 = 0.06
radius_f = 0.06
g = 9.8
L = 0.133 # Length of Rod 0.17
Xf = L*sin(thetaR) # x pos of flywheel
Yf = L*cos(thetaR) # y pos of flywheel
mR = 0.027 # mass of rod in kg           0.033kg
mF = 0.06 # mass of flywheels in kg     0.588kg
mm = 0.11 #mass of motor
M = 0.5*mR + mF
thetaF = 0 #sping angle of flywheel
thetaF = -math.radians(thetaF)
Torque_M = 0 # input torque of motor
Torque_M2 = 0 # input torque of motor

t = 0
dt = 0.01
i = 0
k =  0.006 #0.00157  #coeffecient of friction
thetadotR = 0
thetadotR2 = 0

thetadotF = 0
IR = (1/3)*mR*L*L  #Moment of interti of rod
IF = 0.5*mF*((radius_f1*radius_f1) + (radius_f2*radius_f2))  #Moment of intertia of flywheel
IL = IR + mF*L*L #For conviniance let IL = IR + mF*L*L
#Controller
error = 0
error2 = 0

kp2 = 165.53 #165.53   150     # 1.3  300  150
ki2 = 32.77 # 32.77    50      #eleminates steady state error  0.2   80
kd2 = 1.665  #  1.665    2         #prevent overshooting   0.2   5

kp = 150 #165.53   150     # 1.3  300  150
ki = 50 # 32.77    50      #eleminates steady state error  0.2   80
kd = 2 #  1.665    2         #prevent overshooting   0.2   5


previous_integral = 0
previous_error = 0
stall_torque = 1.1 # Nm

previous_integral2 = 0
previous_error2 = 0
stall_torque2 = 1.1 # Nm

#for the motor
Rm = 2.5 #internal resitance of motor
kt = 0.33#0.3568 #torque constant of motor
Lm = 0.9 #0.499 #internal inductance of motor


#Visual modeling
ground = box(color = color.white, pos = vector(0,0,0), size = vector(2.5,0.02,1))
ground_Top = box(color = color.white, pos = vector(0,0.25,0), size = vector(2.5,0.02,1))
ground_Bot = box(color = color.white, pos = vector(0,-0.25,0), size = vector(2.5,0.02,1))
#hingePlatform = box(color = color.white, pos =vector(0,0.125,0.5) , size = vector(0.25,0.25,0.1))
hinge = sphere(color = color.red, radius = 0.01, pos = vector(0,0,0.51) )
fly_Wheel = ring(color = color.red, radius = radius_f, thickness = 0.009, pos = vector(Xf,Yf,hinge.pos.z), axis = vector(0,0,1))
fan_a = box(color = color.red, pos = fly_Wheel.pos, size = vector(0.09,0.01,0.001), axis = vector(1,0,0))
fan_b = box(color = color.red, pos = fly_Wheel.pos, size = vector(0.09,0.01,0.001), axis = vector(0,1,0))
rod = cylinder(color = color.blue, radius = 0.009, pos = hinge.pos - vector(0,0,0.01), axis = fly_Wheel.pos - hinge.pos)
Text_Time = label(pos=vec(-0.5, 0.5, 0), text='Time: 0 Sec', height=15)



#Graphs
#graph of Rod angle
Graph_Motor_Torque = graph(title = 'Motor Torque vs Time', width = 400, height = 200, xtitle = "Time [s]", ytitle = "Torque", fast = False)
f_Motor_Torque = gcurve(color=color.blue)
#graph of Rod angle
Graph_Rod_Angle = graph(title = 'Pendulum Position vs Time', width = 800, height = 400, xtitle = "Time [s]", ytitle = "Position [deg]", fast = False)
f_Rod_Angle = gcurve(color=color.blue, label = "Position Manual")
f_Rod_Angle_ML = gcurve(color=color.red, label = "Position ML")

#graph of Rod Velocity
Graph_Rod_velocity = graph(title = 'Pendulum Velocity vs Time', width = 400, height = 200, xtitle = "Time [s]", ytitle = "Angular Velcity [deg/s]", fast = False)
f_Rod_velocity = gcurve(color=color.blue)
#graph of Rod Accelaration
Graph_Rod_accelaration = graph(title = 'Pendulum Accelaration vs Time', width = 400, height = 200, xtitle = "Time [s]", ytitle = "Angular Accelaration [deg/s^2]", fast = False)
f_Rod_accelaration = gcurve(color=color.blue)
#graph of Flywheel angle
Graph_Flywheel_Angle = graph(title = 'Flywheel Angle vs Time', width = 400, height = 200, xtitle = "Time [s]", ytitle = "Angle [deg]", fast = False)
f_Flywheel_Angle = gcurve(color=color.blue)
#graph of Flywheel Velocity
Graph_Flywheel_velocity = graph(title = 'Reaction Wheel Velocity vs Time', width = 400, height = 200, xtitle = "Time [s]", ytitle = "Angular Velcity [deg/s]", fast = False)
f_Flywheel_velocity = gcurve(color=color.blue)
#graph of Flywheel Accelaration
Graph_Flywheel_accelaration = graph(title = 'Flywheel Accelaration vs Time', width = 400, height = 200, xtitle = "Time [s]", ytitle = "Angular Accelaration [deg/s^2]", fast = False)
f_Flywheel_accelaration = gcurve(color=color.blue)





while t < 20:
    i = i + 1
    Text_Time.text = f'Time: {i/100} Sec'
    
    # rate(100)

    #Control
    error = thetaR + math.radians(desired_angle)
    error2 = thetaR2 + math.radians(desired_angle2)

    # print(math.degrees(error))
    integral = previous_integral + error*dt
    integral2 = previous_integral2 + error2*dt

    P = kp * error
    P2 = kp2 * error2

    I = ki * integral
    I2 = ki2 * integral2

    D = kd * ((error - previous_error)/dt)
    D2 = kd2 * ((error2 - previous_error2)/dt)

    # Torque_M = P + I + D
    # PWM = P + I + D

    # # Clamping anti-windup on integral to prevent windup
    # if PWM > 255 and error > 0:  # if the output is maxed out and error is still positive
    #     integral = previous_integral  # don't accumulate
    # elif PWM < -255 and error < 0:  # if the output is at its minimum and error is still negative
    #     integral = previous_integral  # don't accumulate
    
    # print("Integral: ", integral)

    # #Limits (saturation)
    # if PWM > 255:
    #     PWM = 255
    # elif PWM < -255:
    #     PWM = -255

    # print("PWM: ", PWM)
    # V = 12*(PWM/255) #input voltage

    PWM = P + I + D
    PWM2 = P2 + I + D2


    # Clamping anti-windup on integral to prevent windup
    if PWM > 255 and error > 0:  # if the output is maxed out and error is still positive
        integral = previous_integral  # don't accumulate
    elif PWM < -255 and error < 0:  # if the output is at its minimum and error is still negative
        integral = previous_integral  # don't accumulate

         # Clamping anti-windup on integral to prevent windup
    if PWM2 > 255 and error2 > 0:  # if the output is maxed out and error is still positive
        integral2 = previous_integral2  # don't accumulate
    elif PWM2 < -255 and error2 < 0:  # if the output is at its minimum and error is still negative
        integral2 = previous_integral2  # don't accumulate
        
        


    #Limits (saturation)
    if PWM > 255:
        PWM = 255
    elif PWM < -255:
        PWM = -255

        #Limits (saturation)
    if PWM2 > 255:
        PWM2 = 255
    elif PWM2 < -255:
        PWM2 = -255

    V = 12*(PWM/255) #input voltage
    V2 = 12*(PWM2/255) #input voltage


    Torque_M = kt*V*math.exp((-Rm/Lm))
    Torque_M2 = kt*V2*math.exp((-Rm/Lm))


    print("Torque: ", Torque_M)
    # print("Torque_M", Torque_M)

    print("Torque: ", Torque_M)



    #Simulation
    #thetaddotR = ((0.5*mR + mF)*g*L*thetaR - Torque_M) / ((1/3)*mR*L*L + mF*L*L)  #Linear equation
    #thetaddotR = ((M*g*L*sin(thetaR)) - Torque_M) / IL
    thetaddotR = ((0.5*mR + mF)*g*L*sin(thetaR) - Torque_M) / ((1/3)*mR*L*L + mF*L*L)  #without friction
    
    thetaddotR2 = ((0.5*mR + mF)*g*L*sin(thetaR2) - Torque_M2) / ((1/3)*mR*L*L + mF*L*L)  #without friction
    # thetaddotR = ((0.5*mR + mF)*g*L*sin(thetaR) - Torque_M - (thetadotR*k)) / ((1/3)*mR*L*L + mF*L*L + mm*L*L) #with friction
    thetadotR = thetadotR + thetaddotR*dt
    thetadotR2 = thetadotR2 + thetaddotR2*dt

    thetaR = thetaR + thetadotR*dt
    thetaR2 = thetaR2 + thetadotR2*dt





    #Graph
    
    #f_Motor_Torque.plot(t,-math.degrees(thetaR)) #Plot morot torque
    f_Rod_Angle.plot(t,-math.degrees(thetaR)) #Plot thetaR in degrees
    # f_Motor_Torque.plot(t,Torque_M) #Plot morot torque
    f_Rod_Angle_ML.plot(t,-math.degrees(thetaR2)) #Reference position
    

    previous_integral = integral
    previous_integral2 = integral2
    previous_error = error
    previous_error2 = error2

    t = t + dt








