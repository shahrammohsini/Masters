
from vpython import *
import math
import json
import numpy as np
import time


#Model

#Parameters
desired_angle = 0  #desired control angle
thetaR = 7# angle of rod  (Initial rod angle)
thetaR = -math.radians(thetaR)
print(thetaR)
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
t = 0
i = 0
k =  0.006 #0.00157  #coeffecient of friction
thetadotR = 0
thetadotF = 0
IR = (1/3)*mR*L*L  #Moment of interti of rod
IF = 0.5*mF*radius_f*radius_f  #Moment of intertia of flywheel
IL = IR + mF*L*L #For conviniance let IL = IR + mF*L*L
#Controller
error = 0


#for the motor
Rm = 2.5 #internal resitance of motor
kt = 0.3568 #torque constant of motor
Lm = 0.9 #0.499 #internal inductance of motor
stall_torque = 0.05 # Nm

PWM = 0



#controller
with open(r"C:\Users\100655277\Documents\GitHub\Masters\windows\Python\Research\Reaction_Wheel_inverse_Pendulum\Dynamic_Matrix_Data\Dynamic_Matrix.json", mode='r') as file:
    data_points = json.load(file)
A = np.array(data_points) #Load data into the Dynamic matrix
nu = 3 #Control horizon
#Constants
# N = int(3/0.01 + 1)  #(prediction horizon) number of data points in matrix. ran for 3 sec and collected 100 data points ever sec. the plus 1 is bc of the way I designed my loop
N = len(A)
u = np.zeros(nu) #Voltage (Step input)for model calculation
Y = 0
PHI = np.zeros(nu) #To correct model inaccuracy
desired_angle = 0
setpoint = 0 #deg   Desired angle
r_desired_angle = setpoint*np.ones(N)
u_prev = np.zeros(nu) # input from previous iteration
I_Matrix = np.identity(nu) # create an identity matrix of size 3 by 3
A_T = np.transpose(A)  #Transpose of matrix A
y_hat = np.zeros(N) #Predicted position
# dt = 0.02 #0.65 and 0.7
timer = 0
data_points = []
LAMBDA = 7 #Penalty factor
#Du calculations. A is the dynamixc matrix
LambdaI = LAMBDA * I_Matrix

ATA = A_T.dot(A) #not really a dot product. its actually AT*A. The dot funciton knows and multiplies them because their not vectors
ATA_LambdaI = ATA - LambdaI
ATA_LambdaI_Inv = np.linalg.inv(ATA_LambdaI)
Du = ATA_LambdaI_Inv.dot(A_T)





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
Graph_Rod_Angle = graph(title = 'Pendulum Position vs Time', width = 600, height = 300, xtitle = "Time [s]", ytitle = "Position [deg]", fast = False)
f_Rod_Angle = gcurve(color=color.blue, label = "Position")
f_Rod_Reference_Angle = gcurve(color=color.red, label = "Reference")

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










startTime = time.time()
dt = 0.04

loop_time = 20 #time in sec
while t < loop_time:
    timer = time.time() - startTime
    time.sleep(dt)
    rate(100)

    
    #mathematical in loop calclulations
    V = 12*(PWM/255) #input voltage
    print("V", V)
    Torque_M = kt*V*math.exp((-Rm/Lm))

    #Limits (saturation)
    if Torque_M > stall_torque:
        Torque_M = stall_torque
    elif Torque_M < -stall_torque:
        Torque_M = -stall_torque
    


    print("Torque_M", Torque_M)
    # thetaddotR = ((0.5*mR + mF)*g*L*thetaR - Torque_M) / ((1/3)*mR*L*L + mF*L*L + 0.001)  #Linear equation
    thetaddotR = ((0.5*mR + mF)*g*L*sin(thetaR) - Torque_M) / (((1/3)*mR*L*L + mF*L*L) + 0.001)  #without friction #The 0.001 is just an offset i got experimentally to make moment of inertia more correct
    # thetaddotR = ((0.5*mR + mF)*g*L*sin(thetaR) - Torque_M - (thetadotR*k)) / ((1/3)*mR*L*L + mF*L*L + mm*L*L) #with friction
    thetadotR = thetadotR + thetaddotR*dt
    thetaR = thetaR + thetadotR*dt

    #update the visual model
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
    thetadotw = thetadotF - thetadotR
    fly_Wheel.rotate(angle = (thetadotw/60), axis=vector(0, 0, 1))   #thetadotw/60 because the rotate function expects theta as an input not thetadot
    fan_a.rotate(angle = (thetadotw/60), axis = vector(0,0,1)) 
    fan_b.rotate(angle = (thetadotw/60), axis = vector(0,0,1))


    #MPC
    ym = math.degrees(thetaR)
    
    #MPC
    PHI = ym - y_hat[0] #difference between calculated velocity (y_hat) and measured velocity (ym) to see if the model is inaccurate

    y_hat = y_hat + PHI #correct model inacuracy
    # print("Y_hat = ", y_hat)
    # print("y_hat: ",y_hat)
    error = desired_angle - y_hat
    
    # print("error: ",error[0])
    # delta_u = error.dot(np.linalg.inv(A_T.dot(A) - (LAMBDA * I_Matrix)).dot(A_T)) #optimize input required to remove error
    delta_u = np.dot(Du,error)
    u = u_prev + delta_u # update input with optimized change
    
    if u[0] > 255:
        u[0] = 255

    elif u[0] < -255:
        u[0] = -255

    
    delta_u = u - u_prev #This recalculates delta_u so that it includes the limits added for the next step where we calculate y_hat
    #print(delta_u.size)
    # y_hat = y_hat + delta_u.dot(A) #update predicted output (note the delta_u used includes the limits added)
    y_hat = y_hat + A[:,0]*delta_u[0] #update predicted output (note the delta_u used includes the limits added). A[:,0] all rows of first col

    # print("old yhat: ", y_hat)
    y_hat[:-1] = y_hat[1:] #This line only moves the values of y_hat one to the right

    

    PWM = u[0]
   
    print("PWM: ", PWM)
    u_prev = u

    

    #Save data
    # print(error[0])
    
     #Graph
    
    #f_Motor_Torque.plot(t,-math.degrees(thetaR)) #Plot morot torque
    f_Rod_Angle.plot(t,-math.degrees(thetaR)) #Plot thetaR in degrees
    f_Motor_Torque.plot(t,Torque_M) #Plot morot torque
    f_Rod_Reference_Angle.plot(t,desired_angle) #Reference position
    f_Rod_velocity.plot(t,-math.degrees(thetadotw)) #Plot thetadotR in deg/s
    f_Rod_accelaration.plot(t,-math.degrees(thetaddotR)) #Plot thetddotR in deg/s^2
    #f_Flywheel_Angle.plot(t,-math.degrees(thetaF)) #Plot thetaF in degrees
    f_Flywheel_velocity.plot(t,(thetadotw)) #Plot thetadotf in rotations per second
    #f_Flywheel_accelaration.plot(t,-math.degrees(thetaddotF)) #Plot thetaddotF in deg/s^2
    #print(-math.degrees(thetadotw))

    t = t + dt