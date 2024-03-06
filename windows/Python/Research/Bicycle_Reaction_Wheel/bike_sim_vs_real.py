from vpython import *
import math
import json

#Parameters
desired_angle = 0  #desired control angle
thetaR = -7# angle of rod  (Initial rod angle)
thetaR = -math.radians(thetaR)
print(thetaR)
radius_f1 = 0.05
radius_f2 = 0.06
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
dt = 0.01
i = 0
k =  0.006 #0.00157  #coeffecient of friction
thetadotR = 0
thetadotF = 0
IR = (1/3)*mR*L*L  #Moment of interti of rod
# IF = 0.5*mF*radius_f*radius_f  #Moment of intertia of flywheel
IF = 0.5*mF*((radius_f1*radius_f1) + (radius_f2*radius_f2))  #Moment of intertia of flywheel
print("motor intertia", mm*L*L)
IL = IR + mF*L*L #+ mm*L*L #For conviniance let IL = IR + mF*L*L
Guessed_Moment_of_Intertia_Rod = IL + 0.00045 #Had to add a very small amount to moment of inertia of rod to get it rreally close to the actual model
print(Guessed_Moment_of_Intertia_Rod)
#Controller
error = 0
kp = 1.3  # 1.3
kd = 0.2 #prevent overshooting   0.2
ki = 0.2 #eleminates steady state error  0.2
previous_integral = 0
previous_error = 0
stall_torque = 1.1 # Nm

#for the motor
Rm = 2.5 #internal resitance of motor
kt = 0.33#0.3568 #torque constant of motor
Lm = 0.499 #0.499 #internal inductance of motor


data_points = []



#pull data from real model saved:
with open(f"C:/Users/100655277/Documents/GitHub/Masters/windows/Python/Research/Bicycle_Reaction_Wheel/Data/Bike_Model_validation/First_input.json", mode='r') as file:
    data_points = json.load(file)
Robot_angle_real_Data = [data_point["Robot_angle"] for data_point in data_points]

print("Robot_angle", Robot_angle_real_Data)

#Visual modeling
ground = box(color = color.white, pos = vector(0,0,0), size = vector(2.5,0.02,1))
ground_Top = box(color = color.white, pos = vector(0,0.25,0), size = vector(2.5,0.02,1))
ground_Bot = box(color = color.white, pos = vector(0,-0.25,0), size = vector(2.5,0.02,1))
#hingePlatform = box(color = color.white, pos =vector(0,0.125,0.5) , size = vector(0.25,0.25,0.1))
hinge = sphere(color = color.red, radius = 0.01, pos = vector(0,0,0.51) )
fly_Wheel = ring(color = color.red, radius = radius_f2, thickness = 0.009, pos = vector(Xf,Yf,hinge.pos.z), axis = vector(0,0,1))
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
f_Rod_Angle = gcurve(color=color.blue, label = "Position_Sim")
# f_Rod_Reference_Angle = gcurve(color=color.red, label = "Reference")
f_Real_Rod_Angle_Data = gcurve(color=color.green, label = "Position_Real")

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





while t < 0.28:
    i = i + 1
    Text_Time.text = f'Time: {i/100} Sec'
    
    rate(100)

    #Control
    error = thetaR + math.radians(desired_angle)
    # print(math.degrees(error))
    integral = previous_integral + error*dt
    P = kp * error
    I = ki * integral
    D = kd * ((error - previous_error)/dt)

    PWM = 255

    V = 12*(PWM/255) #input voltage
    Torque_M = kt*V*math.exp((-Rm/Lm))


    # Torque_M = P + I + D  #Uncomment to use PID controller
    # Torque_M =  (126) * (IF) # Torque = (acceleration(rad/s^2)) * moment_of_inertia 
    print("Torque_M: ", Torque_M)
    # Torque_M = 0.1



    #Limits (saturation)
    if Torque_M > stall_torque:
        Torque_M = stall_torque
    elif Torque_M < -stall_torque:
        Torque_M = -stall_torque

    
    # print("Torque: ", Torque_M)





    #Simulation
    #thetaddotR = ((0.5*mR + mF)*g*L*thetaR - Torque_M) / ((1/3)*mR*L*L + mF*L*L)  #Linear equation
    #thetaddotR = ((M*g*L*sin(thetaR)) - Torque_M) / IL
    # thetaddotR = ((0.5*mR + mF)*g*L*sin(thetaR) - Torque_M) / ((1/3)*mR*L*L + mF*L*L)  #without friction
    # thetaddotR = ((0.5*mR + mF)*g*L*sin(thetaR) - Torque_M - (thetadotR*k)) / ((1/3)*mR*L*L + (mF+mm)*L*L) #with friction
    thetaddotR = ((0.5*mR + mF)*g*L*sin(thetaR) - Torque_M) / Guessed_Moment_of_Intertia_Rod  #guessed moment of inertia

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
    thetadotw = thetadotF - thetadotR
    fly_Wheel.rotate(angle = (thetadotw/60), axis=vector(0, 0, 1))   #thetadotw/60 because the rotate function expects theta as an input not thetadot
    fan_a.rotate(angle = (thetadotw/60), axis = vector(0,0,1)) 
    fan_b.rotate(angle = (thetadotw/60), axis = vector(0,0,1))



     
    # Convert thetaR to degrees (0 to 360 degrees)
    Mapped_rod_angle = math.degrees(thetaR) % 360
    print("thetaR:",-math.degrees(thetaR))
    #Graph
    
    #f_Motor_Torque.plot(t,-math.degrees(thetaR)) #Plot morot torque
    f_Rod_Angle.plot(t, math.degrees(thetaR)) #Plot thetaR in degrees  #This is the original line
    f_Real_Rod_Angle_Data.plot(t, Robot_angle_real_Data[i]) #Real data
    # f_Rod_Angle.plot(t, Mapped_rod_angle -360) # Comment this out
    f_Motor_Torque.plot(t,Torque_M) #Plot morot torque
    # f_Rod_Reference_Angle.plot(t,desired_angle) #Reference position
    f_Rod_velocity.plot(t,-math.degrees(thetadotw)) #Plot thetadotR in deg/s
    f_Rod_accelaration.plot(t,-math.degrees(thetaddotR)) #Plot thetddotR in deg/s^2
    #f_Flywheel_Angle.plot(t,-math.degrees(thetaF)) #Plot thetaF in degrees
    f_Flywheel_velocity.plot(t,(thetadotw)) #Plot thetadotf in rotations per second
    #f_Flywheel_accelaration.plot(t,-math.degrees(thetaddotF)) #Plot thetaddotF in deg/s^2
    #print(-math.degrees(thetadotw))

    previous_integral = integral
    previous_error = error
    t = t + dt


    #write this data
    try:
        #append the data into the list
        data_points.append({"Time":t, "Acceleration":27, "Robot_angle":(int(math.degrees(thetaR)))})
        # Create a file to save this data
        with open(f"C:/Users/100655277/Documents/GitHub/Masters/windows/Python/Research/Bicycle_Reaction_Wheel/Data/Bike_Model_validation/Sim_Data.json", mode="w") as file:
            json.dump(data_points, file, indent=1) #The indent makes it esier to read if you just open the json file
        file.close()
    except Exception as e:
        print(f"Error while writing data to the file: {e}")






