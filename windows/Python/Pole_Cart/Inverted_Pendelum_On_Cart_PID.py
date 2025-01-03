from vpython import *
import math

g = 9.8 # m/s^2
m = 1 # mass of pendelum in grams
M = 10 # mass of cart in grams
F = 0 # force input
theta = 15 #initial angle
theta = -math.radians(theta)
thetadot = 0 # inital angular velocity
t = 0
dt = 0.01
Xc = 0 #initial position of cart x cordinate
Xcdot = 0 #initial velocity of cart
L = 2 # length of rod
Xp = Xc - L*sin(theta)
Yp = L*cos(theta)
Xpdot = Xcdot - L*thetadot*cos(theta)
i = 0 #for counting itterations

#PID parameters
kp = 150  # 150 , 10, 2 is stable
ki = 0
kd = 10
error = 0
integral = 0
previous_error = 0

#Visual Modeling
ground = box(color = color.white, pos = vector(0,0,0), size = vector(30,0.02,10))
cart = box(color = color.blue, pos = vector(Xc,0.52,0.5), size = vector(1,1,1))
PendelumMass = sphere(color = color.red, radius = 0.1, make_trail = True, pos = vector(Xp,Yp,0.5))
rod = cylinder(color = color.green,  radius = 0.05, pos = cart.pos, axis = PendelumMass.pos-cart.pos)
# Create a text object to display the number of iterations
Text_Theta = label(pos=vec(5, 4, 0), text='Theta: 0', height=15)
Text_Time = label(pos=vec(-5, 4, 0), text='Time: 0 Sec', height=15)


#Graphs
#graph of pendulum angle
Graph_P_Pos = graph(title = 'Pendulum Position vs Time', width = 400, height = 200, xtitle = "Time [s]", ytitle = "Angle [deg]", fast = False)
f_Pendulum_Pos = gcurve(color=color.blue)
# graph of pendulum velocity
Graph_P_V = graph(title = "Pendulum Velocity vs Time", width = 400, height = 200, xtitle = "Time [s]", ytitle = "Angular velocity [deg/s]", fast = False)
f_Pendulum_Vel = gcurve(color = color.red)
#graph of pendulum accelaration
Graph_P_A = graph(title = 'Pendulum Accelaration vs Time', width = 400, height = 200, xtitle = "Time [s]", ytitle = "Angular Accelaration [deg/s^2]", fast = False)
f_Pendulum_A = gcurve(color=color.orange)
# graph of cart position
Graph_C_P = graph(title = "Cart Position vs Time", width = 400, height = 200, xtitle = "Time [s]", ytitle = "Position [m]", fast = False)
f_Cart_Pos = gcurve(color = color.purple)
# graph of cart velocity
Graph_C_V = graph(title = "Cart Velocity vs Time", width = 400, height = 200, xtitle = "Time [s]", ytitle = "Velocity [m/s]", fast = False)
f_Cart_V = gcurve(color = color.black)



while t < 120:

    Text_Theta.text = f'Theta: {round(-math.degrees(theta),1)}'
    i = i + 1
    Text_Time.text = f'Time: {i/100} Sec'

    rate(100)

    # PID
    error = -theta
    P = kp * error
    I = ki * (integral + error * dt)
    D = kd * ((error - previous_error)/dt)

    F = P + I + D


    #Model simulation
    #Xcddot = (F + m*g*theta)/M   #Equation for accelartaion of cart
    Xcddot = (F + m*g*sin(theta)*cos(theta) - m*L*thetadot*thetadot*sin(theta))/(M + m + m*cos(theta)*cos(theta))  #Non-Linear
    Xcdot = Xcdot + Xcddot*dt  # new velocity of cart
    Xc = Xc + Xcdot*dt  # New position of cart
    cart.pos = vector(Xc,0.52,0.5) # Update position of the cart
    
    #thetaddot = (F + g * theta * (M + m))/(L * M)
    thetaddot = (F*cos(theta) + (M+m)*g*sin(theta) - m*L*thetadot*thetadot*sin(theta)*cos(theta))/(L*(M+m) - m*L*cos(theta)*cos(theta)) #Non-Linear
    thetadot = thetadot + thetaddot*dt
    theta = theta + thetadot*dt
    Xp = Xc - L*sin(theta)
    Yp = L*cos(theta)
    PendelumMass.pos = vector(Xp,Yp,0.5)
    rod.pos = cart.pos
    rod.axis = PendelumMass.pos-cart.pos
    
    previous_error = error



    #Graph
    f_Pendulum_Pos.plot(t,-math.degrees(theta)) #Plot theta in degrees
    f_Pendulum_Vel.plot(t, -math.degrees(thetadot)) #Plot thetadot
    f_Pendulum_A.plot(t, -math.degrees(thetaddot)) #Plot thetadot
    f_Cart_Pos.plot(t, Xc) #Plot cart position
    f_Cart_V.plot(t, Xcdot) #Plot cart velocity





    t = t + dt  #update time
    

           



