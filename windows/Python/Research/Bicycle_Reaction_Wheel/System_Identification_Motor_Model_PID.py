
from vpython import *
import math
# Constants
t = 0 # Initial Time
dt = 0.01 # Time steps = 0.01 seconds
u = 0 #Voltage (input)
#k = 1.736*u + 206.3 # k for bike
#k = 217.6 average k for bike
k = 15.4*u + 305.7 # For motor two   ----k = 403.7825 Average  for motor two
#tau = 0.0441*u + 2.4436 #tau for bike
#tau = 2.73 average tau for bike
tau = 0.1344*u + 2.2241 #tau for motor two
#tau = 3.0975 #Average for motor two
#tau = 2.73
e = math.e
Velocity = 0
desired_velocity = 2000 #RPM
error = 0
previous_integral = 0
previous_error = 0
kp = 3 #3
ki = 0.55 #1 
kd = 1 #1
Graph = graph(title = "Velocity", xtitle = "time [s]", ytitle = "velocity RPM", width=800, height=400, fast = False)
f_velocity = gcurve(color=color.purple, label= "Velocity")


#Simulation
while t < 40: # Run simulation for 1 sec
    #rate(100)  #Don't do more than 100 calculation per sec. This keeps python from running the calculations too fast to see. We picked 100 so we could see what was happening in real time.
    #graph
    f_velocity.plot(t,Velocity)

    #PID controller
    error = desired_velocity - Velocity
    
    integral = previous_integral + error*dt
    P = kp * error
    I = ki * integral
    D = kd * ((error - previous_error)/dt)
    
    
    u = P + I + D
    if u > 12:
        u = 12
    elif u < -12:
        u = -12
    print(t,error, D, u) 

    #Model
    #Y = (k - (k/tau)*e**(-t/tau))*u  # Y is angular velocity in time domain
    Velocity = (k*u*dt + tau*Velocity)/ (tau + dt)  # in discrete domain
    #print(t,Velocity)


    previous_integral = integral
    previous_error = error
    t = t + dt
    
