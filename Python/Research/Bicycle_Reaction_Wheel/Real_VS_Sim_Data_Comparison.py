
from vpython import *
import math
import json
# Constants
Ymin = 0
t = 0 # Initial Time
dt = 0.01 # Time steps = 0.01 seconds
u = 10 #Voltage (Step input)
#k = 1.736*u + 206.3 #Average = 217.6
#k = 15.4*u + 305.7 # For motor two
k = 7.576*u + 400.05 # Better data from python k = 449.29
#k = 217.6 Average for bike # k = 403.7825 Average  for motor two
#tau = 0.0441*u + 2.4436 #for bike 
#tau = 2.73 #Average tau for bike
#tau = 0.1344*u + 2.2241 #tau for motor two
#tau = 2.825 #Average for motor two
tau = -0.0455*u + 1.2455 # Better data from python tau = 0.927
#tau = 3.0975 #Average for motor two
e = math.e
Y = 0

voltage = u #Which file to read
with open(f"Bicycle_Reaction_Wheel\Data\Motor_Two_Data_For_Comparison\{voltage}_Volts.json", mode='r') as file:
    data_points = json.load(file)

time = [data_point["Time"] for data_point in data_points]
velocity = [data_point["Velocity"] for data_point in data_points]





Graph = graph(title = "Velocity", xtitle = "time [s]", ytitle = "velocity RPM", width=800, height=400, fast = False)
f_velocity = gcurve(color=color.red, label= "Sim Velocity")

#Graph1 = graph(title = "Velocity vs time", xtitle = "time [s]", ytitle = "Velocity [rpm]", width=800, height=400, fast = False)
#f__Desired_velocity = gcurve(color=color.red, label= "Desired Velocity")
f__Actual_velocity = gcurve(color=color.blue, label= "Actual Velocity")


#Simulation
while t < 23: # Run simulation for 1 sec
    #rate(100)  #Don't do more than 100 calculation per sec. This keeps python from running the calculations too fast to see. We picked 100 so we could see what was happening in real time.
    #Model
    #Y = (k - (k/tau)*e**(-t/tau))*u  # Y is angular velocity in time domain
    Y = (k*u*dt + tau*Ymin)/ (tau + dt)  # in discrete domain

    print(t,Y)
    #graph
    f_velocity.plot(t,Y)

    t = t + dt
    Ymin = Y


for time, velocity in zip(time, velocity):  #The zip function is used to iterate through both lists simultaneously
    #rate(100) #number of calculations per second allowed
    f__Actual_velocity.plot(time, velocity)
    print(time,velocity)
    