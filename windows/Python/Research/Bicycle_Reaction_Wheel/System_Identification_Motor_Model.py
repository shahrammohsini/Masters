
from vpython import *
import math
# Constants
Ymin = 0
t = 0 # Initial Time
dt = 0.01 # Time steps = 0.01 seconds
u = 9 #Voltage (Step input)
k = 7.576*u + 400.05 # Better data from python k = 449.29
tau = -0.0455*u + 1.2455 # Better data from python tau = 0.927
e = math.e
Y = 0





Graph = graph(title = "Velocity", xtitle = "time [s]", ytitle = "velocity RPM", width=800, height=400, fast = False)
f_velocity = gcurve(color=color.purple, label= "Velocity")


#Simulation
while t < 20: # Run simulation for 1 sec
    #rate(100)  #Don't do more than 100 calculation per sec. This keeps python from running the calculations too fast to see. We picked 100 so we could see what was happening in real time.
    Y = (k*u*dt + tau*Ymin)/ (tau + dt)  # in discrete domain
    print(t,Y)











    #graph
    f_velocity.plot(t,Y)

    t = t + dt
    Ymin = Y
    
