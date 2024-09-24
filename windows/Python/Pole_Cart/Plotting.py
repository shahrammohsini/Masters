
from vpython import *
# Constants


ball = sphere(radius = 0.05, color = color.red, pos = vector(0,0.3,0))

ground = box(color = color.white, pos = vector(0,0,0), size = vector(1,0.02,1))

#print(ball.pos)

#Initial conditions
ball.m = 0.05 #50 gram ball. Damn, heavey!
g = vector(0,-9.8,0)  # accelaration due to graviety
v0 = 0 # Initial Velocity
ball.v = vector(0,v0,0)
t = 0 # Initial Time
dt = 0.01 # Time steps = 0.01 seconds

#graph
Graph2 = graph(title = "Accelaration", xtitle = "time [s]", ytitle = "Accelaration [m/s^2]", width=400, height=200, fast = True) #lets you control features of the graph. fast = False, lets you use an online tool that gives you more features for the graph including print.
f_accelaration = gcurve(color=color.purple, label= "Velocity")
Graph1 = graph(title = "Position Vs Velocity", xtitle = "time [s]", ytitle = "Position and Velocity", width=400, height=200)
f_position = gcurve(color=color.red, label = "Positoin")
f_velocity = gcurve(color=color.blue, label= "Velocity")

#Simulation
while t < 1: # Run simulation for 1 sec
    rate(100)  #Don't do more than 100 calculation per sec. This keeps python from running the calculations too fast to see. We picked 100 so we could see what was happening in real time.

    #kinematics
    F = ball.m*g  #force = ma
    a = F/ball.m  #could've just done a = g but this is just a good practice to keep things clean.
    ball.v = ball.v + a*dt  #Update velocity through time
    ball.pos = ball.pos + ball.v*dt

    #graph
    f_velocity.plot(t,ball.v.y)
    f_position.plot(t,ball.pos.y)
    f_accelaration.plot(t,a.y)

    t = t + dt
    
