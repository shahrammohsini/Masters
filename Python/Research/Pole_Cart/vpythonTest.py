
from vpython import *
# Constants
print ("sdf")


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

#Simulation
while t < 1: # Run simulation for 1 sec
    rate(100)  #Don't do more than 100 calculation per sec. This keeps python from running the calculations too fast to see. We picked 100 so we could see what was happening in real time.

    #kinematics
    F = ball.m*g  #force = ma
    a = F/ball.m  #could've just done a = g but this is just a good practice to keep things clean.
    ball.v = ball.v + a*dt  #Update velocity through time
    ball.pos = ball.pos + ball.v*dt
    t = t + dt
