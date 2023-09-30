from vpython import *
import math

#Parameters
thetaR = 180 # angle of rod
#thetaR = -math.radians(thetaR)
radius_f = 0.05
g = 9.8
L = 1 # Length of Rod
Xf = L*math.sin(thetaR) # x pos of flywheel
Yf = L*math.cos(thetaR) # y pos of flywheel

print(Xf,Yf)
x = math.sin(180)
y = math.cos(180)
print(x,y)

while True:
    rate(100)