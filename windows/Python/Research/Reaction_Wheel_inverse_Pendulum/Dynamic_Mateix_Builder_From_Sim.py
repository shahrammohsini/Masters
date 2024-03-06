import math
import json
import copy
from vpython import *
# Constants
Ymin = 0
t = 0 # Initial Time
dt = 0.001 # Time steps = 0.01 seconds
u = 4 #Voltage (Step input)
k = 7.576*u + 400.05 # Better data from python k = 449.29
tau = -0.0455*u + 1.2455 # Better data from python tau = 0.927
e = math.e
Y = 0


#Model
#Parameters
desired_angle = 0  #desired control angle
thetaR = 0# angle of rod  (Initial rod angle)
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
dt = 0.01
i = 0
k =  0.006 #0.00157  #coeffecient of friction
thetadotR = 0
thetadotF = 0
IR = (1/3)*mR*L*L  #Moment of interti of rod
IF = 0.5*mF*radius_f*radius_f  #Moment of intertia of flywheel
IL = IR + mF*L*L + 0.03 #For conviniance let IL = IR + mF*L*L
#for the motor
Rm = 2.5 #internal resitance of motor
kt = 0.3568 #torque constant of motor
Lm = 0.9 #0.499 #internal inductance of motor













Angle_Array = []




#Simulation
while t < 1.2: # Run simulation for 1 sec
    #rate(100)  #Don't do more than 100 calculation per sec. This keeps python from running the calculations too fast to see. We picked 100 so we could see what was happening in real time.
    
    PWM = 255
    V = 12*(PWM/255) #input voltage
    Torque_M = kt*V*math.exp((-Rm/Lm))
    print("Torque:", Torque_M)
    thetaddotR = ((0.5*mR + mF)*g*L*sin(thetaR) - Torque_M) / (((1/3)*mR*L*L + mF*L*L) + 0.001)  #without friction #The 0.001 is just an offset i got experimentally to make moment of inertia more correct
    # thetaddotR = ((0.5*mR + mF)*g*L*sin(thetaR) - Torque_M - (thetadotR*k)) / ((1/3)*mR*L*L + mF*L*L + mm*L*L) #with friction
    thetadotR = thetadotR + thetaddotR*dt
    thetaR = thetaR + thetadotR*dt
    Angle_Array.append(thetaR/PWM) 
    print("angle: ", thetaR)
    t = t + dt


#create dynamic matrix

# velocity_Array = [1,2,3,4,5,6,7,8,9]
nu = 3
r = len(Angle_Array) #get number of rows
#A = [1,2,3,4,5,6,7,8,9]
Mod_Array = copy.deepcopy(Angle_Array)
for i in range(nu-1):
    Mod_Array.insert(0,0)  # This inserts N-1 zeros into array Mod_Array


C = nu #get number of columns
m = [] #create matrix (array of arrays)

for i in range (r):
    E = []
    for j in range(C):
        v = Mod_Array[(i - 1 + (nu - j))]  #formula to fill dynamic matrix appropriately.
        E.append(v)
    m.append(E)


print(m)

#save the data in a json file
with open(r"C:\Users\100655277\Documents\GitHub\Masters\windows\Python\Research\Reaction_Wheel_inverse_Pendulum\Dynamic_Matrix_Data\Dynamic_Matrix.json", mode="w") as file:
    json.dump(m, file, indent=5) #The indent makes it esier to read if you just open the json file
file.close()
