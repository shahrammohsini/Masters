
from vpython import *
import math
import json
import numpy as np
# Constants
Ymin = 0
t = 0 # Initial Time
dt = 0.01 # Time steps = 0.01 seconds
sim_time = 3 # num of sec the sim ran to create the dynamic matrix
N = int(3/0.01 + 1)  #(prediction horizon) number of data points in matrix. ran for 3 sec and collected 100 data points ever sec. the plus 1 is bc of the way I designed my loop
nu = 3 #control horizon
u = np.zeros(N) #Voltage (Step input)for model calculation
# k = 7.576*u + 400.05 # Better data from python k = 449.29
# tau = -0.0455*u + 1.2455 # Better data from python tau = 0.927
e = math.e
Y = 0

#MPC
PHI = np.zeros(nu) #To correct model inaccuracy
LAMBDA = 1.01 #penalty factor
y_mes = 0  #Measured position
#error = np.zeros((3, 1)) # r_desired_vel - y_hat  #error between desired velocity and predicted velocity
error = 0
r_desired_vel = 1400 #RPM   Desired velocity setpoint
delta_u = np.zeros(N) #Optimized change in input (u) to minimize error 
u_prev = np.zeros(N) # input from previous iteration

I_Matrix = np.identity(nu) # create an identity matrix of size 3 by 3

with open(f"/home/sham/Desktop/pythonFiles/Motor_Two/Data/Dynamic_Matrix/Matrix_Sim.json", mode='r') as file:
    data_points = json.load(file)
A = np.array(data_points) #Load data into the Dynamic matrix
A_T = np.transpose(A)  #Transpose of matrix A
print(A.shape[0], N)
y_hat = delta_u.dot(A) #Predicted position

Graph = graph(title = "Velocity", xtitle = "time [s]", ytitle = "velocity RPM", width=800, height=400, fast = False)
f_velocity = gcurve(color=color.purple, label= "Velocity")
f_input = gcurve(color=color.blue, label= "input")
f_error = gcurve(color=color.red, label= "error")


#Simulation
while t < 20: # Run simulation for 1 sec
    #rate(100)  #Don't do more than 100 calculation per sec. This keeps python from running the calculations too fast to see. We picked 100 so we could see what was happening in real time.
    k = 7.576*u[0] + 400.05 # Better data from python k = 449.29
    tau = -0.0455*u[0] + 1.2455 # Better data from python tau = 0.927
    #print(u)
    Y = ((k*u[0]*dt + tau*Ymin)/ (tau + dt))+ np.random.rand(1)  # in discrete domain
    #print(t,Y)
    ym = Y
    #MPC
    PHI = ym - y_hat #difference between calculated velocity (y_hat) and measured velocity (ym) to see if the model is inaccurate
    y_hat = y_hat + PHI #correct model inacuracy
    error = r_desired_vel - y_hat
    #print(error)
    #delta_u = (1/((A_T * A) - (LAMBDA * I_Matrix))) * A_T * (error)  #optimize input required to remove error
    delta_u = error.dot(np.linalg.inv(A_T.dot(A) - (LAMBDA * I_Matrix)).dot(A_T)) #optimize input required to remove error
   # print(delta_u.size)
    u = u_prev + delta_u # update input with optimized change
    #print(u)
    # Add limit control on input U
    if u[0] > 12:
        u[0] = 12

    elif u[0] < -12:
        u[0] = -12
   
    delta_u = u - u_prev #This recalculates delta_u so that it includes the limits added for the next step where we calculate y_hat
    #print(delta_u.size)
    
    y_hat = y_hat + delta_u.dot(A) #update predicted output (note the delta_u used includes the limits added)
    y_hat[:-1] = y_hat[1:] #This line only moves the values of y_hat one to the right
    #ex: y_hat before = [1,2,3] y_hat after = [2,3,3]. Note the last value is repeated. All this does is predict the next step
   


    #graph
    f_velocity.plot(t,Y)
    # f_input.plot(t,u[0])
    # f_error.plot(t,error[0])

    #update values
    u_prev = u

    t = t + dt
    Ymin = Y
    
