import matplotlib.pyplot as plt
import os
import dynamixel_sdk as dxl
import time
import csv
from scipy import signal
import numpy as np

t = 0
# dt = 0.01 # If this step size is too large the model becomes very innacurate
voltage = 12
previous_velocity = 0
current_Velocity = 0
MAX_VOLTAGE = 12.2

sim_time = []
sim_velocity = []
actual_velocity = []
data = []
prev_theta_m = 0
previous_velocity = 0
dt = 0.01
t = 0

while True:

    prev_time = time.time()

    #sim
    sim_time.append(t)

    data.append([t, voltage, current_Velocity])

    sim_velocity.append(current_Velocity)
    # print(current_Velocity)

    
    # dt = current_time - prev_time
    
    

    current_Velocity = previous_velocity + dt*(-22.37*previous_velocity + 506.7*voltage)
    # print("current_velocity: ", current_Velocity)

    theta_m = prev_theta_m + current_Velocity*dt
    print("theta_m: ", theta_m)
    prev_theta_m = theta_m

    previous_velocity = current_Velocity

    t = t + dt


        # Delay for sampling time
    time.sleep(0.01)