import numpy as np
import matplotlib.pyplot as plt

# Constants
time_step = 0.02  # example time step
time_duration = 10  # total time for simulation
voltage = 12  # example voltage value

# Time array
time = np.arange(0, time_duration, time_step)

# Initial values
theta_P_joint = np.zeros(len(time))
prev_theta_P = 80

# Calculate theta_P_joint over time
for i in range(0, len(time)):
    theta_P_joint[i] = prev_theta_P + time_step * (-(5.557) * prev_theta_P + (36.1 * voltage))
    prev_theta_P = theta_P_joint[i]

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(time, theta_P_joint, label='theta_P_joint')
plt.title('Theta_P_joint vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Theta_P_joint')
plt.legend()
plt.grid(True)
plt.show()
