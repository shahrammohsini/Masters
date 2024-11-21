
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np  # Ensure numpy is imported

# Load the data from CSV files
sim_data = pd.read_csv('step_input_sim.csv')
real_data = pd.read_csv('step_input_real.csv')

# Extract the columns for joints from real and simulated data and convert them to numpy arrays explicitly
real_theta_D_joint = real_data['theta_D_joint'].to_numpy()
real_theta_P_joint = real_data['theta_P_joint'].to_numpy()
real_theta_M_joint = real_data['theta_M_joint'].to_numpy()

sim_theta_D_joint = sim_data['theta_D_joint'].to_numpy()
sim_theta_P_joint = sim_data['theta_P_joint'].to_numpy()
sim_theta_M_joint = sim_data['theta_M_joint'].to_numpy()

real_time = real_data['time'].to_numpy()
sim_time = sim_data['time'].to_numpy()

# Plot the joints for simulation and real data
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(sim_time, sim_theta_D_joint, label='Sim Theta D Joint', color='blue')
plt.plot(real_time, real_theta_D_joint, label='Real Theta D Joint', linestyle='--', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Theta D Joint')
plt.legend()
plt.title('Theta D Joint Comparison')

plt.subplot(3, 1, 2)
plt.plot(sim_time, sim_theta_P_joint, label='Sim Theta P Joint', color='blue')
plt.plot(real_time, real_theta_P_joint, label='Real Theta P Joint', linestyle='--', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Theta P Joint')
plt.legend()
plt.title('Theta P Joint Comparison')

plt.subplot(3, 1, 3)
plt.plot(sim_time, sim_theta_M_joint, label='Sim Theta M Joint', color='blue')
plt.plot(real_time, real_theta_M_joint, label='Real Theta M Joint', linestyle='--', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Theta M Joint')
plt.legend()
plt.title('Theta M Joint Comparison')

plt.tight_layout(pad=2.0)
plt.show()
