
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np  # Ensure numpy is imported

# Load the data from CSV files
sim_data = pd.read_csv('python/Math_Model_of_Finger_Sim_Vs_Real/step_input_sim.csv')
real_data = pd.read_csv('python/Math_Model_of_Finger_Sim_Vs_Real/step_input_real.csv')

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
plt.figure(figsize=(7, 7))

plt.subplot(3, 1, 1)
plt.plot(sim_time, sim_theta_D_joint, label='Sim Distal Joint', color='blue')
plt.plot(real_time, real_theta_D_joint, label='Real Distal Joint', linestyle='--', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Angle (Deg)')
plt.legend()
plt.title('Distal Joint angle Comparison Real VS Sim')

plt.subplot(3, 1, 2)
plt.plot(sim_time, sim_theta_P_joint, label='Sim Proximal Joint', color='blue')
plt.plot(real_time, real_theta_P_joint, label='Real Proximal Joint', linestyle='--', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Angle (Deg)')
plt.legend()
plt.title('Proximal Joint angle Comparison Real VS Sim')

plt.subplot(3, 1, 3)
plt.plot(sim_time, sim_theta_M_joint, label='Sim Metacarpophalangeal Joint', color='blue')
plt.plot(real_time, real_theta_M_joint, label='Metacarpophalangeal', linestyle='--', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Angle (Deg)')
plt.legend()
plt.title('Metacarpophalangeal Joint angle Comparison Real VS Sim')

plt.tight_layout(pad=2.0)
plt.show()
