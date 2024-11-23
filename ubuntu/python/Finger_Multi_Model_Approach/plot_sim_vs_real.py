import pandas as pd
import matplotlib.pyplot as plt

# Load the simulation data
sim_data = pd.read_csv("/home/shahram/Documents/GitHub/Masters/ubuntu/python/Finger_Multi_Model_Approach/Data/Finger_Validation_Sim_Data_Forward_3.csv")
real_data = pd.read_csv("/home/shahram/Documents/GitHub/Masters/ubuntu/python/Finger_Multi_Model_Approach/Data/Finger_Validation_Real_Data_Forward.csv")

# extract the columns for joints from real data
real_theta_D_joint = real_data['Joint_3'].to_numpy()
real_theta_P_joint = real_data['Joint_2'].to_numpy()
real_theta_M_joint = real_data['Joint_1'].to_numpy()
# extract the columns for joints from sim data
sim_theta_D_joint = sim_data['theta_D_joint'].to_numpy()
sim_theta_P_joint = sim_data['theta_P_joint'].to_numpy()
sim_theta_M_joint = sim_data['theta_M_joint'].to_numpy()

real_time = real_data['time']
sim_time = sim_data['time']


# print("sim: ", sim_theta_P_joint)

# Plot the joints for simulation data
plt.figure(figsize=(7, 7))

plt.subplot(3, 1, 1)
plt.plot(sim_time, sim_theta_D_joint, label='Sim Distal Joint', color='blue')
plt.plot(sim_time, real_theta_D_joint, label='Real Distal Joint', linestyle='--', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Angle (Deg)')
plt.legend()
plt.title('Distal Joint angle Comparison Real VS Sim')

plt.subplot(3, 1, 2)
plt.plot(sim_time, sim_theta_P_joint, label='Sim Proximal Joint', color='blue')
plt.plot(sim_time, real_theta_P_joint, label='Real Proximal Joint', linestyle='--', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Angle (Deg)')
plt.legend()
plt.title('Proximal Joint angle Comparison Real VS Sim')

plt.subplot(3, 1, 3)
plt.plot(sim_time, sim_theta_M_joint, label='Sim Metacarpophalangeal Joint', color='blue')
plt.plot(sim_time, real_theta_M_joint, label='Real Theta M Joint', linestyle='--', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Angle (Deg)')
plt.title('Metacarpophalangeal Joint angle Comparison Real VS Sim')

plt.legend()

plt.tight_layout(pad=2.0)
plt.show()