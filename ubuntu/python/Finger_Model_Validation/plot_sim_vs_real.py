import pandas as pd
import matplotlib.pyplot as plt

# Load the simulation data
sim_data = pd.read_csv(r'C:\Users\100655277\Documents\GitHub\Masters\ubuntu\python\Finger_Model_Validation\Validation_Data\finger_validation_sim_data.csv')
real_data = pd.read_csv(r'C:\Users\100655277\Documents\GitHub\Masters\ubuntu\python\Finger_Model_Validation\Validation_Data\finger_Validation_Real_Data.csv')

# extract the columns for joints from real data
real_theta_D_joint = real_data['Joint_3']
real_theta_P_joint = real_data['Joint_2']
real_theta_M_joint = real_data['Joint_1']
# extract the columns for joints from sim data
sim_theta_D_joint = sim_data['theta_D_joint']
sim_theta_P_joint = sim_data['theta_P_joint']
sim_theta_M_joint = sim_data['theta_M_joint']

real_time = real_data['time']
sim_time = sim_data['time']


# print("sim: ", sim_theta_P_joint)

# Plot the joints for simulation data
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(sim_time, sim_theta_D_joint, label='Sim Theta D Joint')
plt.plot(real_time, real_theta_D_joint, label='Real Theta D Joint', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Theta D Joint')
plt.legend()
plt.title('Theta D Joint Comparison')

plt.subplot(3, 1, 2)
plt.plot(sim_time, sim_theta_P_joint, label='Sim Theta P Joint')
plt.plot(real_time, real_theta_P_joint, label='Real Theta P Joint', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Theta P Joint')
plt.legend()
plt.title('Theta P Joint Comparison')

plt.subplot(3, 1, 3)
plt.plot(sim_time, sim_theta_M_joint, label='Sim Theta M Joint')
plt.plot(real_time, real_theta_M_joint, label='Real Theta M Joint', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Theta M Joint')
plt.title('Theta M Joint Comparison')

plt.legend()

plt.tight_layout(pad=2.0)
plt.show()