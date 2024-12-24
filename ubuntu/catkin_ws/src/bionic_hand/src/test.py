import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

# Determine the start and end index for each joint
def get_index_range(data, column, start_threshold, start_steady_state, end_value):
    """
    Finds the range of indices for a joint's data from start to maximum point.

    Args:
        data (pd.DataFrame): The input data.
        column (str): The column name to analyze.
        start_threshold (float): Threshold for the starting steady state.
        start_steady_state (float): Value indicating when the joint starts moving.
        end_value (float): The maximum point for the joint.

    Returns:
        tuple: (start_index, end_index)
    """
    start_idx = 0
    end_idx = len(data) - 1  # Default to the last index
    
    # Find the start index
    for idx, value in enumerate(data[column]):
        if abs(value - start_steady_state) <= start_threshold:
            start_idx = idx
            break

    # Find the end index
    for idx, value in enumerate(data[column][start_idx:], start=start_idx):
        if value >= end_value:
            end_idx = idx
            break

    return start_idx, end_idx

# Load data from the CSV file
input_file = "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/Data/open_loop_data.csv"
data = pd.read_csv(input_file)

# Ensure the required columns exist
required_columns = ['time', 'Joint_3', 'Joint_2', 'Joint_1']
for col in required_columns:
    if col not in data.columns:
        raise ValueError(f"Missing required column: {col}")

# Define thresholds and steady-state values
joint_3_start_threshold = 1.0
joint_3_steady_state = 0  # Starts moving immediately
joint_3_max_value = 80

joint_2_start_threshold = 1.0
joint_2_steady_state = 80  # Starts after Joint 3 reaches 80
joint_2_max_value = 103

joint_1_start_threshold = 1.0
joint_1_steady_state = 103  # Starts after Joint 2 reaches 103
joint_1_max_value = 80

# Get index ranges for each joint
start_idx_joint_3, end_idx_joint_3 = get_index_range(data, 'Joint_3', joint_3_start_threshold, joint_3_steady_state, joint_3_max_value)
start_idx_joint_2, end_idx_joint_2 = get_index_range(data, 'Joint_2', joint_2_start_threshold, joint_2_steady_state, joint_2_max_value)
start_idx_joint_1, end_idx_joint_1 = get_index_range(data, 'Joint_1', joint_1_start_threshold, joint_1_steady_state, joint_1_max_value)

# Extract relevant data for each joint
time_joint_3 = data['time'][start_idx_joint_3:end_idx_joint_3 + 1].values
theta_D = data['Joint_3'][start_idx_joint_3:end_idx_joint_3 + 1].values

time_joint_2 = data['time'][start_idx_joint_2:end_idx_joint_2 + 1].values
theta_P = data['Joint_2'][start_idx_joint_2:end_idx_joint_2 + 1].values

time_joint_1 = data['time'][start_idx_joint_1:end_idx_joint_1 + 1].values
theta_M = data['Joint_1'][start_idx_joint_1:end_idx_joint_1 + 1].values

# Create new time arrays with exactly 100 points for interpolation
new_time_joint_3 = np.linspace(time_joint_3[0], time_joint_3[-1], 100)
new_time_joint_2 = np.linspace(time_joint_2[0], time_joint_2[-1], 100)
new_time_joint_1 = np.linspace(time_joint_1[0], time_joint_1[-1], 100)

# Interpolate data for each joint
interpolate_theta_D = interp1d(time_joint_3, theta_D, kind='linear')
interpolate_theta_P = interp1d(time_joint_2, theta_P, kind='linear')
interpolate_theta_M = interp1d(time_joint_1, theta_M, kind='linear')

new_theta_D = interpolate_theta_D(new_time_joint_3)
new_theta_P = interpolate_theta_P(new_time_joint_2)
new_theta_M = interpolate_theta_M(new_time_joint_1)

# Combine results into a single DataFrame
interpolated_data = pd.DataFrame({
    'theta_D': new_theta_D,
    'theta_P': new_theta_P,
    'theta_M': new_theta_M,
    'time_joint_1': new_time_joint_1
})

# Save the interpolated data
output_file = "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/src/Data/open_loop_data_interpolated.csv"
interpolated_data.to_csv(output_file, index=False)
