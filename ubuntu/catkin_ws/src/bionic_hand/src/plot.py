#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from bionic_hand.msg import FingerPos

# Lists to store the data from topics for each finger
index_theta_M = []
index_theta_P = []
index_theta_D = []
middle_theta_M = []
middle_theta_P = []
middle_theta_D = []
thumb_theta_M = []
thumb_theta_P = []
thumb_theta_D = []
timestamps = []

def position_callback(msg):
    # This function is called every time data is published on the /updated_Finger_Position topic.
    current_time = rospy.get_time() - start_time
    timestamps.append(current_time)

    # Append data for index finger
    index_theta_M.append(msg.index.theta_M)
    index_theta_P.append(msg.index.theta_P)
    index_theta_D.append(msg.index.theta_D)

    # Append data for middle finger
    middle_theta_M.append(msg.middle.theta_M)
    middle_theta_P.append(msg.middle.theta_P)
    middle_theta_D.append(msg.middle.theta_D)

    # Append data for thumb
    thumb_theta_M.append(msg.thumb.theta_M)
    thumb_theta_P.append(msg.thumb.theta_P)
    thumb_theta_D.append(msg.thumb.theta_D)

def listener():
    global start_time
    rospy.init_node('position_plotter_node', anonymous=True)
    start_time = rospy.get_time()

    # Subscribe to the topic
    rospy.Subscriber("/updated_Finger_Positions", FingerPos, position_callback)

    # Spin until the simulation is terminated
    rospy.spin()

    # Once the simulation ends (node shuts down), plot the collected data
    plot_data()

def plot_data():
    """Function to plot the data after the simulation has ended."""
    plt.figure(figsize=(12, 12))

    # Plot for index finger
    plt.subplot(3, 1, 1)
    plt.plot(timestamps, index_theta_M, label='Index Theta_M')
    plt.plot(timestamps, index_theta_P, label='Index Theta_P')
    plt.plot(timestamps, index_theta_D, label='Index Theta_D')
    plt.xlabel('Time (s)')
    plt.ylabel('Position Data')
    plt.title('Index Finger Position Over Time')
    plt.legend()

    # Plot for middle finger
    plt.subplot(3, 1, 2)
    plt.plot(timestamps, middle_theta_M, label='Middle Theta_M')
    plt.plot(timestamps, middle_theta_P, label='Middle Theta_P')
    plt.plot(timestamps, middle_theta_D, label='Middle Theta_D')
    plt.xlabel('Time (s)')
    plt.ylabel('Position Data')
    plt.title('Middle Finger Position Over Time')
    plt.legend()

    # Plot for thumb
    plt.subplot(3, 1, 3)
    plt.plot(timestamps, thumb_theta_M, label='Thumb Theta_M')
    plt.plot(timestamps, thumb_theta_P, label='Thumb Theta_P')
    plt.plot(timestamps, thumb_theta_D, label='Thumb Theta_D')
    plt.xlabel('Time (s)')
    plt.ylabel('Position Data')
    plt.title('Thumb Position Over Time')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    listener()
