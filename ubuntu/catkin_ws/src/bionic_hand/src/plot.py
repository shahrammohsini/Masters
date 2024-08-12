#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from bionic_hand.msg import FingerPos

# Lists to store the data from topics
theta_M = []
theta_P = []
theta_D = []
timestamps = []

def position_callback(msg):
    # This function is called every time data is published on the /Updated_Finger_Position topic.
    current_time = rospy.get_time() - start_time
    timestamps.append(current_time)
    theta_M.append(msg.theta_M)
    theta_P.append(msg.theta_P)
    theta_D.append(msg.theta_D)

def listener():
    global start_time
    rospy.init_node('posiiton_plotter_node', anonymous=True)
    start_time = rospy.get_time()

    # Subscribe to the topic
    rospy.Subscriber("/Updated_Finger_Position", FingerPos, position_callback)

    # Spin until the simulation is terminated
    rospy.spin()

    # Once the simulation ends (node shuts down), plot the collected data
    plot_data()

def plot_data():
    """Function to plot the data after the simulation has ended."""
    plt.figure()
    plt.plot(timestamps, theta_M, label='Theta_M')
    plt.plot(timestamps, theta_P, label='Theta_P')
    plt.plot(timestamps, theta_D, label='Theta_D')
    plt.xlabel('Time')
    plt.ylabel('Position Data')
    plt.title('Finger Position Over Time')
    plt.gcf().autofmt_xdate()  # Beautify the x-labels for timestamps
    plt.legend()
    plt.show()

if __name__ == '__main__':
    listener()
