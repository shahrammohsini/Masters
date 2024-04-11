#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def pose_callback(msg): #call back fucntion processes the data we recieve
    rospy.loginfo(msg) #Prints everything in msg
    #rospy.loginfo("(" + str(msg.x) + ", " + str(msg.y) + ")") # Prints x and y in this format

if __name__ == '__main__':
    rospy.init_node("turtle_pose_subscriber")

    
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback) #subscribe to /turtle1/pose to get data type Pose from it. The data is sent to our call back function

    rospy.loginfo("Node has been started")

    rospy.spin() # Creates an infinat loop in another thread. It'll only do anything when a message is published to the topic. Similar to an interupt. Loop ends when terminal is closed
