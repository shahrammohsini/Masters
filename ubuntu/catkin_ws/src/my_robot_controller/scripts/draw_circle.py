#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist # If you add a package make sure you add it to package.xml file too
if __name__ == '__main__':
    rospy.init_node("draw_circle")
    rospy.loginfo("Node has been started")

    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10) # Create a publisher. "/turtle1/cmd_vel" is the name of the topic we want to publish to. "Twist" is the message data type the topic uses. Can be found with the command rostopic info /trutle1/cmd_vel
    #Note: publishing to a non existing topic will create that topic
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = Twist() # Create an object of class Twist called msg

        # only need to change these two values to turn in a circle
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        pub.publish(msg) # publish the message
        rate.sleep()

