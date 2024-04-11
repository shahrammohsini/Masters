#!/usr/bin/env python3

'''create a closed loop communication between publisher and subscriber nodes to prevent the turtle from hitting the walls'''

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

# This function only activates when a message is published in the "/turtle1/pose" topic because its called in the subscriber.
def pose_callback(msg):
    cmd = Twist()

    #if we're about to hit the wall. Turn.
    if msg.x > 9.0 or msg.x < 2.0 or msg.y > 9.0 or msg.y < 2.0:
        cmd.linear.x = 1.0
        cmd.angular.z = 1.4
    #otherwise go straight
    else:
        cmd.linear.x = 5.0
        cmd.angular.z = 0.0

    pub.publish(cmd) # Publishing inside of this function allows us to only publish when a message is recived from the topic

if __name__ == "__main__":
    rospy.init_node("turtle_controller")
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10)
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    rospy.loginfo("Node has been started")

    rospy.spin()