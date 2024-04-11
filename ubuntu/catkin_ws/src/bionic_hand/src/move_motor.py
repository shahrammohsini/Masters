#!/usr/bin/env python3

import rospy
from dynamixel_sdk_examples.msg import *

if __name__ == '__main__':
    rospy.init_node("move_motor") #Create a new node
    rospy.loginfo("Node has been started")

    pub = rospy.Publisher("/set_position", SetPosition, queue_size=10) #Create a publisher

    rate = rospy.Rate(2) #publish at 2 HZ (0.5 sec)

    while not rospy.is_shutdown():

        msg = SetPosition() #create an object of class SetPosition

        msg.id = 1 #set id
        msg.position = int(input("Enter a new position: ")) #set position
        
        pub.publish(msg) #publish the message

        rate.sleep()


