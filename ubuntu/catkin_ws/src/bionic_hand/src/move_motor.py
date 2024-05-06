#!/usr/bin/env python3

import rospy
from bionic_hand.msg import *

if __name__ == '__main__':
    rospy.init_node("move_motor") #Create a new node
    rospy.loginfo("Node has been started")

    # pub = rospy.Publisher("/set_position", SetPosition, queue_size=10) #Create a publisher
    pub = rospy.Publisher("/set_pwm", SetPWM, queue_size=2) #Create a publisher


    rate = rospy.Rate(100) #publish at 2 HZ (0.5 sec)

    while not rospy.is_shutdown():

        msg = SetPWM() #create an object of class SetPosition

        msg.id = 1 #set id
        msg.pwm = int(input("Enter a new pwm: ")) #set position
        
        pub.publish(msg) #publish the message

        rate.sleep()


