#!/usr/bin/env python3

#the line above is called the shebang line.  It tells the operating system which interperter to uses to execute teh script. I'm telling it to use python3
import rospy

if __name__ == '__main__':

    rospy.init_node("test_node") # initialize this file as a node. "test_node" is the name of the node

    # rospy.loginfo("Hello from test node") # This prints in the terminal as a log
    # rospy.logwarn("This is a warning") # This is how you do a warning
    # rospy.logerr("This is an error") # How you do errors

    # rospy.sleep(1.0) # Wait a sec
    # rospy.loginfo("End of program")


    rospy.loginfo("Test node has been started.")

    rate = rospy.Rate(10) #10 Hz (0.1 sec)

    while not rospy.is_shutdown(): # While the rospy is running. is_shutdown() is looking for a shutdown request. If the node is killed it will become true
        rospy.loginfo("Hello")
        rate.sleep()
