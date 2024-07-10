# #!/usr/bin/env python
# import rospy
# from bionic_hand.msg import FingerPos

# class FingerPositionSubscriber:
#     def __init__(self):
#         # Initialize the ROS node
#         rospy.init_node('finger_position_subscriber', anonymous=True)

#         # Variable to hold the position
#         self.finger_position = None

#         # Set up the subscriber
#         # The callback function is called every time a message is received
#         self.subscriber = rospy.Subscriber("Finger_Position", FingerPos, self.update_finger_position)

#     def update_finger_position(self, msg):
#         """Callback function to handle incoming messages."""
#         self.finger_position = msg
#         # rospy.loginfo(f"Updated finger position: {msg}")
#         return self.finger_position

#     def spin(self):
#         """Keep the node running so it can keep receiving messages."""
#         rospy.spin()

# # if __name__ == '__main__':
# #     fps = FingerPositionSubscriber()
# #     fps.spin()
