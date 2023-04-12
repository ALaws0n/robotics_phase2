#!/usr/bin/env python3

# Import important components
import rospy
from std_msgs.msg import Bool

# Create boolean message
move_signal = Bool()

if __name__ == '__main__':
	# Initialize node
	rospy.init_node('lawson_move_signal', anonymous = True)
	# Create Publisher
	move_pub = rospy.Publisher('/movement', Bool, queue_size = 1)
	# Set boolean value to False
	move_signal.data = False
	# Define loop rate
	loop_rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		# Display value of boolean message
		print(move_signal.data)
		# Publish boolean signal
		move_pub.publish(move_signal)
		
		loop_rate.sleep()
