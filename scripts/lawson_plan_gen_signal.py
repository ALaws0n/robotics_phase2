#!/usr/bin/env python3

# Import important components
import rospy
from std_msgs.msg import Bool

# Create boolean message
generate_plan = Bool()

if __name__ == '__main__':
	# Initialize node
	rospy.init_node('lawson_plan_gen_signal', anonymous = True)
	# Create publisher
	signal_pub = rospy.Publisher('/generate_plan', Bool, queue_size = 1)
	# Set boolean value to True
	generate_plan.data = True
	# Define loop rate
	loop_rate = rospy.Rate(1)
	
	while not rospy.is_shutdown():
		# Display value of boolean message
		print(generate_plan.data)
		# Publish boolean signal
		signal_pub.publish(generate_plan)
		
		loop_rate.sleep()
