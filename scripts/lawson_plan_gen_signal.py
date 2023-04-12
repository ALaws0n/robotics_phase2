#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

generate_plan = Bool()

if __name__ == '__main__':
	rospy.init_node('lawson_plan_gen_signal', anonymous = True)
	
	signal_pub = rospy.Publisher('/generate_plan', Bool, queue_size = 1)
	
	generate_plan.data = True
	
	loop_rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
	
		print(generate_plan.data)
	
		signal_pub.publish(generate_plan)
