#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs

# import plan messages
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import SphereParams

Tool_pose = Twist()
Ball_Point = tf2_geometry_msgs.PointStamped()
Ball_Point.header.frame_id = 'camera_color_optical_frame'

valid_sphere_params = False
valid_tool_pose = False
first_reading = True

def receive_sphere_params(data):
	global Ball_Point
	global valid_sphere_params
	
	Ball_Point.point.x = data.xc
	Ball_Point.point.y = data.yc
	Ball_Point.point.z = data.zc
	
	valid_sphere_params = True
	
def receive_tool_pose(data):
	global valid_tool_pose
	global Tool_pose
	global first_reading
	
	if first_reading:

		Tool_pose.linear.z = data.linear.z	
		Tool_pose.angular.x = data.angular.x
		Tool_pose.angular.y = data.angular.y
		Tool_pose.angular.z = data.angular.z
	
	valid_tool_pose = True
	first_reading = False
	

def generate_plan(ball_pos, tool_pos):
	plan = Plan()
	
	start_pos = Twist()
	start_pos.linear.x = ball_pos.point.x
	start_pos.linear.y = ball_pos.point.y
	start_pos.linear.z = tool_pos.linear.z
	start_pos.angular.x = tool_pos.angular.x
	start_pos.angular.y = tool_pos.angular.y
	start_pos.angular.z = tool_pos.angular.z
	
	plan.points.append(start_pos)
	
	pickup_pos = Twist()
	pickup_pos.linear.x = ball_pos.point.x
	pickup_pos.linear.y = ball_pos.point.y
	# Compensate for length of gripper
	pickup_pos.linear.z = 0.16
	pickup_pos.angular.x = tool_pos.angular.x
	pickup_pos.angular.y = tool_pos.angular.y
	pickup_pos.angular.z = tool_pos.angular.z
	
	plan.points.append(pickup_pos)
	
	waypoint = Twist()
	waypoint.linear.x = (ball_pos.point.x + 0.12)
	waypoint.linear.y = ball_pos.point.y
	waypoint.linear.z = tool_pos.linear.z
	waypoint.angular.x = tool_pos.angular.x
	waypoint.angular.y = tool_pos.angular.y
	waypoint.angular.z = tool_pos.angular.z
	
	plan.points.append(waypoint)
	
	drop_pos = Twist()
	drop_pos.linear.x = waypoint.linear.x
	drop_pos.linear.y = waypoint.linear.y
	# Compensate for length of gripper
	drop_pos.linear.z = 0.16
	drop_pos.angular.x = tool_pos.angular.x
	drop_pos.angular.y = tool_pos.angular.y
	drop_pos.angular.z = tool_pos.angular.z
	
	plan.points.append(drop_pos)
	
	return plan
	
	
if __name__ == '__main__':
	# Initialize node
	rospy.init_node('lawson_phase2', anonymous = True)
	# add ros transformer listener
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	# Subscribe to filtered ball parameters
	sphere_params_sub = rospy.Subscriber('/sphere_params', SphereParams, receive_sphere_params)
	# Subscribe to ur5e toolpose to get angular joint values
	tool_pos_sub = rospy.Subscriber('/ur5e/toolpose', Twist, receive_tool_pose)
	# Publisher for plan
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	
	# 10hz loop rate
	loop_rate = rospy.Rate(10)
	
	
	
	while not rospy.is_shutdown():
	
	
		#print('nothing is valid')
		print(first_reading)

		if valid_sphere_params and valid_tool_pose:
		
			Ball_Point.header.stamp = rospy.get_rostime()
		
			Ball_Point_Base = tfBuffer.transform(Ball_Point, 'base', rospy.Duration(1.0))
			
			plan = generate_plan(Ball_Point_Base, Tool_pose)
			
			plan_pub.publish(plan)
			
			loop_rate.sleep()
			
		
			#print('Ball in camera frame: x= ', format(Ball_Point.point.x, '.3f'), ' y= ', format(Ball_Point.point.y, '.3f'), ' z= ', format(Ball_Point.point.z, '.3f'))
			
			#print('Ball in base frame: x= ', format(Ball_Point_Base.point.x, '.3f'), ' y= ', format(Ball_Point_Base.point.y, '.3f'), ' z= ', format(Ball_Point_Base.point.z, '.3f'))
			
		
	
