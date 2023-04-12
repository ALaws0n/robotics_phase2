#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import *
import tf2_geometry_msgs

# import plan messages
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import SphereParams
from std_msgs.msg import Bool

Tool_pose = Twist()
Ball_Point = tf2_geometry_msgs.PointStamped()
Ball_Point.header.frame_id = 'camera_color_optical_frame'

valid_sphere_params = False
valid_tool_pose = False
first_reading = True
generate_new_plan = True

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
		Tool_pose.linear.x = data.linear.x
		Tool_pose.linear.y = data.linear.y
		Tool_pose.linear.z = data.linear.z	
		Tool_pose.angular.x = data.angular.x
		Tool_pose.angular.y = data.angular.y
		Tool_pose.angular.z = data.angular.z
	
	valid_tool_pose = True
	first_reading = False
	
def receive_generate_signal(data):
	global generate_new_plan
	
	generate_new_plan = data.data
	
def create_waypoint(x, y, z, roll, pitch, yaw):

	waypoint = Twist()
	
	waypoint.linear.x = x
	waypoint.linear.y = y
	waypoint.linear.z = z
	
	waypoint.angular.x = roll
	waypoint.angular.y = pitch
	waypoint.angular.z = yaw
	
	return waypoint
	
def generate_plan(ball_pos, tool_pos):
	plan = Plan()
	
	# Where we start
	plan.points.append(tool_pos)
	# Above the ball
	above_ball = create_waypoint(ball_pos.point.x, ball_pos.point.y, tool_pos.linear.z, tool_pos.angular.x, tool_pos.angular.y, tool_pos.angular.z)
	plan.points.append(above_ball)
	# Pickup position
	pickup_pos = create_waypoint(ball_pos.point.x, ball_pos.point.y, ball_pos.point.z, tool_pos.angular.x, tool_pos.angular.y, tool_pos.angular.z)
	plan.points.append(pickup_pos)
	# Back to above ball
	plan.points.append(above_ball)
	# Above the drop point
	above_drop = create_waypoint(ball_pos.point.x, ball_pos.point.y - 0.30, tool_pos.linear.z, tool_pos.angular.x, tool_pos.angular.y, tool_pos.angular.z)
	plan.points.append(above_drop)
	# Drop position
	drop_pos = create_waypoint(above_drop.linear.x, above_drop.linear.y, ball_pos.point.z, tool_pos.angular.x, tool_pos.angular.y, tool_pos.angular.z) 
	plan.points.append(drop_pos)
	# Go back to above drop
	plan.points.append(above_drop)
	
	
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
	# Subscribe to plan generation node
	plan_gen_sub = rospy.Subscriber('/generate_plan', Bool, receive_generate_signal)
	# Publisher for plan
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	
	# 10hz loop rate
	loop_rate = rospy.Rate(10)
	
	
	
	while not rospy.is_shutdown():
	
		if valid_sphere_params and valid_tool_pose:
		
			Ball_Point.header.stamp = rospy.get_rostime()
		
			Ball_Point_Base = tfBuffer.transform(Ball_Point, 'base', rospy.Duration(1.0))
			
			if generate_new_plan:
				print("generating a plan")
			
				plan = generate_plan(Ball_Point_Base, Tool_pose)
				
				generate_new_plan = False
			
			plan_pub.publish(plan)
			
			loop_rate.sleep()
			
		
			
			
		
	
