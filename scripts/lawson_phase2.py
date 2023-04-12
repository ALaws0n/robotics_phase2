#!/usr/bin/env python3

# Import necessary components
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import *
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import SphereParams
from std_msgs.msg import Bool

# Create message for Tool tip position and position of Ball in camera
Tool_pose = Twist()
Ball_Pos_Cam = tf2_geometry_msgs.PointStamped()
Ball_Pos_Cam.header.frame_id = 'camera_color_optical_frame'

# Initial Boolean values for control logic
valid_sphere_params = False
valid_tool_pose = False
# For first reading of tooltip position
first_reading = True
# Safety measure booleans
generate_new_plan = True
hold_movement = True

def receive_sphere_params(data):
	global Ball_Pos_Cam
	global valid_sphere_params
	
	# Store x,y,z coordinates of the ball w.r.t the camera
	Ball_Pos_Cam.point.x = data.xc
	Ball_Pos_Cam.point.y = data.yc
	Ball_Pos_Cam.point.z = data.zc
	# Flip to true since we have received data
	valid_sphere_params = True
	
def receive_tool_pose(data):
	global valid_tool_pose
	global Tool_pose
	global first_reading
	
	# Grab tooltip position values from initialization point
	if first_reading:
		Tool_pose.linear.x = data.linear.x
		Tool_pose.linear.y = data.linear.y
		Tool_pose.linear.z = data.linear.z	
		Tool_pose.angular.x = data.angular.x
		Tool_pose.angular.y = data.angular.y
		Tool_pose.angular.z = data.angular.z
	
	# Flip to true since we have received data
	valid_tool_pose = True
	# Flip to false as we no longer want to read where the tooltip is
	first_reading = False
	
def receive_generate_signal(data):
	global generate_new_plan
	# Change generate_new_plan value to whatever is received from the /generate_plan topic
	generate_new_plan = data.data
	
def receive_move_signal(data):
	global hold_movement
	# Change hold_movement value to whatever is received from the /movement topic
	hold_movement = data.data
	
def create_waypoint(x, y, z, roll, pitch, yaw):
	# Create new twist message, set values, then return the message
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
	above_drop = create_waypoint(ball_pos.point.x, ball_pos.point.y - 0.25, tool_pos.linear.z, tool_pos.angular.x, tool_pos.angular.y, tool_pos.angular.z)
	plan.points.append(above_drop)
	# Drop position
	drop_pos = create_waypoint(above_drop.linear.x, above_drop.linear.y, ball_pos.point.z, tool_pos.angular.x, tool_pos.angular.y, tool_pos.angular.z) 
	plan.points.append(drop_pos)
	# Go back to above drop
	plan.points.append(above_drop)
	
	# Return plan for publishing
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
	# Subscribe to movement node
	move_sub = rospy.Subscriber('/movement', Bool, receive_move_signal)
	# Publisher for plan
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	
	# 10hz loop rate
	loop_rate = rospy.Rate(10)
	
	
	
	while not rospy.is_shutdown():
		
		# Do nothing until we have received valid data
		if valid_sphere_params and valid_tool_pose:
			
			# Set header stamp for Ball_Pos_Cam
			Ball_Pos_Cam.header.stamp = rospy.get_rostime()
			# Transform ball coordinates in the camera frame to ball coordinates w.r.t the base of the ur5e
			Ball_Pos_Base = tfBuffer.transform(Ball_Pos_Cam, 'base', rospy.Duration(1.0))
			
			# Generate a path plan
			if generate_new_plan:
				print("Generating a plan...")
				plan = generate_plan(Ball_Pos_Base, Tool_pose)
				# Do not generate a plan again, unless the plan_gen_sub receives a value of True
				generate_new_plan = False
			
			# Hold movement until the move_sub receives the signal to begin movement
			if hold_movement:
				print("Holding movement...")
			else:
				# Publish the plan when ready
				plan_pub.publish(plan)
				print("Moving....")
		
			loop_rate.sleep()
			
		
			
			
		
	
