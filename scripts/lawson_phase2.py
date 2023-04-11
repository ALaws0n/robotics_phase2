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


Ball_Point = tf2_geometry_msgs.PointStamped()
Ball_Point.header.frame_id = 'camera_color_optical_frame'

# Value for initial height of tool
linear_z = 0.0
# Values for initial angular values of tool
angular_x = 0.0
angular_y = 0.0
angular_z = 0.0

valid_sphere_params = False
valid_tool_pose = False

def receive_sphere_params(data):
	global Ball_Point
	global valid_sphere_params
	
	Ball_Point.point.x = data.xc
	Ball_Point.point.y = data.yc
	Ball_Point.point.z = data.zc
	
	valid_sphere_params = True
	
def receive_tool_pose(data):
	global angular_x
	global angular_y
	global angular_z
	global linear_z
	
	linear_z = data.linear.z	
	angular_x = data.angular.x
	angular_y = data.angular.y
	angular_z = data.angular.z
	
	valid_tool_pose = True
	

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
	
	plan = Plan()
	
	while not rospy.is_shutdown():
	
	
		#print('nothing is valid')

		if valid_sphere_params: #and valid_tool_pose:
		
			#Ball_Point.header.stamp = rospy.get_rostime()
		
			Ball_Point_Base = tfBuffer.transform(Ball_Point, 'base', rospy.Duration(1.0))
			
		
			print('Ball in camera frame: x= ', format(Ball_Point.point.x, '.3f'), ' y= ', format(Ball_Point.point.y, '.3f'), ' z= ', format(Ball_Point.point.z, '.3f'))
			
			print('Ball in base frame: x= ', format(Ball_Point_Base.point.x, '.3f'), ' y= ', format(Ball_Point_Base.point.y, '.3f'), ' z= ', format(Ball_Point_Base.point.z, '.3f'))
			
		
	
