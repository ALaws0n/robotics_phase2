#!/usr/bin/env python3

import rospy
import math
# import the messages for reading the joint positions and sending joint commands
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header

def ur5e_initialization():
	# publisher for main initialization point
	init_pub = rospy.Publisher('/pos_joint_traj_controller/command', JointTrajectory, queue_size = 10)
	loop_rate = rospy.Rate(10)
	# define variables
	pos_cmd = JointTrajectory()
	pos_cmd_point = JointTrajectoryPoint()
	# Complete message template
	pos_cmd.joint_names.append('elbow_joint')
	pos_cmd.joint_names.append('shoulder_lift_joint')
	pos_cmd.joint_names.append('shoulder_pan_joint')
	pos_cmd.joint_names.append('wrist_1_joint')
	pos_cmd.joint_names.append('wrist_2_joint')
	pos_cmd.joint_names.append('wrist_3_joint')
	# initialize the position command to zero
	for joint_no in range(6):
		pos_cmd_point.positions.append(0.0)
	# set the ideal time to destination
	pos_cmd_point.time_from_start = rospy.Duration(1.0) # here one second 
	# set initial joint positions
	pos_cmd_point.positions[0] = math.pi/2
	pos_cmd_point.positions[1] = -math.pi/3
	pos_cmd_point.positions[3] = -(2*math.pi)/3
	pos_cmd_point.positions[4] = -math.pi/2
	# add trajectory point to the command
	pos_cmd.points.append(pos_cmd_point)
	# define message header
	header = Header()
	while not rospy.is_shutdown():
		header.stamp = rospy.Time.now()
		pos_cmd.header = header
		init_pub.publish(pos_cmd)
		loop_rate.sleep()

if __name__ == '__main__':
	# initalize the node
	rospy.init_node('lawson_phase1', anonymous = True)
	# call ur5e initialization method
	ur5e_initialization()
