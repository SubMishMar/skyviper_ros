#!/usr/bin/env python

##
#
# Control a MAV via mavros
#
##

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist
import numpy as np
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
import time

class MavController:
	"""
	A simple object to help interface with mavros 
	"""
	def __init__(self):

		rospy.init_node("mav_control_node")
		rospy.Subscriber("/mavros/mocap/pose", PoseStamped, self.pose_callback)
		rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)

		self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
		self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
		self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

		# mode 0 = STABILIZE
		# mode 4 = GUIDED
		# mode 9 = LAND
		self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

		self.rc = RCIn()
		self.pose = Pose()
		self.timestamp = rospy.Time()

		self.next_wp = 0
		self.wellCount = 0
		self.dx = 2
		self.dy = 2
		self.dz = 1.2
		# self.all_waypoints = [(0, 0, self.dz),
		# 					  (self.dx, 0, self.dz),
		# 					  (self.dx, self.dx, self.dz),
		# 					  (0, self.dx, self.dz),
		# 					  (0, 0, self.dz),
		# 					  (self.dx, 0, self.dz),
		# 					  (self.dx, self.dx, self.dz),
		# 					  (0, self.dx, self.dz),
		# 					  (0, 0, self.dz),
		# 					  (self.dx, 0, self.dz),
		# 					  (self.dx, self.dx, self.dz),
		# 					  (0, self.dx, self.dz),
		# 					  (0, 0, self.dz)]
		self.all_waypoints = [(self.dx, self.dx, self.dz)]
		self.reachedInitPos = False
		self.sleepCommandGiven = False
	def rc_callback(self, data):
		"""
		Keep track of the current manual RC values
		"""
		self.rc = data

	def pose_callback(self, data):
		"""
		Handle local position information
		"""
		# self.timestamp = data.header.stamp
		# self.pose = data.pose
		# current_x = self.pose.position.x
		# current_y = self.pose.position.y
		# current_z = self.pose.position.z
		# e_x = current_x - self.all_waypoints[self.next_wp][0]
		# e_y = current_y - self.all_waypoints[self.next_wp][1]		
		# print([current_x, current_y, current_z])
		# print("\n")
		# if np.sqrt(e_x*e_x + e_y*e_y) < 0.1 or self.reachedInitPos:
		# 	self.reachedInitPos = True
		# 	print(['At: ', current_x, current_y])
		# 	print('Go to Adj Cell')
		# 	pose = Pose()
		# 	pose.position.x = self.dx + 0.5
		# 	pose.position.y = self.dy + 0.5
		# 	pose.position.z = self.dz
		# 	self.goto(pose)
		# 	print('Sleeping for 10 seconds...')
		# 	if not self.sleepCommandGiven:
		# 		self.sleepCommandGiven = True
		# 		rospy.sleep(10.)
		# 	print('Landing...')			
		# 	self.land()
        pose = Pose()
        pose.position.x = 2
        pose.position.y = 2
        pose.position.z = 1.2
        self.goto(pose)

	def goto(self, pose):
		"""
		Set the given pose as a the next setpoint by sending
		a SET_POSITION_TARGET_LOCAL_NED message. The copter must
		be in GUIDED mode for this to work. 
		"""
		pose_stamped = PoseStamped()
		pose_stamped.header.stamp = self.timestamp
		pose_stamped.pose = pose

		self.cmd_pos_pub.publish(pose_stamped)

	def goto_xyz(self):
		mode_resp = self.mode_service(custom_mode="4")
		pose = Pose()
		pose.position.x = self.all_waypoints[self.next_wp][0]
		pose.position.y = self.all_waypoints[self.next_wp][1]
		pose.position.z = self.all_waypoints[self.next_wp][2]

		self.goto(pose)
		  

	def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
		"""
		Send comand velocities. Must be in GUIDED mode. Assumes angular
		velocities are zero by default. 
		"""
		mode_resp = self.mode_service(custom_mode="4")
		cmd_vel = Twist()
		
		cmd_vel.linear.x = vx
		cmd_vel.linear.y = vy
		cmd_vel.linear.z = vz

		cmd_vel.angular.x = avx
		cmd_vel.angular.y = avy
		cmd_vel.angular.z = avz

		self.cmd_vel_pub.publish(cmd_vel)

	def arm(self):
		"""
		Arm the throttle
		"""
		return self.arm_service(True)
	
	def disarm(self):
		"""
		Disarm the throttle
		"""
		return self.arm_service(False)

	def takeoff(self, height=1.0):
		"""
		Arm the throttle, takeoff to a few feet, and set to guided mode
		"""
		# Set to stabilize mode for arming
		mode_resp = self.mode_service(custom_mode="0")
		self.arm()

		# Set to guided mode 
		mode_resp = self.mode_service(custom_mode="4")

		# Takeoff
		takeoff_resp = self.takeoff_service(altitude=height)

		return takeoff_resp

	def land(self):
		"""
		Set in LAND mode, which should cause the UAV to descend directly, 
		land, and disarm. 
		"""
		resp = self.mode_service(custom_mode="9")
		self.disarm()

def simple_demo():
	"""
	A simple demonstration of using mavros commands to control a UAV.
	"""
	c = MavController()
	rospy.sleep(1)
	# c.takeoff(2.0)
	# rospy.sleep(5)
	print("Waypoint Nav")
	c.goto_xyz()
	rospy.spin()
	# print("Landing")
	# c.land()

if __name__=="__main__":
	simple_demo()

