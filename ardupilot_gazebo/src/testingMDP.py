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
from mdpOp import getWayPoint

class MavController:
    """
    A simple object to help interface with mavros 
    """
    def __init__(self):

        rospy.init_node("mav_control_node")
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
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

        self.waypoint = np.zeros((2, 1))
        self.commandToLand = False
        # self.all_waypoints = [(self.dx, self.dx, self.dz)]

    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose

        current_x = self.pose.position.x
        current_y = self.pose.position.y
        current_z = self.pose.position.z

        wp = getWayPoint(current_x+0.25, current_y+0.25)
        self.waypoint[0] = wp[0] - 0.25
        self.waypoint[1] = wp[1] - 0.25
        print(self.waypoint)
        pose = Pose()
        pose.position.x = self.waypoint[0]
        pose.position.y = self.waypoint[1]
        pose.position.z = 1.2
    
        if self.waypoint[0] == 0 and self.waypoint[1] == 0:
            err_x = current_x - self.waypoint[0]
            err_y = current_y - self.waypoint[1]
            err = np.sqrt(err_x*err_x + err_y*err_y)
            if err <= 0.15:
                self.land()
                self.commandToLand = True
            else:
                if self.commandToLand is False:
                    self.goto(pose)
                
        else:
            self.goto(pose);

        # print([current_x, current_y, current_z])

    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work. 
        """
        mode_resp = self.mode_service(custom_mode="4")
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
    rospy.spin()
    # print("Landing")
    # c.land()

if __name__=="__main__":
    simple_demo()
