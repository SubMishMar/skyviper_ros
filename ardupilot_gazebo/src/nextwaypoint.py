#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from mdpOp import getWayPoint

def callback(data):
    pose = data.pose
    current_x = pose.position.x 
    current_y = pose.position.y 
    wp = getWayPoint(current_x, current_y)
    print(wp)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/mavros/mocap/pose", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
