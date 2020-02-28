#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Vector3Stamped
from dwm1001.msg  import anchor

import sys
import json
from collections import deque

import time
pose = PoseStamped()

def call_bk(data):
    global count
    global pose
    anch = data
    anch.header.seq = path.header.seq + 1
    path.header.frame_id = pose.header.frame_id
    path.header.stamp = rospy.Time.now()
    pose.header.stamp = path.header.stamp
    pose.pose.position.x = anch.vector.x
    pose.pose.position.y = anch.vector.y
    pose.pose.position.z = anch.vector.z
    path.poses.append(pose)

    count = count +1

    if(count>max_append):
	path.poses.pop(0)

    pub.publish(path)
    return path

def callback(data):
    global pose
    pose = data
 #    global count
 #    pose.header.seq = path.header.seq + 1
 #    path.header.frame_id = data.header.frame_id
 #    path.header.stamp = rospy.Time.now()
 #    pose.header.stamp = path.header.stamp
 #    path.poses.append(pose)

 #    count = count +1

 #    if(count>max_append):
	# path.poses.pop(0)

 #    pub.publish(path)
 #    return path

if __name__ == '__main__':
    global count

    rospy.init_node('path_plotter')

    if not rospy.has_param("max_list_append"):
	rospy.logwarn("The parameter max_list_append doesn't exist")

    max_append = rospy.set_param("~max_list_append",1000)
    max_append = 50000

    if not (max_append > 0):
	rospy.logwarn('The parameter max_list_append is not correct')
	sys.exit()
    count=0
    pub = rospy.Publisher('/path',Path, queue_size=1)

    path=Path()
    msg = PoseStamped()

    msg = rospy.Subscriber('/vrpn_client_node/vicon/pose',PoseStamped,callback)
    msg_2 = rospy.Subscriber('/Filtered_pose',Vector3Stamped,call_bk)

    rate = rospy.Rate(30)

    try:
	while not rospy.is_shutdown():
	    rate.sleep()
    except rospy.ROSInterruptException:
	    pass
