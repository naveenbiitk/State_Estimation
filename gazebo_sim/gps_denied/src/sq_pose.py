#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from dwm1001.msg  import anchor
import rospy
import numpy as np


flag = False
uav_pose = PoseStamped()
imu_msg = PoseStamped()
poses_pub = rospy.Publisher("/error_pose", PoseStamped, queue_size=10)


def tag_detect_node():
    global poses_pub
    rospy.init_node('Pose_Stamped')
    pose_sub_1 = rospy.Subscriber("/host/imu/data",Imu, groundt_cb)
    pose_sub = rospy.Subscriber("/dwm1001/tag5617",anchor, estimated_cb)

    rospy.spin()

def groundt_cb(msg):
    global imu_msg
    imu_msg.pose.orientation.w = msg.orientation.w;
    imu_msg.pose.orientation.x = msg.orientation.x;
    imu_msg.pose.orientation.y = msg.orientation.y;
    imu_msg.pose.orientation.z = msg.orientation.z;
    flag = True


def estimated_cb(msg):
    global imu_msg,uav_pose

    if(flag == True):
        uav_pose.header.stamp = rospy.Time.now()
        uav_pose.pose.position.x =  msg.vector.x
        uav_pose.pose.position.y =  msg.vector.y
        uav_pose.pose.position.z =  msg.vector.z
        uav_pose.pose.orientation = imu_msg.pose.orientation

        poses_pub.publish(uav_pose)

        flag = False


if __name__ == '__main__':
    try:
        tag_detect_node()
    except rospy.ROSInterruptException:
        pass
