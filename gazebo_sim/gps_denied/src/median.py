#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
import rospy
import tf.transformations
import numpy as np
#from scipy.signal import savgol_filter
from numpy import median


arr_yaw = []
window_size=13

def tag_detect_node():

    global uwb_filtered_pose_pub
    rospy.init_node('filter')
    uwb_filtered_pose_pub = rospy.Publisher("/yaw_pose", PoseStamped, queue_size=10)
    uwb_tag_sub = rospy.Subscriber("/yaw_angle",PoseStamped, tag_detection_cb)

    rospy.spin()

def tag_detection_cb(msg):


    angles = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,-msg.pose.orientation.z,msg.pose.orientation.w], axes='sxyz')
    yaw = angles[2]


    if len(arr_yaw)<window_size:
        arr_yaw.append(yaw)
    else:
        arr_yaw.pop(0)
        arr_yaw.append(yaw)

        filtered_yaw = median(arr_yaw)

        quaternion = tf.transformations.quaternion_from_euler(angles[0], angles[1], filtered_yaw)
        
        filtered = PoseStamped()
        filtered.header.stamp = rospy.Time.now()
        filtered.header.frame_id = "uwb"
        filtered.pose.orientation.x = quaternion[0]
        filtered.pose.orientation.y = quaternion[1]
        filtered.pose.orientation.z = quaternion[2]
        filtered.pose.orientation.w = quaternion[3]
        uwb_filtered_pose_pub.publish(filtered)





if __name__ == '__main__':
    try:
        tag_detect_node()
    except rospy.ROSInterruptException:
        pass