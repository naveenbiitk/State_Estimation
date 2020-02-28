#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
import rospy
import numpy as np
from numpy import median
from dwm1001.msg  import anchor
from numpy import median
import math


arr_x = []
arr_y = []
arr_z = []
window_size=200
g_x = 0.0
g_y = 0.0
g_z = 0.0
offset_x = 0.0
offset_y = 0.0
offset_z = 0.0
flag = False
c = 0
count_ = 0
error = PoseStamped()
total_error = PoseStamped()
error_pub = rospy.Publisher("/error_pose_1", PoseStamped, queue_size=10)
total_error_pub = rospy.Publisher("/total_error_pose_1", PoseStamped, queue_size=10)

def tag_detect_node():
    global error_pub,total_error_pub
    rospy.init_node('filter_static')
    ikf_sub_1 = rospy.Subscriber("/vrpn_client_node/quad/pose",PoseStamped, groundt_cb)
    #ikf_sub_1 = rospy.Subscriber("/dwm1001/tag5617",anchor, estimated_cb)
    ikf_sub = rospy.Subscriber("/Filtered_pose",PoseStamped, estimated_cb)

    rospy.spin()

def groundt_cb(msg):
    global g_x,g_y,g_z,flag
    g_x = msg.pose.position.x
    g_y = msg.pose.position.y
    g_z = msg.pose.position.z
    flag = True


def estimated_cb(msg):
    global g_x,g_y,g_z,flag,c,count_
    global error,total_error
    global error_pub,total_error_pub
    global offset_x,offset_y,offset_z
    if(math.isnan(msg.pose.position.z)==True):
        return
    if(flag == True):
        error.header.stamp = rospy.Time.now()
        total_error.header.stamp = rospy.Time.now()
        error.pose.position.x =  msg.pose.position.x - g_x  
        error.pose.position.y =  msg.pose.position.y - g_y  
        error.pose.position.z =  msg.pose.position.z - g_z  
 
        error_pub.publish(error)

        total_error.pose.position.x =  (total_error.pose.position.x * c + error.pose.position.x)/(c+1)
        total_error.pose.position.y =  (total_error.pose.position.y * c + error.pose.position.y)/(c+1)
        total_error.pose.position.z =  (total_error.pose.position.z * c + error.pose.position.z)/(c+1)

        #total_error.pose.position.x = total_error.pose.position.x  + error.pose.position.x
        #total_error.pose.position.y = total_error.pose.position.y  + error.pose.position.y
        #total_error.pose.position.z = total_error.pose.position.z  + error.pose.position.z 
        
        c = c+1
        total_error_pub.publish(total_error)
        flag = False
        if count_< 100 :
            count_=count_+1
        elif count_ < 2000 :
            count_=count_+1
            if len(arr_x)<window_size:
                arr_x.append(error.pose.position.x)
                arr_y.append(error.pose.position.y)
                arr_z.append(error.pose.position.z)
            else:
                arr_x.pop(0)
                arr_y.pop(0)
                arr_z.pop(0)
                arr_x.append(error.pose.position.x)
                arr_y.append(error.pose.position.y)
                arr_z.append(error.pose.position.z)
                offset_x = median(arr_x)
                offset_y = median(arr_y)
                offset_z = median(arr_z)
                text_file = open('/home/naveen/static_test.txt', "w")
                text_file.write(str(total_error)+str('\n')+str(offset_x)+str('\n')+str(offset_y)+str('\n')+str(offset_z))
                text_file.close()


if __name__ == '__main__':
    try:
        tag_detect_node()
    except rospy.ROSInterruptException:
        pass
