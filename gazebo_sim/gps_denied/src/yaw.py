#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
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
window_size=10
g_x = 0.0
g_y = 0.0
g_z = 0.0
offset_x = 0.25
offset_y = 0.2
offset_z = 0.0
flag = False
c = 0
count_ = 0
error = PoseStamped()
total_error = PoseStamped()
error_pub = rospy.Publisher("/error_pose", PoseStamped, queue_size=10)
total_error_pub = rospy.Publisher("/total_error_pose", PoseStamped, queue_size=10)

def tag_detect_node():
    global error_pub,total_error_pub
    rospy.init_node('yaw_filter')
    normal_1 = rospy.Subscriber("/normal/dwm1001/anchor5617",anchor, upt_normal_1)
    normal_2 = rospy.Subscriber("/normal/dwm1001/anchor8136",anchor, upt_normal_2)
    normal_3 = rospy.Subscriber("/normal/dwm1001/anchor833B",anchor, upt_normal_3)
    normal_4 = rospy.Subscriber("/normal/dwm1001/anchorD597",anchor, upt_normal_4)

    yaw_1 = rospy.Subscriber("/yaw/dwm1001/anchor5617",anchor, upt_yaw_1)
    yaw_2 = rospy.Subscriber("/yaw/dwm1001/anchor8136",anchor, upt_yaw_2)
    yaw_3 = rospy.Subscriber("/yaw/dwm1001/anchor833B",anchor, upt_yaw_3)
    yaw_4 = rospy.Subscriber("/yaw/dwm1001/anchorD597",anchor, upt_yaw_4)

    control = rospy.Subscriber("/mavros/imu/data",Imu, imu_cb)

    ikf_sub = rospy.Subscriber("/Filtered_pose",PoseStamped , estimated_cb)

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
        error.pose.position.x =  abs(abs(msg.pose.position.x - g_x ) - offset_x)
        error.pose.position.y =  abs(abs(msg.pose.position.y - g_y ) - offset_y)
        error.pose.position.z =  abs(abs(msg.pose.position.z - g_z ) - offset_z)
 
        error_pub.publish(error)

        total_error.pose.position.x =  (total_error.pose.position.x * c + error.pose.position.x)/(c+1)
        total_error.pose.position.y =  (total_error.pose.position.y * c + error.pose.position.y)/(c+1)
        total_error.pose.position.z =  (total_error.pose.position.z * c + error.pose.position.z)/(c+1)

        # total_error.pose.position.x = total_error.pose.position.x  + error.pose.position.x
        # total_error.pose.position.y = total_error.pose.position.y  + error.pose.position.y
        # total_error.pose.position.z = total_error.pose.position.z  + error.pose.position.z 
        
        c = c+1
        # if(c==100):
        #     total_error.pose.position.x = 0
        #     total_error.pose.position.y = 0
        #     total_error.pose.position.z = 0   
        total_error_pub.publish(total_error)
        # text_file = open('/home/naveen/test.txt', "w")
        # text_file.write(str(total_error)+str('\n')+str(offset_x)+str(offset_y)+str(offset_z))
        # text_file.close()
  
        # flag = False
        # if count_<25 :
        #     count_=count_+1
        # elif count_ < 50 :
        #     count_=count_+1
        #     if len(arr_x)<window_size:
        #         arr_x.append(error.pose.position.x)
        #         arr_y.append(error.pose.position.y)
        #         arr_z.append(error.pose.position.z)
        #     else:
        #         arr_x.pop(0)
        #         arr_y.pop(0)
        #         arr_z.pop(0)
        #         arr_x.append(error.pose.position.x)
        #         arr_y.append(error.pose.position.y)
        #         arr_z.append(error.pose.position.z)
        #         offset_x = median(arr_x)
        #         offset_y = median(arr_y)
        #         offset_z = median(arr_z)


if __name__ == '__main__':
    try:
        tag_detect_node()
    except rospy.ROSInterruptException:
        pass
