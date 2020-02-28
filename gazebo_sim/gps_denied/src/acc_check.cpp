#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <math.h>
#include <stdio.h> 
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/Vector3Stamped.h"

ros::Publisher acc_1;
ros::Publisher acc_2;
ros::Publisher veloc;


void imu_acc(const sensor_msgs::Imu::ConstPtr& msg)
{
	    q = Eigen::Quaternionf(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
    R_mat= q.toRotationMatrix();

    ax=imu.linear_acceleration.x;
    ay=imu.linear_acceleration.y;
    az=imu.linear_acceleration.z;
    imuacc << ax,ay,az;
    acc= R_mat*imuacc ;
    imu_msg.pose.orientation.w = imu.orientation.w;
    imu_msg.pose.orientation.x = imu.orientation.x;
    imu_msg.pose.orientation.y = imu.orientation.y;
    imu_msg.pose.orientation.z = imu.orientation.z;
	return;
}

void vicon_vel(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
     if(count_vel==0)
     {
     	prev_x = msg->pose.position.x ;
     	prev_y = msg->pose.position.y ;
     	prev_z = msg->pose.position.z ;
     	count_vel++;
     	vel_last_time = ros::Time::now().toSec();
     }
     else
     {
     	dt_1 = msg->header.stamp.toSec() - vel_last_time;
     	vic_vel.header.stamp = ros::Time::now();
     	vic_vel.vector.x = (msg->pose.position.x - prev_x)/dt_1 ;
     	vic_vel.vector.y = (msg->pose.position.y - prev_y)/dt_1 ;
     	vic_vel.vector.z = (msg->pose.position.z - prev_z)/dt_1 ;
     	
     	prev_x = msg->pose.position.x ;
     	prev_y = msg->pose.position.y ;
     	prev_z = msg->pose.position.z ;
     	
     	veloc.publish(vic_vel);

     	vel_last_time = ros::Time::now().toSec();	
     }

	return;
}

int main(int argc, char** argv){

    ros::init(argc,argv,"check_acc");
         
    //Initializinging the parameters
    ros::NodeHandle nh;
    Initialize(nh);
  
    //Subscriber and Publisher for the data. remapped in the launch file to the topic required
    
    //ros::Duration(1).sleep();
    
    ros::Subscriber Imu = nh.subscribe("imu",10,imu_acc);
    ros::Subscriber vicon_1 = nh.subscribe("vicon_pos",10,vicon_vel);
    ros::Subscriber vicon_2 = nh.subscribe("vicon_vel",10,vicon_acc);

    acc_1 = nh.advertise<geometry_msgs::Vector3Stamped>("imu_acc", 10);
    acc_2 = nh.advertise<geometry_msgs::Vector3Stamped>("vicon_acc", 10);
    veloc = nh.advertise<geometry_msgs::Vector3Stamped>("vicon_velocity", 10);
    
    ros::Rate loop_rate(50);
    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}