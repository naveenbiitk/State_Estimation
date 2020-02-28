#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include <dwm1001/anchor.h> 
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_broadcaster.h>
#define PI (double)acos(-1.0)

int flag=0;
double X,P,Q;
int set_=0;
double theta_meas,theta_meas_1,r,K,P_p,X_p,error_1;
int set_p=0;
double T,m_last_range_time,u,X_e,M,error,yaw_imu_prev;
double x_1,x_2,y_1,y_2,z_1,z_2;

Eigen::Quaternionf q;
Eigen::Quaternionf q_;
Eigen::MatrixXf R_mat(3,3);
Eigen::MatrixXf rpy(3,1);
geometry_msgs::PoseStamped pose;
double error_threshold;
double l;
Eigen::Quaternionf q_imu;
Eigen::MatrixXf rpy_imu(3,1);
double yaw_imu;


void setState(double Y)
{
    X = Y;
}
void setCovariance(double S)
{
    P = S;
}



void Initialize()
{

    boost::shared_ptr<sensor_msgs::Imu const> final_msg;
    ros::Duration one_second(0.5);
    final_msg = ros::topic::waitForMessage<sensor_msgs::Imu>("/mavros/imu/data",one_second);
    if(final_msg != NULL){
    q = Eigen::Quaternionf(final_msg->orientation.w, final_msg->orientation.x, final_msg->orientation.y, final_msg->orientation.z);
    R_mat= q.toRotationMatrix();
    rpy = q.toRotationMatrix().eulerAngles(2, 1, 0);
    ROS_WARN("yaw initialized %f",rpy(2,0)*180/PI);
    X = rpy(2,0)-PI ;
    }
    P = 1;
    
}

void correction_step()
{   
    //l = 0.2;
    theta_meas_1 = (x_2 - x_1) ;
    if(theta_meas_1>l)
    theta_meas_1 = l;    
    //l = std::sqrt( std::pow( x_1 - x_2, 2) + std::pow( y_1 - y_2, 2) + std::pow( z_1 - z_2, 2) ) ;
    theta_meas = (((theta_meas_1/l)*-90.0) + 90.0);
    if(theta_meas > 90)
        theta_meas = (theta_meas )*PI/180;
    else
        theta_meas = theta_meas*PI/180;

    r = std::pow(r,2) ;
    //ROS_WARN("runing");
    K = P / (P + r );
    P_p = ( 1 - K ) * P;
    X_p = X + K * (theta_meas - X);
    // decide to take the range info or not.
    flag = 0;
    //ROS_WARN("measured distance %f, angle %f",theta_meas_1,theta_meas*180/PI);
    error_1 = std::fabs(X_p - X);
    if(error_1 < error_threshold){
        setState(X_p);
        setCovariance(P_p);
        set_= 0;
        return ;
    } else {

        ROS_WARN("Update too large: predicted %f measured-- %f with length %f", X , theta_meas,l);
        ROS_WARN("measurement setup %f with %f",x_1,x_2);
        set_=set_+1;
        if(set_>10)
            Initialize();
        return ;
    }

}


void prediction_step(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    T = msg->header.stamp.toSec() - m_last_range_time;
    if(T>1){
        T = 1;
    } else if(T<0){
        T = 0.02;
    }
    
    u = msg->twist.angular.z;
    X_e = X + T * u;
    M = P + Q;
    m_last_range_time = msg->header.stamp.toSec();
    error = std::fabs(X-X_e);
    //ROS_WARN("predicted %f",X_e);
    if(error < error_threshold){
        //ROS_WARN("\n sucess too large: %f", error);
        set_p=0;
        setState(X_e);
        setCovariance(M);
        return ;
    } else {

        ROS_WARN("\n Estimate too large: %f", error);
        set_p=set_p+1;
        if(set_p>10)
            Initialize();
        return ;
    }

}


void tag1_cb(const dwm1001::anchor::ConstPtr& msg)
{
    x_1 = msg->x;
    y_1 = msg->y;
    z_1 = msg->z;
    if(flag==1)
        correction_step();
}

void tag2_cb(const dwm1001::anchor::ConstPtr& msg)
{
    x_2 = msg->x;
    y_2 = msg->y;
    z_2 = msg->z;
    flag=1;
}


void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    //conerts the acceleration from body frame to earth frame(NED) 
    q_imu = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    //Eigen::MatrixXf rpy(3,1);
    rpy_imu = q_imu.toRotationMatrix().eulerAngles(0, 1, 2);
    //acc_ang.header.stamp = ros::Time::now();
    //acc_ang.vector.x = rpy_imu(0,0)*(180.0/3.14159265358979);
    //acc_ang.vector.y = rpy_imu(1,0)*(180.0/3.14159265358979);
    yaw_imu = rpy_imu(2,0);


}


int main(int argc, char** argv){

    ros::init(argc,argv,"yaw");
    ros::NodeHandle nh;
    //Initializinging the parameters
    Initialize();
    nh.getParam("yaw/tag_r", r);
    nh.getParam("yaw/imu_q", Q);
    nh.getParam("yaw/distance", l);
    nh.getParam("yaw/error_threshold", error_threshold);
    Q = Q*Q;
    tf::TransformBroadcaster br;
    tf::Transform transform;

    //Subscriber and Publisher for the data. remapped in the launch file to the topic required
    ros::Subscriber anchor_1 = nh.subscribe("tag_1",10,tag1_cb);
    ros::Subscriber anchor_2 = nh.subscribe("tag_2",10,tag2_cb);
    ros::Subscriber velocity_sb = nh.subscribe("velocity",10,prediction_step);
    ros::Subscriber imu_sb = nh.subscribe("imu",10,imu_cb);

    ros::Publisher fused_theta = nh.advertise<geometry_msgs::PoseStamped>("yaw_angle", 10);
    transform.setOrigin( tf::Vector3(0.0, 0.0, 1.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    ros::Rate loop_rate(30);
    while(ros::ok()){
        
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "uwb";
    if(isnan(X)==1)
            Initialize();
        //pose.pose.position.x= X*180/PI;
        if(abs(yaw_imu_prev - yaw_imu) > 170*PI/180)
            yaw_imu = yaw_imu_prev;
        //pose.pose.position.y= abs(yaw_imu*180/PI-16);
        yaw_imu_prev = yaw_imu;
        pose.pose.position.z= 0;
	//ROS_WARN("yaw %f compared %f",X*180/PI,yaw_imu*180/PI-16);
        q_ = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX())*Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY())*Eigen::AngleAxisf(X, Eigen::Vector3f::UnitZ());
        pose.pose.orientation.w = q_.w();
        pose.pose.orientation.x = q_.x();
        pose.pose.orientation.y = q_.y();
        pose.pose.orientation.z = q_.z();

        fused_theta.publish(pose);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_origin", "uwb"));
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}       

