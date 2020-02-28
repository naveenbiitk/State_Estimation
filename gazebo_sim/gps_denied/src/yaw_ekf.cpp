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
#define PI (double)acos(-1.0)

int flag=0;
double X,P,Q;
int set_=0;
double theta_meas,r,K,P_p,X_p,error_1;
int set_p=0;
double T,m_last_range_time,u,X_e,M,error;
double x_1,x_2,y_1,y_2;

Eigen::Quaternionf q;
Eigen::Quaternionf q_;
Eigen::MatrixXf R_mat(3,3);
Eigen::MatrixXf rpy(3,1);
geometry_msgs::PoseStamped pose;
double error_threshold;
double l;


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
    X = rpy(2,0) ;
    }
    P = 1;
    
}

void correction_step()
{   
    theta_meas = std::sqrt( std::pow( x_1 - x_2, 2) +
                        std::pow( y_1 - y_2, 2)  ) ;
    theta_meas = acos(theta_meas/l);

    r = std::pow(r,2) ;
    //ROS_WARN("runing");
    K = P / (P + r );
    P_p = ( 1 - K ) * P;
    error_1 = std::fabs(theta_meas - X);
    X_p = X + K * (theta_meas - X);
    // decide to take the range info or not.
    flag = 0;
    if(error_1 < error_threshold){
        setState(X_p);
        setCovariance(P_p);
        set_= 0;
        return ;
    } else {

        ROS_WARN("Update too large: predicted %f measured-- %f", X , theta_meas);
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
    //ROS_WARN("predicted %f#%f$%f",X_e(0),X_e(3),X_e(6));
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
    if(flag==1)
        correction_step();
}

void tag2_cb(const dwm1001::anchor::ConstPtr& msg)
{
    x_2 = msg->x;
    y_2 = msg->y;
    flag=1;
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
    //Subscriber and Publisher for the data. remapped in the launch file to the topic required
    ros::Subscriber anchor_1 = nh.subscribe("tag_1",10,tag1_cb);
    ros::Subscriber anchor_2 = nh.subscribe("tag_2",10,tag2_cb);
    ros::Subscriber velocity_sb = nh.subscribe("velocity",10,prediction_step);

    ros::Publisher fused_theta = nh.advertise<geometry_msgs::PoseStamped>("yaw_angle", 10);
    
    ros::Rate loop_rate(30);
    while(ros::ok()){
        
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "base";
    if(isnan(X)==1)
            Initialize();
        pose.pose.position.x= 0;
        pose.pose.position.y= 0;
        pose.pose.position.z= 0;
        q_ = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX())*Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY())*Eigen::AngleAxisf(X, Eigen::Vector3f::UnitZ());
        pose.pose.orientation.w = q_.w();
        pose.pose.orientation.x = q_.x();
        pose.pose.orientation.y = q_.y();
        pose.pose.orientation.z = q_.z();

        fused_theta.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}       

