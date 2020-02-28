
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

geometry_msgs::PoseStamped pose;
mavros_msgs::State current_state;
Eigen::Quaternionf q;
Eigen::Quaternionf q_2;
Eigen::MatrixXf R_mat(3,3);
Eigen::MatrixXf imuacc(3,1);
geometry_msgs::Vector3Stamped ang;
geometry_msgs::Vector3Stamped acc_;
geometry_msgs::Vector3Stamped acc_ang;
ros::Publisher euler ;
ros::Publisher body_acc;
ros::Publisher acc_1;
ros::Publisher acc_2;
ros::Publisher veloc;

double a,b;
double theta=0.0;
double count_r=0.0;
double wn;

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    //conerts the acceleration from body frame to earth frame(NED) 
    q = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    R_mat= q.toRotationMatrix();
    imuacc << msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z;
    imuacc= R_mat*imuacc ;
    acc_.header.stamp = ros::Time::now();
    acc_.vector.x = imuacc(0,0);
    acc_.vector.y = imuacc(1,0);
    acc_.vector.z = imuacc(2,0)-9.81;
    acc_1.publish(acc_);

   /* Eigen::MatrixXf rpy(3,1);
    rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
    acc_ang.header.stamp = ros::Time::now();
    acc_ang.vector.x = rpy(0,0)*(180.0/3.14159265358979);
    acc_ang.vector.y = rpy(1,0)*(180.0/3.14159265358979);
    acc_ang.vector.z = rpy(2,0)*(180.0/3.14159265358979);

    body_acc.publish(acc_ang);*/

}

double prev_x,prev_y,prev_z,dt_1,vel_last_time;
geometry_msgs::Vector3Stamped vic_vel;
int count_vel=0;


double prev_vx,prev_vy,prev_vz,dt_2,acc_last_time;
geometry_msgs::Vector3Stamped vic_acc;
int count_acc=0;



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 10, state_cb);
    ros::Subscriber Imu=nh.subscribe("/mavros/imu/data",10,imu_cb);

 
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    acc_1 = nh.advertise<geometry_msgs::Vector3Stamped>("imu_acc", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(5.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 1.5;
    pose.pose.position.z = 1.5;
    
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    double pos_x = 0.0;
    double pos_y = 1.5;
    int mode_sp=0;
    while(ros::ok()){
    
  //  if( current_state.mode == "OFFBOARD" )
    nh.getParam("offboard/mode_sp", mode_sp);    

    if (mode_sp == 0)
    {
        nh.getParam("offboard/pos_x", pos_x);
        nh.getParam("offboard/pos_y", pos_y);
        pose.pose.position.x = pos_x;
        pose.pose.position.y = pos_y;
        pose.pose.position.z = 1.5;
    }

    if(mode_sp == 1)
    {
        
        nh.getParam("offboard/elip_a", a);
        nh.getParam("offboard/elip_b", b);
        nh.getParam("offboard/change_rate", wn);
        theta = wn*count_r;

        pose.pose.position.x = a*sin(theta);
        pose.pose.position.y = b*cos(theta);
        pose.pose.position.z = 1.5;

        count_r = count_r + 1.0;
        if(theta > 360)
            theta = 0 ;
    }

	    pose.pose.orientation.x = 0.0;
    	pose.pose.orientation.y = 0.0;
    	pose.pose.orientation.z = 0.0;
    	pose.pose.orientation.w = 1.0;
        
        local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();

    
//
}


return 0;
}
