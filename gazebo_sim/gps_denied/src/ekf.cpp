    // KALMANFILTER3D_UPDATE - updates a EKF with 9 state variables
    // Called each time a new mobile-to-anchor measurement is available

    // INPUTS:
    // X - the 9x1 a priori state vector - [x,xdot,ax_bias,y,ydot,ay_bias,z,zdot,az_bias]'
    // P - the 9x9 a priori covariance matrix
    // T - the delta time between updates
    // tao_acc - variance of acceleratmetor's gaussian noise
    // tao_bias - variance of acceleratmetor bias's noise
    // sigma_r - the estimated standard deviation of the new range measurement
    // r_meas - the actual range measurement
    // OUTPUTS:
    // X - the new state vector estimate
    // P - the new covariance matrix estimate
    // error - the difference between the range measurement and the estimated
    // range measurement (perhaps useful for outlier filtering.)

    // determine whether to perform filter using SNR
    // According to experiments on 2017-03-24,
    // Time Domain UWB range info have incorrect vPeak when satuatured,
    // probably because of Int16 overflow
    // float SNR = 20 * std::log( range_info.vPeak / ( range_info.noise + 0.1) );
    //   if(SNR < m_snr_threshold){
    //       //ROS_WARN("Anchor %d SNR %f too small, discard.", anchor_id, SNR);
    //       return false;
    //   }
    // X(0) = m_position.point.x;
    // X(1) = m_velocity.point.x;
    // X(2) = m_acc_bias.point.x;
    // X(3) = m_position.point.y;
    // X(4) = m_velocity.point.y;
    // X(5) = m_acc_bias.point.y;
    // X(6) = m_position.point.z;
    // X(7) = m_velocity.point.z;
    // X(8) = m_acc_bias.point.z;

    // ------------------------------------------------------------------
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include <dwm1001/anchor.h> 
#include <sensor_msgs/Imu.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/Altitude.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/TwistStamped.h"

// Declarations
double r_pred;
//int count =0 ;
Eigen::MatrixXd H(1,9);
double R;
Eigen::VectorXd K(9);
double error,error_threshold;
double precisionRangeErrEst,precisionRangeMm;
//
//geometry_msgs::Vector3Stamped pose;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped imu_msg;
float ax,ay,az; 
Eigen::Quaternionf q;
Eigen::Quaternionf q_;
Eigen::MatrixXf R_mat(3,3);
Eigen::MatrixXf imuacc(3,1);
Eigen::MatrixXf acc(3,1);
//
Eigen::VectorXd X(9);
Eigen::VectorXd X_p(9);
Eigen::VectorXd X_e(9);
Eigen::MatrixXd F(9,9);
Eigen::MatrixXd block_F(3,3);
Eigen::MatrixXd Q(9,9);
Eigen::VectorXd u(9);
Eigen::MatrixXd B(9,9);
Eigen::MatrixXd P(9,9);
Eigen::MatrixXd P_p(9,9);
Eigen::MatrixXd M(9,9);
Eigen::MatrixXd block_B(3,3);
Eigen::MatrixXd block_Q(3,3);

double r_vel;
Eigen::MatrixXd r_meas_vel(3,1);
Eigen::MatrixXd r_pred_vel(3,1);
Eigen::MatrixXd H_vel(3,9);
Eigen::MatrixXd R_vel(3,3);
Eigen::MatrixXd K_vel(9,3);
Eigen::MatrixXd P_p_vel(9,9);
Eigen::VectorXd X_p_vel(9);

double tao_acc;
double tao_bias,error_1;
double sigma_r,sigma_a,r_meas;
double T,yaw_off,yaw,yaw_rad;
double m_R_scale;
double m_last_range_time;
std_msgs::Float32MultiArray output;

double m_kalman_sigma_a,T_sq,m_tao_acc_sqrt,m_tao_bias_sqrt,T_cub,m_z_damping_factor,m_Q_scale;
double m_snr_threshold;
double x,y,z;
double ax_,ay_;

double r_h,h_offset,h_meas;
Eigen::VectorXd X_h(9);
Eigen::MatrixXd P_h(9,9);
Eigen::VectorXd K_h(9);
Eigen::MatrixXd H_h(1,9);
Eigen::MatrixXd nine_cov = Eigen::MatrixXd::Identity(9,9);
double trans_x,trans_y;
Eigen::MatrixXf rpy(3,1);
int set_=0,set_p=0;

void setState(Eigen::VectorXd Y)
{
    X = Y;
}
void setCovariance(Eigen::MatrixXd S)
{
    P = S;
}



void param(ros::NodeHandle& nh)
{
    nh.getParam("KalmanFilter/start_x", x);
    nh.getParam("KalmanFilter/start_y", y);
    nh.getParam("KalmanFilter/start_z", z);
    nh.getParam("KalmanFilter/error_threshold", error_threshold);
    nh.getParam("KalmanFilter/m_R_scale", m_R_scale);
    nh.getParam("KalmanFilter/precisionRangeErrEst", precisionRangeErrEst);
    nh.getParam("KalmanFilter/m_kalman_sigma_a", m_kalman_sigma_a);
    nh.getParam("KalmanFilter/precisionRangeMm", precisionRangeMm);
    nh.getParam("KalmanFilter/m_tao_acc_sqrt", m_tao_acc_sqrt);
    nh.getParam("KalmanFilter/m_tao_bias_sqrt", m_tao_bias_sqrt );
    nh.getParam("KalmanFilter/m_z_damping_factor", m_z_damping_factor);
    nh.getParam("KalmanFilter/m_Q_scale", m_Q_scale);
    nh.getParam("KalmanFilter/r_h", r_h);
    nh.getParam("KalmanFilter/r_vel", r_vel);
    nh.getParam("KalmanFilter/trans_x", trans_x);
    nh.getParam("KalmanFilter/trans_y", trans_y);
    nh.getParam("KalmanFilter/yaw_offset", yaw_off);
}

void Initialize(ros::NodeHandle& nh)
{
 
    int static count_ =0 ;
    if(count_ == 0)
    {   param(nh);
        count_++; }
    else{
    std::string pose_topic("initialize");
    // nav_msgs::Path edge;
    // sharedEdge = ros::topic::waitForMessage<nav_msgs::Path>("/path_planned/edge",create_path);
    // if(sharedEdge != NULL){
    // edge = *sharedEdge;
    // }
    boost::shared_ptr<dwm1001::anchor const> final_msg;
    ros::Duration one_second(0.5);
    final_msg = ros::topic::waitForMessage<dwm1001::anchor>(pose_topic,nh,one_second);
    if(final_msg != NULL){
    //p_msg  = *final_msg ;
    x = final_msg->x;
    y = final_msg->y;    
    z = final_msg->z;
    }

    }
    //ROS_WARN("m_Q_scale %f", m_Q_scale);
    // ROS_WARN("x %f", x);

    
    nine_cov(0,0) = 0.001;
    nine_cov(3,3) = 0.001;
    nine_cov(6,6) = 0.001;
    // set cov of vel
    nine_cov(1,1) = 0.01;
    nine_cov(4,4) = 0.01;
    nine_cov(7,7) = 0.01;
    // set cov of acc_bias
    nine_cov(2,2) = 0.1;
    nine_cov(5,5) = 0.1;
    nine_cov(8,8) = 0.1;
    setCovariance(nine_cov);
    X<<x,0,0,y,0,0,z,0,0;
    m_last_range_time = ros::Time::now().toSec();
    H_h << 0, 0, 0,
           0, 0, 0,
           1, 0, 0;
    H_vel << 0, 1, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 1, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 1, 0;



}

void Initialize_2()
{
 
 
    std::string pose_topic("initialize");
    boost::shared_ptr<dwm1001::anchor const> final_msg;
    ros::Duration one_second(0.5);
    final_msg = ros::topic::waitForMessage<dwm1001::anchor>(pose_topic,one_second);
    if(final_msg != NULL){
    //p_msg  = *final_msg ;
    x = final_msg->x;
    y = final_msg->y;    
    z = final_msg->z;
    }
    ROS_WARN("initialized");
    //ROS_WARN("m_Q_scale %f", m_Q_scale);
    // ROS_WARN("x %f", x);
/*
    Eigen::MatrixXd nine_cov = Eigen::MatrixXd::Identity(9,9);
    nine_cov(0,0) = 0.001;
    nine_cov(3,3) = 0.001;
    nine_cov(6,6) = 0.001;
    // set cov of vel
    nine_cov(1,1) = 0.01;
    nine_cov(4,4) = 0.01;
    nine_cov(7,7) = 0.01;
    // set cov of acc_bias
    nine_cov(2,2) = 0.1;
    nine_cov(5,5) = 0.1;
    nine_cov(8,8) = 0.1;*/
    setCovariance(nine_cov);
    X<<x,0,0,y,0,0,z,0,0;
    m_last_range_time = ros::Time::now().toSec();

}

void correction_step(const dwm1001::anchor::ConstPtr& msg)
{   
    r_pred = std::sqrt( std::pow(X(0) - msg->x, 2) +
                        std::pow(X(3) - msg->y, 2) +
                        std::pow(X(6) - msg->z, 2) ) + 1e-5;
  
    r_meas = msg->range;
    // H is the linearized measurement matrix
    H << (X(0) - msg->x)/r_pred, 0, 0,
         (X(3) - msg->y)/r_pred, 0, 0,
         (X(6) - msg->z)/r_pred, 0, 0;

    // K is the Kalman Gain
    sigma_r = double(precisionRangeErrEst) / 1000.0;
    R = std::pow(sigma_r,2) * m_R_scale;
    //ROS_WARN("runing");
    K= P*H.transpose() / ( (H*P*H.transpose())(0,0) + R );
    // Update P for the a posteriori covariance matrix
    //std::cout<<K(6)<<'\n';

    P_p = ( Eigen::MatrixXd::Identity(9,9) - K*H ) * P;
    // Return the measurement innovation
    error_1 = std::fabs(r_meas - r_pred);
    // Update the state
    X_p = X + K * (r_meas - r_pred);
    // decide to take the range info or not.
    if(error_1 < error_threshold){
        //ROS_WARN("\n sucess too large: %f", error);
        setState(X_p);
        setCovariance(P_p);
        //set_= 0;
        return ;
    } else {

        ROS_WARN("Anchor id , Update too large: predicted %f measured-- %f", r_pred, r_meas);
        std::cout << msg->device_id << "\n";
        set_=set_+1;
        if(set_>5)
            {Initialize_2();
              set_=0;
            }
        return ;
    }

}



void convert_NED(sensor_msgs::Imu imu)
{
            int static count_ =0 ;
    if(count_ == 0)
       {ROS_WARN("<--------imu_step---------->");
        count_++; }
    //conerts the acceleration from body frame to earth frame(NED) 
    q = Eigen::Quaternionf(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
    R_mat= q.toRotationMatrix();
    //rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
    //ROS_WARN("%f",rpy(2,0)*180/3.14);
    //yaw = rpy(2,0) - (yaw_off*0.017454);

    ax=imu.linear_acceleration.x;
    ay=imu.linear_acceleration.y;
    az=imu.linear_acceleration.z;
    imuacc << ax,ay,az;
    acc= R_mat*imuacc ;
    imu_msg.pose.orientation.w = imu.orientation.w;
    imu_msg.pose.orientation.x = imu.orientation.x;
    imu_msg.pose.orientation.y = imu.orientation.y;
    imu_msg.pose.orientation.z = imu.orientation.z;

}


void prediction_step(const sensor_msgs::Imu::ConstPtr& msg)
{
    convert_NED(*msg);  
    ax=acc(0,0);
    ay=acc(1,0);
    az=acc(2,0)-9.8;
    
    // yaw_rad = 0.0174533*yaw_off;
    // ax_= ax*std::cos(yaw_rad) + ay*std::sin(yaw_rad);
    // ay_=-ax*std::sin(yaw_rad) + ay*std::cos(yaw_rad);
    //ROS_WARN("Acceleration:## %f, ## %f, ## %f", ax,ay,az);
    T = msg->header.stamp.toSec() - m_last_range_time;
    if(T>1){
        T = 1;
    } else if(T<0){
        T = 0.02;
    }
    
    sigma_a = m_kalman_sigma_a;
   // r_meas = double(precisionRangeMm) / 1000.0;
    
    T_sq = std::pow(T,2);
    T_cub = std::pow(T,3);

    // F is a 9x9 State Transition Matrix
    F = Eigen::MatrixXd::Zero(9,9);

    block_F << 1, T, -T_sq/2.0,
               0, 1, -T,
               0, 0, 1;
    F.block<3,3>(0,0) = block_F;
    F.block<3,3>(3,3) = block_F;
    F.block<3,3>(6,6) = block_F;

    // Q is the acceleration model
    tao_acc = m_tao_acc_sqrt * m_tao_acc_sqrt;
    tao_bias = m_tao_bias_sqrt * m_tao_bias_sqrt;
    Q = Eigen::MatrixXd::Zero(9,9);
    
    block_Q << (T_cub*tao_acc/3.0)+(T_cub*T_sq)*tao_bias/20.0, (T_sq*tao_acc/2)+(T_sq*T_sq)*tao_bias/8.0  ,-T_cub*tao_bias/6,
               (T_sq*tao_acc/2.0)+(T_sq*T_sq)*tao_bias/8.0 ,   T*tao_acc+(T_cub*tao_bias/3)               ,-T_sq*tao_bias/2,
               -T_cub*tao_bias/6.0,                         -T_sq*tao_bias/2                          ,T*tao_bias         ;
    Q.block<3,3>(0,0) = block_Q;
    Q.block<3,3>(3,3) = block_Q;
    Q.block<3,3>(6,6) = block_Q * m_z_damping_factor;
    Q *= m_Q_scale;
    //std::cout << block_Q*m_Q_scale << '\n'<<'\n';
    //ROS_WARN("covariance:## %f, ## %f, ## %f",(T_cub*tao_acc/3.0)+(T_cub*T_sq)*tao_bias/20.0 , T*tao_acc+(T_cub*tao_bias/3), T*tao_bias);
   
    u << ax, 0, 0,
         ay, 0, 0,
         az, 0, 0;
    B = Eigen::MatrixXd::Zero(9,9);

    block_B << T_sq/2.0,  0,  0,
               T      ,  0,  0,
               0      ,  0,  0;
    B.block<3,3>(0,0) = block_B;
    B.block<3,3>(3,3) = block_B;
    B.block<3,3>(6,6) = block_B;

    // X is the predicted state vector and the predicted covariance matrix
    X_e = F*X + B * u;
    //if(count%250 == 0)
    //std::cout << P << '\n'<<'\n';
    // M is the predicted covariance matrix
    M = F*P*F.transpose() + Q;

    // time update
    m_last_range_time = msg->header.stamp.toSec();
    //ROS_WARN("runing---12");
    error = X.squaredNorm()-X_e.squaredNorm();
    //ROS_WARN("predicted %f#%f$%f",X_e(0),X_e(3),X_e(6));
    if(isnan(X(0))==1)
        {Initialize_2();
            return;}
    if(error < error_threshold){
        //ROS_WARN("\n sucess too large: %f", error);
        set_p=0;
        setState(X_e);
        setCovariance(M);
        return ;
    } else {

        ROS_WARN("\n Estimate too large: %f", error);
       set_p=set_p+1;
        if(set_p>5)
            {Initialize_2();
             set_p=0;
            }
        return ;
    }

}


void anchor1_cb(const dwm1001::anchor::ConstPtr& msg)
{
    correction_step(msg);
    //ROS_WARN("runing");
}
void anchor2_cb(const dwm1001::anchor::ConstPtr& msg)
{
    correction_step(msg);
}
void anchor3_cb(const dwm1001::anchor::ConstPtr& msg)
{
    correction_step(msg);
}
void anchor4_cb(const dwm1001::anchor::ConstPtr& msg)
{
    correction_step(msg);
}
// void anchor5_cb(const dwm1001::anchor::ConstPtr& msg)
// {
//  correction_step(msg);
// }
// void anchor6_cb(const dwm1001::anchor::ConstPtr& msg)
// {
//  correction_step(msg);
// }
//x,y,z,error_threshold,sigma_r,m_R_scale,precisionRangeErrEst,m_kalman_sigma_a,precisionRangeMm,
//m_tao_acc_sqrt,m_tao_bias_sqrt,m_z_damping_factor,m_Q_scale
void correction_step_vel(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
        int static count_ =0 ;
    if(count_ == 0)
       {ROS_WARN("<--------velocity_step---------->");
        count_++; }

    r_pred_vel << X(1),X(4),X(7);

    r_meas_vel << msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z;
    // K is the Kalman Gain
    R_vel = Eigen::MatrixXd::Identity(3,3) * (r_vel*r_vel);
    //ROS_WARN("R_matrix:## %f",R);
    R_vel = (H_vel * P * H_vel.transpose()) + R_vel ;
    K_vel = P * H_vel.transpose() * R_vel.inverse();
    // Update P for the a posteriori covariance matrix
    P_p_vel = ( Eigen::MatrixXd::Identity(9,9) - K_vel * H_vel ) * P;
    // Update the state
    X_p_vel = X + K_vel * (r_meas_vel - r_pred_vel);
    // decide to take the range info or not.
        //ROS_WARN("\n sucess velocity: %f", );

        setState(X_p_vel);
        setCovariance(P_p_vel);
        return ;

}

void height_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    int static count_ =0 ;
    if(count_ == 0)
       {ROS_WARN("<--------height_step---------->");
        count_++; }

    h_meas = msg->range;
    // K is the Kalman Gain
    r_h = r_h*r_h;
    //ROS_WARN("R_matrix:## %f",R);

   K_h = P*H_h.transpose() / ( (H_h*P*H_h.transpose())(0,0) + r_h );

    // K is the Kalman Gain
    // Update P for the a posteriori covariance matrix
    

    P_h = ( Eigen::MatrixXd::Identity(9,9) - K_h*H_h ) * P;
    // Return the measurement innovation
    // Update the state
    X_h = X + K_h * (h_meas - X(6));
    //std::cout<<K_h<<"--"<<h_meas<<"---"<<X_h(6)<<'\n';
        //ROS_WARN("\n height: %f", msg->local);
        setState(X_h);
        setCovariance(P_h);
     
    
}




int main(int argc, char** argv){

    ros::init(argc,argv,"KalmanFilter");
    ros::NodeHandle nh;
    //Initializinging the parameters
    Initialize(nh);
  
    //Subscriber and Publisher for the data. remapped in the launch file to the topic required
    ros::Subscriber Imu=nh.subscribe("imu",100,prediction_step);
    ros::Subscriber anchor_1 = nh.subscribe("anchor_1",10,anchor1_cb);
    ros::Subscriber anchor_2 = nh.subscribe("anchor_2",10,anchor2_cb);
    ros::Subscriber anchor_3 = nh.subscribe("anchor_3",10,anchor3_cb);
    ros::Subscriber anchor_4 = nh.subscribe("anchor_4",10,anchor4_cb);
    // ros::Subscriber anchor_5 = nh.subscribe("anchor_5",10,anchor5_cb);
    // ros::Subscriber anchor_6 = nh.subscribe("anchor_6",10,anchor6_cb);
    ros::Subscriber height_sb = nh.subscribe("height",10,height_cb);
    ros::Subscriber velocity_sb = nh.subscribe("velocity",10,correction_step_vel);

    ros::Publisher fused = nh.advertise<std_msgs::Float32MultiArray>("Filtered_data", 100);
    ros::Publisher fused_pose = nh.advertise<geometry_msgs::PoseStamped>("Filtered_pose", 10);
    
    ros::Rate loop_rate(50);
    while(ros::ok()){
        //param(nh);
  //    output.data.clear();
        // for (int i = 0; i < 9; i++)
        //  output.data.push_back(X(i));
        
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "base";

        pose.pose.position.x=X(0)+trans_x;
        pose.pose.position.y=X(3)+trans_y;
        pose.pose.position.z=X(6);
        pose.pose.orientation = imu_msg.pose.orientation;

        
        //q_ = Eigen::AngleAxisf(rpy(0,0), Eigen::Vector3f::UnitX())*Eigen::AngleAxisf(rpy(1,0), Eigen::Vector3f::UnitY())*Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
        
        // pose.pose.orientation.w = q_.w();
        // pose.pose.orientation.x = q_.x();
        // pose.pose.orientation.y = q_.y();
        // pose.pose.orientation.z = q_.z();
        //fused.publish(output);
        fused_pose.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}       

