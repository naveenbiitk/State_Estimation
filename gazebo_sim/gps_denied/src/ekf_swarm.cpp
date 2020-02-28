//State Vector- variable acceleration model
	// X(0) = quad_position.point.x;
    // X(1) = quad_velocity.point.x;
    // X(2) = quad_acc.point.x;
    // X(3) = quad_position.point.y;
    // X(4) = quad_velocity.point.y;
    // X(5) = quad_acc.point.y;
    // X(6) = quad_position.point.z;
    // X(7) = quad_velocity.point.z;
    // X(8) = quad_acc.point.z;
	// X(9)  = leader.point.x;
    // X(10) = leader.point.y;
    // X(11) = leader.point.z;
	// X(12) = neighbor.point.x;
	// X(13) = neighbor.point.y;
	// X(14) = neighbor.point.z;
///measeurement Vector
    //1.
	// Y(0) = quad.imu.acc.x
	// Y(1) = 0
	// Y(2) = 0
	// Y(3) = quad.imu.acc.y
	// Y(4) = 0
	// Y(5) = 0
	// Y(6) = quad.imu.acc.z
	// Y(7) = 0
	// Y(8) = 0
    //2.
    // Y(0) = quad.imu.vel.x
    // Y(1) = 0
    // Y(2) = 0
    // Y(3) = quad.imu.vel.y
    // Y(4) = 0
    // Y(5) = 0
    // Y(6) = quad.imu.vel.z
    // Y(7) = 0
    // Y(8) = 0
    //3.
	// Y() = leader_quad.point.x
	// Y() = leader_quad.point.y
	// Y() = leader_quad.point.z
    //4.
	// Y() = neighbor_quad.point.x
	// Y() = neighbor_quad.point.y
	// Y() = neighbor_quad.point.z

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
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"

// Declarations

double r_pred_l,r_meas_l;
Eigen::MatrixXd H_l(1,15);
double R_l;
double m_R_scale_l,sigma_r_l;
Eigen::VectorXd K_l(15);
Eigen::VectorXd X_p_l(15);
Eigen::MatrixXd P_p_l(15,15);
double error_l;
double precisionRangeErrEst_l;
//
double r_pred_n,r_meas_n;
Eigen::MatrixXd H_n(1,15);
double R_n;
double m_R_scale_n,sigma_r_n;
Eigen::VectorXd K_n(15);
Eigen::VectorXd X_p_n(15);
Eigen::MatrixXd P_p_n(15,15);
double error_n;
double precisionRangeErrEst_n;
//
geometry_msgs::Vector3Stamped pose;
geometry_msgs::Vector3Stamped vel;
geometry_msgs::Vector3Stamped acc_;

Eigen::Quaternionf q;
Eigen::MatrixXf R_mat(3,3);
Eigen::MatrixXf imuacc(3,1);
Eigen::MatrixXf acc(3,1);
double r_acc;
Eigen::MatrixXd r_meas_acc(3,1);
Eigen::MatrixXd r_pred_acc(3,1);
Eigen::MatrixXd H_acc(3,15);
Eigen::MatrixXd R_acc(3,3);
Eigen::MatrixXd K_acc(15,3);
Eigen::MatrixXd P_p_acc(15,15);
Eigen::VectorXd X_p_acc(15);
//
double r_vel;
Eigen::MatrixXd r_meas_vel(3,1);
Eigen::MatrixXd r_pred_vel(3,1);
Eigen::MatrixXd H_vel(3,15);
Eigen::MatrixXd R_vel(3,3);
Eigen::MatrixXd K_vel(15,3);
Eigen::MatrixXd P_p_vel(15,15);
Eigen::VectorXd X_p_vel(15);
//
double r_pos_l;
Eigen::MatrixXd pos_meas_l(3,1);
Eigen::MatrixXd pos_pred_l(3,1);
Eigen::MatrixXd H_pos_l(3,15);
Eigen::MatrixXd R_pos_l(3,3);
Eigen::MatrixXd K_pos_l(15,3);
Eigen::MatrixXd P_l_pos(15,15);
Eigen::VectorXd X_l_pos(15);
//
double r_pos_n;
Eigen::MatrixXd pos_meas_n(3,1);
Eigen::MatrixXd pos_pred_n(3,1);
Eigen::MatrixXd H_pos_n(3,15);
Eigen::MatrixXd R_pos_n(3,3);
Eigen::MatrixXd K_pos_n(15,3);
Eigen::MatrixXd P_n_pos(15,15);
Eigen::VectorXd X_n_pos(15);
//
Eigen::VectorXd X(15);
Eigen::VectorXd X_e(15);
Eigen::MatrixXd F(15,15);
Eigen::MatrixXd block_F(3,3);
Eigen::MatrixXd Q(15,15);
Eigen::MatrixXd P(15,15);
Eigen::MatrixXd M(15,15);
Eigen::MatrixXd block_Q(3,3);
Eigen::MatrixXd Q_lead(3,3);
Eigen::MatrixXd Q_neigh(3,3);
double tao_bias,m_y_damping_factor;
double T,error;
double m_last_range_time;
std_msgs::Float32MultiArray output;
double error_threshold,precisionRangeMm;
double T_sq,m_tao_bias_sqrt,T_cub,m_z_damping_factor,m_Q_scale;
double x,y,z,x_l,y_l,z_l,x_n,y_n,z_n;
double q_l,q_n;



void setState(Eigen::VectorXd Y)
{
    X = Y;
}
void setCovariance(Eigen::MatrixXd S)
{
    P = S;
}


void prediction_step()
{
    //ROS_WARN("Acceleration:## %f, ## %f, ## %f", ax,ay,az);


    T = ros::Time::now().toSec() - m_last_range_time;
    if(T>1){
        T = 1;
    } else if(T<0){
        T = 0.01;
    }
    

    T_sq = std::pow(T,2);
    T_cub = std::pow(T,3);
    // F is a 9x9 State Transition Matrix
    F = Eigen::MatrixXd::Zero(15,15);
    block_F << 1, T, -T_sq/2.0,
               0, 1, -T,
               0, 0, 1;
    F.block<3,3>(0,0) = block_F;
    F.block<3,3>(3,3) = block_F;
    F.block<3,3>(6,6) = block_F;
    F.block<6,6>(9,9) = Eigen::MatrixXd::Identity(6,6);

        int static count =0 ;
    if(count == 0)
       {ROS_WARN("prediction_step");
        count++;}
        //std::cout << F << '\n'<<'\n'; }
    // X is the predicted state vector and the predicted covariance matrix
    X_e = F*X ;
    // Q is the acceleration model
    tao_bias = m_tao_bias_sqrt * m_tao_bias_sqrt;
    Q = Eigen::MatrixXd::Zero(15,15);
    
    block_Q << (T_cub*T_sq)/20.0,   (T_sq*T_sq)/8.0 ,   -T_cub/6,
        	   (T_sq*T_sq)/8.0 ,    (T_cub)/3,          -T_sq/2,
         	    T_cub/6.0,		     -T_sq/2 ,               T  ;
    block_Q *= tao_bias;


    Q.block<3,3>(0,0) = block_Q;
    Q.block<3,3>(3,3) = block_Q ;
    Q.block<3,3>(6,6) = block_Q ;
    Q.block<3,3>(9,9) = Q_lead;
    Q.block<3,3>(12,12) = Q_neigh;
    Q *= m_Q_scale;
    //ROS_WARN("covariance:## %f, ## %f, ## %f",(T_cub*tao_acc/3.0)+(T_cub*T_sq)*tao_bias/20.0 , T*tao_acc+(T_cub*tao_bias/3), T*tao_bias);
   
    // if(count%50 == 1)
    // std::cout << X << '\n'<<'\n';
    // M is the predicted covariance matrix
	M = F*P*F.transpose() + Q;

    // time update
    m_last_range_time = ros::Time::now().toSec();
    //count++;
    // setState(X_e);
    // setCovariance(M);
        pose.header.stamp = ros::Time::now();
        pose.vector.x=X(0);
        pose.vector.y=X(3);
        pose.vector.z=X(6);

        vel.header.stamp = ros::Time::now();
        vel.vector.x=X(1);
        vel.vector.y=X(4);
        vel.vector.z=X(7);

        acc_.header.stamp = ros::Time::now();
        acc_.vector.x=X(2);
        acc_.vector.y=X(5);
        acc_.vector.z=X(8);
    //    fused.publish(output);


    if(error < error_threshold){
        //ROS_WARN("\n sucess too large: %f", error);
        setState(X_e);
        setCovariance(M);
        return ;
    } else {

        ROS_WARN("\n Estimate too large: %f", error);
        return ;
    }
    count++;
}



void correction_step_imu(const sensor_msgs::Imu::ConstPtr& msg)
{   
        int static count =0 ;
    if(count == 0)
       {ROS_WARN("imu_step");
        count++; }

    //conerts the acceleration from body frame to earth frame(NED) 
    q = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    R_mat= q.toRotationMatrix();
    
    imuacc << msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z;
    acc= R_mat*imuacc;
    acc(0,0)=acc(0,0);
    acc(1,0)=acc(1,0);
    acc(2,0)=(acc(2,0)-9.8);

    r_pred_acc << X(2),X(5),X(8);

    r_meas_acc << acc(0,0),acc(1,0),acc(2,0);
    // K is the Kalman Gain
    R_acc = Eigen::MatrixXd::Identity(3,3) * (r_acc*r_acc);
    R_acc(1,1) = R_acc(1,1) * m_y_damping_factor;
    R_acc(2,2) = R_acc(2,2) * m_z_damping_factor;
    //ROS_WARN("R_matrix:## %f",R);
    R_acc = (H_acc*P*H_acc.transpose()) + R_acc ;
    K_acc = P*H_acc.transpose() * R_acc.inverse();
    // Update P for the a posteriori covariance matrix
    P_p_acc = ( Eigen::MatrixXd::Identity(15,15) - K_acc * H_acc ) * P;
    // Update the state
    X_p_acc = X + K_acc * (r_meas_acc - r_pred_acc);
    // decide to take the range info or not.

    if(error < error_threshold){
        setState(X_p_acc);
        setCovariance(P_p_acc);
        return ;
    } else {

        ROS_WARN("\n Estimate too large: %f", error);
        return ;
    }
}


void correction_step_vel(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
        int static count =0 ;
    if(count == 0)
       {ROS_WARN("vel_step");
        count++; }

    r_pred_vel << X(1),X(4),X(7);

    r_meas_vel << msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z;
    // K is the Kalman Gain
    R_vel = Eigen::MatrixXd::Identity(3,3) * (r_vel*r_vel);
    R_vel(1,1) = R_vel(1,1) * m_y_damping_factor;
    R_vel(2,2) = R_vel(2,2) * m_z_damping_factor;
    //ROS_WARN("R_matrix:## %f",R);
    R_vel = (H_vel * P * H_vel.transpose()) + R_vel ;
    K_vel = P * H_vel.transpose() * R_vel.inverse();
    // Update P for the a posteriori covariance matrix
    P_p_vel = ( Eigen::MatrixXd::Identity(15,15) - K_vel * H_vel ) * P;
    // Update the state
    X_p_vel = X + K_vel * (r_meas_vel - r_pred_vel);
    // decide to take the range info or not.

    if(error < error_threshold){
        //ROS_WARN("\n sucess velocity: %f", );

        setState(X_p_vel);
        setCovariance(P_p_vel);
        return ;
    } else {

        ROS_WARN("\n Estimate too large: %f", error);
        return ;
    }
}



void correction_step_leader(const dwm1001::anchor::ConstPtr& msg)
{	
        int static count =0 ;
    if(count == 0)
       {ROS_WARN("leader_r_step");
        count++; }

	r_pred_l = std::sqrt( std::pow(X(0) -  X(9), 2) +
                          std::pow(X(3) - X(10), 2) +
                          std::pow(X(6) - X(11), 2) ) + 1e-5;

	r_meas_l = msg->range;
    // H is the linearized measurement matrix
    H_l << (X(0) -  X(9))/r_pred_l, 0, 0,
           (X(3) - X(10))/r_pred_l, 0, 0,
           (X(6) - X(11))/r_pred_l, 0, 0,
           (X(9) - X(0))/r_pred_l, (X(10) - X(3))/r_pred_l, (X(11) - X(6))/r_pred_l,
         	0,	0,	0;

    // K is the Kalman Gain
    sigma_r_l = double(precisionRangeErrEst_l) / 1000.0;
    R_l = std::pow(sigma_r_l,2) * m_R_scale_l;
    //ROS_WARN("R_matrix:## %f",R);
    K_l = P*H_l.transpose() / ( (H_l*P*H_l.transpose())(0,0) + R_l );
    // Update P for the a posteriori covariance matrix
    P_p_l = ( Eigen::MatrixXd::Identity(15,15) - K_l*H_l ) * P;
    // Return the measurement innovation
    error_l = std::fabs(r_meas_l - r_pred_l);
    // Update the state
    X_p_l = X + K_l * (r_meas_l - r_pred_l);
    // decide to take the range info or not.
    if(error_l < error_threshold){
        //ROS_WARN("\n sucess too large: %f", error);
        setState(X_p_l);
        setCovariance(P_p_l);
     
        return ;
    } else {

        ROS_WARN("Anchor id , Update too large: %f -- %f", r_pred_l,r_meas_l);
        std::cout << X << "\n"<<'\n';
        return ;
    }

}



void correction_step_neigh(const dwm1001::anchor::ConstPtr& msg)
{	
        int static count =0 ;
    if(count == 0)
       {ROS_WARN("neghbor_r_step");
        count++; }

	r_pred_n = std::sqrt( std::pow(X(0) - X(12), 2) +
                          std::pow(X(3) - X(13), 2) +
                          std::pow(X(6) - X(14), 2) ) + 1e-5;

	r_meas_n = msg->range;
    // H is the linearized measurement matrix
    H_n << (X(0) - X(12))/r_pred_n, 0, 0,
           (X(3) - X(13))/r_pred_n, 0, 0,
           (X(6) - X(14))/r_pred_n, 0, 0,
         	0,	0,	0,
           (X(12) - X(0))/r_pred_n, (X(13) - X(3))/r_pred_n, (X(14) - X(6))/r_pred_n ;

    // K is the Kalman Gain
    sigma_r_n = double(precisionRangeErrEst_n) / 1000.0;
    R_n = std::pow(sigma_r_n,2) * m_R_scale_n;
    //ROS_WARN("R_matrix:## %f",R);
    K_n = P*H_n.transpose() / ( (H_n*P*H_n.transpose())(0,0) + R_n );
    // Update P for the a posteriori covariance matrix
    P_p_n = ( Eigen::MatrixXd::Identity(15,15) - K_n*H_n ) * P;
    // Return the measurement innovation
    error_n = std::fabs(r_meas_n - r_pred_n);
    // Update the state
    X_p_n = X + K_n * (r_meas_n - r_pred_n);
    // decide to take the range info or not.
    if(error < error_threshold){
        //ROS_WARN("\n sucess too large: %f", error);
        setState(X_p_n);
        setCovariance(P_p_n);
     
        return ;
    } else {

        ROS_WARN("Anchor id , Update too large: %f --", error_n);
        std::cout << msg->device_id << "\n";
        return ;
    }

}


void position_lead_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        int static count =0 ;
    if(count == 0)
       {ROS_WARN("leader_pos_step");
        count++; }
    pos_meas_l << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;

    pos_pred_l << X(9),X(10),X(11);
    // K is the Kalman Gain
    R_pos_l = Eigen::MatrixXd::Identity(3,3) * (r_pos_l*r_pos_l);
    //ROS_WARN("R_matrix:## %f",R);
    R_pos_l = (H_pos_l * P * H_pos_l.transpose()) + R_pos_l;
    K_pos_l = P * H_pos_l.transpose() * R_pos_l.inverse();
    // Update P for the a posteriori covariance matrix
    P_l_pos = ( Eigen::MatrixXd::Identity(15,15) - K_pos_l * H_pos_l ) * P;
    // Update the state
    X_l_pos = X + K_pos_l * (pos_meas_l - pos_pred_l);
    // decide to take the range info or not.
    if(error < error_threshold){
        //ROS_WARN("\n sucess too large: %f", error);
        setState(X_l_pos);
        setCovariance(P_l_pos);
        return ;
    } else {

        ROS_WARN("\n Estimate too large: %f", error);
        return ;
    }
}



void position_neigh_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        int static count =0 ;
    if(count == 0)
       {ROS_WARN("neighbor_pos_step");
        count++; }
    pos_meas_n << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;

    pos_pred_n << X(12),X(13),X(14);

    // K is the Kalman Gain
    R_pos_n = Eigen::MatrixXd::Identity(3,3) * (r_pos_n*r_pos_n);
    //ROS_WARN("R_matrix:## %f",R);
    R_pos_n = (H_pos_n * P * H_pos_n.transpose()) + R_pos_n;
    K_pos_n = P * H_pos_n.transpose() * R_pos_n.inverse();
    // Update P for the a posteriori covariance matrix
    P_n_pos = ( Eigen::MatrixXd::Identity(15,15) - K_pos_n * H_pos_n ) * P;
    // Update the state
    X_n_pos = X + K_pos_n * (pos_meas_n - pos_pred_n);
    // decide to take the range info or not.
    if(error < error_threshold){
        //ROS_WARN("\n sucess too large: %f", error);
        setState(X_n_pos);
        setCovariance(P_n_pos);
        return ;
    } else {

        ROS_WARN("\n Estimate too large: %f", error);
        return ;
    }
}



void param(ros::NodeHandle& nh)
{
    nh.getParam("KalmanFilter_swarm/start_x", x);
    nh.getParam("KalmanFilter_swarm/start_y", y);
    nh.getParam("KalmanFilter_swarm/start_z", z);
    nh.getParam("KalmanFilter_swarm/lead_x", x_l);
    nh.getParam("KalmanFilter_swarm/lead_y", y_l);
    nh.getParam("KalmanFilter_swarm/lead_z", z_l);
    nh.getParam("KalmanFilter_swarm/neighbor_x", x_n);
    nh.getParam("KalmanFilter_swarm/neighbor_y", y_n);
    nh.getParam("KalmanFilter_swarm/neighbor_z", z_n);
    nh.getParam("KalmanFilter_swarm/leader_covariance", q_l);
    nh.getParam("KalmanFilter_swarm/neighbor_covariance", q_n);
    nh.getParam("KalmanFilter_swarm/m_tao_bias_sqrt", m_tao_bias_sqrt );
    nh.getParam("KalmanFilter_swarm/m_y_damping_factor", m_y_damping_factor);
    nh.getParam("KalmanFilter_swarm/m_z_damping_factor", m_z_damping_factor);
    nh.getParam("KalmanFilter_swarm/m_Q_scale", m_Q_scale);
    nh.getParam("KalmanFilter_swarm/error_threshold", error_threshold);
    nh.getParam("KalmanFilter_swarm/m_R_scale_leader", m_R_scale_l);
    nh.getParam("KalmanFilter_swarm/precisionRangeErrEst_leader", precisionRangeErrEst_l);
    nh.getParam("KalmanFilter_swarm/m_R_scale_neighbor", m_R_scale_n);
    nh.getParam("KalmanFilter_swarm/precisionRangeErrEst_neighbor", precisionRangeErrEst_n);
    nh.getParam("KalmanFilter_swarm/r_acc", r_acc);
    nh.getParam("KalmanFilter_swarm/r_vel", r_vel);
    nh.getParam("KalmanFilter_swarm/r_pos_l", r_pos_l);
    nh.getParam("KalmanFilter_swarm/r_pos_n", r_pos_n);

    ROS_WARN("%f, %f, %f ",x,y,z);
}


void Initialize(ros::NodeHandle& nh)
{
 
    param(nh);
 
    Eigen::MatrixXd nine_cov = Eigen::MatrixXd::Identity(15,15);
    nine_cov(0,0) = 0.001;
    nine_cov(3,3) = 0.001;
    nine_cov(6,6) = 0.001;
    // set cov of vel
    nine_cov(1,1) = 0.01;
    nine_cov(4,4) = 0.01;
    nine_cov(7,7) = 0.01;
    // set cov of acc_bias
    nine_cov(2,2) = 0.01;
    nine_cov(5,5) = 0.01;
    nine_cov(8,8) = 0.01;

    nine_cov(9,9)   = 0.001;
    nine_cov(10,10) = 0.001;
    nine_cov(11,11) = 0.001;

    nine_cov(12,12) = 0.001;
    nine_cov(13,13) = 0.001;
    nine_cov(14,14) = 0.001;

    setCovariance(nine_cov);
    X<<x,0,0,y,0,0,z,0,0,x_l,y_l,z_l,x_n,y_n,z_n;

    Q_lead <<  q_l,  0,  0,
                0, q_l, 0,
                0,  0,  q_l ;
    Q_neigh << q_n,  0,  0,
                0, q_n, 0,
                0,  0,  q_n ;
        // H is the linearized measurement matrix
    H_acc << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
        // H is the linearized measurement matrix
    H_vel << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
        // H is the linearized measurement matrix
    H_pos_l << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
        // H is the linearized measurement matrix
    H_pos_n << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;


    m_last_range_time = ros::Time::now().toSec();

}


int main(int argc, char** argv){

    ros::init(argc,argv,"KalmanFilter_swarm");
    ros::NodeHandle nh;     
    //Initializinging the parameters
    Initialize(nh);
    //param(nh);
    //Subscriber and Publisher for the data. remapped in the launch file to the topic required
    ros::Subscriber quad_imu = nh.subscribe("imu",20,correction_step_imu);
    ros::Subscriber quad_vel = nh.subscribe("velocity",20,correction_step_vel);
    ros::Subscriber anchor_1 = nh.subscribe("anchor_lead", 20,correction_step_leader);
    ros::Subscriber anchor_2 = nh.subscribe("anchor_neigh",20,correction_step_neigh);
    ros::Subscriber anchor_3 = nh.subscribe("position_leader",20,position_lead_cb);
    ros::Subscriber anchor_4 = nh.subscribe("position_neighour",20,position_neigh_cb);

    //ros::Publisher fused = nh.advertise<std_msgs::Float32MultiArray>("Filtered_data", 100);
    ros::Publisher fused_pose = nh.advertise<geometry_msgs::Vector3Stamped>("Filtered_pose", 10);
    ros::Publisher fused_vel = nh.advertise<geometry_msgs::Vector3Stamped>("Filtered_velocity", 10);
    ros::Publisher fused_acc = nh.advertise<geometry_msgs::Vector3Stamped>("Filtered_acc", 10);

    ros::Rate loop_rate(50);
    while(ros::ok()){
    //	output.data.clear();
	//	for (int i = 0; i < 15; i++)
	//		output.data.push_back(X(i));
        prediction_step();
		
        fused_pose.publish(pose);
        fused_vel.publish(vel);
        fused_acc.publish(acc_);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}  