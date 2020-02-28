//State Vector- variable acceleration model
	// X(0) = quad_position.point.x;
    // X(1) = m_velocity.point.x;
    // X(2) = m_acc_bias.point.x;
    // X(3) = quad_position.point.y;
    // X(4) = m_velocity.point.y;
    // X(5) = m_acc_bias.point.y;
    // X(6) = quad_position.point.z;
    // X(7) = m_velocity.point.z;
    // X(8) = m_acc_bias.point.z;
	// X(9)  = leader.point.x;
    // X(10) = leader.point.y;
    // X(11) = leader.point.z;
	// X(12) = leader.point.x;
	// X(13) = leader.point.y;
	// X(14) = leader.point.z;
//Input Vector
	// U(0) = quad.imu.acc.x
	// U(1) = 0
	// U(2) = 0
	// U(3) = quad.imu.acc.y
	// U(4) = 0
	// U(5) = 0
	// U(6) = quad.imu.acc.z
	// U(7) = 0
	// U(8) = 0
	// U(9)  = leader_quad.point.x
	// U(10) = leader_quad.point.y
	// U(11) = leader_quad.point.z
	// U(12) = neighbor_quad.point.x
	// U(13) = neighbor_quad.point.y
	// U(14) = neighbor_quad.point.z

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

// Declarations
int count =0 ;
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
float ax,ay,az; 
Eigen::Quaternionf q;
Eigen::MatrixXf R_mat(3,3);
Eigen::MatrixXf imuacc(3,1);
Eigen::MatrixXf acc(3,1);
//
Eigen::VectorXd X(15);
Eigen::VectorXd X_e(15);
Eigen::MatrixXd F(15,15);
Eigen::MatrixXd block_F(3,3);
Eigen::MatrixXd Q(15,15);
Eigen::VectorXd u(15);
Eigen::MatrixXd B(15,15);
Eigen::MatrixXd P(15,15);
Eigen::MatrixXd M(15,15);
Eigen::MatrixXd block_B(3,3);
Eigen::MatrixXd block_Q(3,3);
Eigen::MatrixXd B_I(6,6);
Eigen::MatrixXd Q_lead(3,3);
Eigen::MatrixXd Q_neigh(3,3);
double tao_acc;
double tao_bias;
double sigma_a,r_meas;
double T;
double m_last_range_time;
std_msgs::Float32MultiArray output;
double error_threshold,precisionRangeMm;
double m_kalman_sigma_a,T_sq,m_tao_acc_sqrt,m_tao_bias_sqrt,T_cub,m_z_damping_factor,m_Q_scale;
double m_snr_threshold,error;
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

void convert_NED(sensor_msgs::Imu imu)
{
	//conerts the acceleration from body frame to earth frame(NED) 
    q = Eigen::Quaternionf(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
    R_mat= q.toRotationMatrix();
    
    ax=imu.linear_acceleration.x;
    ay=imu.linear_acceleration.y;
    az=imu.linear_acceleration.z;
    imuacc << ax,ay,az;
    acc= R_mat*imuacc;

}


void prediction_step(const sensor_msgs::Imu::ConstPtr& msg)
{
	convert_NED(*msg);	
  	ax=acc(0,0);
  	ay=-acc(1,0);
  	az=-1*(acc(2,0)-9.8);

    //ROS_WARN("Acceleration:## %f, ## %f, ## %f", ax,ay,az);
    T = msg->header.stamp.toSec() - m_last_range_time;
    if(T>1){
        T = 1;
    } else if(T<0){
        T = 0.01;
    }
    
    sigma_a = m_kalman_sigma_a;
   // r_meas = double(precisionRangeMm) / 1000.0;
    
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

    u << ax, 0, 0,
         ay, 0, 0,
         az, 0, 0,
         x_l,y_l,z_l,
         x_n,y_n,z_n;
    B = Eigen::MatrixXd::Zero(15,15);
    B_I = Eigen::MatrixXd::Identity(6,6);
    block_B << T_sq/2.0,  0,  0,
               T      ,  0,  0,
               0      ,  0,  0;
    B.block<3,3>(0,0) = block_B;
    B.block<3,3>(3,3) = block_B;
    B.block<3,3>(6,6) = block_B;
    B.block<6,6>(9,9) = B_I;

    // X is the predicted state vector and the predicted covariance matrix
    X_e = F*X + B * u;

    // Q is the acceleration model
    tao_acc = m_tao_acc_sqrt * m_tao_acc_sqrt;
    tao_bias = m_tao_bias_sqrt * m_tao_bias_sqrt;
    Q = Eigen::MatrixXd::Zero(15,15);
    
    block_Q << (T_cub*tao_acc/3.0)+(T_cub*T_sq)*tao_bias/20.0, (T_sq*tao_acc/2)+(T_sq*T_sq)*tao_bias/8.0  ,-T_cub*tao_bias/6,
        	   (T_sq*tao_acc/2.0)+(T_sq*T_sq)*tao_bias/8.0 ,   T*tao_acc+(T_cub*tao_bias/3) 			  ,-T_sq*tao_bias/2,
         	   -T_cub*tao_bias/6.0,						    -T_sq*tao_bias/2 						  ,T*tao_bias         ;

    Q_lead <<  q_l,  0,  0,
    			0, q_l, 0,
    			0,  0,  q_l ;
    Q_neigh << q_n,  0,  0,
    			0, q_n, 0,
    			0,  0,  q_n ;

    Q.block<3,3>(0,0) = block_Q;
    Q.block<3,3>(3,3) = block_Q;
    Q.block<3,3>(6,6) = block_Q * m_z_damping_factor;
    Q *= m_Q_scale;
    Q.block<3,3>(9,9) = Q_lead;
    Q.block<3,3>(12,12) = Q_neigh;
    //ROS_WARN("covariance:## %f, ## %f, ## %f",(T_cub*tao_acc/3.0)+(T_cub*T_sq)*tao_bias/20.0 , T*tao_acc+(T_cub*tao_bias/3), T*tao_bias);
   
    // if(count%250 == 0)
    // std::cout << P << '\n'<<'\n';
    // M is the predicted covariance matrix
	M = F*P*F.transpose() + Q;

    // time update
    m_last_range_time = msg->header.stamp.toSec();
    //count++;
    error = X.squaredNorm()-X_e.squaredNorm();
    if(error < error_threshold){
        //ROS_WARN("\n sucess too large: %f", error);
        setState(X_e);
        setCovariance(M);
        return ;
    } else {

        ROS_WARN("\n Estimate too large: %f", error);
        return ;
    }

}


void correction_step_leader(const dwm1001::anchor::ConstPtr& msg)
{	
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

        ROS_WARN("Anchor id , Update too large: %f --", error_l);
        std::cout << msg->device_id << "\n";
        return ;
    }

}


void correction_step_neigh(const dwm1001::anchor::ConstPtr& msg)
{	
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


// void anchor_lead_cb(const dwm1001::anchor::ConstPtr& msg)
// {
// 	correction_step_leader(msg);
// }

// void anchor_neigh_cb(const dwm1001::anchor::ConstPtr& msg)
// {
// 	correction_step_neigh(msg);
// }

void position_lead_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	x_l =  msg->pose.position.x;
    y_l =  msg->pose.position.y;
    z_l =  msg->pose.position.z;
}

void position_neigh_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	x_n =  msg->pose.position.x;
    y_n =  msg->pose.position.y;
    z_n =  msg->pose.position.z;
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
    nh.getParam("KalmanFilter_swarm/error_threshold", error_threshold);
    nh.getParam("KalmanFilter_swarm/m_R_scale_leader", m_R_scale_l);
    nh.getParam("KalmanFilter_swarm/precisionRangeErrEst_leader", precisionRangeErrEst_l);
    nh.getParam("KalmanFilter_swarm/m_R_scale_neighbor", m_R_scale_n);
    nh.getParam("KalmanFilter_swarm/precisionRangeErrEst_neighbor", precisionRangeErrEst_n);
    nh.getParam("KalmanFilter_swarm/m_kalman_sigma_a", m_kalman_sigma_a);
    nh.getParam("KalmanFilter_swarm/precisionRangeMm", precisionRangeMm);
    nh.getParam("KalmanFilter_swarm/leader_covariance", q_l);
    nh.getParam("KalmanFilter_swarm/neighbor_covariance", q_n);
    nh.getParam("KalmanFilter_swarm/m_tao_acc_sqrt", m_tao_acc_sqrt);
    nh.getParam("KalmanFilter_swarm/m_tao_bias_sqrt", m_tao_bias_sqrt );
    nh.getParam("KalmanFilter_swarm/m_z_damping_factor", m_z_damping_factor);
    nh.getParam("KalmanFilter_swarm/m_Q_scale", m_Q_scale);
}


void Initialize(ros::NodeHandle& nh)
{
 
    param(nh);
    // ROS_WARN("m_Q_scale %f", m_Q_scale);
    // ROS_WARN("x %f", x);

    m_kalman_sigma_a = 0.125;
    m_snr_threshold = -100; 
 
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
    m_last_range_time = ros::Time::now().toSec();

}


int main(int argc, char** argv){

    ros::init(argc,argv,"KalmanFilter_swarm");
    ros::NodeHandle nh;     
    //Initializinging the parameters
    Initialize(nh);
  
    //Subscriber and Publisher for the data. remapped in the launch file to the topic required
    ros::Subscriber Imu=nh.subscribe("imu",100,prediction_step);
    ros::Subscriber anchor_1 = nh.subscribe("anchor_lead", 100,correction_step_leader);
    ros::Subscriber anchor_2 = nh.subscribe("anchor_neigh",100,correction_step_neigh);
    ros::Subscriber anchor_3 = nh.subscribe("position_leader",100,position_lead_cb);
    ros::Subscriber anchor_4 = nh.subscribe("position_neighour",100,position_neigh_cb);

    //ros::Publisher fused = nh.advertise<std_msgs::Float32MultiArray>("Filtered_data", 100);
    ros::Publisher fused_pose = nh.advertise<geometry_msgs::Vector3Stamped>("Filtered_pose", 10);
    ros::Publisher fused_vel = nh.advertise<geometry_msgs::Vector3Stamped>("Filtered_velocity", 10);
    ros::Publisher fused_acc = nh.advertise<geometry_msgs::Vector3Stamped>("Filtered_acc", 10);

    ros::Rate loop_rate(50);
    while(ros::ok()){
        param(nh);
    //	output.data.clear();
	//	for (int i = 0; i < 15; i++)
	//		output.data.push_back(X(i));
		
        pose.header.stamp = ros::Time::now();
        pose.vector.x=X(0);
        pose.vector.y=X(3);
        pose.vector.z=X(6);

        vel.header.stamp = ros::Time::now();
        vel.vector.x=X(1);
        vel.vector.y=X(4);
        vel.vector.z=X(7);

        acc_.header.stamp = ros::Time::now();
        acc_.vector.x=ax;
        acc_.vector.y=ay;
        acc_.vector.z=az;
    //    fused.publish(output);
        fused_pose.publish(pose);
        fused_vel.publish(vel);
        fused_acc.publish(acc_);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}  