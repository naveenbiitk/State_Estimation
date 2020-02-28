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
double R_l;
double m_R_scale_l,sigma_r_l;
//double error_l;
double precisionRangeErrEst_l;
Eigen::MatrixXd H_l(1,15);
Eigen::VectorXd K_l(15);
Eigen::VectorXd Ze_r_l(15);
Eigen::MatrixXd Om_r_l(15,15);
Eigen::VectorXd X_r_l(15);
//
double r_pred_n,r_meas_n;
double R_n;
double m_R_scale_n,sigma_r_n;
//double error_n;
double precisionRangeErrEst_n;
Eigen::MatrixXd H_n(1,15);
Eigen::VectorXd K_n(15);
Eigen::VectorXd Ze_r_n(15);
Eigen::MatrixXd Om_r_n(15,15);
Eigen::VectorXd X_r_n(15);
//
Eigen::Quaternionf q;
Eigen::MatrixXf R_mat(3,3);
Eigen::MatrixXf imuacc(3,1);
Eigen::MatrixXf acc(3,1);
double r_acc;
Eigen::MatrixXd r_meas_acc(3,1);
Eigen::MatrixXd H_acc(3,15);
Eigen::MatrixXd R_acc(3,3);
Eigen::MatrixXd K_acc(15,3);
Eigen::MatrixXd Om_acc(15,15);
Eigen::VectorXd Ze_acc(15);
Eigen::VectorXd X_acc(15);
//
double r_vel;
Eigen::MatrixXd r_meas_vel(3,1);
Eigen::MatrixXd H_vel(3,15);
Eigen::MatrixXd R_vel(3,3);
Eigen::MatrixXd K_vel(15,3);
Eigen::MatrixXd Om_vel(15,15);
Eigen::VectorXd Ze_vel(15);
Eigen::VectorXd X_vel(15);
//
double r_pos_l;
Eigen::MatrixXd pos_meas_l(3,1);
Eigen::MatrixXd H_pos_l(3,15);
Eigen::MatrixXd R_pos_l(3,3);
Eigen::MatrixXd K_pos_l(15,3);
Eigen::MatrixXd Om_pos_l(15,15);
Eigen::VectorXd Ze_pos_l(15);
Eigen::VectorXd X_pos_l(15);
//
double r_pos_n;
Eigen::MatrixXd pos_meas_n(3,1);
Eigen::MatrixXd H_pos_n(3,15);
Eigen::MatrixXd R_pos_n(3,3);
Eigen::MatrixXd K_pos_n(15,3);
Eigen::MatrixXd Om_pos_n(15,15);
Eigen::VectorXd Ze_pos_n(15);
Eigen::VectorXd X_pos_n(15);
//
double R_h,r_h,h_meas;
Eigen::MatrixXd H_h(1,15);
Eigen::MatrixXd Om_h(15,15);
Eigen::VectorXd Ze_h(15);
Eigen::VectorXd X_h(15);
//
Eigen::VectorXd X(15);
Eigen::VectorXd u(15);
Eigen::VectorXd X_e(15);
Eigen::MatrixXd F(15,15);
Eigen::MatrixXd block_F(3,3);
Eigen::MatrixXd B(15,15);
Eigen::MatrixXd block_B(3,3);
Eigen::MatrixXd Q(15,15);
Eigen::MatrixXd block_Q(3,3);
Eigen::MatrixXd Q_lead(3,3);
Eigen::MatrixXd Q_neigh(3,3);
Eigen::MatrixXd Om_p(15,15);
Eigen::VectorXd Ze_p(15);
Eigen::MatrixXd Om(15,15);
Eigen::VectorXd Ze(15);
geometry_msgs::Vector3Stamped pose;
geometry_msgs::Vector3Stamped vel;
geometry_msgs::Vector3Stamped acc_;
std_msgs::Float32MultiArray output;
double tao_bias,m_y_damping_factor;
double T,error;
double m_last_range_time;
double error_threshold;
double T_sq,m_tao_bias_sqrt,T_cub,m_z_damping_factor,m_Q_scale;
double x,y,z,x_l,y_l,z_l,x_n,y_n,z_n;
double q_l,q_n;



void setVector(Eigen::VectorXd Y)
{
    Ze = Y;
    
}
void setMatrix(Eigen::MatrixXd S)
{
    Om = S;
}

//void prediction_step(const sensor_msgs::Imu::ConstPtr& msg)
void prediction_step()
{
    T = ros::Time::now().toSec() - m_last_range_time;
    int static count =0 ;
    if(count < 5)
       {ROS_WARN("prediction_step");
        count++;
        T = 0.0301735 ;}
    

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


    //std::cout << F << '\n'<<'\n'; 

    // Q is the acceleration model
    tao_bias = m_tao_bias_sqrt * m_tao_bias_sqrt;
    Q = Eigen::MatrixXd::Zero(15,15);
    
    block_Q << (T_cub*T_sq)/20.0,   (T_sq*T_sq)/8.0 ,   -T_cub/6,
        	   (T_sq*T_sq)/8.0 ,    (T_cub)/3,          -T_sq/2,
         	    T_cub/6.0,		     -T_sq/2 ,               T  ;
    block_Q *= tao_bias;


    Q.block<3,3>(0,0) = block_Q;
    Q.block<3,3>(3,3) = block_Q * m_y_damping_factor ;
    Q.block<3,3>(6,6) = block_Q * m_z_damping_factor;
    Q.block<3,3>(9,9) = Q_lead;
    Q.block<3,3>(12,12) = Q_neigh;
    Q *= m_Q_scale;
    //ROS_WARN("covariance:## %f, ## %f, ## %f",(T_cub*tao_acc/3.0)+(T_cub*T_sq)*tao_bias/20.0 , T*tao_acc+(T_cub*tao_bias/3), T*tao_bias);
   
    // if(count%50 == 1)
    // std::cout << X << '\n'<<'\n';
    // M is the predicted covariance matrix
    // X is the predicted state vector
    X = Om.inverse() * Ze;
    Om_p = (F *Om.inverse() * F.transpose()) + Q ;
    Om_p = Om_p.inverse();
    X = F * X ;
    Ze_p = Om_p * X;
   //  q = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
   //  R_mat= q.toRotationMatrix();
   //  imuacc << msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z;
   //  acc= R_mat*imuacc;
   //  B = Eigen::MatrixXd::Zero(15,15);
   //  block_B << T_sq/2.0,  0,  0,
   //             T      ,  0,  0,
   //             1      ,  0,  0;
   //  B.block<3,3>(0,0) = block_B;
   //  B.block<3,3>(3,3) = block_B;
   //  B.block<3,3>(6,6) = block_B;
   //  B.block<6,6>(9,9) = Eigen::MatrixXd::Identity(6,6);;
   //  u<<acc(0,0),0,0,
   //    -acc(1,0),0,0,
   // acc(2,0)-9.8,0,0,
   // 0,0,0,0,0,0; 

   //  X = F * X + B * u;
   //  Ze_p = Om_p * X; 
    // time update
    
    //count++;
    // setState(X_e);
    // setCovariance(M);
        
        pose.vector.x=X(0);
        pose.vector.y=X(3);
        pose.vector.z=X(6);

        
        vel.vector.x=X(1);
        vel.vector.y=X(4);
        vel.vector.z=X(7);


        acc_.vector.x=X(2);
        acc_.vector.y=X(5);
        acc_.vector.z=X(8);
    //    fused.publish(output);


    // if(error < error_threshold){
    //     //ROS_WARN("\n sucess too large: %f", error);
        setVector(Ze_p);
        setMatrix(Om_p);
        //std::cout << X << '\n'<<'\n';
    //     return ;
    // } else {

    //     ROS_WARN("\n Estimate too large: %f", error);
    //     return ;
    // }
    // count++;
    m_last_range_time = ros::Time::now().toSec();    
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

    r_meas_acc << acc(0,0),acc(1,0),acc(2,0);
    // K is the Kalman Gain
    R_acc = Eigen::MatrixXd::Identity(3,3) * (r_acc*r_acc);
    R_acc(1,1) = R_acc(1,1) ;
    R_acc(2,2) = R_acc(2,2) * m_z_damping_factor;
    //ROS_WARN("R_matrix:## %f",R);
    // Update the state
    R_acc = R_acc.inverse();
    Om_acc = Om + (H_acc.transpose() * R_acc * H_acc) ; 
    Ze_acc = Ze + (H_acc.transpose() * R_acc * r_meas_acc) ;

    X_acc = Om_acc.inverse() * Ze_acc;

if(abs(X_acc(0)-X(0)) < error_threshold && abs(X_acc(3)-X(3)) < error_threshold && abs(X_acc(6)-X(6)) < error_threshold )
    {   setVector(Ze_acc);
        setMatrix(Om_acc);
    }

}


void correction_step_vel(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    int static count =0 ;
    if(count == 0)
       {ROS_WARN("vel_step");
        count++; }


    r_meas_vel << msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z;
    // K is the Kalman Gain
    R_vel = Eigen::MatrixXd::Identity(3,3) * (r_vel*r_vel);
    R_vel(1,1) = R_vel(1,1) ;
    R_vel(2,2) = R_vel(2,2) * m_z_damping_factor;
    //ROS_WARN("R_matrix:## %f",R);
    R_vel = R_vel.inverse();
    Om_vel = Om + (H_vel.transpose() * R_vel * H_vel) ; 
    Ze_vel = Ze + (H_vel.transpose() * R_vel * r_meas_vel) ;

        setVector(Ze_vel);
        setMatrix(Om_vel);
    X_vel = Om_vel.inverse() * Ze_vel;

if(abs(X_vel(0)-X(0)) < error_threshold && abs(X_vel(3)-X(3)) < error_threshold && abs(X_vel(6)-X(6)) < error_threshold )
    {   setVector(Ze_vel);
        setMatrix(Om_vel);
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
    R_l = 1/R_l;
    Om_r_l = Om + (H_l.transpose() * R_l * H_l) ; 
    Ze_r_l = Ze + (H_l.transpose() * R_l * (r_meas_l - r_pred_l + (H_l*X)(0,0) )) ;

        
    X_r_l = Om_r_l.inverse() * Ze_r_l;

if(abs(X_r_l(0)-X(0)) < error_threshold && abs(X_r_l(3)-X(3)) < error_threshold && abs(X_r_l(6)-X(6)) < error_threshold )
    {   setVector(Ze_r_l);
        setMatrix(Om_r_l);
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
    R_n = 1/R_n;
    Om_r_n = Om + (H_n.transpose() * R_n * H_n) ; 
    Ze_r_n = Ze + (H_n.transpose() * R_n * (r_meas_n - r_pred_n + (H_n*X)(0,0) )) ;

        
    X_r_n = Om_r_n.inverse() * Ze_r_n;

if(abs(X_r_n(0)-X(0)) < error_threshold && abs(X_r_n(3)-X(3)) < error_threshold && abs(X_r_n(6)-X(6)) < error_threshold )
    {   setVector(Ze_r_n);
        setMatrix(Om_r_n);
    }

}


void position_lead_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    int static count =0 ;
    if(count == 0)
       {ROS_WARN("leader_pos_step");
        count++; }
    pos_meas_l << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
    // K is the Kalman Gain
    R_pos_l = Eigen::MatrixXd::Identity(3,3) * (r_pos_l*r_pos_l);
    //ROS_WARN("R_matrix:## %f",R);

    R_pos_l = R_pos_l.inverse();
    Om_pos_l = Om + (H_pos_l.transpose() * R_pos_l * H_pos_l) ; 
    Ze_pos_l = Ze + (H_pos_l.transpose() * R_pos_l * pos_meas_l) ;

    X_pos_l = Om_pos_l.inverse() * Ze_pos_l;

if(abs(X_pos_l(0)-X(0)) < error_threshold && abs(X_pos_l(3)-X(3)) < error_threshold && abs(X_pos_l(6)-X(6)) < error_threshold )
    {   setVector(Ze_pos_l);
        setMatrix(Om_pos_l);
    }
}



void position_neigh_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    int static count =0 ;
    if(count == 0)
       {ROS_WARN("neighbor_pos_step");
        count++; }
    pos_meas_n << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
    // K is the Kalman Gain
    R_pos_n = Eigen::MatrixXd::Identity(3,3) * (r_pos_n*r_pos_n);
    //ROS_WARN("R_matrix:## %f",R);

    R_pos_n = R_pos_n.inverse();
    Om_pos_n = Om + (H_pos_n.transpose() * R_pos_n * H_pos_n) ; 
    Ze_pos_n = Ze + (H_pos_n.transpose() * R_pos_n * pos_meas_n) ;

    X_pos_n = Om_pos_n.inverse() * Ze_pos_n;

if(abs(X_pos_n(0)-X(0)) < error_threshold && abs(X_pos_n(3)-X(3)) < error_threshold && abs(X_pos_n(6)-X(6)) < error_threshold )
    {   setVector(Ze_pos_n);
        setMatrix(Om_pos_n);
    }

}


void height_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    int static count =0 ;
    if(count == 0)
       {ROS_WARN("height_cb");
        count++; }
    h_meas = msg->pose.position.z;
    // K is the Kalman Gain
    R_h = r_h*r_h;
    //ROS_WARN("R_matrix:## %f",R);

    R_h = 1/R_h;
    Om_h = Om + (H_h.transpose() * R_h * H_h) ; 
    Ze_h = Ze + (H_h.transpose() * R_h * h_meas) ;

        setVector(Ze_h);
        setMatrix(Om_h);
    X_h = Om_h.inverse() * Ze_h;

if(abs(X_h(0)-X(0)) < error_threshold && abs(X_h(3)-X(3)) < error_threshold && abs(X_h(6)-X(6)) < error_threshold )
    {   setVector(Ze_h);
        setMatrix(Om_h);
    }


}



void param(ros::NodeHandle& nh)
{
    nh.getParam("IKalmanFilter_swarm/start_x", x);
    nh.getParam("IKalmanFilter_swarm/start_y", y);
    nh.getParam("IKalmanFilter_swarm/start_z", z);
    nh.getParam("IKalmanFilter_swarm/lead_x", x_l);
    nh.getParam("IKalmanFilter_swarm/lead_y", y_l);
    nh.getParam("IKalmanFilter_swarm/lead_z", z_l);
    nh.getParam("IKalmanFilter_swarm/neighbor_x", x_n);
    nh.getParam("IKalmanFilter_swarm/neighbor_y", y_n);
    nh.getParam("IKalmanFilter_swarm/neighbor_z", z_n);
    nh.getParam("IKalmanFilter_swarm/leader_covariance", q_l);
    nh.getParam("IKalmanFilter_swarm/neighbor_covariance", q_n);
    nh.getParam("IKalmanFilter_swarm/m_tao_bias_sqrt", m_tao_bias_sqrt );
    nh.getParam("IKalmanFilter_swarm/m_y_damping_factor", m_y_damping_factor);
    nh.getParam("IKalmanFilter_swarm/m_z_damping_factor", m_z_damping_factor);
    nh.getParam("IKalmanFilter_swarm/m_Q_scale", m_Q_scale);
    nh.getParam("IKalmanFilter_swarm/error_threshold", error_threshold);
    nh.getParam("IKalmanFilter_swarm/m_R_scale_leader", m_R_scale_l);
    nh.getParam("IKalmanFilter_swarm/precisionRangeErrEst_leader", precisionRangeErrEst_l);
    nh.getParam("IKalmanFilter_swarm/m_R_scale_neighbor", m_R_scale_n);
    nh.getParam("IKalmanFilter_swarm/precisionRangeErrEst_neighbor", precisionRangeErrEst_n);
    nh.getParam("IKalmanFilter_swarm/r_acc", r_acc);
    nh.getParam("IKalmanFilter_swarm/r_vel", r_vel);
    nh.getParam("IKalmanFilter_swarm/r_pos_l", r_pos_l);
    nh.getParam("IKalmanFilter_swarm/r_pos_n", r_pos_n);
    nh.getParam("IKalmanFilter_swarm/r_h", r_h);

    ROS_WARN("%f, %f, %f ",x,y,z);
}


void Initialize(ros::NodeHandle& nh)
{
 
    param(nh);

    X<<x,0,0,y,0,0,z,0,0,x_l,y_l,z_l,x_n,y_n,z_n; 
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
    nine_cov = nine_cov*100;
    nine_cov = nine_cov.inverse();
    setMatrix(nine_cov);
    setVector(nine_cov*X);

  
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

    H_h << 0,0,0,0,0,0,1,0,0,0,0,0,0,0,0;

    m_last_range_time = ros::Time::now().toSec();

}


int main(int argc, char** argv){

    ros::init(argc,argv,"IKalmanFilter_swarm");
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
    ros::Subscriber height = nh.subscribe("height",20,height_cb);

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
		pose.header.stamp = ros::Time::now();
        vel.header.stamp = ros::Time::now();
        acc_.header.stamp = ros::Time::now();
        fused_pose.publish(pose);
        fused_vel.publish(vel);
        fused_acc.publish(acc_);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}  