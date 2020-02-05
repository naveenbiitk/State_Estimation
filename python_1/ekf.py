
import math

import matplotlib.pyplot as plt
import numpy as np


DT = 0.1  # time tick [s]
# Covariance for EKF simulation
Q = np.diag([
    0.480,  # variance of location on x-axis
    0.480,  # variance of location on y-axis
    np.deg2rad(10.5),  # variance of yaw angle
    0.9  # variance of velocity
]) ** 2  # predict state covariance

#np.diag([2.0, np.deg2rad(40.0)]) ** 2

R = np.diag([525]) ** 2 # Observation x,y position covariance



def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F.dot(x) + B.dot(u)

    return x




def jacob_f(x, u):
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF



def ekf_estimation(xEst, PEst, z, u):
    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacob_f(xPred, u)
    PPred = np.matmul( jF , np.matmul(PEst , jF.T) ) + Q

    #  Update
    # xEst = xPred
    # PEst = PPred
    # print(xEst)
    for i in range(len(z[:, 0])):
    	dx = xEst[0, 0] - z[i, 1]
    	dy = xEst[1, 0] - z[i, 2]
    	zPred = (dx*dx) + (dy*dy)

        y = (z[i,0]*z[i,0]) - zPred
        #print(str(z[i,0])+' '+str(zPred))
        #if( y < 4):
        jH = np.array([
       		     [2*(xEst[0,0]-z[i,1]),
	   	    	  2*(xEst[1,0]-z[i,2]), 0, 0] ])

        S = np.matmul( jH , np.matmul( PPred , jH.T) ) + R
        K = np.matmul( PPred ,  (jH.T/S) )

        xEst = xPred + (K*y) 
        #print('K*y is'+str(K[0,0]*y))
    	PEst = np.matmul( (np.eye(len(xEst)) - np.matmul(K , jH)) , PPred )

    return xEst, PEst
