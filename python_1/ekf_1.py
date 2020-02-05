
import math

import matplotlib.pyplot as plt
import numpy as np


DT = 0.1  # time tick [s]
# Covariance for EKF simulation
Q = np.diag([
    0.020,  # variance of location on x-axis
    0.020,  # variance of location on y-axis
    np.deg2rad(1.05),  # variance of yaw angle
    0.09  # variance of velocity
]) ** 2  # predict state covariance

#np.diag([2.0, np.deg2rad(40.0)]) ** 2

R = np.diag([0.3, 0.3]) ** 2 # Observation x,y position covariance



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


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = np.matmul(H,x)

    return z


def ekf_estimation(xEst, PEst, z, u):
    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacob_f(xPred, u)
    PPred = np.matmul( jF , np.matmul(PEst , jF.T) ) + Q

    #  Update
    # xEst = xPred
    # PEst = PPred
    #print(xEst)
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    zPred = observation_model(xPred)

    y = z- zPred 
    S = np.matmul( jH , np.matmul( PPred , jH.T) ) + R
    K = np.matmul( PPred ,  np.matmul( jH.T, np.linalg.inv(S) ) )

    xEst = xPred + np.matmul(K,y) 
        #print('K*y is'+str(K[0,0]*y))
    PEst = np.matmul( (np.eye(len(xEst)) - np.matmul(K , jH)) , PPred )

        
    return xEst, PEst
