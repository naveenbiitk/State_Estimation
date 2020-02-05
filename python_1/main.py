import math

import matplotlib.pyplot as plt
import numpy as np
from pf import pf_localization
from ekf_1 import ekf_estimation
from leastsq import lsq_estimation



#  Simulation parameter
# Q_sim = np.diag([0.2]) ** 2
# R_sim = np.diag([1.0, np.deg2rad(30.0)]) ** 2
Q_sim = np.diag([0.4]) ** 2
R_sim = np.diag([0.8, np.deg2rad(10.0)]) ** 2

count =0
DT = 0.1  # time tick [s]
SIM_TIME = 65.0  # simulation time [s]
MAX_RANGE = 200.0  # maximum observation range


show_animation = True

#constant velocity inuput
def calc_input():
    v = 1.0  # [m/s]
    yaw_rate = 0.1  # [rad/s]
    u = np.array([[v, yaw_rate]]).T
    return u




def observation(xTrue, xd, u, RF_ID):
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.zeros((0, 3))

    for i in range(len(RF_ID[:, 0])):

        dx = xTrue[0, 0] - RF_ID[i, 0]
        dy = xTrue[1, 0] - RF_ID[i, 1]
        d = math.hypot(dx, dy)
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Q_sim[0, 0] ** 0.5  # add noise
            zi = np.array([[dn, RF_ID[i, 0], RF_ID[i, 1]]])
            z = np.vstack((z, zi))

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0] ** 0.5
    ud2 = u[1, 0] + np.random.randn() * R_sim[1, 1] ** 0.5
    ud = np.array([[ud1, ud2]]).T

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


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




def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eig_val, eig_vec = np.linalg.eig(Pxy)

    if eig_val[0] >= eig_val[1]:
        big_ind = 0
        small_ind = 1
    else:
        big_ind = 1
        small_ind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)

    # eig_val[big_ind] or eiq_val[small_ind] were occasionally negative numbers extremely
    # close to 0 (~10^-20), catch these cases and set the respective variable to 0
    try:
        a = math.sqrt(eig_val[big_ind])
    except ValueError:
        a = 0

    try:
        b = math.sqrt(eig_val[small_ind])
    except ValueError:
        b = 0

    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eig_vec[big_ind, 1], eig_vec[big_ind, 0])
    Rot = np.array([[math.cos(angle), -math.sin(angle)],
                    [math.sin(angle), math.cos(angle)]])
    fx = Rot.dot(np.array([[x, y]]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():
    print(__file__ + " start!!")
    count = 0.0
    time = 0.0

    # RF_ID positions [x, y]
    RFi_ID = np.array([[15.0,  0.0],
                       [15.0,  20.0],
                       [-15.0, 0.0],
                       [-15.0, 20.0]])

    # State Vector [x y yaw v]'
    xEst_1 = np.zeros((4, 1))
    xEst_2 = np.zeros((4, 1))
    xEst_3 = np.zeros((4, 1))
    xTrue = np.zeros((4, 1))
    PEst_2 = np.eye(4)
    # Particle filter parameter
    NP = 100  # Number of Particle
    NTh = NP / 2.0  # Number of particle for re-sampling
    px = np.zeros((4, NP))  # Particle store
    pw = np.zeros((1, NP)) + 1.0 / NP  # Particle weight
    xDR = np.zeros((4, 1))  # Dead reckoning

    # history
    hxEst_1 = np.zeros((4, 1))
    hxEst_2 = np.zeros((4, 1))
    hxEst_3 = np.zeros((4, 1))
    hxTrue = xTrue
    hxDR = np.zeros((4, 1))
    hxcount = np.zeros((1, 1))
    temp = np.zeros((1, 1))

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFi_ID)

        xEst_1, PEst_1, px, pw = pf_localization(px, pw, z, ud) # particle filter
        
        xEst_3 = lsq_estimation(xEst_3, z, ud) # least square filter

        z_new = np.array([[ xEst_3[0,0] ], [xEst_3[1,0] ]])

        xEst_2, PEst_2 = ekf_estimation(xEst_2,PEst_2, z_new, ud) # extended kalman filter


        # store data history
        hxEst_1 = np.hstack((hxEst_1, xEst_1))
        hxEst_2 = np.hstack((hxEst_2, xEst_2))
        hxEst_3 = np.hstack((hxEst_3, xEst_3))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))


#For comparing the error between different filters
        # count = count + 1
        # temp[0,0] = math.hypot(xTrue[0,0]- xEst_1[0,0], xTrue[1,0]- xEst_1[1,0]) 
        # hxEst_1 = np.hstack((hxEst_1,temp ))
        # temp[0,0] = math.hypot(xTrue[0,0]- xEst_2[0,0], xTrue[1,0]- xEst_2[1,0])
        # hxEst_2 = np.hstack((hxEst_2, temp))
        # temp[0,0] = math.hypot(xTrue[0,0]- xEst_3[0,0], xTrue[1,0]- xEst_3[1,0])
        # hxEst_3 = np.hstack((hxEst_3, temp))
        # temp[0,0] = math.hypot(xTrue[0,0]- xDR[0,0], xTrue[1,0]- xDR[1,0])
        # hxDR = np.hstack((hxDR, temp))
        # temp[0,0] = count        
        # hxcount = np.hstack((hxcount, temp))

        
        #print(hxcount.shape)
        #print(hxEst.shape)
        # print('\n')
        #print('True '+str(xTrue[0,0])+' '+str(xEst_3[0,0])+' '+str(xTrue[1,0])+' '+str(xEst_3[1,0]))
        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

            plt.gcf().set_size_inches(25, 25)
            plt.axis(xlim=(-25, 25), ylim=(-10, 30))

            for i in range(len(z[:, 0])):
                plt.plot([xTrue[0, 0], z[i, 1]], [xTrue[1, 0], z[i, 2]], "-k")


            plt.plot(RFi_ID[:, 0], RFi_ID[:, 1], "*k")
            plt.plot(px[0, :], px[1, :], ".r")
            plt.plot(np.array(hxTrue[0, :]).flatten(),
                     np.array(hxTrue[1, :]).flatten(), "-b",label='Original path')
            plt.plot(np.array(hxDR[0, :]).flatten(),
                       np.array(hxDR[1, :]).flatten(), "k",label='Dead Reckoning')
            plt.plot(np.array(hxEst_1[0, :]).flatten(),
                      np.array(hxEst_1[1, :]).flatten(), "-r",label='Particle filter')
            plt.plot(np.array(hxEst_3[0, :]).flatten(),
                      np.array(hxEst_3[1, :]).flatten(), "y", label='Least square')
            plt.plot(np.array(hxEst_2[0, :]).flatten(),
                      np.array(hxEst_2[1, :]).flatten(), "g", label='Extended Kalman filter')

#For comparing the total error
            # plt.plot(hxcount[0],hxDR[0], "k",label='Dead Reckoning')
            # plt.plot(hxcount[0, :],hxEst_1[0, :], "-r",label='Particle filter')
            # plt.plot(hxcount[0, :],hxEst_3[0, :], "y", label='Least square')
            # plt.plot(hxcount[0, :],hxEst_2[0, :], "g", label='Extended Kalman filter')
            
            #plot_covariance_ellipse(xEst_1, PEst_1)
            #plt.axis("equal")
            plt.legend()
            plt.grid(True)
            plt.pause(0.001)

            # if time == 59:
            #     plt.savefig('books_read.png')
            #     print('yes')


if __name__ == '__main__':
    main()