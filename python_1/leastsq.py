import math

import matplotlib.pyplot as plt
import numpy as np

def lsq_estimation(X_est, z, u):
	dt=0.01
	Ax = z[0, 1]
	Ay = z[0, 2]
	Asq = (Ax*Ax) + (Ay*Ay)
	d1sq = z[0,0]*z[0,0]
	
	Bx = z[1, 1]
	By = z[1, 2]
	Bsq = (Bx*Bx) + (By*By)
	d2sq = z[1,0]*z[1,0]
	
	Cx = z[2, 1]
	Cy = z[2, 2]
	Csq = (Cx*Cx) + (Cy*Cy)
	d3sq = z[2,0]*z[2,0]
	
	Dx = z[3, 1]
	Dy = z[3, 2]
	Dsq = (Dx*Dx) + (Dy*Dy)
	d4sq = z[3,0]*z[3,0]        
	
	H = np.array([
			[Bx-Ax, By-Ay],
			[Cx-Ax, Cy-Ay],
			[Dx-Ax, Dy-Ay]
		])
	H = 2*H
	Y = np.array([
			[d1sq - d2sq + Bsq - Asq],
			[d1sq - d3sq + Csq - Asq],
			[d1sq - d4sq + Dsq - Asq]
		])
	
	X =  np.matmul(np.matmul( np.linalg.inv( np.matmul( H.T, H) ) , H.T), Y)
	
	X_est[0] = X[0]
	X_est[1] = X[1]
	X_est[2] = X_est[2]+u[1]*dt
	X_est[3] = u[0]
	
	return X_est	