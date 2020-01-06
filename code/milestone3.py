import modern_robotics as mr
import numpy as np 
import csv
from numpy import genfromtxt

# finding Tse/X
configs = genfromtxt('configs_x.csv', delimiter=',')
# first find T0e
M0e = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]]) # end effector frame relative to base frame of arm at home config
B =  np.array([[0,0,0,0,0],[0,-1,-1,-1,0],[1,0,0,0,1],[0,-0.5076,-0.3526,-0.2176,0],[0.033,0,0,0,0],[0,0,0,0,0]]) # screw axes for arm at home config relative to end effector frame
#J_angles = configs[0][3:8] # list of joint coordinates from CSV file
J_angles = np.array([0,0,0.2,-1.6,0])
T0e = mr.FKinBody(M0e,B,J_angles)
# Get Tsb from milestone1 code (Tsbi1)

# Define Tb0 (fixed offset)
Tb0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])

# Compute Tse/X
#X = np.matmul(np.matmul(Tsb,Tb0),T0e)
X = np.array([[0.170,0,0.985,0.387],[0,1,0,0],[-0.985,0,0.170,0.570],[0,0,0,1]])

# Compute end effector Jacobian (Finding Jbase and Jarm)
Jarm = mr.JacobianBody(B,J_angles) # arm jacobian equivalent to jacobian body
# finding Jbase
l = 0.47/2 # half the forward-backward distance of wheels (l)
w = 0.3/2 # half the side-side distance of wheels (w) in meters
r = 0.0475 # radius of each wheel 
F6 = r/4*np.array([[0,0,0,0],[0,0,0,0],[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],[1,1,1,1],[-1,1,-1,1],[0,0,0,0]])
Te0 = mr.TransInv(T0e) # getting inverse of T0e
T0b = mr.TransInv(Tb0) # getting inverse of Tb0
Teb = np.matmul(Te0,T0b)
Adj_Teb = mr.Adjoint(Teb) # adjoint of Teb
#u = np.array([10,10,10,10]) # wheel speeds
Jbase = np.dot(Adj_Teb,F6)
Je = np.concatenate((Jbase,Jarm),axis=1)

# Computing the kinematic task-space feedforward plus feedback control law
Xd = np.array([[0,0,1,0.5],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
Xdnext = np.array([[0,0,1,0.6],[0,1,0,0],[-1,0,0,0.3],[0,0,0,1]])
#Kp = np.zeros((6,6))
Kp = np.identity(6)
#Ki = np.zeros((6,6))
Ki = np.identity(6)
step = 0.01
# Computing Vd
Xd_inv = mr.TransInv(Xd)
skew_Vd = (1/step)*mr.MatrixLog6(np.matmul(Xd_inv,Xdnext))
Vd = mr.se3ToVec(skew_Vd)
# Computing AdjointX
X_inv = mr.TransInv(X)
X_adj = np.matmul(X_inv,Xd)
Adj_X = mr.Adjoint(X_adj)
Vd_Adj = np.dot(Adj_X,Vd)
# Computing error twist Xerr
skew_Xerr = mr.MatrixLog6(X_adj)
Xerr = mr.se3ToVec(skew_Xerr)
# Compute command end effector twist
Xerr_int = Xerr*step
V = Vd_Adj + np.dot(Kp,Xerr) + np.dot(Ki,Xerr_int)
Je_ps = np.linalg.pinv(Je)
WJ_speeds = np.dot(Je_ps,V)
