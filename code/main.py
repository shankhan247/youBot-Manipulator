import modern_robotics as mr
import numpy as np 
import csv
import matplotlib.pyplot as plt

error = np.zeros((N,6))
"""
To use this code, run via IDE.
I noticed that for some reason when I run this code via command line, it does not seem to update the CSV file.
I copy and paste this code into iPython to run, and then import CSV file into V-REP.
"""
def TrajectoryGenerator(N):

    # define inputs
    # initial config of end effector to s frame
    #Tsei = np.array([[1,0,0,0.293],[0,1,0,0.094],[0,0,1,0.848],[0,0,0,1]]) 
    Tsei = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]]) 
    # grasping config of end effector frame to initial cube frame
    Tceg = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]]) 
    # standoff config of end effector to initial cube frame
    Tces = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.15],[0,0,0,1]])
    # initial cube config to s frame
    Tsci = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
    #Tsci = np.array([[0,0,1,1.2],[0,1,0,0],[-1,0,0,0.025],[0,0,0,1]]) # custom cube config for newTask
    # desired sube config to s frame
    Tscd = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])
    #Tscd = np.array([[0,0,1,0],[0,-1,0,-1.2],[-1,0,0,0.025],[0,0,0,1]]) # custom cube config for newTask

    # finding the grasping config of end effector to s frame
    Tseg = np.matmul(Tsci, Tceg)
    # finding the standoff config of end effector to s frame
    Tses = np.matmul(Tsci, Tces)

    # step 1
    traj1 = mr.ScrewTrajectory(Tsei,Tses,3,300,5)
    # step 2
    traj2 = mr.ScrewTrajectory(Tses,Tseg,1,100,5)
    # step 4
    traj4 = mr.ScrewTrajectory(Tseg,Tses,1,100,5)
    # step 5
    Tsesf = np.matmul(Tscd, Tces)
    traj5 = mr.ScrewTrajectory(Tses,Tsesf,3,300,5)
    # step 6
    Tsesd = np.matmul(Tscd, Tceg)
    traj6 = mr.ScrewTrajectory(Tsesf,Tsesd,1,100,5)
    # step 7
    traj7 = mr.ScrewTrajectory(Tsesd,Tsesf,1,100,5)

    # Saving trajectory in CSV file
    traj = np.zeros((N,13))
    for i in range(300):
        traj[i][0] = traj1[i][0][0]
        traj[i][1] = traj1[i][0][1]
        traj[i][2] = traj1[i][0][2]
        traj[i][3] = traj1[i][1][0]
        traj[i][4] = traj1[i][1][1]
        traj[i][5] = traj1[i][1][2]
        traj[i][6] = traj1[i][2][0]
        traj[i][7] = traj1[i][2][1]
        traj[i][8] = traj1[i][2][2]
        traj[i][9] = traj1[i][0][3]
        traj[i][10] = traj1[i][1][3]
        traj[i][11] = traj1[i][2][3]
        traj[i][12] = 0
    for i in range(300,400):
        traj[i][0] = traj2[i-300][0][0]
        traj[i][1] = traj2[i-300][0][1]
        traj[i][2] = traj2[i-300][0][2]
        traj[i][3] = traj2[i-300][1][0]
        traj[i][4] = traj2[i-300][1][1]
        traj[i][5] = traj2[i-300][1][2]
        traj[i][6] = traj2[i-300][2][0]
        traj[i][7] = traj2[i-300][2][1]
        traj[i][8] = traj2[i-300][2][2]
        traj[i][9] = traj2[i-300][0][3]
        traj[i][10] = traj2[i-300][1][3]
        traj[i][11] = traj2[i-300][2][3]
        traj[i][12] = 0
    for i in range(400,470):
        traj[i][0] = traj2[99][0][0]
        traj[i][1] = traj2[99][0][1]
        traj[i][2] = traj2[99][0][2]
        traj[i][3] = traj2[99][1][0]
        traj[i][4] = traj2[99][1][1]
        traj[i][5] = traj2[99][1][2]
        traj[i][6] = traj2[99][2][0]
        traj[i][7] = traj2[99][2][1]
        traj[i][8] = traj2[99][2][2]
        traj[i][9] = traj2[99][0][3]
        traj[i][10] = traj2[99][1][3]
        traj[i][11] = traj2[99][2][3]
        traj[i][12] = 1
    for i in range(470,570):
        traj[i][0] = traj4[i-470][0][0]
        traj[i][1] = traj4[i-470][0][1]
        traj[i][2] = traj4[i-470][0][2]
        traj[i][3] = traj4[i-470][1][0]
        traj[i][4] = traj4[i-470][1][1]
        traj[i][5] = traj4[i-470][1][2]
        traj[i][6] = traj4[i-470][2][0]
        traj[i][7] = traj4[i-470][2][1]
        traj[i][8] = traj4[i-470][2][2]
        traj[i][9] = traj4[i-470][0][3]
        traj[i][10] = traj4[i-470][1][3]
        traj[i][11] = traj4[i-470][2][3]
        traj[i][12] = 1
    for i in range(570,870):
        traj[i][0] = traj5[i-570][0][0]
        traj[i][1] = traj5[i-570][0][1]
        traj[i][2] = traj5[i-570][0][2]
        traj[i][3] = traj5[i-570][1][0]
        traj[i][4] = traj5[i-570][1][1]
        traj[i][5] = traj5[i-570][1][2]
        traj[i][6] = traj5[i-570][2][0]
        traj[i][7] = traj5[i-570][2][1]
        traj[i][8] = traj5[i-570][2][2]
        traj[i][9] = traj5[i-570][0][3]
        traj[i][10] = traj5[i-570][1][3]
        traj[i][11] = traj5[i-570][2][3]
        traj[i][12] = 1
    for i in range(870,970):
        traj[i][0] = traj6[i-870][0][0]
        traj[i][1] = traj6[i-870][0][1]
        traj[i][2] = traj6[i-870][0][2]
        traj[i][3] = traj6[i-870][1][0]
        traj[i][4] = traj6[i-870][1][1]
        traj[i][5] = traj6[i-870][1][2]
        traj[i][6] = traj6[i-870][2][0]
        traj[i][7] = traj6[i-870][2][1]
        traj[i][8] = traj6[i-870][2][2]
        traj[i][9] = traj6[i-870][0][3]
        traj[i][10] = traj6[i-870][1][3]
        traj[i][11] = traj6[i-870][2][3]
        traj[i][12] = 1
    for i in range(970,1040):
        traj[i][0] = traj6[99][0][0]
        traj[i][1] = traj6[99][0][1]
        traj[i][2] = traj6[99][0][2]
        traj[i][3] = traj6[99][1][0]
        traj[i][4] = traj6[99][1][1]
        traj[i][5] = traj6[99][1][2]
        traj[i][6] = traj6[99][2][0]
        traj[i][7] = traj6[99][2][1]
        traj[i][8] = traj6[99][2][2]
        traj[i][9] = traj6[99][0][3]
        traj[i][10] = traj6[99][1][3]
        traj[i][11] = traj6[99][2][3]
        traj[i][12] = 0
    for i in range(1040,1140):
        traj[i][0] = traj7[i-1040][0][0]
        traj[i][1] = traj7[i-1040][0][1]
        traj[i][2] = traj7[i-1040][0][2]
        traj[i][3] = traj7[i-1040][1][0]
        traj[i][4] = traj7[i-1040][1][1]
        traj[i][5] = traj7[i-1040][1][2]
        traj[i][6] = traj7[i-1040][2][0]
        traj[i][7] = traj7[i-1040][2][1]
        traj[i][8] = traj7[i-1040][2][2]
        traj[i][9] = traj7[i-1040][0][3]
        traj[i][10] = traj7[i-1040][1][3]
        traj[i][11] = traj7[i-1040][2][3]
        traj[i][12] = 0
    
    np.savetxt("trajectory.csv",traj,delimiter=",")

    return traj


def NextStates(config,u,thetadot,step,max_speed,r,w,l):

    for k in range(4):
        if u[k] > max_speed:
            u[k] = max_speed
        if u[k] < -max_speed:
            u[k] = -max_speed
    for k in range(5):
        if thetadot[k] > max_speed:
            thetadot[k] = max_speed
        if thetadot[k] < -max_speed:
            thetadot[k] = -max_speed        
    new_Jangles = config[3:8]+thetadot*step # computing new joint angles using Euler step
    new_Wangles = config[8:12]+u*step # computing new wheel angles using Euler step
    # finding chassis config using odometry
    delta_theta = (new_Wangles - config[8:12]) # getting wheel displacement
    Vb = np.dot((r/4*np.array([[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],[1,1,1,1],[-1,1,-1,1]])),delta_theta)
    Vb6 = np.array([0,0,Vb[0],Vb[1],Vb[2],0]) # twist in 6D
    Vb6_skew_angvel = mr.VecToso3(Vb6[:3]) # skew symmetry of spatial twist angular velocities
    Vb6_linvel = np.array([[Vb6[3],Vb6[4],Vb6[5]]]) # array of linear velocities from spatial twist
    mat_Vb6 = np.concatenate((Vb6_skew_angvel,Vb6_linvel.T),axis=1)
    last_row = np.array([[0,0,0,0]])
    mat_Vb6 = np.concatenate((mat_Vb6,last_row)) # 4x4 matrix of spatial twist
    Tbibi1 = mr.MatrixExp6(mat_Vb6) # matrix exponential to find config of chassis at i+1 relative to i
    Tsbi = np.array([[np.cos(config[0]),-np.sin(config[0]),0,config[1]],[np.sin(config[0]),np.cos(config[0]),0,config[2]],
    [0,0,1,0.0963],[0,0,0,1]]) # config of chassis at i relative to s
    Tsbi1 = np.matmul(Tsbi,Tbibi1) # config of chassis at current iteration relative to s
    # finding chassis configs
    chass_phi = np.arccos(Tsbi1[0][0])
    chass_x = Tsbi1[0][3]
    chass_y = Tsbi1[1][3]
    new_config = np.array([chass_phi,chass_x,chass_y,new_Jangles[0],new_Jangles[1],new_Jangles[2],new_Jangles[3],new_Jangles[4],new_Wangles[0],new_Wangles[1],new_Wangles[2],new_Wangles[3],0])

    return new_config

def FeedBackControl(J_angles,Tsb,Xd,Xdnext,i):

    # first find T0e
    M0e = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]]) # end effector frame relative to base frame of arm at home config
    B =  np.array([[0,0,0,0,0],[0,-1,-1,-1,0],[1,0,0,0,1],[0,-0.5076,-0.3526,-0.2176,0],[0.033,0,0,0,0],[0,0,0,0,0]]) # screw axes for arm at home config relative to end effector frame

    T0e = mr.FKinBody(M0e,B,J_angles)
    # Get Tsb from milestone1 code (Tsbi1)

    # Define Tb0 (fixed offset)
    Tb0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])

    # Compute Tse/X
    X = np.matmul(np.matmul(Tsb,Tb0),T0e)
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
    Jbase = np.dot(Adj_Teb,F6)
    Je = np.concatenate((Jbase,Jarm),axis=1)

    # Computing the kinematic task-space feedforward plus feedback control law
    #Kp = np.zeros((6,6))
    Kp = np.identity(6)*2
    #Ki = np.zeros((6,6))
    Ki = np.identity(6)*2
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
    error[i] = Xerr
    # Compute command end effector twist
    Xerr_int = Xerr*step
    V = Vd_Adj + np.dot(Kp,Xerr) + np.dot(Ki,Xerr_int)
    Je_ps = np.linalg.pinv(Je)
    WJ_speeds = np.dot(Je_ps,V)

    return WJ_speeds

# Calling function to generate trajectories
N = 1140
traj = TrajectoryGenerator(N) 

# intial configuration of youBot
chassx = 0
chassy = 0
chassphi = 0
J1 = 0
J2 = 0
J3 = 0
J4 = 0
J5 = 0
W1 = 0
W2 = 0
W3 = 0
W4 = 0
config = np.zeros((N,13))
config[0][:14] = [chassphi,chassx,chassy,J1,J2,J3,J4,J5,W1,W2,W3,W4,0]

for i in range(N-1):
    J_angles = config[i][3:8] # joint coordinates 1-5
    Tsbi = np.array([[np.cos(config[i][0]),-np.sin(config[i][0]),0,config[i][1]],[np.sin(config[i][0]),np.cos(config[i][0]),0,config[i][2]],
    [0,0,1,0.0963],[0,0,0,1]]) # config of chassis at i relative to s
    Xd = np.array([[traj[i][0],traj[i][1],traj[i][2],traj[i][9]],[traj[i][3],traj[i][4],traj[i][5],traj[i][10]],
    [traj[i][6],traj[i][7],traj[i][8],traj[i][11]],[0,0,0,1]])
    Xdnext = np.array([[traj[i+1][0],traj[i+1][1],traj[i+1][2],traj[i+1][9]],[traj[i+1][3],traj[i+1][4],traj[i+1][5],traj[i+1][10]],
    [traj[i+1][6],traj[i+1][7],traj[i+1][8],traj[i+1][11]],[0,0,0,1]])
    speeds = FeedBackControl(J_angles,Tsbi,Xd,Xdnext,i)
    u = speeds[:4] # extract wheel speeds
    thetadot = speeds[4:9] # extract joint speeds
    step = 0.01
    max_speed = 12.3
    l = 0.47/2 # half the forward-backward distance of wheels (l)
    w = 0.3/2 # half the side-side distance of wheels (w) in meters
    r = 0.0475 # radius of each wheel
    config[i+1] = NextStates(config[i],u,thetadot,step,max_speed,r,w,l)
    config[i+1][12] = traj[i+1][12] # gripper state 

np.savetxt("configs_best.csv",config,delimiter=",")

# Plot Xerr
time = np.linspace(0,11.4,1140)
err1 = np.zeros((N))
err2 = np.zeros((N))
err3 = np.zeros((N))
err4 = np.zeros((N))
err5 = np.zeros((N))
err6 = np.zeros((N))
for i in range(N):
    err1[i] = error[i][0]
    err2[i] = error[i][1]
    err3[i] = error[i][2]
    err4[i] = error[i][3]
    err5[i] = error[i][4]
    err6[i] = error[i][5]

plt.plot(err1,time)
plt.plot(err2,time)
plt.plot(err3,time)
plt.plot(err4,time)
plt.plot(err5,time)
plt.plot(err6,time)
plt.xlabel('time (s)')
plt.ylabel('error')
plt.title('Xerr Plot')
plt.show()

np.savetxt("log_Xerr",error,delimiter=",")
