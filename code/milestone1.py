import modern_robotics as mr
import numpy as np 
import csv

def NextStates(config,u,thetadot,step,max_speed,r,w,l,CSVFile):
    for i in range(100):
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
        config = np.array([chass_phi,chass_x,chass_y,new_Jangles[0],new_Jangles[1],new_Jangles[2],new_Jangles[3],new_Jangles[4],new_Wangles[0],new_Wangles[1],new_Wangles[2],new_Wangles[3],0])
        print chass_phi
        print chass_x
        # writing config to csv file
        for k in range(12):
            CSVFile[i+1][k] = config[k]
        print config

    return CSVFile


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

config = np.array([chassphi,chassx,chassy,J1,J2,J3,J4,J5,W1,W2,W3,0])
CSVFile = np.zeros((101,12))
for i in range(12):
    CSVFile[0][i] = config[i]

# Joint speed and wheel speed controls
Jspeed_ctrl = np.array([5,5,5,5,5])
W_ctrl = np.array([10,10,10,10])

# defining timestep in s
step = 0.01

# maximum angular speeds of joints and wheels
max_speed = 12.3

# half the forward-backward distance of wheels (l) and half the side-side distance of wheels (w) in meters
l = 0.47/2
w = 0.3/2

# radius of each wheel
r = 0.0475

CSV = NextStates(config,W_ctrl,Jspeed_ctrl,step,max_speed,r,w,l,CSVFile)
np.savetxt("configs_x.csv",CSV,delimiter=",")