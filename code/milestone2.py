import modern_robotics as mr
import numpy as np 
import csv

# define inputs
# initial config of end effector to s frame
Tsei = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.5],[0,0,0,1]]) 
# grasping config of end effector frame to initial cube frame
Tceg = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]]) 
# standoff config of end effector to initial cube frame
Tces = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.15],[0,0,0,1]])
# initial cube config to s frame
Tsci = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
# desired sube config to s frame
Tscd = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])


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

with open('step1_traj','wb') as jfile:
        writer = csv.writer(jfile)
        # write trajectory values generated for each iteration
        for i in range(300):
            writer.writerow([traj1[i][0][0], traj1[i][0][1], traj1[i][0][2], 
            traj1[i][1][0], traj1[i][1][1], traj1[i][1][2],
            traj1[i][2][0], traj1[i][2][1], traj1[i][2][2], 
            traj1[i][0][3], traj1[i][1][3], traj1[i][2][3], 0])
        for i in range(100):
            writer.writerow([traj2[i][0][0], traj2[i][0][1], traj2[i][0][2], 
            traj2[i][1][0], traj2[i][1][1], traj2[i][1][2],
            traj2[i][2][0], traj2[i][2][1], traj2[i][2][2], 
            traj2[i][0][3], traj2[i][1][3], traj2[i][2][3], 0])
        for i in range(70):
            writer.writerow([traj2[99][0][0], traj2[99][0][1], traj2[99][0][2], 
            traj2[99][1][0], traj2[99][1][1], traj2[99][1][2],
            traj2[99][2][0], traj2[99][2][1], traj2[99][2][2], 
            traj2[99][0][3], traj2[99][1][3], traj2[99][2][3], 1])
        for i in range(100):
            writer.writerow([traj4[i][0][0], traj4[i][0][1], traj4[i][0][2], 
            traj4[i][1][0], traj4[i][1][1], traj4[i][1][2],
            traj4[i][2][0], traj4[i][2][1], traj4[i][2][2], 
            traj4[i][0][3], traj4[i][1][3], traj4[i][2][3], 1])    
        for i in range(300):
            writer.writerow([traj5[i][0][0], traj5[i][0][1], traj5[i][0][2], 
            traj5[i][1][0], traj5[i][1][1], traj5[i][1][2],
            traj5[i][2][0], traj5[i][2][1], traj5[i][2][2], 
            traj5[i][0][3], traj5[i][1][3], traj5[i][2][3], 1])
        for i in range(100):
            writer.writerow([traj6[i][0][0], traj6[i][0][1], traj6[i][0][2], 
            traj6[i][1][0], traj6[i][1][1], traj6[i][1][2],
            traj6[i][2][0], traj6[i][2][1], traj6[i][2][2], 
            traj6[i][0][3], traj6[i][1][3], traj6[i][2][3], 1])
        for i in range(70):
            writer.writerow([traj6[99][0][0], traj6[99][0][1], traj6[99][0][2], 
            traj6[99][1][0], traj6[99][1][1], traj6[99][1][2],
            traj6[99][2][0], traj6[99][2][1], traj6[99][2][2], 
            traj6[99][0][3], traj6[99][1][3], traj6[99][2][3], 0])            
        for i in range(100):
            writer.writerow([traj7[i][0][0], traj7[i][0][1], traj7[i][0][2], 
            traj7[i][1][0], traj7[i][1][1], traj7[i][1][2],
            traj7[i][2][0], traj7[i][2][1], traj7[i][2][2], 
            traj7[i][0][3], traj7[i][1][3], traj7[i][2][3], 0])                


    
