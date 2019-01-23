# This program implements an LQR Controller for the 3D Drone drone dynamics using 
# differential flatness property as state and input references generator.
# 
# The system matrix corresponds to the 'linearization' of the system
# done through the computed torque method. 
#
# For a reference of the implemented system refer to: Scholarsarchive, B., Mclain, T., Beard, R. W., Mclain, T. ;, Beard, R. W. ;, Leishman, R. C. ;,Mclain, T. (2011).
# Differential Flatness Based Control of a Rotorcraft For Aggressive Maneuvers BYU ScholarsArchive Citation Differential Flatness Based Control of a Rotorcraft For
# Aggressive Maneuvers, (September), 2688-2693. Retrieved from https://scholarsarchive.byu.edu/facpub%0Ahttps://scholarsarchive.byu.edu/facpub/1949
#
# For a reference of the differential flatness proof for the drone dynamics refer
# to: Mellinger, D.,  Kumar, V. (2011). Minimum snap trajectory generation and control for quadrotors. Proceedings - IEEE International Conference on Robotics and 
# Automation, 2520-2525. https://doi.org/10.1109/ICRA.2011.5980409
#
# And this paper for important corrections on the demostration done in the previous paper:
# Faessler, M., Franchi, A., & Scaramuzza, D. (2017). Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories.
# https://doi.org/10.1109/LRA.2017.2776353
#


import numpy as np
import control as ctl
import matplotlib.pyplot as plt
import argparse
from mpl_toolkits.mplot3d import Axes3D


# Create a parser for the command line arguments
def createCommandLineParser():
	parser1 = argparse.ArgumentParser(description='3D Quadcopter linear controller simulation')
	#parser1.add_argument('-f', help='Path to log file.', default='log201811071627_1080p_0.txt')
	#parser1.add_argument('-b', help='Histogram bin size', default=200)
	#parser1.add_argument('-g', help='Path to gpx file.', default='20181112/2018._11._12._noon_5_09_18.gpx')
	#parser1.add_argument('-r', help='Video Resolution', default='1280*720.gpx')
	parser1.add_argument('-t', help='Total Simulation time', default=10.0)
	#parser1.add_argument('-a', help='Name of GPS Track image', default='GPS Track image file name with extension')
	#parser1.add_argument('-k', help='Name of GPS Data image', default='GPS Data image file name with extension')
	#parser1.add_argument('-x', help='LOG File format: 1: oldest , 2: new  3:newest ', default="old")
	#parser1.add_argument('-y', help='Number of camera for which latency map will be draw, 5 = all cameras', default="5")
	#parser1.add_argument('-q', help='Colormap to be used [viridis_r, magma_r, inferno_r]', default="inferno_r")
	#parser1.add_argument('-o', help='Bin number for Encoding Latency Histogram', default="5")
	#parser1.add_argument('-p', help='Test Place', default="Downtown")

	#parser.add_argument('-w', help='Path to rotated image', default='r1.jpg')
	args = parser1.parse_args()
	return args

def getInputMatrices(A_,B_,C_,D_):

	aug1 = np.concatenate((A_,B_), axis = 1)
	aug2 = np.concatenate((C_,D_), axis = 1)
	# create [[A,B],[C,D]] matrix
	inp = np.concatenate((aug1,aug2), axis = 0)
	print(inp)

	# create multiplying matrix
	mult = np.zeros((inp.shape[0],1))
	mult[mult.shape[0]-1] = 1.0

	# compute inv(inp)*mult
	inp = np.linalg.inv(inp)*mult

	# nx is all values but last, nu is simply last value
	n_x = inp[0:(inp.shape[0]-1)]
	n_u = inp[-1]

	return n_u, n_x




# get arguments to the program
args = createCommandLineParser()

fig = plt.figure(figsize=(20,10))
fig.suptitle(" LQR Controller for a dif-flat generated trajectory tracking quadrotor")
ax0 = fig.add_subplot(3,2,1)#, projection='3d')
ax1 = fig.add_subplot(3,2,2)
ax2 = fig.add_subplot(3,2,3)#, projection='3d')
ax3 = fig.add_subplot(3,2,4)
ax4 = fig.add_subplot(3,2,5)
ax5 = fig.add_subplot(3,2,6)

# define simulation time
t_max = int(args.t)
dt = 0.01
t = np.arange(0.0,t_max,dt)

# define step input gain:  u(t) = R*u(t)
# for each control variable
Rx = 10.0
refx = Rx*np.ones_like(t)
Ry = 11.0
refy = Ry*np.ones_like(t)
Rz = 9.0
refz = Rz*np.ones_like(t)

Ra = 10.0
refa = Ra*np.ones_like(t)
Rb = 11.0
refb = Rb*np.ones_like(t)
Rc = 9.0
refc = Rc*np.ones_like(t)


	
m = 0.5
I = np.array([[0.00025, 0, 2.55e-6],
              [0, 0.000232, 0],
              [2.55e-6, 0, 0.0003738]])

invI = np.linalg.inv(I)
print(invI)



#define A matrix (It is the linearization with computer-torque method)
A = np.matrix(
	[[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

# input matrices
B = np.matrix(
	[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[1.0/m, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 1.0/m, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 1.0/m, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
	[0.0, 0.0, 0.0, invI.item(0,0), invI.item(0,1), invI.item(0,2), 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, invI.item(1,0), invI.item(1,1), invI.item(1,2), 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, invI.item(2,0), invI.item(2,1), invI.item(2,2), 0.0, 0.0, 0.0]])

# output matrices
Cx = np.matrix([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #np.matrix()
Cy = 0.0

# transmission matrix for all systems
D = np.matrix([0.0])

# Thanks to the addivity property of linear differential equations
# and that our system is a system of linear ordinary differential equations
# we can compute the output as the sum of the effects of each of the inputs
# y(u1, u2, u3, u4, u5 , u6) = y(u1) + y(u2) + y(u3) + y(u4) + y(u5) + y(u6)
#
# So now I proceed to compute the output as a sum each of the inputs



######################################################
##                                              	##
##            CREATE STATE SPACE SYSTEMS    		##
##            and their step step 					##
##													##
######################################################

# plot open loop system...just for fun
x_ol_u1 = ctl.ss(A,B[:,0],Cx,D)
x_ol_u2 = ctl.ss(A,B[:,1],Cx,D)
x_ol_u3 = ctl.ss(A,B[:,2],Cx,D)
x_ol_u4 = ctl.ss(A,B[:,3],Cx,D)
x_ol_u5 = ctl.ss(A,B[:,4],Cx,D)
x_ol_u6 = ctl.ss(A,B[:,5],Cx,D)

# get closed loop system step response   refy1 is an step signal
#ty, y1=ctl.step_response(cl_ss_y1, T = t) 
tx, x_u1, s = ctl.forced_response(x_ol_u1, T=t, U=refx)
tx, x_u2, s = ctl.forced_response(x_ol_u2, T=t, U=refy)
tx, x_u3, s = ctl.forced_response(x_ol_u3, T=t, U=refz)
tx, x_u4, s = ctl.forced_response(x_ol_u4, T=t, U=refa)
tx, x_u5, s = ctl.forced_response(x_ol_u5, T=t, U=refb)
tx, x_u6, s = ctl.forced_response(x_ol_u6, T=t, U=refc)


# plot responses
ax0.plot(t,x_u1,linestyle = '-',color ='r', label = "x_u1")
ax0.plot(t,x_u2,linestyle = '-',color ='g', label = "x_u2")
ax0.plot(t,x_u3,linestyle = '-',color ='b', label = "x_u3")
ax0.plot(t,x_u4,linestyle = '-',color ='m', label = "x_u4")
ax0.plot(t,x_u5,linestyle = '-',color ='c', label = "x_u5")
ax0.plot(t,x_u6,linestyle = '-',color ='k', label = "x_u6")

ax0.plot(t,x_u1+x_u2+x_u3+x_u4+x_u5+x_u6,linestyle = '--',color ='y', label = "x")


ax0.set_title("Step response of open loop flat system", fontsize='small')
ax0.legend(loc='center right', shadow=True, fontsize='small')
ax0.set_xlabel("time {s}")
ax0.set_ylabel("x {m}")

######################################################
##                                              	##
##            FULL STATE FEEDBACK LQR CONTROLLERS 	##
##													##
######################################################


# Define performance index weights
Q = np.diag([40, 20, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]);
R = np.diag([2, 2, 2, 2, 2, 2]);

# calcucate LQR
(K, X, E) = ctl.lqr(A,B,Q,R)
K = np.matrix(K)
#print(K[1,:])

######################################################
##                                              	##
##            COMPUTE CONTROLLED OUTPUT 		 	##
##													##
######################################################


# again will compute output as a sum of the effects of each input

# plot open loop system...just for fun
#Nu, Nx = getInputMatrices(A,B[:,0],Cx,D)
#x_cl_u1 = ctl.ss(A-B[:,0]*K[0,:], B[:,0]*(Nu +K[0,:]*Nx)*Rx , Cx, D)
#x_cl_u2 = ctl.ss(A-B[:,1]*K[1,:], B[:,1]*(Nu +K[1,:]*Nx)*Ry , Cx, D)
#x_cl_u3 = ctl.ss(A-B[:,2]*K[2,:], B[:,2]*(Nu +K[2,:]*Nx)*Rz , Cx, D)
#x_cl_u4 = ctl.ss(A-B[:,3]*K[3,:], B[:,3]*(Nu +K[3,:]*Nx)*Ra , Cx, D)
#x_cl_u5 = ctl.ss(A-B[:,4]*K[4,:], B[:,4]*(Nu +K[4,:]*Nx)*Rb , Cx, D)
#x_cl_u6 = ctl.ss(A-B[:,5]*K[5,:], B[:,5]*(Nu +K[5,:]*Nx)*Rc , Cx, D)

#Nu, Nx = getInputMatrices(A,B[:,0],Cx,D)
Nu = np.matrix([0.0])
Nx = np.matrix([[1.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
x_cl_u1 = ctl.ss(A-B[:,0]*K[0,:], B[:,0]*(Nu +K[0,:]*Nx)*1.0 , Cx, D)
x_cl_u2 = ctl.ss(A-B[:,1]*K[1,:], B[:,1]*(Nu +K[1,:]*Nx)*1.0 , Cx, D)
x_cl_u3 = ctl.ss(A-B[:,2]*K[2,:], B[:,2]*(Nu +K[2,:]*Nx)*1.0 , Cx, D)
x_cl_u4 = ctl.ss(A-B[:,3]*K[3,:], B[:,3]*(Nu +K[3,:]*Nx)*1.0 , Cx, D)
x_cl_u5 = ctl.ss(A-B[:,4]*K[4,:], B[:,4]*(Nu +K[4,:]*Nx)*1.0 , Cx, D)
x_cl_u6 = ctl.ss(A-B[:,5]*K[5,:], B[:,5]*(Nu +K[5,:]*Nx)*1.0 , Cx, D)

# get closed loop system step response   refy1 is an step signal
#ty, y1=ctl.step_response(cl_ss_y1, T = t) 
tx, x_u1, s = ctl.forced_response(x_cl_u1, T=t, U=refx)
tx, x_u2, s = ctl.forced_response(x_cl_u2, T=t, U=refy)
tx, x_u3, s = ctl.forced_response(x_cl_u3, T=t, U=refz)
tx, x_u4, s = ctl.forced_response(x_cl_u4, T=t, U=refa)
tx, x_u5, s = ctl.forced_response(x_cl_u5, T=t, U=refb)
tx, x_u6, s = ctl.forced_response(x_cl_u6, T=t, U=refc)

# plot responses
ax1.plot(t,x_u1,linestyle = '-',color ='r', label = "x_u1")
ax1.plot(t,x_u2,linestyle = '-',color ='g', label = "x_u2")
ax1.plot(t,x_u3,linestyle = '-',color ='b', label = "x_u3")
ax1.plot(t,x_u4,linestyle = '-',color ='m', label = "x_u4")
ax1.plot(t,x_u5,linestyle = '-',color ='c', label = "x_u5")
ax1.plot(t,x_u6,linestyle = '-',color ='k', label = "x_u6")

ax1.plot(t,x_u1+x_u2+x_u3+x_u4+x_u5+x_u6,linestyle = '--',color ='y', label = "x")


ax1.set_title("Step response of closed loop flat system", fontsize='small')
ax1.legend(loc='center right', shadow=True, fontsize='small')
ax1.set_xlabel("time {s}")
ax1.set_ylabel("x {m}")

plt.show()

