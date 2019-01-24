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
	#print(inp)

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


	
m = 0.18
I = np.array([[0.00025, 0, 2.55e-6],
              [0, 0.000232, 0],
              [2.55e-6, 0, 0.0003738]])

invI = np.linalg.inv(I)
print(invI)



# define system matrix for translation variables
# (It is the result of linearization with computer-torque method)
At = np.matrix(
	[[0.0,1.0],
	[0.0,0.0]])

# input matrices
Bt = np.matrix(
	[[0.0],
	[1.0]])

# output matrices
Ct = np.matrix([1.0,0.0]) #np.matrix()

# system matrix for rotational variables
Ar = np.matrix([0.0])

# input matrix
Br = np.matrix([1.0]) 

# output matrix
Cr = np.matrix([1.0])

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
x_ol = ctl.ss(At,Bt,Ct,D)
wx_ol = ctl.ss(Ar,Br,Cr,D)

#x_ol_u3 = ctl.ss(A,B[:,2],Cx,D)
#x_ol_u4 = ctl.ss(A,B[:,3],Cx,D)
#x_ol_u5 = ctl.ss(A,B[:,4],Cx,D)
#x_ol_u6 = ctl.ss(A,B[:,5],Cx,D)

# get closed loop system step response   refy1 is an step signal
#ty, y1=ctl.step_response(cl_ss_y1, T = t) 
tx, x, s = ctl.forced_response(x_ol, T=t, U=refx)
tx, wx, s = ctl.forced_response(wx_ol, T=t, U=refy)
#tx, x_u3, s = ctl.forced_response(x_ol_u3, T=t, U=refz)
#tx, x_u4, s = ctl.forced_response(x_ol_u4, T=t, U=refa)
#tx, x_u5, s = ctl.forced_response(x_ol_u5, T=t, U=refb)
#tx, x_u6, s = ctl.forced_response(x_ol_u6, T=t, U=refc)


# plot responses
ax0.plot(t,x,linestyle = '-',color ='r', label = "x")
ax0.plot(t,wx,linestyle = '-',color ='g', label = "wx")
#ax0.plot(t,x_u3,linestyle = '-',color ='b', label = "x_u3")
#ax0.plot(t,x_u4,linestyle = '-',color ='m', label = "x_u4")
#ax0.plot(t,x_u5,linestyle = '-',color ='c', label = "x_u5")
#ax0.plot(t,x_u6,linestyle = '-',color ='k', label = "x_u6")


ax0.set_title("Step response of open loop flat system", fontsize='small')
ax0.legend(loc='center right', shadow=True, fontsize='small')
ax0.set_xlabel("time {s}")
ax0.set_ylabel("x {m}")

######################################################
##                                              	##
##            FULL STATE FEEDBACK LQR CONTROLLERS 	##
##													##
######################################################

# output performance matrix for translation variables
Qt = np.diag([10.0,1.0])
# input effort matrix for translation variables
Rt = np.array([1.0])

# calculate LQR gains
(Kt, X, E) = ctl.lqr(At,Bt,Qt,Rt)
Kt = np.matrix(Kt)
print("LQR gains for translation Dynamics: {}".format(Kt))


# create closed loop system...
Nu, Nx = getInputMatrices(At,Bt,Ct,D)
print("Translation Variables Input Matrices are: Nu{}, Nx{}".format(Nu,Nx))

x_cl = ctl.ss(At-Bt*Kt, Bt*(Nu +Kt*Nx)*1.0 , Ct, D)

# output performance matrix for rotational variables
Qr = np.diag([10.0])
# input effort matrix for rotational variables
Rr = np.array([1.0])

# calculate LQR gains
(Kr, X, E) = ctl.lqr(Ar,Br,Qr,Rr)
Kr = np.matrix(Kr)
print("LQR gains for rotational Dynamics: {}".format(Kr))


# create closed loop system...
Nu, Nx = getInputMatrices(Ar,Br,Cr,D)
print("Rotation Variables Input Matrices are: Nu{}, Nx{}".format(Nu,Nx))

wx_cl = ctl.ss(Ar-Br*Kr, Br*(Nu +Kr*Nx)*1.0 , Cr, D)



######################################################
##                                              	##
##            COMPUTE CONTROLLED OUTPUT 		 	##
##													##
######################################################


tx, x, s = ctl.forced_response(x_cl, T=t, U=refx)
tx, wx, s = ctl.forced_response(wx_cl, T=t, U=refy)

ax1.plot(t,x,linestyle = '-',color ='r', label = "x")
ax1.plot(t, refx,linestyle = '--', color = "k", label = 'x ref')

ax1.plot(t,wx,linestyle = '-',color ='g', label = "wx")
ax1.plot(t, refy,linestyle = '--', color = "k", label = 'wx ref')



ax1.set_title("Step response of closed loop flat system", fontsize='small')
ax1.legend(loc='center right', shadow=True, fontsize='small')
ax1.set_xlabel("time {s}")
ax1.set_ylabel("x {m}, wx {rad}")

plt.show()

