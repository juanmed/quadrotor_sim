# 3D Control of Quadcopter

import numpy as np
import control as ctl
import matplotlib.pyplot as plt
from scipy import signal
import argparse
from mpl_toolkits.mplot3d import Axes3D




# Create a parser for the command line arguments
def createCommandLineParser():
	parser1 = argparse.ArgumentParser(description='3D Quadcopter linear controller simulation')
	parser1.add_argument('-f', help='Path to log file.', default='log201811071627_1080p_0.txt')
	parser1.add_argument('-b', help='Histogram bin size', default=200)
	parser1.add_argument('-g', help='Path to gpx file.', default='20181112/2018._11._12._noon_5_09_18.gpx')
	parser1.add_argument('-r', help='Video Resolution', default='1280*720.gpx')
	parser1.add_argument('-t', help='Total Simulation time', default=10.0)
	parser1.add_argument('-a', help='Name of GPS Track image', default='GPS Track image file name with extension')
	parser1.add_argument('-k', help='Name of GPS Data image', default='GPS Data image file name with extension')
	parser1.add_argument('-x', help='LOG File format: 1: oldest , 2: new  3:newest ', default="old")
	parser1.add_argument('-y', help='Number of camera for which latency map will be draw, 5 = all cameras', default="5")
	parser1.add_argument('-q', help='Colormap to be used [viridis_r, magma_r, inferno_r]', default="inferno_r")
	parser1.add_argument('-o', help='Bin number for Encoding Latency Histogram', default="5")
	parser1.add_argument('-p', help='Test Place', default="Downtown")

	#parser.add_argument('-w', help='Path to rotated image', default='r1.jpg')
	args = parser1.parse_args()
	return args

def getInputMatrices(A,B,C,D):
	aug1 = np.concatenate((A,B), axis = 1)
	aug2 = np.concatenate((C,D), axis = 1)
	# 
	inp = np.concatenate((aug1,aug2), axis = 0)
	#
	mult = np.matrix([[0],[0],[0],[0],[1]])

	inp = np.linalg.inv(inp)*mult

	Nx = inp[(0,1,2,3),]  #First three rows
	Nu = inp[4,]          # Last row

	#print(inp)
	#print(" Y position input matrices")
	#print(Nx)
	#print(Nu)

	return Nu, Nx

def getInputMatrices2(A,B,C,D):
	aug1 = np.concatenate((A,B), axis = 1)
	aug2 = np.concatenate((C,D), axis = 1)
	# 
	inp = np.concatenate((aug1,aug2), axis = 0)
	#
	mult = np.matrix([[0],[0],[1]])

	inp = np.linalg.inv(inp)*mult

	Nx = inp[(0,1),]  #First three rows
	Nu = inp[2,]          # Last row

	#print(inp)
	#print(" Y position input matrices")
	#print(Nx)
	#print(Nu)

	return Nu, Nx

# get arguments to the program
args = createCommandLineParser()


fig = plt.figure(figsize=(20,10))
fig.suptitle(" 2D (Y-Z) Quadrotor Control and State Estimation")
ax0 = fig.add_subplot(2,2,1)
ax1 = fig.add_subplot(2,2,2)
ax2 = fig.add_subplot(2,2,3, projection='3d')
ax3 = fig.add_subplot(2,2,4, projection='3d') 

# define constants
g = 9.81 #m/s2
b = 0.0  # body drag constant
c = 0.2 #air friction constant

# quadrotor physical constants
m = 0.18  #kg  mass of the quadrotor
Ixx =0.00025   # kg*m2 moment of inertia around X-axis (quadrotor rotates around X-axis)
Iyy =0.000232   # kg*m2 
Izz =0.0003738   # kg*m2
Ktao =1.5e-9           # Drag torque constant for motors
Kt =6.11e-8             # Thrust constant for motors


# quadrotor geometry constants
t1 = np.pi/2   		# rads
t2 = 3*np.pi/4  	#rads
t3 = 5*np.pi/4		# rads
t4 = 7*np.pi/4		# rads
l = 0.086 			#m  arm lenght from center of mass to each rotor


# define time range for analysis
t_max = int(args.t)  #s
t = np.arange(0.0,t_max,0.01)


air = 0.0  # set to 1 to activate air model, otherwise set to 0

# equilibrium state


# X  axis dynamics
Ax = np.matrix(
	[[0.0,1.0,0.0,0.0],
	[0.0,0.0,g,0.0],
	[0.0,0.0,0.0,1.0],
	[0.0,0.0,0.0,0.0]])

Bx = np.matrix(
	[[0.0],
	[0.0],
	[0.0],
	[np.sin(t1)*l/Ixx]])

# X axis dynamics with body drag
Ax_bd = np.matrix(
	[[0.0,1.0,0.0,0.0],
	[0.0,-b/m,g,0.0],
	[0.0,0.0,0.0,1.0],
	[0.0,0.0,0.0,0.0]])

# X position output matrix
Cx = np.matrix([1.0,0.0,0.0,0.0])
# X velocity output matrix
Cx_dot = np.matrix([0.0,1.0,0.0,0.0])
# Pitch angle output matrix
Cp = np.matrix([0.0,0.0,1.0,0.0])
# Pitch angle velocity output matrix
Cp_dot = np.matrix([0.0,0.0,0.0,1.0])


# Y  dynamics
Ay = np.matrix(
	[[0.0,1.0,0.0,0.0],
	[0.0,0.0,-1.0*g,0.0],
	[0.0,0.0,0.0,1.0],
	[0.0,0.0,0.0,0.0]])

By = np.matrix(
	[[0.0],
	[0.0],
	[0.0],
	[np.sin(t1)*l/Iyy]])

# Y dynamics with body drag
Ay_bd = np.matrix(
	[[0.0,1.0,0.0,0.0],
	[0.0,-b/m,-1.0*g,0.0],
	[0.0,0.0,0.0,1.0],
	[0.0,0.0,0.0,0.0]])

# Y position output matrix
Cy = np.matrix([1.0,0.0,0.0,0.0])
# Y velocity output matrix
Cy_dot = np.matrix([0.0,1.0,0.0,0.0])
# Roll angle output matrix
Cr = np.matrix([0.0,0.0,1.0,0.0])
# Roll angle velocity output matrix
Cr_dot = np.matrix([0.0,0.0,0.0,1.0])


# Z axis dynamics
Az = np.matrix(
	[[0.0,1.0],
	[0.0,0.0]])

Bz = np.matrix(
	[[0.0],
	[1.0/m]])

# Z axis dynamics with body drag
Az_bd = np.matrix(
	[[0.0,1.0],
	[0.0,-b/m]])

# Z position output matrix
Cz = np.matrix([1.0,0.0])
# Z velocity matrix
Cz_dot = np.matrix([0.0,1.0])

# Yaw dynamics
Ayaw = np.matrix(
	[[0.0,1.0],
	[0.0,0.0]])

Byaw = np.matrix(
	[[0.0],
	[Ktao/(Kt*Izz)]]) 

# Yaw angle output matrix
Cyaw = np.matrix([1.0,0.0])
# Yaw angle velocity output matrix
Cyaw_dot = np.matrix([0.0,1.0])

# Transmission matrix
D = np.matrix([0.0])

######################################################
##                                              	##
##            CREATE STATE SPACE SYSTEMS    		##
##            and their step step 					##
##													##
######################################################

# Create open loop systems for X,Y,Z, and Yaw angles
x_ol = ctl.ss(Ax, Bx, Cx, D)
y_ol = ctl.ss(Ay, By, Cy, D)
z_ol = ctl.ss(Az, Bz, Cz, D)
yaw_ol = ctl.ss(Ayaw,Byaw,Cyaw,D)

# Create open loop systems for X,Y,Z with body drag. The rotation dynamics (yaw angle)
# are not affected by drag
x_ol_bd = ctl.ss(Ax_bd, Bx, Cx, D)
y_ol_bd = ctl.ss(Ay_bd, By, Cy, D)
z_ol_bd = ctl.ss(Az_bd, Bz, Cz, D)


# get step response of the systems
tx, x = ctl.step_response(x_ol, T = t)
ty, y = ctl.step_response(y_ol, T = t)
tz, z = ctl.step_response(z_ol, T = t)
tyaw, yaw=ctl.step_response(yaw_ol, T = t)

# get step response of the systems with body drag
tx, x_bd = ctl.step_response(x_ol_bd, T = t)
ty, y_bd = ctl.step_response(y_ol_bd, T = t)
tz, z_bd = ctl.step_response(z_ol_bd, T = t)

# plot open loop response per axis (x,y,z) and orientation (yaw)
ax0.plot(t,x,linestyle = "-",color ='r', label = "x")
ax0.plot(t,y,linestyle = '-',color ='g', label = "y")
ax0.plot(t,z,linestyle = '-',color ='b', label = "z")
ax0.plot(t,yaw,linestyle = '-',color ='m', label = "yaw")

# plot open loop response per axis (x,y,z) with body drag
ax0.plot(t,x_bd,linestyle = '--',color ='firebrick', label = "x_bd")
ax0.plot(t,y_bd,linestyle = '--',color ='mediumseagreen', label = "y_bd")
ax0.plot(t,z_bd,linestyle = '--',color ='royalblue', label = "z_bd")

ax0.set_title("Step response for open loop system (bd means 'with body drag')", fontsize='small')
ax0.legend(loc='upper left', shadow=True, fontsize='small')
ax0.set_xlabel("time {s}")
ax0.set_ylabel("Position {m}")

# plot open loop response of 3D position
ax2.scatter(x,y, z, color = 'r')
ax2.scatter(x_bd,y_bd,z_bd, color = 'g')
ax2.set_title("Open Loop system response {3D} (red : no drag, green: body drag)")
ax2.set_xlabel('x {m}')
ax2.set_ylabel('y {m}')
ax2.set_zlabel('z {m}')


######################################################
##                                              	##
##            Create input reference values    		##
##            i.e. desired values 					##
##													##
######################################################


# define step input gain:  u(t) = R*u(t)
# for each controlled state variable : x,y,z and yaw
Rx = 9.0
refx = Rx*np.ones_like(t)
Ry = 10.0
refy = Ry*np.ones_like(t)
Rz = 11.0
refz = Rz*np.ones_like(t)
Ryaw = np.pi/3.0
refyaw = Ryaw*np.ones_like(t)
# ticks
p_ticks = np.arange(0,max([Rx,Ry,Rz])+2,1)        #position ticks



######################################################
##                                              	##
##            FULL STATE FEEDBACK CONTROLLERS 		##
##													##
######################################################


#    *****        SIMPLE DYNAMICS            ******  #

# Define performance index weights For X,Y Dynamics
# This performance index puts much greater emphasis on position performance
# and very low emphasis on control effort
Qx1 = np.diag([100.0, 1.0, 1.0, 1.0]);
Qu1a = np.diag([1.0]);

# Calculate LQR gains for X dynamics
(K, X, E) = ctl.lqr(Ax, Bx, Qx1, Qu1a)
Kx = np.matrix(K)
print("LQR gains for X Dynamics: {}".format(Kx))

# Calculate input matrices for Reference
Nu_x, Nx_x = getInputMatrices(Ax,Bx,Cx,D)
print("X Dynamics Input Matrices are: Nu_x{}, Nx_x{}".format(Nu_x,Nx_x))

# create controlled state space system for X dynamics
cl_ss_x = ctl.ss(Ax-Bx*Kx, Bx*(Nu_x + Kx*Nx_x)*Rx,Cx,D)


# Calculate LQR gains for Y dynamics
(K, X, E) = ctl.lqr(Ay, By, Qx1, Qu1a)
Ky = np.matrix(K)
print("LQR gains for Y Dynamics: {}".format(Ky))

# Calculate input matrices for Reference
Nu_y, Nx_y = getInputMatrices(Ay,By,Cy,D)
print("Y Dynamics Input Matrices are: Nu_y {}, Nx_y {}".format(Nu_y,Nx_y))

# create controlled state space system for Y dynamics
cl_ss_y = ctl.ss(Ay-By*Ky, By*(Nu_y + Ky*Nx_y)*Ry,Cy,D) 

# Define performance index weights for Z, Yaw Dynamics
# This performance index puts much greater emphasis on position performance
# and very low emphasis on control effort
Qx1 = np.diag([100., 1.]);
Qu1a = np.diag([1.0]);

# Calculate LQR gains for Z dynamics
(K, X, E) = ctl.lqr(Az, Bz, Qx1, Qu1a)
Kz = np.matrix(K)
print("LQR gains for Z Dynamics: {}".format(Kz))

# Calculate input matrices for Reference
Nu_z, Nx_z = getInputMatrices2(Az,Bz,Cz,D)
print("Z Dynamics Input Matrices are: Nu_z {}, Nx_z {}".format(Nu_z,Nx_z))
# create controlled state space system for Z dynamics
cl_ss_z = ctl.ss(Az-Bz*Kz, Bz*(Nu_z + Kz*Nx_z)*Rz,Cz,D) 


# Calculate LQR gains for YAW dynamics
(K, X, E) = ctl.lqr(Ayaw, Byaw, Qx1, Qu1a)
Kyaw = np.matrix(K)
print("LQR gains for Yaw Dynamics: {}".format(Kyaw))

# Calculate input matrices for Reference
Nu_yaw, Nx_yaw = getInputMatrices2(Ayaw,Byaw,Cyaw,D)
print("Yaw Dynamics Input Matrices are: Nu_yaw {}, Nx_yaw {}".format(Nu_yaw,Nx_yaw))

# create controlled state space system for X dynamics
cl_ss_yaw = ctl.ss(Ayaw-Byaw*Kyaw, Byaw*(Nu_yaw + Kyaw*Nx_yaw)*Ryaw,Cyaw,D) 


#    *****     DYNAMICS  WITH BODY DRAG      ******  #

Qx1 = np.diag([100.0, 1.0, 1.0, 1.0]);
Qu1a = np.diag([1.0]);

# LQR controller for X Dynamics with body drag
(K, X, E) = ctl.lqr(Ax_bd, Bx, Qx1, Qu1a)
Kx = np.matrix(K)
print("LQR gains for X Dynamics with Body Drag: {}".format(Kx))
# Calculate input matrices for Reference
Nu_x_bd, Nx_x_bd = getInputMatrices(Ax_bd,Bx,Cx,D)
print("X Dynamics Input Matrices with Body Drag are: Nu_x_bd {}, Nx_x_bd {}".format(Nu_x_bd,Nx_x_bd))

# create controlled state space system for X dynamics with body drag
cl_ss_x_bd = ctl.ss(Ax_bd-Bx*Kx, Bx*(Nu_x_bd + Kx*Nx_x_bd)*Rx,Cx,D)

# Calculate LQR gains for Y dynamics with body drag
(K, X, E) = ctl.lqr(Ay_bd, By, Qx1, Qu1a)
Ky = np.matrix(K)
print("LQR gains for Y Dynamics with Body Drag: {}".format(Ky))

# Calculate input matrices for Reference
Nu_y_bd, Nx_y_bd = getInputMatrices(Ay_bd,By,Cy,D)
print("Y Dynamics Input Matrices with Body Drag are: Nu_y_bd {}, Nx_y_bd {}".format(Nu_y_bd,Nx_y_bd))

# create controlled state space system for Y dynamics
cl_ss_y_bd = ctl.ss(Ay_bd-By*Ky, By*(Nu_y_bd + Ky*Nx_y_bd)*Ry,Cy,D) 


Qx1 = np.diag([100, 1.0]);
Qu1a = np.diag([1.0]);

# Calculate LQR gains for Z dynamics with body drag
(K, X, E) = ctl.lqr(Az_bd, Bz, Qx1, Qu1a)
Kz = np.matrix(K)
print("LQR gains for Z Dynamics with Body Drag: {}".format(Kz))

# Calculate input matrices for Reference
Nu_z_bd, Nx_z_bd = getInputMatrices2(Az_bd,Bz,Cz,D)
print("Z Dynamics Input Matrices with Body Drag are: Nu_z_bd {}, Nx_z_bd {}".format(Nu_z_bd,Nx_z_bd))

# create controlled state space system for Z dynamics
cl_ss_z_bd = ctl.ss(Az_bd-Bz*Kz, Bz*(Nu_z_bd + Kz*Nx_z_bd)*Rz,Cz,D) 




# get step responses for controlled systems
tx, x = ctl.step_response(cl_ss_x, T = t)
ty, y = ctl.step_response(cl_ss_y, T = t)
tz, z = ctl.step_response(cl_ss_z, T = t)
tyaw, yaw = ctl.step_response(cl_ss_yaw, T = t)


# get step response for controlled systems with body drag
tx, x_bd = ctl.step_response(cl_ss_x_bd, T = t)
ty, y_bd = ctl.step_response(cl_ss_y_bd, T = t)
tz, z_bd = ctl.step_response(cl_ss_z_bd, T = t)

# plot controlled step response
ax1.plot(t,x,color ='r', label = "x")
ax1.plot(t, refx,linestyle = '--', color = "k", label = 'x ref')
ax1.plot(t,y,color="g",label="y")
ax1.plot(t, refy,linestyle = '--', color = "c", label = 'y ref')
ax1.plot(t,z,color="b",label="z")
ax1.plot(t, refz,linestyle = '--', color = "sandybrown", label = 'z ref')
ax1.plot(t,yaw,color="m",label="yaw")
ax1.plot(t, refyaw,linestyle = '--', color = "mediumvioletred", label = 'yaw ref')

# plot open loop response per axis (x,y,z) with body drag
ax1.plot(t,x_bd,linestyle = '-.',color ='firebrick', label = "x_bd")
ax1.plot(t,y_bd,linestyle = '-.',color ='mediumseagreen', label = "y_bd")
ax1.plot(t,z_bd,linestyle = '-.',color ='royalblue', label = "z_bd")

#ax1.set_title("Closed loop with  LQR controller gains K1={:.2f}, K2={:.2f}, K3={:.2f}, K4={:.2f}".format(Kx.item(0,0),Kx.item(0,1),Kx.item(0,2),Kx.item(0,3)), fontsize='small')
ax1.set_title("LQR Controller Response (bd means 'with body drag')")
ax1.legend(loc='center right', shadow=True, fontsize='small')
ax1.set_xlabel("time {s}")
ax1.set_ylabel("Position {m}")
ax1.set_yticks(p_ticks)

ax3.scatter(x,y,z, color = 'r')
ax3.scatter(x_bd,y_bd,z_bd, color = 'g')
ax3.set_title("Closed Loop response with LQR Controller {3D} (red : no drag, green: body drag)")
ax3.set_xlabel('x {m}')
ax3.set_ylabel('y {m}')
ax3.set_zlabel('z {m}')



#*****************************************#
#          DEFINE A PERSONALIZED INPUT SIGNAL
#*****************************************#

# reseed to generate different results
np.random.seed()

# Redefine the System just to remove the input Reference gain R
# so that that it follows input signal and not (input signal)*R
cl_ss_x = ctl.ss(Ax-Bx*Kx, Bx*(Nu_x + Kx*Nx_x)*1.0,Cx,D)
cl_ss_y = ctl.ss(Ay-By*Ky, By*(Nu_y + Ky*Nx_y)*1.0,Cy,D) 
cl_ss_z = ctl.ss(Az-Bz*Kz, Bz*(Nu_z + Kz*Nx_z)*1.0,Cz,D) 
cl_ss_yaw = ctl.ss(Ayaw-Byaw*Kyaw, Byaw*(Nu_yaw + Kyaw*Nx_yaw)*1.0,Cyaw,D) 

cl_ss_x_bd = ctl.ss(Ax_bd-Bx*Kx, Bx*(Nu_x_bd + Kx*Nx_x_bd)*1.0,Cx,D)
cl_ss_y_bd = ctl.ss(Ay_bd-By*Ky, By*(Nu_y_bd + Ky*Nx_y_bd)*1.0,Cy,D) 
cl_ss_z_bd = ctl.ss(Az_bd-Bz*Kz, Bz*(Nu_z_bd + Kz*Nx_z_bd)*1.0,Cz,D) 

signalx = np.ones_like(t)
signaly = np.ones_like(t)
signalz = np.ones_like(t)
random_timesx = list()
random_timesy = list()
random_timesz = list()

# define number of time slots
slotsx = np.random.randint(0,3)
slotsy = np.random.randint(0,3)
slotsz = np.random.randint(0,3)

# generate random time divisions
for i in range(slotsx):
	random_timesx.append(int((len(signalx)-1)*np.random.random_sample()))
for i in range(slotsy):
	random_timesy.append(int((len(signaly)-1)*np.random.random_sample()))
for i in range(slotsz):
	random_timesz.append(int((len(signalz)-1)*np.random.random_sample()))


# add the index of the last element
random_timesx.append(len(signalx)-1)
random_timesy.append(len(signaly)-1)
random_timesz.append(len(signalz)-1)

random_timesx = np.unique(np.sort(random_timesx)).flatten()
random_timesy = np.unique(np.sort(random_timesy)).flatten()
random_timesz = np.unique(np.sort(random_timesz)).flatten()


# Store random input signal
start = 0
for i in random_timesx:
	signalx[start:i] = signalx[start:i]*(Rx*np.random.random_sample())
	start = i

start = 0
for i in random_timesy:
	signaly[start:i] = signaly[start:i]*(Ry*np.random.random_sample())
	start = i

start = 0
for i in random_timesz:
	signalz[start:i] = signalz[start:i]*(Rz*np.random.random_sample())
	start = i	


# Create an spiral signal to track...not sure what will happen
k = 1.0
w = 0.2
signalx = k*(1 - np.cos(w*t))
signaly = k*np.sin(w*t)
signalz = 0.5*t


# evaluate the forced response of both systems
t, x, u = ctl.forced_response(cl_ss_x, T=t, U=signalx)
t, y, u = ctl.forced_response(cl_ss_y, T=t, U=signaly)
t, z, u = ctl.forced_response(cl_ss_z, T=t, U=signalz)

# evaluate the forced response of both systems
t, x_bd, u = ctl.forced_response(cl_ss_x_bd, T=t, U=signalx)
t, y_bd, u = ctl.forced_response(cl_ss_y_bd, T=t, U=signaly)
t, z_bd, u = ctl.forced_response(cl_ss_z_bd, T=t, U=signalz)

fig2 = plt.figure(figsize=(20,10))
track0 = fig2.add_subplot(1,2,1, projection="3d")
errors = fig2.add_subplot(1,2,2)

# plot 3D trajectory and 3D quadrotor position
track0.plot(x,y,z, color = "r", label = "quadrotor")
track0.plot(x_bd,y_bd,z_bd, color = "g", label = "quadrotor with drag")
track0.plot(signalx,signaly,signalz,color="b",label="command")
track0.set_title("Closed Loop response with LQR Controller to random input signal {3D}")
track0.set_xlabel('x {m}')
track0.set_ylabel('y {m}')
track0.set_zlabel('z {m}')
track0.text(signalx[0], signaly[0], signalz[0], "start", color='red')
track0.text(signalx[-1], signaly[-1], signalz[-1], "finish", color='red')

# plot errors
errors.plot(t,signalx-x, color = "r", label = "x error")
errors.plot(t,signaly-y, color = "g", label = "y error")
errors.plot(t,signalz-z, color = "b", label = "z error")

errors.plot(t,signalx-x_bd,linestyle = '-.', color = "firebrick", label = "x_bd error")
errors.plot(t,signaly-y_bd,linestyle = '-.', color = "mediumseagreen", label = "y_bd error")
errors.plot(t,signalz-z_bd,linestyle = '-.', color = "royalblue", label = "z_bd error")

errors.set_title("Position error for reference tracking")
errors.set_xlabel("time {s}")
errors.set_ylabel(" Position {m}")
errors.legend(loc='lower right', shadow=True, fontsize='small')







plt.show()