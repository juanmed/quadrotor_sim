# This file contains the resulting system matrices
# for the 2D Differentially Flat system

import numpy as np
import control as ctl
import matplotlib.pyplot as plt
#from scipy import signal
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

# Calculate input matrices for zero-error tracking
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

	return Nu, Nx

def get_u2(y5,y6,y7,y8,v1,v2):
	a = y6 + g
	b = -y6*v1 -y8*y7 -g*v1+y8*y7+v2*y5
	c = -y6*y7 -g*y7 +y8*y5
	u2 = ((a**2)*b - (2.0*c*a*y8)) / (a**4)
	return u2*Ixx/l


# get arguments to the program
args = createCommandLineParser()


fig = plt.figure(figsize=(20,10))
fig.suptitle(" 2D (Y-Z) Quadrotor Control using Differential Flatness Property")
ax0 = fig.add_subplot(2,2,1)
ax1 = fig.add_subplot(2,2,2)
ax2 = fig.add_subplot(2,2,3)#, projection='3d')
ax3 = fig.add_subplot(2,2,4)#, projection='3d') 

#track = plt.figure(figsize=(20,10))
#track.suptitle(" 2D (Y-Z) Quadrotor Command Tracking ")
#track0 = track.add_subplot(1,1,1)

# define constants
g = 9.81 #m/s2
m = 1.0  #kg  mass of the quadrotor
l = 0.2 #m  arm lenght from center of mass to each rotor
Ixx = (m*l**2)/2.0  # moment of inertia around X-axis (quadrotor rotates around X-axis)
b = 0.01  # air drag/friction force
c = 0.2 #air friction constant

# define time range for analysis
t_max = int(args.t)  #s
dt = 0.01
t = np.arange(0.0,t_max,dt)

# define step input gain:  u(t) = R*u(t)
# for each control variable
Ry1 = 10.0
refy1 = Ry1*np.ones_like(t)
Ry2 = 11.0
refy2 = Ry2*np.ones_like(t)
Rpsi = 10.0
refpsi = Rpsi*np.ones_like(t)

# ticks
p_ticks = np.arange(0,max([Ry1,Ry2])+2,1)        #position ticks
angle_ticks = np.arange(-np.pi,np.pi,np.pi/4)  #angle ticks

# System matrix 1 of flat system
A = np.matrix(
	[[0.0,1.0,0.0,0.0],
	[0.0,0.0,1.0,0.0],
	[0.0,0.0,0.0,1.0],
	[0.0,0.0,0.0,0.0]])

# Input matrix 1 for flat system
B = np.matrix(
	[[0.0],
	[0.0],
	[0.0],
	[1.0]])

# Output 1 matrix
Cy1 = np.matrix([1.0,0.0,0.0,0.0])
Cy3 = np.matrix([0.0,1.0,0.0,0.0])
Cy5 = np.matrix([0.0,0.0,1.0,0.0])
Cy7 = np.matrix([0.0,0.0,0.0,1.0])

# Transmision matrix
D = np.matrix([0.0])


#									#
#         OPEN LOOP SYSTEM       	#
#									#

# Open Loop System Definition
y1_ol = ctl.ss(A,B,Cy1,D)
y3_ol = ctl.ss(A,B,Cy3,D)
y5_ol = ctl.ss(A,B,Cy5,D)
y7_ol = ctl.ss(A,B,Cy7,D)


# get open loop system response
ty, y1=ctl.step_response(y1_ol, T = t)
ty, y3=ctl.step_response(y3_ol, T = t)
ty, y5=ctl.step_response(y5_ol, T = t)
ty, y7=ctl.step_response(y7_ol, T = t)

# plot responses
ax0.plot(t,y1, linestyle = '-',color ='r', label = "y1")
ax0.plot(t,y3,linestyle = '-',color ='g', label = "y3")
ax0.plot(t,y5,linestyle = '-',color ='b', label = "y5")
ax0.plot(t,y7,linestyle = '-',color ='m', label = "y7")

ax0.set_title("Step response of open loop flat system", fontsize='small')
ax0.legend(loc='center right', shadow=True, fontsize='small')
ax0.set_xlabel("time {s}")
ax0.set_ylabel("y1,y3,y5,y7")


#***********************************************#
#   FULL STATE FEEDBACK CLOSED LOOP SYSTEM      #
#			VIA POLE PLACEMENT					#
#***********************************************#


# Gains computation by pole placement

# desired poles
dp = np.array([-2.0,-2.0,-2.0+0.5j,-2.0-0.5j])
# compute gains by Ackermans formula
K1 = ctl.acker(A,B,dp)
K1 = np.matrix(K1)
print("Pole placement gains for y1: {}".format(K1))
# compute gains by Ackermans formula
K2 = ctl.acker(A,B,dp)
K2 = np.matrix(K2)
print("Pole placement gains for y2: {}".format(K2))

# Calculate input matrices for Reference
Nu_y1, Nx_y1 = getInputMatrices(A,B,Cy1,D)

# and create controlled system, y1 & y2 dynamics are the same so use the same matrices
cl_ss_y1 = ctl.ss(A-B*K1, B*(Nu_y1 + K1*Nx_y1)*1.0,Cy1,D) 
cl_ss_y2 = ctl.ss(A-B*K2, B*(Nu_y1 + K2*Nx_y1)*1.0,Cy1,D) 


# get closed loop system step response   refy1 is an step signal
#ty, y1=ctl.step_response(cl_ss_y1, T = t) 
t, y1, s_y1 = ctl.forced_response(cl_ss_y1, T=t, U=refy1)
t, y2, s_y2 = ctl.forced_response(cl_ss_y2, T=t, U=refy2)


# calculate commanded input signal:   u = -K*x  where x is the state
v1_pp = list()
v2_pp = list()
for state1, state2 in zip(s_y1.T,s_y2.T):
	inp1 = 1.0*np.matrix(state1)*K1.T
	inp2 = 1.0*np.matrix(state2)*K2.T
	v1_pp.append(inp1.item(0,0))   # inp.item(0,0) gets element in np.matrix... so strange syntax
	v2_pp.append(inp2.item(0,0))

# get state variables and input evolution
y1_pp = s_y1[0,:]
y3_pp = s_y1[1,:]
y5_pp = s_y1[2,:]
y7_pp = s_y1[3,:]
v1_pp = np.array(v1_pp)

y2_pp = s_y2[0,:]
y4_pp = s_y2[1,:]
y6_pp = s_y2[2,:]
y8_pp = s_y2[3,:]
v2_pp = np.array(v2_pp)


#compute phi angle
phi_pp = np.arctan2(-y5_pp,y6_pp+g)

# plot controlled step response and other states
ax1.plot(t,y1_pp,linestyle = '-',color ='r', label = "y1 pp")
#ax1.plot(t,y3_pp,linestyle = '-',color ='g', label = "y3 pp")
#ax1.plot(t,y5_pp,linestyle = '-',color ='b', label = "y5 pp")
#ax1.plot(t,y7_pp,linestyle = '-',color ='m', label = "y7 pp")
ax1.plot(t,y2_pp,linestyle = '-',color ='maroon', label = "y2 pp")
#ax1.plot(t,y4_pp,linestyle = '-',color ='lightgreen', label = "y4 pp")
#ax1.plot(t,y6_pp,linestyle = '-',color ='skyblue', label = "y6 pp")
#ax1.plot(t,y8_pp,linestyle = '-',color ='pink', label = "y8 pp")

ax1.plot(t, refy1,linestyle = '--', color = "k", label = 'y1 ref')
ax1.plot(t, refy2,linestyle = '--', color = "k", label = 'y2 ref')

ax1t = ax1.twinx()
ax1t.plot(t, phi_pp,linestyle = ':', color = "c", label = 'phi pp')


# Compute inputs to the system
u1 = map(lambda y5, y6: m*np.sqrt((y5**2) + (y6+g)**2) ,y5_pp,y6_pp)
u2 = map(lambda y5,y6,y7,y8,v1,v2: get_u2(y5,y6,y7,y8,v1,v2), y5_pp,y6_pp,y7_pp,y8_pp,v1_pp,v2_pp)

# and draw them
#ax1t = ax1.twinx()
ax3.plot(t,u1,linestyle = '-',color ='r', label = "u1 pp")
ax3.plot(t,u2,linestyle = '-',color ='g', label = "u2 pp")

ax2.plot(y1_pp,y2_pp, linestyle = '-',color ='r', label = "x-x pos pp")



#***********************************************#
#   FULL STATE FEEDBACK CLOSED LOOP SYSTEM      #
#			VIA LQR CONTROLLER					#
#***********************************************#

# Performance index weights For X,Y Dynamics: Qx1, Qy1a
# This performance index puts much greater emphasis on position performance
# and very low emphasis on control effort
Qx1 = np.diag([100.0, 1.0, 1.0, 1.0]);
Qu1a = np.diag([1.0]);

# LQR controller gains for Y1 Dynamics
(K, X, E) = ctl.lqr(A, B, Qx1, Qu1a)
K1 = np.matrix(K)
print("LQR gains For y1: {}".format(K1))
# LQR controller gains for Y2 Dynamics
(K, X, E) = ctl.lqr(A, B, Qx1, Qu1a)
K2 = np.matrix(K)
print("LQR gains For y2: {}".format(K2))

# and create controlled system
cl_ss_y1 = ctl.ss(A-B*K1, B*(Nu_y1 + K1*Nx_y1)*1.0,Cy1,D)
cl_ss_y2 = ctl.ss(A-B*K2, B*(Nu_y1 + K2*Nx_y1)*1.0,Cy1,D) 


# get closed loop system step response,   refy1 is an step signal
#ty, y1=ctl.step_response(cl_ss_y1, T = t) 
t, y1, s_y1 = ctl.forced_response(cl_ss_y1, T=t, U=refy1)
t, y2, s_y2 = ctl.forced_response(cl_ss_y2, T=t, U=refy2)


# calculate commanded input signal:   u = -K*x  where x is the state
v1_lqr = list()
v2_lqr = list()
for state1, state2 in zip(s_y1.T,s_y2.T):
	inp1 = 1.0*np.matrix(state1)*K1.T
	inp2 = 1.0*np.matrix(state2)*K2.T
	v1_lqr.append(inp1.item(0,0))   # inp.item(0,0) gets element in np.matrix... so strange syntax
	v2_lqr.append(inp2.item(0,0))

# get state variables and input evolution
y1_lqr = s_y1[0,:]
y3_lqr = s_y1[1,:]
y5_lqr = s_y1[2,:]
y7_lqr = s_y1[3,:]
v1_lqr = np.array(v1_lqr)

y2_lqr = s_y2[0,:]
y4_lqr = s_y2[1,:]
y6_lqr = s_y2[2,:]
y8_lqr = s_y2[3,:]
v2_lqr = np.array(v2_lqr)

#compute phi angle
phi_lqr = np.arctan2(-y5_lqr,y6_lqr+g)

# plot controlled step response and other states
ax1.plot(t,y1_lqr,linestyle = '-.',color ='r', label = "y1 lqr")
#ax1.plot(t,y3_lqr,linestyle = '-.',color ='g', label = "y3 lqr")
#ax1.plot(t,y5_lqr,linestyle = '-.',color ='b', label = "y5 lqr")
#ax1.plot(t,y7_lqr,linestyle = '-.',color ='m', label = "y7 lqr")
ax1.plot(t,y2_lqr,linestyle = '-.',color ='maroon', label = "y2 lqr")
#ax1.plot(t,y4_lqr,linestyle = '-.',color ='lightgreen', label = "y4 lqr")
#ax1.plot(t,y6_lqr,linestyle = '-.',color ='skyblue', label = "y6 lqr")
#ax1.plot(t,y8_lqr,linestyle = '-.',color ='pink', label = "y8 lqr")

ax1t.plot(t, phi_lqr,linestyle = ':', color = "aqua", label = 'phi lqr' )

# compute inputs to system
u1 = map(lambda y5, y6: m*np.sqrt((y5**2) + (y6+g)**2) ,y5_lqr,y6_lqr)
u2 = map(lambda y5,y6,y7,y8,v1,v2: get_u2(y5,y6,y7,y8,v1,v2), y5_lqr,y6_lqr,y7_lqr,y8_lqr,v1_lqr,v2_lqr)

# and plot them
#ax1t = ax1.twinx()
ax3.plot(t,u1,linestyle = '-.',color ='r', label = "u1 lqr")
ax3.plot(t,u2,linestyle = '-.',color ='g', label = "u2 lqr")

ax2.plot(y1_lqr,y2_lqr,linestyle = '-',color ='g', label = "x-y pos lqr")
ax2.set_title("2D Position", fontsize='small')
ax2.legend(loc='center right', shadow=True, fontsize='small')
ax2.set_xlabel("x {m}")
ax2.set_ylabel("y {m}")


#ax1.set_title("Closed loop with  LQR controller gains K1={:.2f}, K2={:.2f}, K3={:.2f}, K4={:.2f}".format(Kx.item(0,0),Kx.item(0,1),Kx.item(0,2),Kx.item(0,3)), fontsize='small')
ax1.set_title("System Response with Flatness based controller")
ax1.legend(loc='center right', shadow=True, fontsize='small')
ax1.set_xlabel("time {s}")
ax1.set_ylabel("y1 {m}")
ax1.set_yticks(p_ticks)

ax1t.set_ylabel("roll angle {rad}")
#ax1t.set_yticks(np.linspace(min(v1_pp),max(v1_pp),6))
ax1t.legend(loc='lower right', shadow=True, fontsize='small')

#plot inputs
#ax3.plot(t,u1, linestyle = '-',color ='royalblue', label = "u1")
#ax3.plot(t,u2,linestyle = '-',color ='g', label = "u2")

ax3.set_title("System Input", fontsize='small')
ax3.legend(loc='center right', shadow=True, fontsize='small')
ax3.set_xlabel("time {s}")
ax3.set_ylabel("")


#***********************************************#
#    SYSTEM DYNAMICS SIMULATION 			    #
#	 using differential flatness				#
#***********************************************#
	





u1 = map(lambda y5, y6: m*np.sqrt((y5**2) + (y6+g)**2) ,y5_pp,y6_pp)
u2 = map(lambda y5,y6,y7,y8,v1,v2: get_u2(y5,y6,y7,y8,v1,v2), y5_pp,y6_pp,y7_pp,y8_pp,v1_pp,v2_pp)


# define an input manually
t_max = len(t)
#u2 = np.zeros_like(t) #u_lqr
#u1 = np.zeros_like(t) #u_pp
#u1[0:200] = g*5
#u1[200:t_max] = m*g
#u2[0:100] = -1.0


# initial conditions
ry_0 = 0.0
vy_0 = 0.0
rz_0 = 0.0
vz_0 = 0.0
phi_0 = 0.0
omega_0 = 0.0

def get_ay_n(phi_n,u1_n):
	ay_n1 = -1.0*np.sin(phi_n)*u1_n/m
	return ay_n1

def get_vy_n1(vn,an,an_1,dt):
	vy_n1 = vn + 0.5*dt*(an_1+an)
	return vy_n1

def get_ry_n1(rn,vn,an,dt):
	ry_n1 = rn+ dt*vn + 0.5*(dt**2)*an
	return ry_n1


def get_az_n(phi_n,u1_n):
	ay_n = np.cos(phi_n)*u1_n/m  - g
	return ay_n

def get_vz_n1(vn,an,an_1,dt):
	vy_n1 = vn + 0.5*dt*(an_1+an)
	return vy_n1

def get_rz_n1(rn,vn,an,dt):
	ry_n1 = rn+ dt*vn + 0.5*(dt**2)*an
	return ry_n1

def get_alpha(u2_n):
	alpha = u2_n*l/Ixx
	return alpha

def get_omega_n1(omega_n,alpha_n,alpha_n1,dt):
	omega_n1 = omega_n + 0.5*dt*(alpha_n1+alpha_n)
	return omega_n1

def get_phi_n1(phi_n, omega_n, alpha_n,dt):
	phi_n1 = phi_n + dt*omega_n + 0.5*(dt**2)*alpha_n
	return phi_n1

Kp = 1.0
Kd = 0.01
Ki = .01
err = 0
def get_u1_pid(rz_real,vz_real,error):
	error = error + (Ry2 - rz_real)
	u1v = Kp*(Ry2 - rz_real) + Kd*(vz_real) + Ki*error
	return u1v, error


alpha = get_alpha(np.array(u2))

# initialize all arrays
ry = np.array([ry_0])
vy = np.array([vy_0])
ay = np.array([get_ay_n(phi_0,u1[0])])
rz = np.array([rz_0])
vz = np.array([vz_0])
az = np.array([get_az_n(phi_0,u1[0])])
omega = np.array([omega_0])
phi = np.array([phi_0])

#u1 = np.array([get_u1_pid(rz[0],vz[0],err)])


for n in range(1,len(alpha)):

	#u1_n, err = get_u1_pid(rz[n-1],vz[n-1], err)
	#u1 = np.append(u1,u1_n)


	# compute roll (phi) angula acceleration alpha, angular velocity omega,
	# and angular position phi 
	phi_1 = get_phi_n1(phi[n-1],omega[n-1],alpha[n-1],dt)
	omega_1 = get_omega_n1(omega[n-1], alpha[n-1], alpha[n], dt)
	# add new omega and phi values to each lists
	omega = np.append(omega,omega_1)
	phi = np.append(phi,phi_1)

	# compute Y axis accelerations, positions and velocities
	ay_n = get_ay_n(phi[n],u1[n])
	# add new acceleration value to list
	ay = np.append(ay,ay_n)

	ry_1 = get_ry_n1(ry[n-1],vy[n-1],ay[n-1],dt)
	vy_1 = get_vy_n1(vy[n-1],ay[n-1],ay[n],dt)
	# add new position and velocity to each list
	ry = np.append(ry,ry_1)
	vy = np.append(vy,vy_1)

	# compute Z axis accelerations, positions and velocities
	az_n = get_az_n(phi[n],u1[n])
	# add new acceleration value to list
	az = np.append(az,az_n)
	
	rz_1 = get_rz_n1(rz[n-1],vz[n-1],az[n-1],dt)
	vz_1 = get_vz_n1(vz[n-1],az[n-1],az[n],dt)
	# add new position and velocity to each list
	rz = np.append(rz,rz_1)
	vz = np.append(vz,vz_1)





# plot responses
#ax2.plot(t,alpha, linestyle = '-',color ='r', label = "alpha")
#ax2.plot(t,omega,linestyle = '-',color ='g', label = "omega")
#ax2.plot(t,phi,linestyle = '-',color ='b', label = "phi")
#ax2.plot(t,u2,linestyle = '-',color ='m', label = "u2")




# plot responses
#ax3.plot(t,ry, linestyle = '-',color ='r', label = "ry")
#ax3.plot(t,vy,linestyle = '-',color ='g', label = "vy")
#ax3.plot(t,ay,linestyle = '-',color ='b', label = "ay")
#ax3.plot(t,rz, linestyle = '--',color ='r', label = "rz")
#ax3.plot(t,vz,linestyle = '--',color ='g', label = "vz")
#ax3.plot(t,az,linestyle = '--',color ='b', label = "az")






plt.show()