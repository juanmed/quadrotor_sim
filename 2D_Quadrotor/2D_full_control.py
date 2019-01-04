#2D quadrotor controller and estimator
# The method for analyzing the systems in python was inspired on
# https://www.cds.caltech.edu/~murray/wiki/index.php/Python-control/Example:_Vertical_takeoff_and_landing_aircraft


import numpy as np
import control as ctl
import matplotlib.pyplot as plt
from scipy import signal

fig = plt.figure(figsize=(20,10))
fig.suptitle(" 2D (Y-Z) Quadrotor Control and State Estimation")
ax0 = fig.add_subplot(2,2,1)
ax1 = fig.add_subplot(2,2,2)
ax2 = fig.add_subplot(2,2,3)
ax3 = fig.add_subplot(2,2,4) 

track = plt.figure(figsize=(20,10))
track.suptitle(" 2D (Y-Z) Quadrotor Command Tracking ")
track0 = track.add_subplot(1,1,1)

# define constants
g = 9.81 #m/s2
m = 1.0  #kg  mass of the quadrotor
l = 0.2 #m  arm lenght from center of mass to each rotor
Ixx = (m*l**2)/2.0  # moment of inertia around X-axis (quadrotor rotates around X-axis)
b = 0.01  # air drag/friction force
c = 0.2 #air friction constant

air = 1.0  # set to 1 to activate air model, otherwise set to 0


# define time
t_max = 20.0  #s
t = np.arange(0.0,t_max,0.1)

# define step input gain:  u(t) = R*u(t)
# for each control variable
Ry = 10.0
refy = Ry*np.ones_like(t)
Rz = 11.0
refz = Rz*np.ones_like(t)
Rpsi = 10.0
refpsi = Rpsi*np.ones_like(t)

# ticks
p_ticks = np.arange(0,max([Ry,Rz])+2,1)        #position ticks
angle_ticks = np.arange(-np.pi,np.pi,np.pi/4)  #angle ticks

# equilibrium points
xe = [0.0,0.0,0.0,0.0,0.0,0.0]
ue = [m*g,0.0]

#define A matrix (It is the linearization)
A = np.matrix(
	[[-air*c/m,0.0,0.0,0.0,0.0,-np.cos(xe[5])*ue[0]/m],
	[0.0,-air*c/m,0.0,0.0,0.0,-np.sin(xe[5])*ue[0]/m],
	[1.0,0.0,0.0,0.0,0.0,0.0],
	[0.0,1.0,0.0,0.0,0.0,0.0],
	[0.0,0.0,0.0,0.0,0.0,0.0],
	[0.0,0.0,0.0,0.0,1.0,0.0]])

# Since python control works only with SISO systems,
# define two input matrices, Bu1  for u1 input related to translation quantities
# and Bu2 for u2 input related to rotation quantities
Bu1 = np.matrix(
	[[0.0],
	[1.0/m],
	[0.0],
	[0.0],
	[0.0],
	[0.0]])

Bu2 = np.matrix(
	[[0.0],
	[0.0],
	[0.0],
	[0.0],
	[l/Ixx],
	[0.0]])

# define output matrices for the various control variables
Cy = np.matrix([0.0,0.0,1.0,0.0,0.0,0.0])
Cz = np.matrix([0.0,0.0,0.0,1.0,0.0,0.0])
Cpsi = np.matrix([0.0,0.0,0.0,0.0,0.0,1.0])

# transmission matrix
D = np.matrix([0.0])

# Open Loop Systems Response
y_ol = ctl.ss(A,Bu1,Cy,D)
z_ol = ctl.ss(A,Bu1,Cz,D)
psi_ol = ctl.ss(A,Bu2,Cpsi,D)

ty, y=ctl.step_response(y_ol, T = t)
tz, z=ctl.step_response(z_ol, T = t)
tpsi, psi=ctl.step_response(psi_ol, T = t)


ax0.plot(t,y, linestyle = '-',color ='r', label = "Y position")
ax0.plot(t,z, linestyle = ':',color ='g', label = "Z position")
ax0t = ax0.twinx()
#ax0t.plot(t,psi,linestyle = '--',color ='b', label = "Roll  Angle" )
ax0.set_title("Step response of open loop system", fontsize='small')
ax0.legend(loc='center right', shadow=True, fontsize='small')
ax0.set_xlabel("time {s}")
ax0.set_ylabel("Position {m}")
#ax0.set_yticks(p_ticks)
#ax0t.set_ylabel("Angle {rad}")
#ax0t.set_yticks(angle_ticks)


#*****************************************#
#          FULL STATE FEEDBACK CONTROLLER 
#*****************************************#
# Define performance index weights
Qx1 = np.diag([1, 1, 1, 1, 1, 1]);
Qu1a = np.diag([1,1]);
B = np.concatenate((Bu1,Bu2), axis=1)
(K, X, E) = ctl.lqr(A, B, Qx1, Qu1a)
K = np.matrix(K)
#print(K)

# Calculate reference input matrices for each reference input
# z, y, psi
aug1 = np.concatenate((A,Bu1), axis = 1)
aug2 = np.concatenate((Cy,D), axis = 1)
y_input = np.concatenate((aug1,aug2), axis = 0)
aug1 = np.concatenate((A,Bu1), axis = 1)
aug2 = np.concatenate((Cz,D), axis = 1)
z_input = np.concatenate((aug1,aug2), axis = 0)
aug1 = np.concatenate((A,Bu2), axis = 1)
aug2 = np.concatenate((Cpsi,D), axis = 1)
psi_input = np.concatenate((aug1,aug2), axis = 0)
mult = np.matrix([ [0],[0],[0],[0],[0],[1]])
#print(y_input,z_input,psi_input)


#*****************************************#
#     SEPARATE SYSTEM DYNAMICS 
#*****************************************#

# Lateral Dynamics

A_lat = np.matrix(
	[[-air*c/m,0.0,0.0,-np.cos(xe[5])*ue[0]/m],
	[1.0,0.0,0.0,0.0],
	[0.0,0.0,0.0,0.0],
	[0.0,0.0,1.0,0.0]])

B_lat = np.matrix(
	[[0.0],
	[0.0],
	[l/Ixx],
	[0.0]])

C_laty = np.matrix([0.0,1.0,0.0,0.0])
C_latpsi = np.matrix([0.0,0.0,0.0,1.0])

# Vertical Dynamics

A_vert = np.matrix(
	[[-air*c/m,0.0],
	[1.0,0.0]])

B_vert = np.matrix(
	[[1.0/m],
	[0.0]])

C_vertz = np.matrix([0.0,1.0])

#print(A_lat,B_lat,C_laty, C_latpsi, A_vert, B_vert, C_vertz)

#*****************************************#
#          FULL STATE FEEDBACK CONTROLLER FOR LATERAL AND VERTICAL DYNAMICS
#*****************************************#

#*****************************************#
#          LATERAL DYNAMICS
#*****************************************#


# Define performance index weights For Lateral Dynamics
Qx1 = np.diag([10, 100, 1, 1]);
Qu1a = np.diag([1]);
(K, X, E) = ctl.lqr(A_lat, B_lat, Qx1, Qu1a)
K_lat = np.matrix(K)
print("LQR gains for Lateral Dynamics")
print(K_lat)

# Calculate Reference input matrices
# First for Y position 
aug1 = np.concatenate((A_lat,B_lat), axis = 1)
aug2 = np.concatenate((C_laty,D), axis = 1)
y_input = np.concatenate((aug1,aug2), axis = 0)
mult = np.matrix([[0],[0],[0],[0],[1]])

y_input = np.linalg.inv(y_input)*mult

print(y_input)
Nx_y = y_input[(0,1,2,3),]  #First three rows
Nu_y = y_input[4,]          # Last row
print(" Y position input matrices")
print(Nx_y)
print(Nu_y)

#Now for psi .... maybe not possible
aug1 = np.concatenate((A_lat,B_lat), axis = 1)
aug2 = np.concatenate((C_latpsi,D), axis = 1)
psi_input = np.concatenate((aug1,aug2), axis = 0)
mult = np.matrix([[0],[0],[0],[1]])
#psi_input = np.linalg.inv(psi_input)*mult

# create State Space system and plot step response
cl_ss_y = ctl.ss(A_lat-B_lat*K_lat, B_lat*(Nu_y + K_lat*Nx_y)*Ry,C_laty,D)
ty, y=ctl.step_response(cl_ss_y, T = t)


ax1.plot(t,y,linestyle = '-',color ='r', label = "Y position")
ax1.plot(t, refy,linestyle = '--', color = "k", label = 'Y desired')
ax1.set_title("Closed loop with  LQR controller gains K1={:.2f}, K2={:.2f}, K3={:.2f}, K4={:.2f}".format(K_lat.item(0,0),K_lat.item(0,1),K_lat.item(0,2),K_lat.item(0,3)), fontsize='small')
ax1.legend(loc='center right', shadow=True, fontsize='small')
ax1.set_xlabel("time {s}")
ax1.set_ylabel("Position {m}")
ax1.set_yticks(p_ticks)

#*****************************************#
#          VERTICAL DYNAMICS
#*****************************************#

# Define performance index weights for Vertical Dynamics
Qx1 = np.diag([10, 100]);
Qu1a = np.diag([1]);
(K, X, E) = ctl.lqr(A_vert, B_vert, Qx1, Qu1a)
K_vert = np.matrix(K)
print("LQR gains for Vertical Dynamics")
print(K_vert)

# Calculate Reference input matrices
# First for Y position 
aug1 = np.concatenate((A_vert,B_vert), axis = 1)
aug2 = np.concatenate((C_vertz,D), axis = 1)
z_input = np.concatenate((aug1,aug2), axis = 0)
mult = np.matrix([[0],[0],[1]])

z_input = np.linalg.inv(z_input)*mult

Nx_z = z_input[(0,1),]  #First two rows
Nu_z = z_input[2,]          # Last row
print(" Y position input matrices")
print(Nx_z)
print(Nu_z)

# create State Space system and plot step response
cl_ss_z = ctl.ss(A_vert-B_vert*K_vert, B_vert*(Nu_z + K_vert*Nx_z)*Rz,C_vertz,D)
tz, z=ctl.step_response(cl_ss_z, T = t)


ax1.plot(t,z,linestyle = '-',color ='g', label = "Z position")
ax1.plot(t, refz,linestyle = '--', color = "m", label = 'Z desired')
ax1.legend(loc='center right', shadow=True, fontsize='small')




#*****************************************#
#          FULL STATE ESTIMATOR FOR LATERAL AND VERTICAL DYNAMICS
#*****************************************#

#*****************************************#
#          LATERAL DYNAMICS
#*****************************************#
# seed the generator for repeatable results

np.random.seed(2)

# add 1% noise
noise_percent = 0.01
A_latn = noise_percent*np.random.rand(4,4)+A_lat
B_latn = noise_percent*np.random.rand(4,1)+B_lat
C_latyn = noise_percent*np.random.rand(1,4)+C_laty
C_latpsin = noise_percent*np.random.rand(1,4)+C_latpsi
Dn = noise_percent*np.random.randn()+D


# Define performance index weights For Lateral Dynamics
Qx1 = np.diag([10, 100, 1, 1]);
Qu1a = np.diag([1]);
(L, X, E) = ctl.lqr(A_latn.T, C_latyn.T , Qx1, Qu1a)
L_lat = np.matrix(L.T)    # Remember to transpose the resulting gains
print("Estimator gains: {},{},{},{}".format(L_lat.item(0,0),L_lat.item(1,0),L_lat.item(2,0),L_lat.item(3,0))) 

# define reference input matrix
Nbar_y = Nu_y + K_lat*Nx_y

# define estimated system and its step output
cl_ess_y = ctl.ss(A_latn-B_latn*K_lat - L_lat*C_latyn + L_lat*C_laty, B_latn*Nbar_y*Ry, C_latyn, D)
ty, y=ctl.step_response(cl_ess_y, T = t)

ax2.plot(t, y,linestyle = '-',color ='r', label = "Y position")
ax2.plot(t, refy,linestyle = '--', color = "k", label = 'Y desired')
ax2.set_title("Closed loop with LQR estimator gains L1={:.1f}, L2={:.2f}, L3={:.2F}, L4={:.2f}".format(L_lat.item(0,0),L_lat.item(1,0),L_lat.item(2,0), L_lat.item(3,0)), fontsize='small')
ax2.legend(loc='lower right', shadow=True, fontsize='small')
ax2.set_xlabel("time {s}")
ax2.set_ylabel("Position {m}")
ax2.set_yticks(p_ticks)


#*****************************************#
#          VERTICAL DYNAMICS
#*****************************************#

# add 1% noise
noise_percent = 0.01
A_vertn = noise_percent*np.random.rand(2,2)+A_vert
B_vertn = noise_percent*np.random.rand(2,1)+B_vert
C_vertzn = noise_percent*np.random.rand(1,2)+C_vertz


# Define performance index weights For Lateral Dynamics
Qx1 = np.diag([10, 100]);
Qu1a = np.diag([1]);
(L, X, E) = ctl.lqr(A_vertn.T, C_vertzn.T , Qx1, Qu1a)
L_vert = np.matrix(L.T)    # Remember to transpose the resulting gains
print("Estimator gains: {},{}".format(L_lat.item(0,0),L_lat.item(1,0)))

# define reference input matrix
Nbar_z = Nu_z + K_vert*Nx_z

# define estimated system and its step output
cl_ess_z = ctl.ss(A_vertn-B_vertn*K_vert - L_vert*C_vertzn + L_vert*C_vertz, B_vertn*Nbar_z*Rz, C_vertzn, D)
tz, z=ctl.step_response(cl_ess_z, T = t)

ax2.plot(t,z,linestyle = '-',color ='g', label = "Z position")
ax2.plot(t, refz,linestyle = '--', color = "m", label = 'Z desired')
ax2.legend(loc='center right', shadow=True, fontsize='small')




#*****************************************#
#          DEFINE A PERSONALIZED INPUT SIGNAL
#*****************************************#

# reseed to generate different results
np.random.seed()

# Redefine the System just to remove the input Reference gain R
# so that that it follows input signal and not (input signal)*R
cl_ess_y = ctl.ss(A_latn-B_latn*K_lat - L_lat*C_latyn + L_lat*C_laty, B_latn*Nbar_y*1.0, C_latyn, D)
cl_ess_z = ctl.ss(A_vertn-B_vertn*K_vert - L_vert*C_vertzn + L_vert*C_vertz, B_vertn*Nbar_z*1.0, C_vertzn, D)


signaly = np.ones_like(t)
signalz = np.ones_like(t)
random_timesy = list()
random_timesz = list()

# define number of time slots
slotsy = np.random.randint(0,5)
slotsz = np.random.randint(0,5)

for i in range(slotsy):
	random_timesy.append(int((len(signaly)-1)*np.random.random_sample()))
for i in range(slotsz):
	random_timesz.append(int((len(signalz)-1)*np.random.random_sample()))

# add the index of the last element
random_timesy.append(len(signaly)-1)
random_timesz.append(len(signalz)-1)

random_timesy = np.unique(np.sort(random_timesy)).flatten()
random_timesz = np.unique(np.sort(random_timesz)).flatten()

#print(random_times)
start = 0
for i in random_timesy:
	signaly[start:i] = signaly[start:i]*(Ry*np.random.random_sample())
	start = i

start = 0
for i in random_timesz:
	signalz[start:i] = signalz[start:i]*(Rz*np.random.random_sample())
	start = i	

# evaluate the forced response of both systems
t, y, x = ctl.forced_response(cl_ess_y, T=t, U=signaly)
t, z, x = ctl.forced_response(cl_ess_z, T=t, U=signalz)


ax3.plot(t, y,linestyle = '-',color ='r', label = "Y position")
ax3.plot(t, signaly, linestyle = '--',color = "y", label = 'Y desired')
ax3.plot(t, z,linestyle = '-',color ='b', label = "Z position")
ax3.plot(t, signalz, linestyle = '--',color = "c", label = 'Z desired')
ax3.set_title("Reference Tracking",fontsize='small')
ax3.legend(loc='upper right', shadow=True, fontsize='small')
ax3.set_xlabel("time {s}")
ax3.set_ylabel("Position {m}")
ax3.set_yticks(p_ticks)

track0.plot(y,z, linestyle='-', color = 'r', label = 'Quadrotor Position')
track0.plot(signaly,signalz, linestyle='-', color = 'b', label = 'Desired Position')
track0.legend(loc='upper right', shadow=True, fontsize='small')
track0.set_xlabel("Y Position {m}")
track0.set_ylabel("Z Position {m}")
track0.text(signaly[0],signalz[0], "Start")
track0.text(signaly[-1],signalz[-1], "Finish")


plt.show()