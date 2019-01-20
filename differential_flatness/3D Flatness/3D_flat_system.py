# This program contains  a differential flatness controller for a quadrotor
#


import numpy as np
import matplotlib.pyplot as plt
import argparse
import control as ctl

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

def gen_helix_trajectory(t):
	"""
		This function returns the trajectory: position, velocity,
		acceleration, jerk and snap an object going through a 3D helix 
		should have.

	"""
	a = 1.0
	b = 1.0
	c = 1.0

	wx = 1.0
	wy = 1.0

	x_0 = 1.0
	y_0 = 1.0
	z_0 = 0.0

	# positions in helix
	x = a*np.cos(wx*t) + x_0
	y = b*np.sin(wy*t) + y_0
	z = c*t
	psi = 0.0*np.ones_like(t)

	# velocities in helix
	v_x = -a*wx*np.sin(wx*t)
	v_y = b*wy*np.cos(wy*t)
	v_z = c*np.ones_like(t)
	psi_rate = 0.0*np.ones_like(t)

	# accelerations in helix
	a_x = -(wx**2)*(x - x_0)
	a_y = -(wy**2)*(y - y_0)
	a_z = 0.0*np.ones_like(t)
	psi_dd = 0.0*np.ones_like(t)

	# jerks in helix
	j_x = -(wx**2)*(v_x)
	j_y = -(wy**2)*(v_y)
	j_z = 0.0*np.ones_like(t)
	psi_ddd = 0.0*np.ones_like(t)

	# snap in helix
	s_x = -(wx**2)*(a_x)
	s_y = -(wy**2)*(a_y)
	s_z = 0.0*np.ones_like(t)
	psi_dddd = 0.0*np.ones_like(t)

	return [x,y,z,v_x,v_y,v_z,a_x,a_y,a_z,j_x,j_y,j_z,s_x,s_y,s_z, psi, psi_rate, psi_dd, psi_ddd, psi_dddd]

def get_x(sigma1):
	return sigma1

def get_y(sigma2):
	return sigma2

def get_z(sigma3):
	return sigma3

def get_psi(sigma4):
	return sigma4

def get_u1(m,t):
	u1 = m*np.linalg.norm(t)
	return u1

def get_zb(t):
	zb = t/(np.linalg.norm(t))
	return zb

def get_u1_dot(z_b,j):
	u1_dot = m*z_b.T*j
	return u1_dot

def get_t_vector(sigma1, sigma2,sigma3):
	# construct the t vector at each point in time
	t_vec = np.matrix([[sigma1],[sigma2],[sigma3+g]])
	return t_vec

def get_xc(sigma4):
	x_c = np.matrix([[np.cos(sigma4)],[np.sin(sigma4)],[0.0]])
	return x_c

def get_yc(sigma4):
	y_c = np.matrix([[-1.0*np.sin(sigma4)],[np.cos(sigma4)],[0.0]])
	return y_c	

def get_xb(y_c,z_b):
	a = np.cross(y_c, z_b, axis = 0)
	a = np.matrix(a)
	return a/np.linalg.norm(a)

def get_yb(z_b,x_b):
	a = np.cross(z_b,x_b, axis = 0)
	a = np.matrix(a)
	return a/np.linalg.norm(a)

def get_wx(y_b,j,u_1):
	wx = -1.0*m*((y_b.T)*j)/u_1
	return wx

def get_wy(x_b,j,u_1):
	wy = m*((x_b.T)*j)/u_1
	return wy

def get_wz(psi_rate,x_c,x_b,w_y,y_c,z_b):
	"""
		Will compute as wz = (a + b)/c
	"""
	a = psi_rate*(x_c.T)*x_b
	b = w_y*(y_c.T)*z_b
	c = np.linalg.norm(np.cross(y_c,z_b,axis = 0))
	wz = (a+b)/c
	return wz

def get_wy_dot(x_b,s,u_1_dot,w_y,u_1,w_x,w_z):
	"""
		Will use wy_dot = (a + b + c)/d
	"""
	a = (x_b.T)*s 
	b = -2.0*u_1_dot*w_y/m
	c = -1.0*u_1*w_x*w_z/m
	d = u_1/m
	w_y_dot = (a+b+c)/d
	return w_y_dot

def get_wx_dot(y_b,s,u_1_dot,w_x,u_1,w_y,w_z):

	"""
		Will use wx_dot = (a + b + c)/d
	"""
	a = (y_b.T)*s
	b = 2.0*u_1_dot*w_x/m
	c = -1.0*u_1*w_y*w_z/m
	d = u_1/m
	w_x_dot = (a+b+c)/d
	return w_x_dot

def get_wz_dot(psi_acc,x_c,x_b,psi_rate,w_z,y_b,w_y,z_b,w_x,y_c,w_y_dot):
	"""
		Will compute as w_z_dot = (a+b+c+d+e+f)/g

	"""
	a = psi_acc*(x_c.T)*x_b
	b = 2.0*psi_rate*w_z*(x_c.T)*y_b
	c = -2.0*psi_rate*w_y*(x_c.T)*z_b
	d = -1.0*w_x*w_y*(y_c.T)*z_b
	e = -1.0*w_x*w_z*(y_c.T)*z_b
	f = w_y_dot*(y_c.T)*z_b
	g = np.linalg.norm(np.cross(y_c,z_b,axis = 0))
	w_z_dot = (a+b+c+d+e+f)/g
	return w_z_dot

def get_ux(w_dot_,w_):
	u_x = I*w_dot_ + np.matrix(np.cross(w_, I*w_, axis = 0))
	return u_x

# get arguments to the program
args = createCommandLineParser()

fig = plt.figure(figsize=(20,10))
fig.suptitle(" 3D (Y-Z) Quadrotor Control using Differential Flatness Property")
ax0 = fig.add_subplot(2,2,1, projection='3d')
ax1 = fig.add_subplot(2,2,2)
ax2 = fig.add_subplot(2,2,3)#, projection='3d')
ax3 = fig.add_subplot(2,2,4)#, projection='3d') 

# define constants
g = 9.81 #m/s2
b = 0.01  # air drag/friction force
#c = 0.2 #air friction constant

# quadrotor physical constants
m = 0.18  #kg  mass of the quadrotor
Ktao =0.02           # Drag torque constant for motors
Kt =0.2             # Thrust constant for motors
I = np.matrix([[0.00025, 0, 2.55e-6],
              [0, 0.000232, 0],
              [2.55e-6, 0, 0.0003738]]);

# define simulation time
t_max = int(args.t)
dt = 0.01
t = np.arange(0.0,t_max,dt)

# generate trajectory
helix_traj = gen_helix_trajectory(t)

# get t vector
t_vector = map(lambda a,b,c: get_t_vector(a,b,c), helix_traj[6], helix_traj[7],helix_traj[8]) 

# get ref u1 input
u1 = map(lambda t: get_u1(m,t),t_vector)

# get ref zb (base frame e3)
zb = map(lambda t: get_zb(t),t_vector)
#print(zb[0:5])

# get ref yc (c frame e2)
yc = map(lambda psi: get_yc(psi),helix_traj[15])

# get ref xb (body frame e1)
xb = map(lambda a,b: get_xb(a,b),yc,zb)

# get ref yb (body frame e2)
yb = map(lambda a,b: get_yb(a,b),zb,xb)

# get ref jerk 
j = map(lambda a,b,c: np.matrix([[a],[b],[c]]),helix_traj[9],helix_traj[10],helix_traj[11])

# get ref wx (x component of angular velocity in body frame)
wx = map(lambda a,b,c: get_wx(a,b,c),yb,j,u1)

# get ref wy (y component of angular velocity in body frame)
wy = map(lambda a,b,c: get_wy(a,b,c),xb,j,u1)

# get ref xc (c frame e1)
xc = map(lambda a: get_xc(a),helix_traj[15])

# get ref wz (z component of angular velocity in body frame)
wz = map(lambda a,b,c,d,e,f: get_wz(a,b,c,d,e,f),helix_traj[16],xc,xb,wy,yc,zb)

# get referenece u1_dot (derivative of input 1)
u1_dot = map(lambda a,b: get_u1_dot(a,b),zb,j)

# get ref snap
s = map(lambda a,b,c: np.matrix([[a],[b],[c]]), helix_traj[12], helix_traj[13], helix_traj[14])

# get wy_dot (y component of angular acceleration in body frame)
wy_dot = map(lambda a,b,c,d,e,f,g: get_wy_dot(a,b,c,d,e,f,g), xb,s,u1_dot,wy,u1,wx,wz )   #get_wy_dot(x_b,s,u_1_dot,w_y,u_1,w_x,w_z)

# get wx_dot (x component of angular acceleration in body frame)
wx_dot = map(lambda a,b,c,d,e,f,g: get_wx_dot(a,b,c,d,e,f,g), yb,s,u1_dot,wx,u1,wy,wz )	  #get_wx_dot(y_b,s,u_1_dot,w_x,u_1,w_y,w_z)

# get wz_dot (z component of angular acceleration in body frame)
wz_dot = map(lambda a,b,c,d,e,f,g,h,i,j,k: get_wz_dot(a,b,c,d,e,f,g,h,i,j,k), helix_traj[17],xc,xb,helix_traj[16],wz,yb,wy,zb,wx,yc,wy_dot) ##get_wz_dot(psi_acc,x_c,x_b,psi_rate,w_z,y_b,w_y,z_b,w_x,y_c,w_y_dot)

# make w_dot vector. Need to extract each component and form it into a vector for each time step
w_dot = map(lambda a,b,c: np.matrix([ [a.item(0)], [b.item(0)], [c.item(0)] ]),wx_dot,wy_dot,wz_dot )

# make w vector. Again extract each component and put it into a vector for each time step
w = map(lambda a,b,c,: np.matrix([ [a.item(0)], [b.item(0)], [c.item(0)] ]), wx, wy, wz)

# get u_x (it is the vector of [u2,u3,u4] inputs)
ux = map(lambda a,b: get_ux(a,b), w_dot, w )

ax0.plot(helix_traj[0],helix_traj[1],helix_traj[2], color = "r", label = "Ref Traj")
ax0.set_title("Reference helix trajectory {3D}")
ax0.set_xlabel('x {m}')
ax0.set_ylabel('y {m}')
ax0.set_zlabel('z {m}')
ax0.legend(loc='lower right', shadow=True, fontsize='small')

ax1.plot(t,u1, color = "r", label = "u1")
ax1.set_title("Input U1 evolution")
ax1.set_xlabel('t {s}')
ax1.set_ylabel('u1 {N}')
ax1.legend(loc='lower right', shadow=True, fontsize='small')

ax2.plot(t,map(lambda v: v.item(0),ux), color = "r", label = "u2")
ax2.plot(t,map(lambda v: v.item(1),ux), color = "g", label = "u3")
ax2.plot(t,map(lambda v: v.item(2),ux), color = "b", label = "u4")
ax2.set_title("xb evolution")
ax2.set_xlabel('t {s}')
ax2.set_ylabel('{unit less}')
ax2.legend(loc='lower right', shadow=True, fontsize='small')

ax3.plot(t,map(lambda v: v.item(0),wz_dot), color ="r")#map(lambda v: np.linalg.norm(v),yc))#map(lambda v: v.item(0),wy))
ax3.plot(t,map(lambda v: v.item(0),wz), color ="g")
plt.show()
