# This program contains  a differential flatness controller for a quadrotor
#


import numpy as np
import matplotlib.pyplot as plt
import argparse
import control as ctl

from mpl_toolkits.mplot3d import Axes3D

#from math import sin, cos, asin, atan2, sqrt

def RotToRPY(R):
	"""
		Euler angle convention is ZYX, which means first apply
		rotaion of psi-degrees around Z axis, then rotation of
		theta-degrees around new Y axis, and then rotation of 
		phi-degrees around new X axis.

		The rotation R received should be from body to world frame.
	"""
	theta = np.arcsin(-1.0*R.item(2,0))
	phi = np.arctan2(R.item(2,1)/np.cos(theta),R.item(2,2)/np.cos(theta))
	psi = np.arctan2(R.item(1,0)/np.cos(theta),R.item(0,0)/np.cos(theta))

	return np.matrix([ [phi], [theta], [psi] ]) 

# Create a parser for the command line arguments
def createCommandLineParser():
	parser1 = argparse.ArgumentParser(description='3D Quadcopter linear controller simulation')
	#parser1.add_argument('-f', help='Path to log file.', default='log201811071627_1080p_0.txt')
	#parser1.add_argument('-b', help='Histogram bin size', default=200)
	#parser1.add_argument('-g', help='Path to gpx file.', default='20181112/2018._11._12._noon_5_09_18.gpx')
	#parser1.add_argument('-r', help='Video Resolution', default='1280*720.gpx')
	parser1.add_argument('-t', help='Total Simulation time', default=20.0)
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
	a = 2.0
	b = 2.0
	c = 5.0

	wx = 0.5
	wy = 1.0

	x_0 = 1.0
	y_0 = 1.0
	z_0 = 0.0	

	# positions in helix
	x = a*np.cos(wx*t) + x_0
	y = b*np.sin(wy*t) + y_0
	z = c*t
	#psi = 0.0*np.ones_like(t)
	#tangent_vector = map(lambda a,b,c: np.matrix([[a],[b],[0]]),-a*wx*np.sin(wx*t),b*wy*np.cos(wy*t),c)
	psi = np.sin(t)
	#psi = np.arccos( )

	# velocities in helix
	v_x = -a*wx*np.sin(wx*t)
	v_y = b*wy*np.cos(wy*t)
	v_z = c*np.ones_like(t)
	psi_rate = np.cos(t)#0.0*np.ones_like(t)

	# accelerations in helix
	a_x = -(wx**2)*(x - x_0)
	a_y = -(wy**2)*(y - y_0)
	a_z = 0.0*np.ones_like(t)
	psi_dd = -1.0*np.sin(t)#0.0*np.ones_like(t)

	# jerks in helix
	j_x = -(wx**2)*(v_x)
	j_y = -(wy**2)*(v_y)
	j_z = 0.0*np.ones_like(t)
	psi_ddd = -1.0*np.cos(t)#0.0*np.ones_like(t)

	# snap in helix
	s_x = -(wx**2)*(a_x)
	s_y = -(wy**2)*(a_y)
	s_z = 0.0*np.ones_like(t)
	psi_dddd = np.sin(t) #0.0*np.ones_like(t)

	return [x,y,z,v_x,v_y,v_z,a_x,a_y,a_z,j_x,j_y,j_z,s_x,s_y,s_z, psi, psi_rate, psi_dd, psi_ddd, psi_dddd]

def gen_helix_trajectory2(t):
	"""
		This function returns the trajectory: position, velocity,
		acceleration, jerk and snap an object going through a 3D helix 
		should have.
	"""
	a = 2.0
	b = 2.0
	c = 5.0

	wx = 0.5
	wy = 1.0

	x_0 = 1.0
	y_0 = 1.0
	z_0 = 0.0

	# positions in helix
	x = a*np.cos(wx*t) + x_0
	y = b*np.sin(wy*t) + y_0
	z = c*t
	#psi = 0.0*np.ones_like(t)
	#tangent_vector = map(lambda a,b,c: np.matrix([[a],[b],[0]]),-a*wx*np.sin(wx*t),b*wy*np.cos(wy*t),c)
	psi = np.sin(t)
	#psi = np.arccos( )

	# velocities in helix
	v_x = -a*wx*np.sin(wx*t)
	v_y = b*wy*np.cos(wy*t)
	v_z = c*np.ones_like(t)
	psi_rate = np.cos(t)#0.0*np.ones_like(t)

	# accelerations in helix
	a_x = -(wx**2)*(x - x_0)
	a_y = -(wy**2)*(y - y_0)
	a_z = 0.0*np.ones_like(t)
	psi_dd = -1.0*np.sin(t)#0.0*np.ones_like(t)

	# jerks in helix
	j_x = -(wx**2)*(v_x)
	j_y = -(wy**2)*(v_y)
	j_z = 0.0*np.ones_like(t)
	psi_ddd = -1.0*np.cos(t)#0.0*np.ones_like(t)

	# snap in helix
	s_x = -(wx**2)*(a_x)
	s_y = -(wy**2)*(a_y)
	s_z = 0.0*np.ones_like(t)
	psi_dddd = np.sin(t) #0.0*np.ones_like(t)

	# pack everything
	pos = np.array([[x],[y],[z]])
	vel = np.array([[v_x],[v_y],[v_z]])
	acc = np.array([[a_x],[a_y],[a_z]])
	jerk = np.array([[j_x],[j_y],[j_z]])
	snap = np.array([[s_x],[s_y],[s_z]])

	return [pos,vel,acc,jerk,snap, psi, psi_rate, psi_dd, psi_ddd, psi_dddd]


def gen_trajectory3(t, t_max):

	x,y,z,v_x,v_y,v_z,a_x,a_y,a_z,j_x,j_y,j_z,s_x,s_y,s_z = [list(),list(),list(),
															list(),list(),list(),
															list(),list(),list(),
															list(),list(),list(),
															list(),list(),list()]

	psi, psi_rate, psi_dd, psi_ddd, psi_dddd = list(),list(),list(),list(),list()

	a = 2.0
	b = 2.0
	c = 5.0

	wx = 0.5
	wy = 1.0

	x_0 = 1.0
	y_0 = 1.0
	z_0 = 0.0

	for (i,j) in enumerate(t):
		if( j < t_max):

			# positions in helix
			x.append(a*np.cos(wx*j) + x_0)
			y.append(b*np.sin(wy*j) + y_0)
			z.append(c*j + z_0)
			#psi = 0.0*np.ones_like(t)
			#tangent_vector = map(lambda a,b,c: np.matrix([[a],[b],[0]]),-a*wx*np.sin(wx*t),b*wy*np.cos(wy*t),c)
			psi.append(np.sin(j))
			#psi = np.arccos( )

			# velocities in helix
			v_x.append(-a*wx*np.sin(wx*j))
			v_y.append(b*wy*np.cos(wy*j))
			v_z.append(c)
			psi_rate.append(np.cos(j))#0.0*np.ones_like(t)

			# accelerations in helix
			a_x.append(-(wx**2)*(x[i] - x_0))
			a_y.append(-(wy**2)*(y[i] - y_0))
			a_z.append(0.0)
			psi_dd.append(-1.0*np.sin(j))#0.0*np.ones_like(t)

			# jerks in helix
			j_x.append(-(wx**2)*(v_x[i]))
			j_y.append(-(wy**2)*(v_y[i]))
			j_z.append(0.0)
			psi_ddd.append(-1.0*np.cos(j))#0.0*np.ones_like(t)

			# snap in helix
			s_x.append(-(wx**2)*(a_x[i]))
			s_y.append(-(wy**2)*(a_y[i]))
			s_z.append(0.0) #*np.ones_like(t)
			psi_dddd.append(np.sin(j)) #0.0*np.ones_like(t)

		else:
			# positions in helix
			x.append(a*np.cos(wx*t_max) + x_0)
			y.append(b*np.sin(wy*t_max) + y_0)
			z.append(c*t_max + z_0)
			psi.append(np.sin(t_max))
			#psi = np.arccos( )

			# velocities in helix
			v_x.append(0.0)
			v_y.append(0.0)
			v_z.append(0.0)
			psi_rate.append(0.0)#np.cos(i))

			# accelerations in helix
			a_x.append(0.0)
			a_y.append(0.0)
			a_z.append(0.0)
			psi_dd.append(0.0)#-1.0*np.sin(i))#

			# jerks in helix
			j_x.append(0.0)
			j_y.append(0.0)
			j_z.append(0.0)
			psi_ddd.append(0.0)#-1.0*np.cos(i))#0.0*np.ones_like(t)

			# snap in helix
			s_x.append(0.0)
			s_y.append(0.0)
			s_z.append(0.0) #*np.ones_like(t)
			psi_dddd.append(0.0)#np.sin(i)) #0.0*np.ones_like(t)



	return [x,y,z,v_x,v_y,v_z,a_x,a_y,a_z,j_x,j_y,j_z,s_x,s_y,s_z, psi, psi_rate, psi_dd, psi_ddd, psi_dddd]


def get_x(sigma1):
	return sigma1

def get_y(sigma2):
	return sigma2

def get_z(sigma3):
	return sigma3

def get_psi(sigma4):
	return sigma4

def get_u1(t):
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
	#print("w_z_dot type is: {}".format(type(w_z_dot)))
	return w_z_dot

# This correspond to [u1, u2, u3]
def get_ux(w_dot_,w_):
	u_x = I*w_dot_ + np.matrix(np.cross(w_, I*w_, axis = 0))
	return u_x

def get_ua(u_1,z_b):
	"""
		ua = -g*z_w +u1*z_b/m
	"""
	u_a = -g*np.matrix([[0.0],[0.0],[1.0]]) + u_1*z_b/m
	return u_a

def get_ub(w_,M):
	u_b = invI*(-1.0*np.cross(w_, I*w_, axis = 0) + M)
	return u_b

def get_uc(w_,ori):
	"""
	"""
	phi_ = ori.item(0)
	theta_ = ori.item(1)
	psi_ = ori.item(2)

	peta = np.matrix([
		[1.0, np.sin(phi_)*np.tan(theta_), np.cos(phi_)*np.tan(theta_)],
		[0.0, np.cos(phi_),-1.0*np.sin(phi_)],
		[0.0, np.sin(phi_)/np.cos(theta_), np.cos(phi_)/np.cos(theta_)]])
	u_c = peta*w_

	return u_c 

def compute_ref(trajectory):
	"""
		Compute all reference states and inputs from the given desired trajectory point using 
		differential flatness property.
	"""
	# first convert all input np.array to np.matrices to simplify 
	# computation.
	# This should be changed to use np.arrays only as np.matrix is not recommended anymore

	# extract all quantities from given trajectory
	pos_traj = trajectory[0] 
	vel_traj = trajectory[1]
	acc_traj = trajectory[2]
	jerk_traj = trajectory[3]
	snap_traj = trajectory[4]
	yaw_traj = trajectory[5]
	yaw_dot_traj = trajectory[6]
	yaw_ddot_traj = trajectory[7]

	# convert all vectors from np.array to np.matrix for compatibility and
	# ease
	pos_traj = np.matrix(pos_traj)
	vel_traj = np.matrix(vel_traj)
	acc_traj = np.matrix(acc_traj)
	jerk_traj = np.matrix(jerk_traj)
	snap_traj = np.matrix(snap_traj)
	yaw_traj = np.matrix(yaw_traj)
	yaw_dot_traj = np.matrix(yaw_dot_traj)
	yaw_ddot_traj = np.matrix(yaw_ddot_traj)

	t_vec = get_t_vector(acc_traj.item(0), acc_traj.item(1), acc_traj.item(2))

	u_1 = get_u1(t_vec)
	z_b = get_zb(t_vec)
	y_c = get_yc(yaw_traj.item(0))
	x_b = get_xb(y_c,z_b)
	y_b = get_yb(z_b,x_b)
	j_ = np.matrix([[jerk_traj.item(0)],[jerk_traj.item(1)],[jerk_traj.item(2)]])
	w_x = get_wx(y_b,j_,u_1)
	w_y = get_wy(x_b,j_,u_1)
	x_c = get_xc(yaw_traj.item(0))
	w_z = get_wz(yaw_dot_traj.item(0),x_c,x_b,w_y,y_c,z_b)
	u_1_dot = get_u1_dot(z_b,j_)
	s_ = np.matrix([[snap_traj.item(0)],[snap_traj.item(1)],[snap_traj.item(2)]])
	w_y_dot = get_wy_dot(x_b,s_,u_1_dot,w_y,u_1,w_x,w_z)
	w_x_dot = get_wx_dot(y_b, s_, u_1_dot, w_x, u_1, w_y, w_z)
	w_z_dot = get_wz_dot(yaw_ddot_traj.item(0),x_c, x_b, yaw_dot_traj.item(0), w_z, y_b, w_y, z_b, w_x, y_c, w_y_dot)
	
	# make angular acceleration vector w_dot
	# remember each element is a 1x1 matrix so have to extract that element...
	w_dot_ = np.matrix([[w_x_dot.item(0)],[w_y_dot.item(0)],[w_z_dot.item(0)]]) 

	# make angular velocity vector w
	w_ = np.matrix([[w_x.item(0)],[w_y.item(0)],[w_z.item(0)]])

	# get vector of torque inputs u2, u3, u4
	u_x = get_ux(w_dot_, w_)

	# get rotation matrix from base frame to world frame
	# for current desired trajectory point.
	# This matrix represents the orientation of the quadrotor
	R_ = np.concatenate((x_b, y_b, z_b), axis = 1)

	# Get rall pitch yaw angles assuming ZYX Euler angle convention
	# This means: first rotate psi degrees around Z axis,
	# then theta degrees around Y axis, and lastly phi degrees around X axis
	or_ = RotToRPY(R_)

	# compute u_a input for system reference
	# can be computed as follows or simply the received acc_traj
	# vector after conversion to matrix. Both are exactly the same quantity
	u_a = get_ua(u_1, z_b)

	# compute u_b input for system reference
	u_b = get_ub(w_, u_x)

	# compute u_c input for system reference
	u_c = get_uc(w_,or_)

	# we need to return back the 1) reference state vector and 2) reference inputs
	# The reference state vector is : (x,y,z, v_x, v_y, v_z, phi, theta, psi, p, q, r)
	# where x,y,z: are position coordinates
	# v_x, v_y, v_z: are velocities
	# phi, theta, psi: are orientation euler angles as described above
	# p, q, r: are angular velocities in body frame X,Y,Z axis respectively

	# we send the received pos_traj, and vel_traj vectors as the reference pos and vel vectors
	# because that is the result from the differential flatness output selection
	return [pos_traj, vel_traj, or_, w_, u_a, u_b, u_c]

def main():
		
	# get arguments to the program
	args = createCommandLineParser()

	fig = plt.figure(figsize=(20,10))
	fig.suptitle(" 3D (Y-Z) Quadrotor Control using Differential Flatness Property")
	ax0 = fig.add_subplot(3,2,1, projection='3d')
	ax1 = fig.add_subplot(3,2,2)
	ax2 = fig.add_subplot(3,2,3)#, projection='3d')
	ax3 = fig.add_subplot(3,2,4)#, projection='3d') 
	ax4 = fig.add_subplot(3,2,5)
	ax5 = fig.add_subplot(3,2,6)

	fig1 = plt.figure(figsize=(20,10))
	fig1.suptitle(" 3D (Y-Z) Quadrotor Control using Differential Flatness Property")
	fig1ax0 = fig1.add_subplot(2,2,1, projection='3d')
	fig1ax1 = fig1.add_subplot(2,2,2)
	fig1ax2 = fig1.add_subplot(2,2,3)#, projection='3d')
	fig1ax3 = fig1.add_subplot(2,2,4)#, projection='3d') 


	# define simulation time
	t_max_ = int(args.t)
	dt = 0.01
	t = np.arange(0.0,t_max_,dt)

	# generate trajectory
	helix_traj = gen_trajectory3(t,t_max_*3/4)#gen_helix_trajectory(t)

	helix_traj2 = gen_helix_trajectory2(t)


	# get t vector
	t_vector = map(lambda a,b,c: get_t_vector(a,b,c), helix_traj[6], helix_traj[7],helix_traj[8]) 

	# get ref u1 input
	u1 = map(lambda t: get_u1(t),t_vector)

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

	# generate Rotation matrices Rbw (from body to world frame)
	R = map(lambda a,b,c: np.concatenate((a,b,c),axis = 1),xb,yb,zb)

	# obtain roll pitch yaw angles
	orientation = map(lambda a: RotToRPY(a),R)

	# get ua input for system reference
	ua = map(lambda a,b,: get_ua(a,b),u1,zb)

	# get ub input for system reference
	ub = map(lambda a,b: get_ub(a,b),w,ux)

	# get uc input for system reference
	uc = map(lambda a,b:  get_uc(a,b),w,orientation)


	ax0.plot(helix_traj[0],helix_traj[1],helix_traj[2], color = "r", label = "Ref Traj")
	ax0.set_title("Reference helix trajectory {3D}")
	ax0.set_xlabel('x {m}')
	ax0.set_ylabel('y {m}')
	ax0.set_zlabel('z {m}')
	ax0.legend(loc='lower right', shadow=True, fontsize='small')

	#ax1.plot(t,u1, color = "r", label = "u1")
	ax1.plot(t,map(lambda v: v.item(0),u1), color = "g", label = "u1")
	#ax1.plot(t,map(lambda v: v.item(1),uc), color = "b", label = "uc y")
	#ax1.plot(t,map(lambda v: v.item(2),uc), color = "c", label = "uc z")
	ax1.set_title("Input U1 evolution")
	ax1.set_xlabel('t {s}')
	ax1.set_ylabel('{N}')
	ax1.legend(loc='lower right', shadow=True, fontsize='small')




	ax2.plot(t,map(lambda v: v.item(0)*180.0/np.pi,orientation), color = "r", label = "phi")
	ax2.plot(t,map(lambda v: v.item(1)*180.0/np.pi,orientation), color = "g", label = "theta")
	ax2.plot(t,map(lambda v: v.item(2)*180.0/np.pi,orientation), color = "b", label = "psi")
	ax2.set_title("orientation evolution")
	ax2.set_xlabel('t {s}')
	ax2.set_ylabel('{rad}')
	ax2.legend(loc='lower right', shadow=True, fontsize='small')

	#ax3.plot(t,map(lambda v: v.item(0),wz_dot), color ="r")#map(lambda v: np.linalg.norm(v),yc))#map(lambda v: v.item(0),wy))
	#ax3.plot(t,map(lambda v: v.item(0),wz), color ="g")

	#ax3.plot(t,u1, color = "r", label = "u1")
	ax3.plot(t,map(lambda v: v.item(0),wx_dot), color = "g", label = "ref wx dot")
	ax3.plot(t,map(lambda v: v.item(0),wy_dot), color = "b", label = "ref wy dot")
	ax3.plot(t,map(lambda v: v.item(0),wz_dot), color = "c", label = "ref wz dot")
	ax3.set_title("angular acceleration evolution")
	ax3.set_xlabel('t {s}')
	ax3.set_ylabel('{rad/s2}')
	ax3.legend(loc='lower right', shadow=True, fontsize='small')

	ax4.plot(t,helix_traj[0], color="r", label = 'ref x')
	ax4.plot(t,helix_traj[1], color="g", label = 'ref y')
	ax4.plot(t,helix_traj[2], color="b", label = 'ref z')
	ax4.plot(t,helix_traj[15], color="c", label = 'ref psi')
	ax4.set_title("Reference position")
	ax4.set_xlabel('t {s}')
	ax4.set_ylabel('{m},{rad}')
	ax4.legend(loc='lower right', shadow=True, fontsize='small')


	ax5.plot(t,map(lambda v: v.item(0),ux), color = "r", label = "ref u1")
	ax5.plot(t,map(lambda v: v.item(1),ux), color = "g", label = "ref u2")
	ax5.plot(t,map(lambda v: v.item(2),ux), color = "b", label = "ref u3")
	ax5.set_title("Input torques u2, u3, u4 evolution")
	ax5.set_xlabel('t {s}')
	ax5.set_ylabel('{N/m}')
	ax5.legend(loc='lower right', shadow=True, fontsize='small')

	if(False):

		# making the following computations just to check computer_ref method.
		# The output of the above code should be the same for the below code,
		# i.e. the two figures generated have graph the exact same curves.
		ref = map(lambda a,b,c,e,f,g,h,i,j,k,l,m,n,o,p,r,s,t: compute_ref([np.array([[a],[b],[c]]),
																			   np.array([[e],[f],[g]]),
																			   np.array([[h],[i],[j]]),
																			   np.array([[k],[l],[m]]),
																			   np.array([[n],[o],[p]]),r,s,t]),
																			   helix_traj[0],helix_traj[1],helix_traj[2],
																			   helix_traj[3],helix_traj[4],helix_traj[5],
																			   helix_traj[6],helix_traj[7],helix_traj[8],
																			   helix_traj[9],helix_traj[10],helix_traj[11],
																			   helix_traj[12],helix_traj[13],helix_traj[14],
																			   helix_traj[15],helix_traj[16],helix_traj[17])

		ref_pos_x = map(lambda a: a[0].item(0),ref)
		ref_pos_y = map(lambda a: a[0].item(1),ref)
		ref_pos_z = map(lambda a: a[0].item(2),ref)

		ref_uc_x = map(lambda a: a[6].item(0),ref)
		ref_uc_y = map(lambda a: a[6].item(1),ref)
		ref_uc_z = map(lambda a: a[6].item(2),ref)

		ref_phi = map(lambda a: a[2].item(0)*180.0/np.pi,ref)
		ref_theta = map(lambda a: a[2].item(1)*180.0/np.pi,ref)
		ref_psi = map(lambda a: a[2].item(2)*180.0/np.pi,ref)

		ref_wx_dot = map(lambda a: a[5].item(0),ref)
		ref_wy_dot = map(lambda a: a[5].item(1),ref)
		ref_wz_dot = map(lambda a: a[5].item(2),ref)

		fig1ax0.plot(ref_pos_x,ref_pos_y, ref_pos_z, color = 'r', label = "Ref Traj")
		fig1ax0.set_title("Reference helix trajectory {3D}")
		fig1ax0.set_xlabel('ref x {m}')
		fig1ax0.set_ylabel('ref y {m}')
		fig1ax0.set_zlabel('ref z {m}')
		fig1ax0.legend(loc='lower right', shadow=True, fontsize='small')

		#ax1.plot(t,u1, color = "r", label = "u1")
		fig1ax1.plot(t,ref_uc_x, color = "g", label = "uc x")
		fig1ax1.plot(t,ref_uc_y, color = "b", label = "uc y")
		fig1ax1.plot(t,ref_uc_z, color = "c", label = "uc z")
		fig1ax1.set_title("orientation rate evolution")
		fig1ax1.set_xlabel('t {s}')
		fig1ax1.set_ylabel('{rad/s}')
		fig1ax1.legend(loc='lower right', shadow=True, fontsize='small')

		fig1ax2.plot(t,ref_phi, color = "r", label = "phi")
		fig1ax2.plot(t,ref_theta, color = "g", label = "theta")
		fig1ax2.plot(t,ref_psi, color = "b", label = "psi")
		fig1ax2.set_title("orientation evolution")
		fig1ax2.set_xlabel('t {s}')
		fig1ax2.set_ylabel('{rad}')
		fig1ax2.legend(loc='lower right', shadow=True, fontsize='small')

		fig1ax3.plot(t,ref_wx_dot, color = "g", label = "ref wx dot")
		fig1ax3.plot(t,ref_wy_dot, color = "b", label = "ref wy dot")
		fig1ax3.plot(t,ref_wz_dot, color = "c", label = "ref wz dot")
		fig1ax3.set_title("angular acceleration evolution")
		fig1ax3.set_xlabel('t {s}')
		fig1ax3.set_ylabel('{rad/s2}')
		fig1ax3.legend(loc='lower right', shadow=True, fontsize='small')

	plt.show()


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

invI = np.linalg.inv(I)

if __name__ == '__main__':
	main()

