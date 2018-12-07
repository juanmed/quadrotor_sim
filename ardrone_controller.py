# design a controller for an ardrone 

import control
import slycot
import numpy as np
import matplotlib.pyplot as plt


#intertial frame definition by axis
xw = np.array([ [1],[0],[0] ])
yw = np.array([ [0],[1],[0] ])
zw = np.array([ [0],[0],[1] ])



#define rotations


def roll_rotation(phi):
	"""
	Rotation by 'phi' radians around X-axis of the body frame
	"""
	rotation = np.array([ [1.0,0.0,0.0],[0.0,np.cos(phi),np.sin(phi)],[0.0,-1.0*np.sin(phi),np.cos(phi)] ])
	return rotation

def pitch_rotation(theta):
	"""
	Rotation by 'theta' radians around Y-axis of the body frame
	"""
	rotation = np.array([ [np.cos(theta),0.0,-1.0*np.sin(theta)],[0.0,1.0,0.0],[np.sin(theta),0.0,np.cos(theta)] ])
	return rotation

def yaw_rotation(psi):
	"""
	Rotation by 'psi' radians around Z-axis of the body frame
	"""
	rotation = np.array([ [np.cos(psi),np.sin(psi),0.0],[-1.0*np.sin(psi),np.cos(psi),0.0],[0.0, 0.0, 1] ])
	return rotation


mass = 1.5 #kg
body_width = 0.3 #m
body_height = 0.16 #m
mass_rotor = 0.005 #kg
arm_length =0.09 #m
rotor_offset_top = 0.023 #m
radius_rotor = 0.1 #m
motor_constant = 8.54858e-06 #kg*m/s2
moment_constant = 0.016 #m
time_constant_up = 0.0125#s
time_constant_down = 0.025#s
max_rot_velocity = 838 #rad/s

body_inertia = np.array([ [0.0347563, 0, 0],[0, 0.0458929, 0],[0, 0, 0.0977] ])# kg*m2

rotor_inertia = 