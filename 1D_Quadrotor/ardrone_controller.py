# Simulation and control of an ardrone quadrotor 

import control
import slycot
import numpy as np
import matplotlib.pyplot as plt




#define rotations from world_frame to body_frame
def roll_rotation(phi):
	"""
	Rotation by 'phi' radians around X-axis of the body frame

	Important: this rotation is from  the World frame -> Body Frame
	"""
	rotation = np.array([ [1.0,0.0,0.0],[0.0,np.cos(phi),np.sin(phi)],[0.0,-1.0*np.sin(phi),np.cos(phi)] ])
	return rotation

def pitch_rotation(theta):
	"""
	Rotation by 'theta' radians around Y-axis of the body frame

	Important: this rotation is from  the World frame -> Body Frame
	"""
	rotation = np.array([ [np.cos(theta),0.0,-1.0*np.sin(theta)],[0.0,1.0,0.0],[np.sin(theta),0.0,np.cos(theta)] ])
	return rotation

def yaw_rotation(psi):
	"""
	Rotation by 'psi' radians around Z-axis of the body frame

	Important: this rotation is from  the World frame -> Body Frame
	"""
	rotation = np.array([ [np.cos(psi),np.sin(psi),0.0],[-1.0*np.sin(psi),np.cos(psi),0.0],[0.0, 0.0, 1] ])
	return rotation

def rotation_matrix(phi,theta,psi):
	"""
		Calculate the rotation matrix for rotation of phi, theta, psi radians around the X,Y,Z axis (roll,pitch,yaw)
		respectively and in Z,Y,X order (i.e. first rotation around Z, then Y then X axis)

		Important: this rotation is from  the World frame -> Body Frame
	"""
	# calculate individual rotations
	roll_rot =  roll_rotation(phi)
	pitch_rot = pitch_rotation(theta)
	yaw_rot =  yaw_rotation(psi)

	# calculate full rotation
	rm = np.dot(roll_rot,np.dot(pitch_rot,yaw_rot))
	return rm

def get_rotor_thrust(omega, frame):
	"""
	Returns the trust of a rotor with rotational velocity = omage,  over a frame 'frame'.
	The thrust always points in the z direction of a frame {x,y,z}
	"""
	# compute thrust magnitude and insert into vector
	#thrust = c_t * air_density * rotor_area * rotor_radius**2 *omega
	thrust = c_t_lumped * omega
	thrust_vector = np.array([ [0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,thrust] ])
	thrust_vector = np.dot(np.transpose(thrust_vector),frame)
	return thrust_vector
	

#inertial (world) frame definition by axis
xw = np.array([ [1.0],[0.0],[0.0] ])
yw = np.array([ [0.0],[1.0],[0.0] ])
zw = np.array([ [0.0],[0.0],[1.0] ])
world_frame = np.array([xw,yw,zw])
#print("World frame: {}".format(world_frame))

# Environment parameters
g_w = -9.81*zw    #gravity [m/s2], in -z direction
wind = 1*xw    # wind speed  m/s


# -------------------------------------------
#      QUAD ROTOR PARAMETERS
# -------------------------------------------

# BODY
mass = 1.5 #kg
body_width = 0.3 #m
body_height = 0.16 #m
arm_length =0.09 #m
rotor_offset_top = 0.023 #m
radius_rotor = 0.1 #m
arm0_angle = np.pi/4  #rad
arm1_angle = arm0_angle+(np.pi/2) #rad
arm2_angle = arm0_angle+(2*np.pi/2) #rad
arm3_angle = arm0_angle+(3*np.pi/2) #rad
body_inertia = np.array([ [0.0347563, 0, 0],[0, 0.0458929, 0],[0, 0, 0.0977] ]) # kg*m2



#rotor parameters
mass_rotor = 0.005 #kg
motor_constant = 8.54858e-06 #kg*m/s2
moment_constant = 0.016 #m
time_constant_up = 0.0125#s
time_constant_down = 0.025#s
max_rot_velocity = 838 #rad/s
rotor_inertia = list()
c_t_lumped = 1.0 #rotor thrust coefficient





# calculate the body frame based on rotation angles
roll = 0.0
pitch = np.pi/2
yaw = 0.0
rm = rotation_matrix(roll, pitch, yaw)
body_frame = np.dot(rm, world_frame)
#print("Body Frame: {}".format(body_frame))

# -------------------------------------------
#      Forces over the quadrotor
# -------------------------------------------

#rotational velocities of rotors
omega0 = 20.0
omega1 = 20.0
omega2 = 20.0
omega3 = 20.0

# thrusts generated by each rotor
thrust0 = get_rotor_thrust(omega0, body_frame)
thrust1 = get_rotor_thrust(omega1, body_frame)
thrust2 = get_rotor_thrust(omega2, body_frame)
thrust3 = get_rotor_thrust(omega3, body_frame)
#print("Force vectors: T1{},T2{},T3{},T4{}".format(thrust0, thrust1, thrust2, thrust3))

#convert gravity to body frame
g_b = np.dot(rm,g_w)
print("Gravity in body frame: {}".format(g_b))


