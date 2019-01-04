# This script simulates the dynamics of a quadrotor.
# The objective is
# 	1. Be able to call "input" functions and the "state" of the quadrotor
#	should change according to this inputs and the environment
#	2. Develop a dynamic model of the quadrotor that is as close as possible to reality
# 	3. 
#
#
#   Inputs are 
#		a.  The thrust of the rotors
#		b.  The moments generated by the rotors around the center of mass CoM of the quadrotor
#
#	The states are
#		a. The position of the quadrotor and its first derivative (linear velocity)
#		b. The angular position of the quadrotor and its first derivate (angular velocity)
#	
#	Since position and angular position are 3x1 vectors, the total number of degress of freedom
#	is 6: the quadrotor is a 6DoF system
#
#



import numpy as np
import matplotlib.pyplot as plt
import argparse
from mpl_toolkits.mplot3d import Axes3D

# compute acceleration of the quadrotor

class quadrotor:
	
def get_instant_Accel():
	a = m

