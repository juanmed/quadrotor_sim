# control of the altitude

import numpy as np
import control as ctl
import matplotlib.pyplot as plt

m = 1.0 #kg
g = -9.8 #m/s2
F = 5.0 # N
A = np.array([ [0.0,0.0],[1.0,0.0]] ) # system matrix
B = np.array([[F/m+g],[0.0]])  #  input matrix
C = np.array([1.0,0.0]) # output is position
D = 0.0


step = plt.figure(figsize=(20,10))
ax0 = step.add_subplot(2,2,1)
ax1 = step.add_subplot(2,2,2)
ax2 = step.add_subplot(2,2,3)
ax3 = step.add_subplot(2,2,4)


#open loop system
ol_ss = ctl.ss(A,B,C,D) 

#draw root locus and step input
#locus1, k1 = ctl.root_locus(ol_ss, PrintGain = True)
t0,y0=ctl.step_response(ol_ss)
ax0.plot(t0,y0)
ax0.set_title("Response of open loop system")


poles_des = np.array([-1+5j,-1-5j])
K = ctl.acker(A,B,poles_des)

# calculate new A matrix
A1 = A - np.dot(B,K)
B1 = np.array([ [1.0],[0.0]])

cl_ss = ctl.ss(A1,B1,C,D)
#locus1, k1 = ctl.root_locus(cl_ss, PrintGain = True)
#calculate new step response
t1,y1=ctl.step_response(cl_ss)
ax1.plot(t1,y1)
ax1.set_title("Step response with gains: {}".format(K))

# introduce reference
# add the reference input
Nx = np.array([ [0.5],[0.0]])
Nu = np.array([-1.0])

B2 = np.dot(B,Nu + np.dot(K,Nx))

ref_ss = ctl.ss(A1,B2,C,D)
#calculate new step response
t2,y2=ctl.step_response(ref_ss)
ax2.plot(t2,y2)
ax2.set_title("Step Response with reference input")


plt.show()