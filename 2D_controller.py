# control in the plane y - z

import numpy as np
import control as ctl
import matplotlib.pyplot as plt

step = plt.figure(figsize=(20,10))
ax0 = step.add_subplot(2,2,1)
ax1 = step.add_subplot(2,2,2)
ax2 = step.add_subplot(2,2,3)
ax3 = step.add_subplot(2,2,4)



g = 9.8 #m/s2
m = 1 #kg
l = 0.2 #m
Ixx = (m*(l**2)) /2#
A = np.array([ [0,0,0,0,0,-1.0*g],[1,0,0,0,0,0],[0,0,0,0,0,0],[0,0,1,0,0,0],[0,0,0,0,0,0],[0,0,0,0,1,0] ])
B = np.array([ [0,0],[0,0],[1.0/m,0],[0,0],[0,l/Ixx],[0,0]])
C = np.array([0.0,1.0,0.0,0.0,0.0,0.0 ])
D = np.array([0.0,0.0 ])

#open loop system
ol_ss = ctl.ss(A,B,C,D) 

#draw root locus and step input
#locus1, k1 = ctl.root_locus(ol_ss, PrintGain = True)
t0,y0=ctl.step_response(ol_ss)
ax0.plot(t0,y0)
ax0.set_title("Response of open loop system")

plt.show()