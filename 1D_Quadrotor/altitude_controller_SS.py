# Height model for quadrotor using state space



import numpy as np
import control as ctl
import matplotlib.pyplot as plt

# define time
t = np.arange(0.0,20.0,0.1)

hand = plt.figure(figsize=(20,10))
hand.suptitle(" Hand Calculations ")
ax0 = hand.add_subplot(2,2,1)
ax1 = hand.add_subplot(2,2,2)
ax2 = hand.add_subplot(2,2,3)
ax3 = hand.add_subplot(2,2,4) 

sw = plt.figure(figsize=(20,10))
sw.suptitle(" Software Calculations ")
sw0 = sw.add_subplot(2,2,1)
sw1 = sw.add_subplot(2,2,2)
sw2 = sw.add_subplot(2,2,3)
sw3 = sw.add_subplot(2,2,4) 

A = np.array([ [0.0,0.0], [1.0,0.0] ])
B = np.array([[1.0],[0.0]])
C = np.array([0.0,1.0])
D = 0

#define system
ol_sys_ss = ctl.ss(A,B,C,D)
t0,y0=ctl.step_response(ol_sys_ss, T = t)
#real, imag, freq = ctl.nyquist_plot(ol_sys_s, labelFreq = 10, omega = 2*np.pi*freq)
sw0.plot(t0,y0)
sw0.set_title("Step response of open loop system")


#desired poles
dp = np.array([-3,-4])
K = ctl.acker(A,B,dp)


#Reference input matrices
Nu = 0
Nx = np.array([[0],[1]])

# reference input value for step reference
R = 3

A1 = A -np.dot(B,K)
B1 = np.dot(B,(Nu + np.dot(K,Nx)))*R
#print(A1,B1)
cl_sys_ss = ctl.ss(A1,B1,C,D)

t1,y1=ctl.step_response(cl_sys_ss, T = t)
#real, imag, freq = ctl.nyquist_plot(ol_sys_s, labelFreq = 10, omega = 2*np.pi*freq)
sw1.plot(t1,y1)
sw1.set_title("Step response of close loop system")


# -------------------------------------
#       HAND: RESPONSE OF OPEN LOOP SYSTEM
# -------------------------------------
print(K.item(0,1))
p = (-12.0*R/4.0)*(1-np.exp(-4.0*t)) + (12.0*R/3.0)*(1-np.exp(-3.0*t))
# plot open loop system response
ax0.plot(t,p,color = "r", label = "Closed loop With K1={:.2f}, K2={:.2f}".format(K.item(0,0),K.item(0,1)))
ax0.legend(loc='upper right', shadow=True, fontsize='small')
plt.show()