# Height model for quadrotor

import numpy as np
import control as ctl
import matplotlib.pyplot as plt

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

# -------------------------------------
#       HAND: RESPONSE OF OPEN LOOP SYSTEM
# -------------------------------------

# define time
t = np.arange(0.0,20.0,0.1)

# define control parameters
g = 9.8 #m/s2
m = 1.0 #kg
F2 = 0.0 # N
F1 = 0.0 # N
u = (F1 + F2 - m*g)/m # N

# initial conditions
p0 = 10.0 #m
v0 = 10.0 # m/s
p = (u)*(t**2)  + v0*t + p0 
# plot open loop system response
ax0.plot(t,p,color = "r", label = "Open Loop Position")
ax0.legend(loc='upper right', shadow=True, fontsize='small')

# -------------------------------------
#       SW: RESPONSE OF OPEN LOOP SYSTEM
# -------------------------------------

num = np.array([1])
den = np.array([1,0,0])
ol_sys_s = ctl.tf(num,den) # open loop s-domain system

freq = np.arange(0,1000,10)
#mag, phase, omega = ctl.bode(ol_sys_s, dB = True, Plot = True , legend = "Open Loop system Bode")
#locus1, k1 = ctl.root_locus(ol_sys_s, PrintGain = True)
t0,y0=ctl.step_response(ol_sys_s, T = t)
#real, imag, freq = ctl.nyquist_plot(ol_sys_s, labelFreq = 10, omega = 2*np.pi*freq)
sw0.plot(t0,y0)
sw0.set_title("Step response of open loop system")


# -------------------------------------
#       HAND: RESPONSE OF CLOSED LOOP SYSTEM WITH P controller
# -------------------------------------

# after introducing a proportional controler K
K = 2.0  # controller gain 
R = 4.0  # reference value
F1 = (m*R*np.cos(np.sqrt(K)*t)+m*g)/2.0
F2 = (m*R*np.cos(np.sqrt(K)*t)+m*g)/2.0
u = (F1 + F2 - m*g)/m # N
# plot open loop system response
p = -R*np.cos(np.sqrt(K)*t)/K + R#+ p0 + v0*t 
ax1.plot(t,p,color = "r", label = "Closed Loop with P (K = {}) controller".format(K))
ax1.legend(loc='upper right', shadow=True, fontsize='small')
	

# -------------------------------------
#       SW: RESPONSE OF CLOSED LOOP SYSTEM WITH P controller
# -------------------------------------

num = np.array([K])
den = np.array([1])
G = ctl.tf(num,den) # open loop s-domain system
# a simple system G(s) = 1 
g1_sys = ctl.tf([1],[1])
new = ctl.series(G,ol_sys_s)
new = ctl.feedback(new, g1_sys, sign = -1)
# define a gain with the value of the reference
gr_sys = ctl.tf([R],[1])
# and put it in the into path (so that step response is 1*g1_sys)
p_control_ss = ctl.series(gr_sys,new)
print("Transfer function of system with K controller\n{}".format(p_control_ss))
# draw step response
t1,y1=ctl.step_response(p_control_ss, T = t)
sw1.plot(t1,y1)
sw1.set_title("Step response of closed loop system")

#real, imag, freq = ctl.nyquist_plot(p_control_ss, labelFreq = 10, omega = 2*np.pi*freq)


# -------------------------------------
#       HAND: RESPONSE OF CLOSED LOOP SYSTEM WITH PI controller
# -------------------------------------

Kp = 9.0
Kd = 2*np.sqrt(Kp) + 10

a = -Kd + np.sqrt(Kd**2 - (4.0*Kp))
b = -Kd - np.sqrt(Kd**2 - (4.0*Kp))
p = (Kp + Kd*a)/(a-b)
q = (Kp + Kd*b)/(b-a)

p = R*(p/a)*(1.0 - np.exp(1.0*a*t)) + R*(q/b)*(1.0-np.exp(1.0*b*t))
# plot open loop system response
ax2.plot(t,p,color = "r", label = "Closed loop With Kp={:.2f}, Kd={:.2f}".format(Kp,Kd))
ax2.legend(loc='upper right', shadow=True, fontsize='small')


# -------------------------------------
#       SW: RESPONSE OF CLOSED LOOP SYSTEM WITH PI controller
# -------------------------------------
num = np.array([Kd, Kp])
den = np.array([1])
G = ctl.tf(num,den) # open loop s-domain controller
# a simple system G(s) = 1 
g1_sys = ctl.tf([1],[1])
new = ctl.series(G,ol_sys_s)
new = ctl.feedback(new, g1_sys, sign = -1)
# define a gain with the value of the reference
gr_sys = ctl.tf([R],[1])
# and put it in the into path (so that step response is 1*g1_sys)
pd_control_ss = ctl.series(gr_sys,new)
print("Transfer function of system with Kp = {0}, Kd = {1} controller\n{2}".format(Kp, Kd, pd_control_ss))

# draw step response
t2,y2=ctl.step_response(pd_control_ss, T = t)
sw2.plot(t2,y2)
sw2.set_title("Closed loop With Kp={:.2f}, Kd={:.2f}".format(Kp,Kd))

plt.show()
