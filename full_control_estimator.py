# Altitude controller with full state feedback

import numpy as np
import control as ctl
import matplotlib.pyplot as plt
from scipy import signal

fig = plt.figure(figsize=(20,10))
fig.suptitle(" Quadrotor Height LQR Controller and Estimator")
ax0 = fig.add_subplot(2,2,1)
ax1 = fig.add_subplot(2,2,2)
ax2 = fig.add_subplot(2,2,3)
ax3 = fig.add_subplot(2,2,4) 

# define constants
g = 9.81 #m/s2 gravity
m = 1.0  #kg
c = 0.2 #air friction constant

#define equilibrium point
xe = [0,10]
u = [m*g]

# define time
t_max = 5.0  #s
t = np.arange(0.0,t_max,0.1)

air = 1.0  # set to 1 to activate air model, otherwise set to 0


# define system matrices
A = np.matrix([[-air*c/m,0.0],[1.0,0.0]])
B = np.matrix([[1.0],[0.0]])
Ch = np.matrix([0.0,1.0])  # define matrix for position output
Cv = np.matrix([1.0,0.0])
D = 0

# define step input gain:  u(t) = R*u(t)
R = 10.0
ref = R*np.ones_like(t)

h_ticks = np.arange(0,14,1)
v_ticks = np.arange(0,12,1)


#*****************************************#
#            OPEN LOOP SYSTEM RESPONSE
#*****************************************#
ol_ss_p = ctl.ss(A,B,Ch,D)
ol_ss_v = ctl.ss(A,B,Cv,D)
th, h=ctl.step_response(ol_ss_p, T = t)
tv, v=ctl.step_response(ol_ss_v, T = t)

#real, imag, freq = ctl.nyquist_plot(ol_sys_s, labelFreq = 10, omega = 2*np.pi*freq)
ax0.plot(th, h,linestyle = '-',color ='r', label = "Height")
ax0t = ax0.twinx()
ax0t.plot(tv, v, linestyle = '--', color = 'g', label = "Velocity")
ax0.plot(t, ref, color = "k", label = 'desired')
ax0.set_title("Step response of close loop system", fontsize='small')
ax0.legend(loc='lower right', shadow=True, fontsize='small')
ax0.set_xlabel("time {s}")
ax0.set_ylabel("Height {m}")
ax0.set_yticks(h_ticks)
ax0t.set_ylabel("Velocity {m/s}")
ax0t.set_yticks(v_ticks)

#*****************************************#
#          FULL STATE FEEDBACK CONTROLLER 
#*****************************************#

#desired controller poles  p1= -3, p2, -4
dcp = np.array([-3.0,-3.0])
K = ctl.acker(A,B,dcp)
print("Controller gains: {},{}".format(K.item(0,0),K.item(0,1))) 

# The system with full state feedback is modified as 
#      x'(t) = (A-BK)x + B(Nu + KNx)*r
#      

Nx = np.matrix([[0.0],[1.0]])
Nu = 0.0

#**************** ADDED LQR Controller
# Define performance index weights for Vertical Dynamics
Qx1 = np.diag([10, 100]);
Qu1a = np.diag([1]);
(K, X, E) = ctl.lqr(A, B, Qx1, Qu1a)
K = np.matrix(K)
print("LQR gains for Vertical Dynamics")
print(K)

xd = np.matrix([[0.0], [R]]); 
cl_ss_p = ctl.ss(A-B*K, B*(Nu + K*Nx)*R, Ch, D)
cl_ss_v = ctl.ss(A-B*K, B*(Nu + K*Nx)*R, Cv, D)
th, h=ctl.step_response(cl_ss_p, T = t)
tv, v=ctl.step_response(cl_ss_v, T = t)

#real, imag, freq = ctl.nyquist_plot(ol_sys_s, labelFreq = 10, omega = 2*np.pi*freq)
ax1.plot(th, h,linestyle = '-',color ='r', label = "Height")
ax1t = ax1.twinx()
ax1t.plot(tv, v, linestyle = '--', color = 'g', label = "Velocity")
ax1.plot(t, ref, color = "k", label = 'desired')
ax1.set_title("Closed loop with LQR controller gains K1={:.2f}, K2={:.2f}".format(K.item(0,0),K.item(0,1)), fontsize='small')
ax1.legend(loc='lower right', shadow=True, fontsize='small')
ax1.set_xlabel("time {s}")
ax1.set_ylabel("Height {m}")
ax1.set_yticks(h_ticks)
ax1t.set_ylabel("Velocity {m/s}")
ax1t.set_yticks(v_ticks)


#*****************************************#
#          FULL STATE ESTIMATOR
#*****************************************#

#desired estimator poles p1 = -15, p2 = -20
dep = np.array([-1.0, -1.0])
# remember poles of det(sI - (A-LC)) = det(sI - (A-LC).T) = det(sI - (A.T - C.T*L.T))
A_T= np.transpose(A)
C_T = np.transpose(Ch)
L = np.transpose(ctl.acker(A_T,C_T,dep))
print("Estimator gains: {},{}".format(L.item(0,0),L.item(1,0))) 



#-------------- ADDED LQR Estimator
# Define performance index weights For Lateral Dynamics
Qx1 = np.diag([100, 100]);
Qu1a = np.diag([1]);
(L, X, E) = ctl.lqr(A_T, C_T , Qx1, Qu1a)
L = np.matrix(L.T)    # Remember to transpose the resulting gains
print("Estimator gains: {},{}".format(L.item(0,0),L.item(1,0))) 



# define reference input matrix
Nbar = Nu + K*Nx

# Define estimator system matrices. This matrices are supposedly not equal
# to the real system matrices since it belong to a "model" of the system
# that will contain some error. 
#
# Here the model inaccuracy is model as a small error in each matrix. 
# The estimator model with reference input is
#     
#   `x' = (An -BnK -LCn)`x' + Ly + Bn*Nbar*r    
#    u  = -Kx + Nbar*r
#
#   where `x : state estimation
#          y : measured output from real system = Cx
#          r : reference/command
#
# can be simplified to
#   `x' = (An - BnK - LCn -LC)`x' + B*Nbar*r 
#    y  = ()

# seed the generator for repeatable results
np.random.seed(1)
# add 1% noise
noise_percent = 0.01
An = noise_percent*np.random.rand(2,2)+A
Bn = noise_percent*np.random.rand(2,1)+B
Chn = noise_percent*np.random.rand(1,2)+Ch
Cvn = noise_percent*np.random.rand(1,2)+Cv
Dn = noise_percent*np.random.randn()+D

cl_ess_p = ctl.ss(An-Bn*K - L*Chn + L*Ch, B*Nbar*R, Ch, D)
cl_ess_v = ctl.ss(An-Bn*K - L*Cvn + L*Cv, B*Nbar*R, Cv, D)

th, h=ctl.step_response(cl_ess_p, T = t)
tv, v=ctl.step_response(cl_ess_v, T = t)

#real, imag, freq = ctl.nyquist_plot(ol_sys_s, labelFreq = 10, omega = 2*np.pi*freq)
ax2.plot(th, h,linestyle = '-',color ='r', label = "Height")
ax2t = ax2.twinx()
ax2t.plot(tv, v, linestyle = '--', color = 'g', label = "Velocity")
ax2.plot(t, ref, color = "k", label = 'desired')
ax2.set_title("Closed loop with LQR controller gains K1={:.1f}, K2={:.2f}, and estimator gains L1={:.1f}, L2={:.2f}".format(K.item(0,0),K.item(0,1),L.item(0,0), L.item(1,0)), fontsize='small')
ax2.legend(loc='lower right', shadow=True, fontsize='small')
ax2.set_xlabel("time {s}")
ax2.set_ylabel("Height {m}")
ax2.set_yticks(h_ticks)
ax2t.set_ylabel("Velocity {m/s}")
ax2t.set_yticks(v_ticks)



#*****************************************#
#          DEFINE A PERSONALIZED INPUT SIGNAL
#*****************************************#


# Redefine the System just to remove the input Reference gain R
# so that that it follows input signal and not (input signal)*R
cl_ess_p = ctl.ss(An-Bn*K - L*Chn + L*Ch, B*Nbar, Ch, D)


signal = np.ones_like(t)
random_times = list()
slots = np.random.randint(0,5)
for i in range(slots):
	random_times.append(int((len(signal)-1)*np.random.random_sample()))

random_times.append(len(signal)-1)
random_times = np.unique(np.sort(random_times)).flatten()

print(random_times)
start = 0
for i in random_times:
	signal[start:i] = signal[start:i]*(R*np.random.random_sample())
	start = i

t, h, x = ctl.forced_response(cl_ess_p, T=t, U=signal)

ax3.plot(t, h,linestyle = '-',color ='c', label = "Height")
ax3.plot(t, signal, linestyle = '--',color = "k", label = 'desired')
ax3.set_title("Reference Tracking",fontsize='small')
ax3.legend(loc='upper right', shadow=True, fontsize='small')
ax3.set_xlabel("time {s}")
ax3.set_ylabel("Height {m}")
ax3.set_yticks(h_ticks)

"""
# using estimator and controller in the feedback path
# In this case the regulator is
#  `x' = (A -BK -LC)`x + Ly    where the input is y = Cx, from the real system
#   
cl_ess_p2 = ctl.ss(A-B*K - L*Ch, L*R, Ch, D)
cl_ess_v2 = ctl.ss(A-B*K - L*Cv, L*R, Cv, D)

# make feedback interconnection
cl_sys_p = ctl.feedback(ol_ss_p, cl_ess_p2, sign = +1)
cl_sys_v = ctl.feedback(ol_ss_v, cl_ess_v2, sign = +1)

# input an step signal
th, h=ctl.step_response(cl_sys_p, T = t)
tv, v=ctl.step_response(cl_sys_v, T = t)

#real, imag, freq = ctl.nyquist_plot(ol_sys_s, labelFreq = 10, omega = 2*np.pi*freq)
ax3.plot(th, h,linestyle = '-',color ='r', label = "Height")
ax3t = ax3.twinx()
ax3.plot(tv, v, linestyle = '--', color = 'g', label = "Velocity")
ax3.plot(t, ref, color = "k", label = 'desired')
ax3.set_title("Estimator Feedback with K1={:.1f}, K2={:.2f}, and estimator gains L1={:.1f}, L2={:.2f}".format(K.item(0,0),K.item(0,1),L.item(0,0), L.item(1,0)),fontsize='small')
ax3.legend(loc='center right', shadow=True, fontsize='small')
ax3.set_xlabel("time {s}")
ax3.set_ylabel("Height {m}")
ax3.set_yticks(h_ticks)
ax3t.set_ylabel("Velocity {m/s}")
ax3t.set_yticks(v_ticks)
"""
plt.show()