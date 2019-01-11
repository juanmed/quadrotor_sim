# Digital Control of Height

import control as ctl
import numpy as np
import matplotlib.pyplot as plt



# system transfer function  G(s)= 1 / s*s
num = np.array([1])
den = np.array([1,0,0])
G = ctl.tf(num,den) # open loop s-domain system

# System bandwidth
wb = np.sqrt(2)
# Sampling frequency = 20~25 * wb
ws = 25.0*wb
# ws = 2*pi/Ts
Ts = 2.0*np.pi/ws
print("Sampling period: {}".format(Ts))

# get digitization using Tustin method
d_Tustin = ctl.sample_system(G, Ts, method='tustin')
print("Tustin digization: {}".format(d_Tustin))

#get digitized PI controller with Kp = 9, Kd = 16 using ZOH 
Kp = 9.0
Kd = 16.0
num = np.array([Kd,Kp])
den = np.array([1])
Dc = ctl.tf(num,den)

dc_zoh = ctl.sample_system(G, Ts, method = 'zoh')
print("PI ZOH Digitization {}".format(dc_zoh))

dc_mpz = ctl.sample_system(G, Ts, method = 'matched')
print("PI MPZ Digitization {}".format(dc_mpz))


