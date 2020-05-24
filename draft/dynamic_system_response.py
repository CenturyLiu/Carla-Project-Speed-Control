# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import control
import carla
from scipy import signal
import matplotlib.pyplot as plt
import numpy as np
from collections import deque


num = [1.0]
den = [1., 1.,0.5]
sys1 = control.tf(num,den,0.002)

t = np.linspace(0, 1, 500, endpoint=False)
#sig = np.sin(2 * np.pi * t)
pwm = signal.square(2 * np.pi * 5 * t)

num_low_pass = [1.0]
den_low_pass = [1.0 , 10.0]
low_pass_c = control.tf(num_low_pass,den_low_pass)
low_pass_d = control.sample_system(low_pass_c, 0.002) 

sys = low_pass_d

sys = control.tf2ss(sys)

T, yout, xout = control.forced_response(sys,U=pwm,X0=0)

init_value = deque(maxlen = 10)
init_value.append(0)
U0 = deque(maxlen = 10)
U0.append(pwm[0])
y = []
for ii in range(1,500):
    U0.append(pwm[ii])
    _,y0,x0 = control.forced_response(sys,U=U0,X0 = init_value[0])
    init_value.append(x0[-1])
    y.append(y0[-1])
    
    

plt.subplot(3,1,1)
plt.plot(t,pwm)
plt.subplot(3,1,2)
plt.plot(T,yout)
plt.subplot(3,1,3)
plt.plot(y)


