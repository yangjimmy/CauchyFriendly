#!/usr/bin/env python
# coding: utf-8

# Pendulum dynamics equation
# \begin{align}
# \dfrac{dx_1}{dt} &= x_2 \\
# \dfrac{dx_2}{dt} &= -\frac{m g l_c}{J} sin(x_1) - (\frac{B}{J} + \frac{K_m^2}{JR}) x_2
# \end{align}


import numpy as np 
import cauchy_estimator as ce 
import gaussian_filters as gf
import matplotlib.pyplot as plt
from pendulum_helper import *
import time

mp = PendulumParams()

# Initial condition
x0 = np.array([[np.pi/2], [0]])
P0 = np.eye(2) * mp.v_PSD**.5
mp.set_initial(x0, P0)

propagations = 5000

# Dynamic simulation without noise
thetas, _, _, _ = mp.dynamic_simulation(propagations, noise=False)
Ts = np.arange(propagations+1) * mp.dt
plt.figure()
plt.suptitle("Pendulum Trajectory (angle: top), (anglular rate: bottom)")
plt.subplot(211)
plt.plot(Ts, thetas[:, 0])
plt.subplot(212)
plt.plot(Ts, thetas[:, 1])
plt.show()

# Dynamic simulation with noise
xs, zs, ws, vs = mp.dynamic_simulation(propagations, noise=True)
ce.plot_simulation_history(None, (xs,zs,ws,vs), None)

for k in range(propagations):
    x_kf, P_kf = mp.Kalman_step(zs[k+1])
xs_kf = np.array(mp.xs_kf)
Ps_kf = np.array(mp.Ps_kf)
# Plot Simulation results
ce.plot_simulation_history( None, (xs,zs,ws,vs), (xs_kf, Ps_kf) )

num_windows = 4
start_time = time.time()
mp.cauchy_start(num_windows)
for zk in zs:
    mp.cauchy_step(zk)
mp.cauchy_shutdown()

print(time.time() - start_time)

ce.plot_simulation_history( mp.cauchyEst.moment_info, (xs,zs,ws,vs), (xs_kf, Ps_kf) )
