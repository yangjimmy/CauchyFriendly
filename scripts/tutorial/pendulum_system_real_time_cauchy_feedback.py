#!/usr/bin/env python
# coding: utf-8

# ----- Import libraries -----
import numpy as np 
import cauchy_estimator as ce 
import gaussian_filters as gf
import matplotlib.pyplot as plt
from pendulum_device import C2000_Communication as C2000
import time
from pendulum_helper import *

# ----- Define system parameters -----

mp = PendulumParams(dt=0.001,num_controls=1)
# Initial condition
x0 = np.array([[np.pi/2], [0]])
P0 = np.eye(2) * mp.v_PSD**.5
mp.set_initial(x0, P0)

# Determine number of propagations
# exp_time = 3.5 # s
exp_time = 0.5
propagations = int(exp_time / mp.dt) + 1

# Start Cauchy filter
num_windows = 4
mp.cauchy_start(num_windows)

# Initialize serial communication
motor_device = C2000('/dev/ttyUSB0')
motor_device.idle()

# reset pendulum
input("Is the pendulum resting at the zero position? ")
motor_device.reset()

# Setup experiment
input("Is the pendulum set in place? ")
print("Drop the pendulumn!")

estimator_start = False
control_enable = False
shutdown = False
z_prev = motor_device.get_states()[0][0]

step = 0
dt = mp.dt
timer = dt
start_time = time.time()
u = 0

# Log
state_log = []
measurement_log = []
control_log = []

# K = -np.array([0.0289, 0.0048])
# K = -np.array([0.31, 0])
K = -np.array([0.0077, 0.0154])


while True:
    if time.time() - start_time >= timer - dt*.01:
        # timer
        timer = timer + dt
        while True:
            x_meas, status, tag_v = motor_device.get_states()
            if status:
                break
            else:
                time.sleep(0.0001)
        z = x_meas[0][0]
        # trigger protection
        # if z > 1.7:
        #     motor_device.idle()
        #     shutdown = True
        #     print(f"Early exit ..., angle at {z}")
        #     break
        if abs(z - z_prev) > 0.01 and not estimator_start:
            estimator_start = True
            actual_start_time = time.time()
            print(f"Estimator actually starts at {actual_start_time - start_time} lol")
        if estimator_start:
            step = step + 1
            # Run Kalman filter step
            mp.Kalman_step(z, u)

            # Run Cauchy filter step
            xhat, Phat, wavg_xhat, wavg_Phat = mp.cauchy_step(z, u)
            
            # Enabling the controller
            if not control_enable and time.time() - actual_start_time > .2:
                control_enable = True
            if control_enable:
                u = K @ xhat
                u = PendulumParams.saturation(u,.3)        # Control command saturation protectoin
                # u = 0
                motor_device.run(u,step)

            # Store returned states, measurement
            state_log.append(x_meas.copy())
            measurement_log.append(z.copy())
            control_log.append(u)
            print(step - tag_v)

        if step >= propagations:
            break


# End serial connection and Cauchy filter
motor_device.disconnect()
mp.cauchy_shutdown()
print(f"Estimators run for {time.time() - actual_start_time} s")

# Data processing
state_log = np.array(state_log)
measurement_log = np.array(measurement_log)
control_log = np.array(control_log)                 # control history
cauchy_moment_info = mp.cauchyEst.moment_info       # Cauchy filter statistics
c_means = np.array(cauchy_moment_info["x"])
c_covars = np.array(cauchy_moment_info["P"])
cerr_norm_factors = np.array(cauchy_moment_info["cerr_fz"])
cerr_means = np.array(cauchy_moment_info["cerr_x"])
cerr_covars = np.array(cauchy_moment_info["cerr_P"])

Ts = np.arange(step) * mp.dt
# Plot measured states and estimated states
plt.figure()
plt.suptitle("Pendulum Trajectory (angle: top), (anglular rate: bottom)")
plt.subplot(211)
plt.plot(Ts, state_log[:, 0],'--')
plt.plot(Ts, c_means[:,0])
plt.grid()
plt.legend(["Measurement","Cauchy"])
plt.subplot(212)
plt.plot(Ts, state_log[:, 1],'--')
plt.plot(Ts, c_means[:,1])
plt.grid()
plt.savefig('All_fb')

# Plot covariance and error btw measured and estimated states
plt.figure()
plt.suptitle("Pendulum system 1-sigma bound and error")

plt.subplot(211)
plt.plot(Ts,  np.sqrt(c_covars[:,0,0]),'g')
plt.plot(Ts, -np.sqrt(c_covars[:,0,0]),'g',label='_nolegend_')
plt.plot(Ts, state_log[:,0].reshape(-1) - c_means[:,0].reshape(-1),'k')
plt.legend(["Cauchy 1-sig bound","Cauchy error"],loc="upper right")
plt.grid()

plt.subplot(212)
plt.plot(Ts,  np.sqrt(c_covars[:,1,1]),'g')
plt.plot(Ts, -np.sqrt(c_covars[:,1,1]),'g',label='_nolegend_')  
plt.plot(Ts, state_log[:,1].reshape(-1) - c_means[:,1].reshape(-1),'k')
plt.grid()
plt.savefig("All_var_fb")

plt.figure()
plt.plot(Ts, control_log)
plt.grid()

plt.show()


