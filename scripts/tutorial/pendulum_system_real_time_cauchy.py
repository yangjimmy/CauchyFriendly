#!/usr/bin/env python
# coding: utf-8

# Pendulum dynamics equation
# \begin{align}
# \dfrac{dx_1}{dt} &= x_2 \\
# \dfrac{dx_2}{dt} &= -\frac{m g l_c}{J} sin(x_1) - (\frac{B}{J} + \frac{K_m^2}{JR}) x_2
# \end{align}

# ----- Import libraries -----
import numpy as np 
import cauchy_estimator as ce 
import gaussian_filters as gf
import matplotlib.pyplot as plt
from scripts.tutorial.pendulum_device import C2000_Communication as C2000
import time
# import serial_com.C2000_Communication as C2000
# import serial communication


# ----- Define system parameters -----
class PendulumParams:    
    def __init__(self, dt=0.001):
        self.g = 9.81
        self.B = 8.1055e-6
        self.L = .23e-3
        self.R = 3.85
        self.V_s = 10.7
        self.K_m = 0.0228
        self.m = 0.03937
        self.l_c = 0.0254
        self.J_motor = 1.67e-6
        self.J_rod = 2.12e-5
        self.w_PSD = .01
        self.EncRes = 400
        self.w_PSD = .01
        self.H = np.array([1.0,0.0])
        self.Gamma_c = np.array([[0.0],[1.0]])
        
        self.dt = dt
        self.sr = 1/self.dt
        
        self.J = self.J_motor + self.J_rod
        self.Enc_n = 2 * np.pi / self.EncRes
        self.VelRes = self.EncRes / self.dt
        self.v_PSD = self.Enc_n**2 / 12 / self.dt
        self.w_PSD = self.w_PSD / self.dt
        
mp = PendulumParams() # Lets just make a simple globally viewable object to get ahold of these parameters when we want them

# ----- Define system dynamics -----
# The ODE
def pend_ode(x):
    dx_dt = np.zeros(2)
    dx_dt[0] = x[1]
    # dx_dt[1] = -mp.g / mp.L * np.sin(x[0]) - mp.c * x[1]
    dx_dt[1] = -(mp.B/mp.J + mp.K_m**2/(mp.J*mp.R))*x[1]-mp.m*mp.g*mp.l_c*np.sin(x[0])/mp.J
    return dx_dt 

# Nonlinear transition model from t_k to t_k+1...ie: dt
def nonlin_transition_model(x):
    return ce.runge_kutta4(pend_ode, x, mp.dt)

# Jacobian
def jacobian_pendulum_ode(x):
    Jac = np.zeros((2,2))
    # Jac[0,1] = 1
    # Jac[1,0] = -mp.g/mp.L*np.cos(x[0])    
    # Jac[1,1] = -mp.c
    Jac[0,1] = 1
    Jac[1,0] = -mp.m*mp.g*mp.l_c*np.cos(x[0])/mp.J
    Jac[1,1] = -(mp.B/mp.J + mp.K_m**2/(mp.J*mp.R))
    return Jac



# ----- Estimator step -----


# Kalman step
def Kalman_step(x_kf, P_kf, z, taylor_order = 2):
    """
    Take the variables and measurement for the Kalman filter

    Parameters
    -----
    x_kf : nx1 vector
        estimated states
    P_kf : nxn matrix
        estimated variance
    z    : px1 vector
        measurement
    
    Return
    -----
    nx1 nd.array
        propagated estimated states
    nxn nd.array
        propagated estimated variance
    """
    Jac_F = jacobian_pendulum_ode(x_kf)
    Phi_k, W_k = ce.discretize_nl_sys(Jac_F, Gamma_c, W_c, mp.dt, taylor_order, with_Gamk = False, with_Wk = True)
    # Propagate covariance and state estimates
    P_kf = Phi_k @ P_kf @ Phi_k.T + W_k
    x_kf = nonlin_transition_model(x_kf)
    # Form Kalman Gain, update estimate and covariance
    K = P_kf @ H.T @ np.linalg.inv(H @ P_kf @ H.T + V)
    zbar = H @ x_kf
    r = z - zbar
    x_kf += K @ r 
    P_kf = (I2 - K @ H) @ P_kf @ (I2 - K @ H).T + K @ V @ K.T
    return x_kf, P_kf


# Cauchy setting
# This is the callback function correpsonding to the decription for point 1.) above 
def dynamics_update(c_duc):
    taylor_order = 2
    pyduc = ce.Py_CauchyDynamicsUpdateContainer(c_duc)
    ## Propagate x 
    xk = pyduc.cget_x()
    xbar = nonlin_transition_model(xk) # propagate from k -> k+1
    pyduc.cset_x(xbar)
    pyduc.cset_is_xbar_set_for_ece() # need to call this!
    ## Phi, Gamma, beta may update
    Jac_F = jacobian_pendulum_ode(xk)
    Phi_k, Gam_k = ce.discretize_nl_sys(Jac_F, Gamma_c, None, mp.dt, taylor_order, with_Gamk=True, with_Wk=False)
    pyduc.cset_Phi(Phi_k)
    pyduc.cset_Gamma(Gam_k)
    #pyduc.cset_beta(beta)

# This is the callback function correpsonding to the decription for point 2.) above 
def nonlinear_msmt_model(c_duc, c_zbar):
    pyduc = ce.Py_CauchyDynamicsUpdateContainer(c_duc)
    ## Set zbar
    xbar = pyduc.cget_x() # xbar
    zbar = H @ xbar # for other systems, call your nonlinear h(x) function
    pyduc.cset_zbar(c_zbar, zbar)

# This is the callback function correpsonding to the decription for point 3.) above 
def msmt_model_jacobian(c_duc):
    pyduc = ce.Py_CauchyDynamicsUpdateContainer(c_duc)
    ## Set H: for other systems, call your nonlinear jacobian function H(x)
    pyduc.cset_H(H) # we could write some if condition to only set this once, but its such a trivial overhead, who cares




# ----- System initial condition -----

theta_vec0 = np.array([np.pi/2, 0]) # initial angle of 45 degrees at 0 radians/sec
theta_k = theta_vec0.copy()
thetas = [theta_k]




exp_time = 5 # s
propagations = int(exp_time / mp.dt) + 1



# Creating the dynamic simulation
H = mp.H
Gamma_c = mp.Gamma_c
V = np.array([[mp.v_PSD]])
W_c = np.array([[mp.w_PSD]])
I2 = np.eye(2)
H = H.reshape((1,2))

# Setting up and running the EKF
# The gaussian_filters module has a "run_ekf" function baked in, but we'll just show the whole thing here
P0_kf = np.eye(2) * 0.003
x0_kf = np.random.multivariate_normal(theta_vec0, P0_kf) # lets initialize the Kalman filter slightly off from the true state position

xs_kf = [x0_kf.copy()] 
Ps_kf = [P0_kf.copy()] 
x_kf = x0_kf.copy()
P_kf = P0_kf.copy()

# Cauchy setting
scale_g2c = 1.0 / 1.3898 # scale factor to fit the cauchy to the gaussian
beta = np.array([mp.w_PSD / mp.dt])**0.5 * scale_g2c
gamma = np.array([V[0,0]**0.5]) * scale_g2c
x0_ce = x0_kf.copy()
A0 = np.eye(2)
p0 = np.diag(P0_kf)**0.5 * scale_g2c 
b0 = np.zeros(2)

num_controls = 0
print_debug = True
swm_print_debug = False 
win_print_debug = False
num_windows = 4
cauchyEst = ce.PySlidingWindowManager("nonlin", num_windows, swm_print_debug, win_print_debug)
cauchyEst.initialize_nonlin(x0_ce, A0, p0, b0, beta, gamma, dynamics_update, nonlinear_msmt_model, msmt_model_jacobian, num_controls, mp.dt)


# Start experiment
state_log = []
measurement_log = []

# ***** TODO: Initialize serial communication *****

motor_device = C2000('/dev/ttyUSB0')
motor_device.idle()

# reset pendulum

input("Is the pendulum resting at the zero position? ")
motor_device.reset()
# start the estimator
input("Is the pendulum set in place? ")

print("Drop the pendulumn!")
estimator_start = False
z_prev = motor_device.get_states()[0][0]

step = 0
dt = 0.001
timer = .001
start_time = time.time()
states_log = []
while True:
    if time.time() - start_time >= timer - dt*.01:
        timer = timer + dt
        status = False
        while True:
            x_meas, status = motor_device.get_states()
            if status:
                break
            else:
                time.sleep(0.00001)
        z = x_meas[0][0]
        if abs(z - z_prev) > 0.1 and not estimator_start:
            estimator_start = True
            print(f"Estimator actually starts at {time.time() - start_time} lol")
        if estimator_start:
            # print(estimator_start)
            # print(z)
            # x_kf, P_kf = Kalman_step(x_kf,P_kf, z)
            cauchyEst.step(z, None)
            # Store returned states, measurement
            state_log.append(x_meas.copy())
            measurement_log.append(z.copy())
            
            # Store estimates
            # xs_kf.append(x_kf.copy())
            # Ps_kf.append(P_kf.copy())
            step = step + 1
        if step > 5000:
            break

cauchyEst.shutdown()


state_log = np.array(state_log)
measurement_log = np.array(measurement_log)
# xs_kf = np.array(xs_kf)
# Ps_kf = np.array(Ps_kf)
# Plot Simulation results 
# ce.plot_simulation_history( None, (xs,zs,ws,vs), (xs_kf, Ps_kf) )
end_time = time.time()
print(end_time - start_time)

# ***** TODO: end serial communication *****
motor_device.disconnect()


# ***** TODO: plot result *****
cauchy_moment_info = cauchyEst.moment_info
means = np.array(cauchy_moment_info["x"])
covars = np.array(cauchy_moment_info["P"])
cerr_norm_factors = np.array(cauchy_moment_info["cerr_fz"])
cerr_means = np.array(cauchy_moment_info["cerr_x"])
cerr_covars = np.array(cauchy_moment_info["cerr_P"])

Ts = np.arange(propagations) * mp.dt


# plt.figure()
# plt.suptitle("Pendulum Trajectory (angle: top), (anglular rate: bottom)")
# plt.subplot(211)
# plt.plot(Ts, xs_kf[1:, 0])
# plt.plot(Ts, state_log[:, 0])
# plt.grid()
# plt.legend(["Kalman","Measurement"])
# plt.subplot(212)
# plt.plot(Ts, xs_kf[1:, 1])
# plt.plot(Ts, state_log[:, 1])
# plt.grid()
# plt.savefig('Kalman')
# plt.show()

plt.figure()
plt.suptitle("Pendulum Trajectory (angle: top), (anglular rate: bottom)")
plt.subplot(211)
plt.plot(Ts,means[:,0])
plt.plot(Ts, state_log[:, 0])
plt.grid()
plt.legend(["Cauchy","Measurement"])
plt.subplot(212)
plt.plot(Ts,means[:,1])
plt.plot(Ts, state_log[:, 1])
plt.grid()
plt.savefig('Cauchy')
plt.show()
