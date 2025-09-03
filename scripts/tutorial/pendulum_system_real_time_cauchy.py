import numpy as np 
import cauchy_estimator as ce
import gaussian_filters as gf
import matplotlib.pyplot as plt
from pendulum_helper import *
import time
from pendulum_device import *

port = '/dev/ttyAMA0'
baudrate = 460800
motor = C2000_Communication(port, baudrate)
mp = PendulumParams(dt=0.004, num_controls=1)

# Initial condition
x0 = np.array([[np.pi/2], [0]])
P0 = np.eye(2) * mp.v_PSD**.5
mp.set_initial(x0, P0)

exp_time = 5 # s
propagations = int(exp_time / mp.dt) + 1

num_windows = 4
mp.cauchy_start(num_windows)

# prepare for the estimator
step = 0
u = 0.0
estimator_start = False
states_log = []
measurement_log = []

# reset pendulum
input("Is the pendulum resting at the zero position? ")
motor.reset_encoder()
print(motor.get_states())

# start the estimator
input("Is the pendulum set in place? ")
_, _, states = motor.get_states()
z_prev = states[0,0]
print(f"Pendulum set at {z_prev}")
_, _, states = motor._get_real_states()
z_prev = states[0,0]
print(f"Pendulum set at {z_prev}")

print(f"Zero pos: {motor.zero_pos}")

input("Drop the pendulum")



estimator_start = True
start_time = time.time()
estimator_time = start_time


while True:
    motor.run(step, control_cmd=u)
    counter, status, x_meas = motor.get_states()    
    z = x_meas[0, 0]
    # print(z)
    if not estimator_start:
        if abs(z_prev - z) > 0.1:
            estimator_start = True
            estimator_time = time.time()
            ini_counter, status, states = motor.reset_counter()
            print(f"Estimator starts at {estimator_time - start_time}, z: {z}, z_prev: {z_prev}, initial counter: {ini_counter}")
    else:
        if status == 1:
            print(f"unsync, sent {step}, received {counter + ini_counter}")
            motor.idle()
            unsync = True
            break
        # cauchyEst.step(z, None)
        mp.cauchy_step(z, u)
        
        # Store returned states, measurement
        states_log.append(x_meas.copy())
        measurement_log.append(z.copy())
        
        step = step + 1
        if step > propagations:
            break

mp.cauchy_shutdown()
end_time = time.time()
print(f"estimator took {end_time - estimator_time} s")

motor.disconnect()

# cauchy_moment_info = mp.cauchyEst.moment_info
# means = np.array(cauchy_moment_info["x"])
# covars = np.array(cauchy_moment_info["P"])
# cerr_norm_factors = np.array(cauchy_moment_info["cerr_fz"])
# cerr_means = np.array(cauchy_moment_info["cerr_x"])
# cerr_covars = np.array(cauchy_moment_info["cerr_P"])

# Ts = np.arange(propagations + 1) * mp.dt
# states_log = np.array(states_log)

# plt.figure()
# plt.suptitle("Pendulum Trajectory (angle: top), (anglular rate: bottom)")
# plt.subplot(211)
# plt.plot(Ts,means[:,0])
# plt.plot(Ts, states_log[:, 0])
# plt.grid()
# plt.legend(["Cauchy","Measurement"])
# plt.subplot(212)
# plt.plot(Ts,means[:,1])
# plt.plot(Ts, states_log[:, 1])
# plt.grid()
# plt.savefig('Cauchy')
# plt.show()



