import numpy as np 
import cauchy_estimator as ce
import gaussian_filters as gf
import matplotlib.pyplot as plt
from pendulum_helper import *
import time
from pendulum_device import *
import pdb

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
time.sleep(0.01)
input("Is the pendulum set in place? ")
# time.sleep(1)
# _, _, states = motor.get_states()
# z_prev = states[0,0]
# print(f"Pendulum set at {z_prev}")
# _, _, states = motor._get_real_states()
# z_prev = states[0,0]
# print(f"Pendulum set at {states[0,0]}")
ini_counter, status, states = motor.reset_counter(hardware_reset=True)

_, _, z_prev = motor.get_meas()
print(f"z prev: {z_prev}")
# print(f"Zero pos: {motor.zero_pos}")

# input("Drop the pendulum")
print("Drop the pendulum")

estimator_start = False
start_time = time.time()
estimator_time = start_time

ini_counter, status, states = motor.reset_counter(hardware_reset=True)

while True:
    counter, status, x_meas = motor.get_states()   
    motor.run(step, control_cmd=u) 
    z = x_meas[0, 0]
    # print(z)
    if not estimator_start: # estimator_start == False
        if abs(z_prev - z) > 0.1:
            estimator_start = True
            estimator_time = time.time()
            mp.cauchy_start(num_windows)
            ini_counter, status, states = motor.reset_counter(hardware_reset=True)
            # print(f"Estimator starts at {estimator_time - start_time}, z: {z}, z_prev: {z_prev}, initial counter: {ini_counter}")
            print(f"Estimator starts at {estimator_time - start_time}, z: {z}, z_prev: {z_prev}")
            ini_counter = 0
    else: # estimator_start == True
        if status == 1 and step != 0: # C2000 counter resets at step = 0, does not output 0 on that iteration
            # only start looking at synching once motor resets
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

cauchy_moment_info = mp.cauchyEst.moment_info
means = np.array(cauchy_moment_info["x"])
covars = np.array(cauchy_moment_info["P"])
cerr_norm_factors = np.array(cauchy_moment_info["cerr_fz"])
cerr_means = np.array(cauchy_moment_info["cerr_x"])
cerr_covars = np.array(cauchy_moment_info["cerr_P"])

Ts = np.arange(propagations + 1) * mp.dt
states_log = np.array(states_log)
measurement_log = np.array(measurement_log)

plt.figure()
plt.suptitle("Pendulum Trajectory (angle: top), (anglular rate: bottom)")
plt.subplot(211)
plt.plot(Ts,means[:,0],linewidth=1)
plt.plot(Ts,states_log[:,0],linewidth=1)
plt.grid()
plt.legend(["Cauchy","Measurement"])
plt.subplot(212)
plt.plot(Ts,means[:,1],linewidth=1)
plt.plot(Ts,states_log[:,1],linewidth=1)
plt.grid()
plt.savefig('Cauchy.png')
plt.show()

plt.figure()
plt.suptitle("Pendulum Trajectory Error (angle: top), (anglular rate: bottom)")
plt.subplot(211)
plt.plot(Ts,np.squeeze(states_log[:,0])-means[:,0],linewidth=1)
plt.grid()
plt.subplot(212)
plt.plot(Ts,np.squeeze(states_log[:,1])-means[:,1],linewidth=1)
plt.grid()
plt.savefig('Cauchy error.png')
plt.show()

# print(np.asarray([states_log, measurement_log]))
print(covars.shape)

N = len(states_log)

save_data = np.concatenate((states_log.reshape(N, 2), means.reshape(N,2), covars[:,0,0].reshape(N,1), covars[:,1,1].reshape(N,1)), axis=1)
np.savetxt("data_log.csv", save_data, delimiter=",")

