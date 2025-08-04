from serial_com import C2000_Communication as C2000
import time
import matplotlib.pyplot as plt
import numpy as np
import sys

motor_device = C2000('/dev/ttyUSB0')

ans = input("Is the pendulum resting at the zero position? ")
motor_device.reset()
motor_device.get_states()
# ans = input("Start estimator ?")
# motor_device.clear_buffer()

step = 0
dt = 0.001
timer = .001
start_time = time.time()
states_log = []

motor_device.run(.2)

while True:
    if time.time() - start_time >= timer - dt*.01:
        timer = timer + dt
        status = False
        while True:
            states, status = motor_device.get_states()
            if status:
                break
            else:
                time.sleep(0.00001)
        # print(states)
        step = step + 1
        states_log.append(states.copy())
    if step > 5000:
        break

print(time.time() - start_time)
motor_device.disconnect()

states_log = np.array(states_log)

plt.figure()
plt.suptitle("Pendulum Trajectory (angle: top), (anglular rate: bottom)")
plt.subplot(211)
plt.plot(states_log[:,0])
plt.subplot(212)
plt.plot(states_log[:,1])
plt.show()
