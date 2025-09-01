from pendulum_device import C2000_Communication as C2000
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
count = 0

while True:
    if time.time() - start_time >= timer - dt*.01:
        timer = timer + dt
        status = False
        
        while True:
            
            states, status, tag_v = motor_device.get_states()
            if status:
                print("New states!")
                motor_device.run(-0.4, step)
                break
            else:
                time.sleep(0.0002)
        # print(states)
        if tag_v == step:
            count += 1
        print(step, tag_v - step)
        step = step + 1
        states_log.append(states.copy())
    if step > 5000:
        break

print(time.time() - start_time)
motor_device.disconnect()
print(count)

states_log = np.array(states_log)

plt.figure()
plt.suptitle("Pendulum Trajectory (angle: top), (anglular rate: bottom)")
plt.subplot(211)
plt.plot(states_log[:,0])
plt.subplot(212)
plt.plot(states_log[:,1])
plt.show()
