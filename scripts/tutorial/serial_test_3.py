from pendulum_device import *
import time

port = '/dev/ttyAMA0'
baudrate = 460800

motor = C2000_Communication(port, baudrate)

Ts = 4e-3

unsync_counter = 0
motor.reset_encoder()
motor.idle()
input("Just input")
start_time = time.time()
# time.sleep(0.004)
# motor.reset_counter()
# ini_counter, status, states = motor.get_states()  
unsync = False
ini_counter, status, t_pos, t_vel, z = motor.reset_counter(hardware_reset = True)
# print(ini_counter)
# NOTE: ini_counter does not reset here! it resets on falling edge of reset signal, which is on the next motor.get_states() call

duration = 5 # s

for i in range(0, int(duration / Ts)):
    motor.run(i, control_cmd=0.0)
    counter, status, t_pos, t_vel, z = motor.get_states()
    # print(states[0,0])
    if status != 0 and i != 0:
        print(f"Unsync, status {status}, sent {i}, received {counter}")
        motor.idle()
        unsync = True
        break

if not unsync:
    print("Run sucessfully")
print(f"Program took {time.time() - start_time} s.")
# print(f"Program was unsync for {unsync_counter} samples.")
motor.disconnect()

print(ini_counter)