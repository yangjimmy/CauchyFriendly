from pendulum_device import *
import time

port = '/dev/ttyAMA0'
baudrate = 460800

motor = C2000_Communication(port, baudrate)

unsync_counter = 0
motor.reset_encoder()
motor.idle()
start_time = time.time()
input("Just input")
time.sleep(0.004)
# motor.reset_counter()
# ini_counter, status, states = motor.get_states()
unsync = False
ini_counter, status, states = motor.reset_counter()
print(ini_counter)
for i in range(0,501):
    motor.run(i, control_cmd=0.0)
    counter, status, states = motor.get_states()
    # print(states[0,0])
    if status != 0:
        print(f"Status {status}, sent {i}, received {counter}")
        motor.idle()
        unsync = True
        break

if not unsync:
    print("Run sucessfully")
print(f"Program took {time.time() - start_time} s.")
# print(f"Program was unsync for {unsync_counter} samples.")
motor.disconnect()

