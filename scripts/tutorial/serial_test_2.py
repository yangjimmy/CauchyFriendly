from pendulum_device import *
import time

motor = C2000_Communication()
motor.reset()

motor.reset_counter()
start_time = time.time()
cmd = 0.5
for i in range(1000):
    print(motor.receive_msg())
    cmd *= -1
    motor.run(1000, 0.2)
    time.sleep(0.0005)
print(time.time() - start_time)

motor.disconnect()


# import serial
# import struct
# import numpy
# import time



# def receive_msg(ser):
#     # try:
#     while True:
#         byte = ser.read(1)
#         if byte == b'S':
#             payload = ser.read(12)
#             terminator = ser.read(1)
#             if terminator == b'E':
#                 msg = struct.unpack('<HHff', payload)
#                 return msg
# def transmit_msg(ser, msg):
#         """
#         Send message to serial port
#         Parameters:
#         -----
#         msg : [uint16, uint16, float]
#             message to send
#         """
#         payload = struct.pack('<Hf', *msg)
#         message = b'S' + payload + b'E'
#         ser.write(message)
# port = '/dev/ttyUSB0'
# baudrate = 5e6
# ser = serial.Serial(port, baudrate=baudrate,timeout=0.001)

# for i in range(1000):
#     print(receive_msg(ser))
#     transmit_msg(ser, [4, -.2])


# ser.close()

