import serial
import struct
import numpy as np
import time
import threading
import math

class C2000_Communication():
    """
    Establish communication via serial communication.
    """
    def __init__(self,port='/dev/ttyUSB0',baudrate=5e6,timeout=0.001,divider=5):
        """ Initialize C2000_Communication. Connect to serial port

        Paramaters:
        -----
        port : str
        baudrate : int
            baud rate for communication
        timeout : float
            timeout before communication fails
        """
        # ***** TODO: timeout setting *****
        print(f"Starting serial coommunication with {port} ...")
        self.port=port
        self.baudrate = baudrate
        self.ser = serial.Serial(self.port, self.baudrate, timeout=timeout)
        
        self.zero_count = 0
        self.latest = False
        self.t_pos = 0
        
        self.running = True
        self.ini_counter = 0

        self.divider = divider
        self.truth_encoder_res = 2000
        self.truth_count_size = 2 * np.pi / self.truth_encoder_res
        self.encoder_res = int(2000 / divider)
        self.count_size = 2 * np.pi / self.encoder_res
        
    
    def __del__(self):
        self.disconnect()
    
    def disconnect(self):
        self.running = False
        if self.ser.is_open:
            print(f"Disconnect with {self.port}")
            self.idle()
            self.ser.close()
    
    def receive_msg(self):
        """
        Receive message from the serial port with the format of [uint16, uint16, float, float].
        """
        while self.running and self.ser.is_open:
            # try:
            byte = self.ser.read(1)
            if byte == b'S':
                payload = self.ser.read(6)
                terminator = self.ser.read(1)
                if terminator == b'E' and len(payload) == 6:
                    msg = struct.unpack('<HHh', payload)
                    return msg
            # except Exception as e:
            #     print(f"[Receive Error] {e}")
            # time.sleep(.00005)

    def transmit_msg(self, msg):
        """
        Send message to serial port
        Parameters:
        -----
        msg : [uint16, uint16, float]
            message to send
        """
        msg[0] += self.ini_counter
        msg[0] &= 0xffff
        payload = struct.pack('<HHf', *msg)
        message = b'S' + payload + b'E'
        self.ser.write(message)
        # print(msg[0])
    
    def idle(self, counter=0):
        """
        Set the motor device into idle status
        """
        # counter &= 0xffff
        self.transmit_msg([counter, 0, 0.0])
    
    def reset_counter(self, hardware_reset=True):
        """
        Reset the time counter on C2000
        """
        if hardware_reset:
            self.ini_counter = 0
            self.transmit_msg([0, 1, 0.0])
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            counter, status, t_pos, t_vel, z = self.get_states()
        else:
            counter, status, t_pos, t_vel, z = self.get_states()
            self.ini_counter = counter
            print(f"Counter reseted at {counter}")
        return counter, status, t_pos, t_vel, z

    def run(self, counter, control_cmd=0.0):
        # counter &= 0xffff
        if control_cmd == 0.0:
            self.transmit_msg([counter, 0, float(control_cmd)])
        else:
            control_cmd = self.saturation(control_cmd, 1)
            self.transmit_msg([counter, 2, float(control_cmd)])
    
    def reset_counter_run(self,control_cmd=0.0):
        # counter &= 0xffff
        self.transmit_msg([0, 3, float(control_cmd)])
        counter, status, states = self.get_states()
        self.ini_counter = counter
        return counter, status, states

    def _get_real_count(self):
        msg = self.receive_msg()
        return msg[0], msg[1], msg[2]
        # counter = msg[0]
        # status = msg[1]
        # count = msg[2]
        # return counter, status, count

    def reset_encoder(self):
        _, status, count = self._get_real_count()
        self.zero_count = count
        print(f"Reset complete at {count}!")
    
    def get_states(self):
        counter, status, count = self._get_real_count()
        t_pos = (count - self.zero_count) * self.truth_count_size
        t_vel = (t_pos - self.t_pos) / 0.004
        self.t_pos = t_pos
        z = round((count - self.zero_count)/self.divider) * self.count_size
        return counter, status, t_pos, t_vel, z
    
    @staticmethod
    def saturation(x, lb, ub=None):
        if ub is None:
            lb = -abs(lb)
            ub =  abs(lb)
        if lb > ub:
            lb, ub = ub, lb
        if x > ub:
            x = ub
        if x < lb:
            x = lb
        return x
