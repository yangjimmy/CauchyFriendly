import serial
import struct
import numpy as np
import time
import threading

class C2000_Communication():
    """
    Establish communication via serial communication.
    """
    def __init__(self,port='/dev/ttyUSB0',baudrate=5e6,timeout=0.001):
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
        
        self.zero_pos = np.zeros([2,1])
        self.latest = False
        
        self.running = True
    
    def __del__(self):
        self.disconnect()
    
    def disconnect(self):
        self.running = False
        if self.ser.is_open:
            print(f"Diconnect with {self.port}")
            self.idle()
            self.ser.close()
    
    def receive_msg(self):
        while self.running and self.ser.is_open:
            # try:
            byte = self.ser.read(1)
            if byte == b'S':
                payload = self.ser.read(12)
                terminator = self.ser.read(1)
                if terminator == b'E' and len(payload) == 12:
                    msg = struct.unpack('<HHff', payload)
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
        payload = struct.pack('<HHf', *msg)
        message = b'S' + payload + b'E'
        self.ser.write(message)

    def idle(self):
        """
        Set the motor device into idle status
        """
        self.transmit_msg([0, 0, 0.0])

    def reset_counter(self):
        """
        Reset the time counter on C2000
        """
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.transmit_msg([0, 1, 0.0])
    
    def run(self, counter, control_cmd=0.0):
        self.transmit_msg([counter, 2, float(control_cmd)])
    
    def reset_counter_run(self,control_cmd=0.0):
        self.transmit_msg([0, 3, float(control_cmd)])

    def _get_real_states(self):
        msg = self.receive_msg()
        counter = msg[0]
        status = msg[1]
        states = np.array(msg[2:4]).reshape([2,1])
        return counter, status, states

    def reset(self):
        _, status, states = self._get_real_states()
        self.zero_pos = states
        print(f"Reset complete at {states}!")
    
    def get_states(self):
        counter, status, states = self._get_real_states()
        return counter, status, (states - self.zero_pos)

        
    
    
