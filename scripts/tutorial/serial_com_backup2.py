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
        self.port     = port
        self.baudrate = baudrate
        self.ser      = serial.Serial(self.port, self.baudrate, timeout=timeout)
        self.running = True

        self.data_size   = 4
        self.data_length = 3
        self.zero_pos    = np.zeros([2,1])
        self.latest      = False
        self.latest_msg  = None
    
    def __del__(self):
        self.disconnect()
    
    def disconnect(self):
        self.running = False
        if self.ser.is_open:
            print(f"Diconnect with {self.port}")
            self.idle()
            self.ser.close()

    def clear_buffer(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
    
    def get_latest_msg(self):
        # try:
        count = 0
        while count < 16:
            byte = self.ser.read(1)
            if byte == b'S':
                payload = self.ser.read(self.data_size * self.data_length)
                terminator = self.ser.read(1)
                if terminator == b'E' and len(payload) == self.data_size * self.data_length:
                    msg = struct.unpack('<'+'f'*self.data_length, payload)
                    return msg
        # except Exception as e:
        #     print(f"[Receive Error] {e}")
    def transmit_msg(self, msg):
        """
        Send message to serial port
        Parameters:
        -----
        msg : int list
            message to send
        """

        payload = struct.pack('<'+'f'*len(msg), *msg)
        message = b'S' + payload + b'E'
        self.ser.write(message)

    # send status
    # idle : 0
    # run  : 1
    def idle(self):
        """
        Set the motor device into idle status
        """
        self.transmit_msg([0,0])

    def run(self,control_cmd):
        self.transmit_msg([1,control_cmd])

    def reset(self):
        msg = self.get_latest_msg()
        self.zero_pos = np.array(msg[1:3]).reshape([2,1])
        print("Reset complete!")
        
    def get_states(self):
        msg = self.get_latest_msg()
        if msg is None or len(msg) < self.data_length:
            return None  # or raise an exception
        value = np.array(msg[1:3]).reshape([2,1])
        return (value - self.zero_pos)
    
    # receive message status check
    # idle :         0
    # running:       1
    # overspeed:     2
    # stall:         3
    def status_check(self,msg):
        status= msg[0]
        if status == 2:
            print("Motor is overspeeding!!")
        elif status == 3:
            print("Motor is stalling")
