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
        
        self.data_size = 4
        self.data_length = 3
        self.zero_pos = np.zeros([2,1])
        self.latest = False
        
        self.running = True
        self.latest_msg = None
        self.lock = threading.Lock()

        self.recv_threading = threading.Thread(target=self._receive_loop,daemon=True)
        self.recv_threading.start()
    
    def __del__(self):
        self.disconnect()
    
    def disconnect(self):
        self.running = False
        if self.ser.is_open:
            print(f"Diconnect with {self.port}")
            self.idle()
            self.ser.close()
    
    def _receive_loop(self):
        while self.running and self.ser.is_open:
            try:
                byte = self.ser.read(1)
                if byte == b'S':
                    payload = self.ser.read(self.data_size * self.data_length)
                    terminator = self.ser.read(1)
                    if terminator == b'E' and len(payload) == self.data_size * self.data_length:
                        msg = struct.unpack('<'+'f'*self.data_length, payload)
                        with self.lock:
                            self.latest_msg = msg
                            self.latest = True
            except Exception as e:
                print(f"[Receive Error] {e}")
            time.sleep(.00005)

    def get_latest_msg(self):
        with self.lock:
            if self.latest:
                self.latest = False
                return self.latest_msg, True
            else:
                return self.latest_msg, False

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
        msg, _ = self.get_latest_msg()
        self.zero_pos = np.array(msg[1:3]).reshape([2,1])
        print(f"Reset complete at {self.get_states()}!")
        
    def get_states(self):
        msg, status = self.get_latest_msg()
        if msg is None or len(msg) < self.data_length:
            return None  # or raise an exception
        value = np.array(msg[1:3]).reshape([2,1])
        return (value - self.zero_pos), status
    
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
