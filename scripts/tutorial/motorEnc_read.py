import serial.tools.list_ports 
import struct
import sys
import time
import csv 
import matplotlib.pyplot as plt
import numpy as np
#ports = serial.tools.list_ports.comports() 
serialInst = serial.Serial() 

# portList = []; 

# for onePort in ports: 
#     portList.append(str(onePort)) 
#     print(str(onePort)) 

# val = input("select Port: COM")

# for x in range(0,len(portList)): 
#     if portList[x].startswith("COM"+str(val)): 
#         portVar = "COM" + str(val)
#         print(portList[x])

portVar = "COM10"; 
serialInst.baudrate = 115200
serialInst.timeout = 1; # 1 second
serialInst.parity = serial.PARITY_NONE; 
serialInst.stopbits = serial.STOPBITS_ONE; 
serialInst.bytesize = serial.EIGHTBITS; 
serialInst.port = portVar; 
serialInst.open() 

print("buffer size: ",serialInst.in_waiting) 

packetBytes = serialInst.readline(); 
#packdecodedBytes = struct.unpack("<f",packetBytes); 
print(packetBytes)
#print(str(packetBytes,'UTF-8'))
#print(packetBytes.decode('utf'))
#print(packdecodedBytes)
flag1 = True; 
X = np.array([]); 
V = np.array([]); 
start_Time = time.time(); 
time_now = start_Time; 
x_prev = 0; 
v_prev = 0;
path = "C:/Users/omars/Documents/CrazyflieProject/singleCF0/MotorEncData"
log_memory = [[0,0,0]];  
while flag1: 
    dt = time.time()-time_now; 
    if dt>=0.01:
        time_now = time.time(); 
        packetBytes = serialInst.readline(); 
        #print(len(packetBytes[0:4]))
        #print(len(packetBytes[5:9]))
        print(len(packetBytes),dt);
        if len(packetBytes)==9: 
            x =struct.unpack("<f",packetBytes[0:4])
            v = struct.unpack("<f",packetBytes[4:8])
            print(x[0]); 
            print(v[0]); 
            #log_memory.append([dt,x[0],v[0]])
            X  = np.append(X,x[0]); 
            V  = np.append(V,v[0]); 
            x_prev = x[0]; 
            v_prev = v[0]; 
        else: 
            print(x_prev)
            print(v_prev)
            #log_memory.append([dt,x_prev,v_prev])
            X  = np.append(X,x_prev)
            V  = np.append(V,v_prev)
        serialInst.reset_input_buffer(); 
    if time.time()-start_Time > 10:
        flag1 = False; 

n = len(log_memory); 
print(n)

# with open(path+"/test_data3.csv","w") as f: 
#           writer = csv.writer(f); 
#           writer.writerows(log_memory[1:n]); 

plt.figure()
plt.plot(X)
plt.plot(V)
plt.grid(True)
plt.show()



