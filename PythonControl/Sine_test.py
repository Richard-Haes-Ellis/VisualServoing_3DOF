
import serial
import time
import struct
import binascii
import numpy as np

ser = serial.Serial("/dev/ttyACM0",9600)
time.sleep(3)

t = 0
a = 0
b = 0
c = 0

def sendPacket(v1,v2,v3):
    """Packs a python 4 byte integer to an arduino unsigned long"""
    packet = struct.pack('>biiii',126,v1,v2,v3,2147483647) 
    # print(binascii.hexlify(bytearray(packet)))
    return packet    #should check bounds

while True:
    
    t = t + 0.05
    a = 2500.0*np.sin(t)
    b = 2500.0*np.sin(t)
    c = 2500.0*np.sin(t)
    # send and receive via pyserial
    ser.write(sendPacket(int(a),int(b),int(0)))
    time.sleep(0.1)
