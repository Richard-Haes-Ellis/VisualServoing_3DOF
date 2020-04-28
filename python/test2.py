
import json
import serial
import time
import struct

ser = serial.Serial("/dev/ttyACM0",9600)
time.sleep(5)


def packIntegerAsULong(v1,v2,v3):
    """Packs a python 4 byte unsigned integer to an arduino unsigned long"""
    return struct.pack('III', v1,v2,v3)    #should check bounds

while True:
    # send and receive via pyserial
    ser.write(('\n').encode())
    ser.write(packIntegerAsULong(100,342,534))
    time.sleep(2)
