
import serial
import time
import struct
import binascii

ser = serial.Serial("/dev/ttyACM1",9600)
time.sleep(3)


a = 100
b = -50
c = -500

def sendPacket(v1,v2,v3):
    """Packs a python 4 byte integer to an arduino unsigned long"""
    packet = struct.pack('>biiii',126,v1,v2,v3,2147483647) 
    print(binascii.hexlify(bytearray(packet)))
    return packet    #should check bounds

while True:

    a = a + 10
    b = b - 10
    c = c + 10
    
    # send and receive via pyserial
    ser.write(sendPacket(a,b,c))
    # time.sleep(0.01)
