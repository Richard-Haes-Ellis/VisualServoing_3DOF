import numpy as np
import serial
import cv2
import time
import struct
import binascii

ser = serial.Serial("/dev/ttyACM0",9600)
time.sleep(3)
cap = cv2.VideoCapture(0)

# face_cascade = cv2.CascadeClassifier('haarcascades/haarcascade_frontalface_default.xml')
# face_cascade = cv2.CascadeClassifier('haarcascades/haarcascade_profileface.xml')
# face_cascade = cv2.CascadeClassifier('haarcascades/haarcascade_frontalcatface_extended.xml')
# face_cascade = cv2.CascadeClassifier('haarcascades/haarcascade_frontalcatface.xml')
# face_cascade = cv2.CascadeClassifier('haarcascades/haarcascade_frontalface_alt_tree.xml')
face_cascade = cv2.CascadeClassifier('haarcascades/haarcascade_frontalface_alt.xml')
# face_cascade = cv2.CascadeClassifier('haarcascades/haarcascade_frontalface_alt2.xml')
# face_cascade = cv2.CascadeClassifier('haarcascades/haarcascade_smile.xml')
# face_cascade = cv2.CascadeClassifier('haarcascades/haarcascade_frontalface_alt.xml')

ret, frame = cap.read()
height, width = frame.shape[:2]


dt = 1.0/30.0 # 30 fps

epx_1 = 0
epy_1 = 0
eix = 0
eiy = 0

kp = 1
ki = 0
kd = 0

frame_rate = 10
prev = 0

def sendPacket(v1,v2,v3):
    """Packs a python 4 byte integer to an arduino unsigned long"""
    packet = struct.pack('>biiii',126,int(v1),int(v2),int(v3),2147483647) 
    # print(binascii.hexlify(bytearray(packet)))
    return packet    #should check bounds

while(True):
    time_elapsed = time.time() - prev
    if time_elapsed > dt:
        # print('Elapsed time {}'.format(time_elapsed))
        prev = time.time()
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        for (x,y,w,h) in faces:
            gray = cv2.rectangle(gray,(x,y),(x+w,y+h),(255,0,0),2)
            gray = cv2.line(gray, (int(x+w/2), int(y+h/2)), (int(width/2), int(height/2)), (0,255,0), 2)

        # Display the resulting frame
        cv2.imshow('frame',gray)
        if 'x' in locals():
            # Calculate p error
            epx = (x+w/2)-(width/2)
            epy = (y+h/2)-(height/2)

            # Calculate i error

            eix = eix + epx*dt
            eiy = eiy + epy*dt

            # Calculate d error
            edx = (epx-epx_1)/dt
            edy = (epy-epy_1)/dt
            
            #print(' Errx:{}    Erry:{}'.format(p_error_x,p_error_y))
            #print('iErrx:{}   iErry:{}'.format(i_error_x,i_error_y))
            #print('dErrx:{}   dErry:{}'.format(d_error_x,d_error_x))

            # Calculate control signal
            x = (kp*epx + ki*eix + kd*edx)
            y = -(kp*epy + ki*eiy + kd*edy)
            
            print('x:{}   y:{}'.format(x,y))

            # Send to microcontroller
            ser.write(sendPacket(0,x,y))

            epx_1 = epx
            epy_1 = epy

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

       


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
