import numpy as np
import json
import serial
import cv2

# ser = serial.Serial("/dev/ttyACM0",9600)
cap = cv2.VideoCapture(0)

face_cascade = cv2.CascadeClassifier('haarcascades/haarcascade_frontalface_default.xml')
ret, frame = cap.read()
height, width = frame.shape[:2]

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    for (x,y,w,h) in faces:
        img = cv2.rectangle(gray,(x,y),(x+w,y+h),(255,0,0),2)
        img = cv2.line(img, (x+w/2, y+h/2), (width/2, height/2), (0,255,0), 2)

    # Display the resulting frame
    cv2.imshow('frame',img)

    # Calculate p error

    # Calculate i error

    # Calculate d error

    # Calculate control signal

    # Saturate

    # Send to microcontroller

    # controlSignal = {
    # "m1": 5,
    # "m2": 6452,
    # "m3": 73456
    # }

    # data = json.dumps(controlSignal)

    # ser.write(data.encode('ascii'))


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
