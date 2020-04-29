import cv2
import numpy as np 
import dlib
from imutils import face_utils

cap = cv2.VideoCapture(0)
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

def jaw_line(n,width=3,color=(255,0,0)):
    if n<16:
        x1 = landmarks.part(n).x
        y1 = landmarks.part(n).y
        x2 = landmarks.part(n+1).x
        y2 = landmarks.part(n+1).y
        cv2.line(frame,(x1,y1),(x2,y2),color,width)

def left_eyebrow(n,width=3,color=(255,0,0)):
    if n>=17 and n<=20:
        x1 = landmarks.part(n).x
        y1 = landmarks.part(n).y
        x2 = landmarks.part(n+1).x
        y2 = landmarks.part(n+1).y
        cv2.line(frame,(x1,y1),(x2,y2),color,width)

def right_eyebrow(n,width=3,color=(255,0,0)):
    if n>=22 and n<=25:
        x1 = landmarks.part(n).x
        y1 = landmarks.part(n).y
        x2 = landmarks.part(n+1).x
        y2 = landmarks.part(n+1).y
        cv2.line(frame,(x1,y1),(x2,y2),color,width)

def left_eye(n,width=3,color=(255,0,0)):
    if n>=36 and n<=40:
        x1 = landmarks.part(n).x
        y1 = landmarks.part(n).y
        x2 = landmarks.part(n+1).x
        y2 = landmarks.part(n+1).y
        cv2.line(frame,(x1,y1),(x2,y2),color,width)

def right_eye(n,width=3,color=(255,0,0)):
    if n>=42 and n<=46:
        x1 = landmarks.part(n).x
        y1 = landmarks.part(n).y
        x2 = landmarks.part(n+1).x
        y2 = landmarks.part(n+1).y
        cv2.line(frame,(x1,y1),(x2,y2),color,width)

def nose(n,width=3,color=(255,0,0)):
    if n>=27 and n< 35:
        x1 = landmarks.part(n).x
        y1 = landmarks.part(n).y
        x2 = landmarks.part(n+1).x
        y2 = landmarks.part(n+1).y
        cv2.line(frame,(x1,y1),(x2,y2),color,width)

def mouth(n,width=3,color=(255,0,0)):
    if n>=48 and n< 67:
        x1 = landmarks.part(n).x
        y1 = landmarks.part(n).y
        x2 = landmarks.part(n+1).x
        y2 = landmarks.part(n+1).y
        cv2.line(frame,(x1,y1),(x2,y2),color,width)

while True:
    _, frame = cap.read()
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    faces = detector(gray)

    for face in faces:
        # left = face.left()
        # right = face.right()
        # top = face.top()
        # bottom = face.bottom()
        # cv2.rectangle(frame,(left,top),(right,bottom),(0,255,0),3)

        landmarks = predictor(gray, face)

        for n in range(0, 68):
            # x = landmarks.part(n).x
            # y = landmarks.part(n).y
            # cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)
            jaw_line(n,2)
            left_eyebrow(n,2)
            right_eyebrow(n,2)    
            left_eye(n,2)   
            right_eye(n,2)
            nose(n,2)
            mouth(n,2)
            
    # output = face_utils.visualize_facial_landmarks(frame, landmarks)
    cv2.imshow("Gray Frame",frame)

    key = cv2.waitKey(1)

    if key == 27 or key == ord('q'):
        break
