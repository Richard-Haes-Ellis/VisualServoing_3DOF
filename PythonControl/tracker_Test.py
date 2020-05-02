# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import struct
import time
import serial
import binascii
import cv2

# grab the appropriate object tracker using our dictionary of
# OpenCV object tracker objects
tracker = cv2.TrackerKCF_create()

# initialize the bounding box coordinates of the object we are going
# to track
initBB = None

print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(1.0)

# initialize the FPS throughput estimator
fps = None
loop_frequency = 30
dt = 1.0/loop_frequency # 30 fps
prev = 0

# Error variables
epx_1 = 0
epy_1 = 0
eix = 0
eiy = 0

# PID parameters Kp Ki Kd
kp = 60
ki = 0.25
kd = 1.8

# Open serial connection to arduino 
ser = serial.Serial("/dev/ttyACM0",9600)
time.sleep(3)

def sendPacket(v1,v2,v3):
    """Packs a python 4 byte integer to an arduino unsigned long"""
    packet = struct.pack('>biiii',126,int(v1),int(v2),int(v3),2147483647) 
    # print(binascii.hexlify(bytearray(packet)))
    return packet    #should check bounds


# loop over frames from the video stream
while True:
	time_elapsed = time.time() - prev
	if(time_elapsed > dt):
        # print('Elapsed time {}'.format(time_elapsed))
		prev=time.time()
		# grab the current frame from VideoStream
		frame = vs.read()
		# resize the frame (so we can process it faster) and grab the frame dimensions
		frame = imutils.resize(frame, width=500)
		(H, W) = frame.shape[:2]
		# check to see if we are currently tracking an object
		if initBB is not None:
			# grab the new bounding box coordinates of the object
			(success, box) = tracker.update(frame)
			# check to see if the tracking was a success
			if success:
				(x, y, w, h) = [int(v) for v in box]
				cv2.rectangle(frame, (x, y), (x + w, y + h),
					(0, 255, 0), 2)	
				cv2.line(frame, (int(x+w/2), int(y+h/2)), (int(W/2), int(H/2)),
					(0,255,0), 2)
				#### CONTROL ####
				# Calculate p error
				epx = (x+w/2)-(W/2)
				epy = (y+h/2)-(H/2)

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
				u_x = (kp*epx + ki*eix + kd*edx)
				u_y = -(kp*epy + ki*eiy + kd*edy)

				# Send to microcontroller
				ser.write(sendPacket(0,u_x,u_y))

				epx_1 = epx
				epy_1 = epy
			else:
				# Nothing is being tracked
				# Stop any controller variables or control signals
				u_x = 0
				u_y = 0
				ser.write(sendPacket(0,0,0))
				print("Lost target!")

			# update the FPS counter
			# fps.update()
			# fps.stop()
			# initialize the set of information we'll be displaying on
			# the frame
			info = [
				("Tracker", "kcf"),
				("Success", "Yes" if success else "No"),
				("FPS", "{:.5f}".format(time_elapsed)),
				#("Control signals x:{} y:{}".format(x,y)),
			]
			# loop over the info tuples and draw them on our frame
			for (i, (k, v)) in enumerate(info):
				text = "{}: {}".format(k, v)
				cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
					cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
						# show the output frame

		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF
		# if the 's' key is selected, we are going to "select" a bounding
		# box to track
		if key == ord("s"):
			# select the bounding box of the object we want to track (make
			# sure you press ENTER or SPACE after selecting the ROI)
			initBB = cv2.selectROI("Frame", frame, fromCenter=False,
				showCrosshair=True)
			# start OpenCV object tracker using the supplied bounding box
			# coordinates, then start the FPS throughput estimator as well
			tracker.init(frame, initBB)
			fps = FPS().start()
				# if the `q` key was pressed, break from the loop
		elif key == ord("q"):
			break

# release the pointer
vs.stop()

# close all windows
cv2.destroyAllWindows()