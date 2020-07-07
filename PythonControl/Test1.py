# import the necessary packages

import matplotlib.pyplot as plt
from imutils.video import VideoStream
import imutils
import time
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

positionX = []
positionY = []
Time = []

cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
# set your desired size
cv2.resizeWindow('Frame', 1080, 1920)

# loop over frames from the video stream
while True:
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
				positionX.append(x+w/2)
				positionY.append(y+h/2)
				Time.append(time.time())

			else:
				# Nothing is being tracked
				print("Lost target!")

			info = [
				("Tracker", "kcf"),
				("Success", "Yes" if success else "No")
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
				# if the `q` key was pressed, break from the loop
		elif key == ord("q"):
			break

# release the pointer
vs.stop()
fig1, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)
ax1.plot(Time, positionX, Time, positionY)
ax1.set_title('Posición - timepo')
ax1.set_xlabel('Tiempo X')
ax1.set_ylabel('Posición X-Y')

ax2.plot(positionX, positionY)
ax2.set_title('Posición en la imagen')
ax2.set_xlabel('Posición X')
ax2.set_ylabel('Posición Y')

plt.show()

# close all windows
cv2.destroyAllWindows()