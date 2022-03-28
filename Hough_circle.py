# -*- coding: utf-8 -*-
"""
Created on Tue Nov 19 19:24:30 2019

@author: oscar
"""

import cv2 
import numpy as np
global circles

cam = cv2.VideoCapture(0)
while True:
    # Read each frame in video stream
    ret, frame = cam.read()
    # Display each frame in video stream
    
    cap_cnt=0
    radius = 4
    num_corners=100
    img = cv2.medianBlur(frame,5)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 100)
    if circles is not None:
	# convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        

        for (x, y, r) in circles:
		# draw the circle in the output image, then draw a rectangle
		# corresponding to the center of the circle
		cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
		cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
    
    
    
    cv2.imshow("Hough Circles", frame)
    if not ret:
        break
# Monitor keystrokes
    k = cv2.waitKey(1)

    if k & 0xFF == ord('q'):
        # q key pressed so quit
        print("Quitting...")
        cv2.destroyAllWindows()
        break
    elif k & 0xFF == ord('c'):
        # c key pressed so capture frame to image file
        cap_name = "capture_{}.png".format(cap_cnt)
        cv2.imwrite(cap_name, frame)
        print("Saving {}!".format(cap_name))
        # Increment Capture Counter for next frame to capture
        cap_cnt += 1
cam.release()
print('Released')
    