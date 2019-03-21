# -*- coding: utf-8 -*-
"""
Created on Wed Feb 20 09:55:27 2019

@author: Sramana Dan
"""

import cv2
import time

cam = cv2.VideoCapture(1)

cv2.namedWindow("test")

img_counter = 0
print(cv2.VideoCapture(1).isOpened())
#print(cam.read())
#time.sleep(0.1)  # If you don't wait, the image will be dark
#return_value, image = cam.read()
#cv2.imwrite("opencv.jpg", image)

while True:
    ret, frame = cam.read()
    cv2.imshow("test", frame)
    if not ret:
        break
    k = cv2.waitKey(1)

    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    elif k%256 == 32:
        # SPACE pressed
        img_name = "opencv_frame_{}.png".format(img_counter)
        cv2.imwrite(img_name, frame)
        print("{} written!".format(img_name))
        img_counter += 1

cam.release()

cv2.destroyAllWindows()
