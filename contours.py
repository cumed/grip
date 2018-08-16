# -*- coding: utf-8 -*-
"""
Created on Tue Jul 3 15:03:12 2018

@author: sramana

"""

import cv2
import numpy as np
from matplotlib import pyplot as plt
import numpy.linalg as la

img = [] # 
lineRep = []
array = [0,10,20]


# uncomment after connecting pi cam. 
#l = len(bends)
#camera.start_preview()
#for i in range(l):
#    sleep(4)
#    camera.capture('/home/pi/Desktop/img%s.jpg' % i)
#camera.stop_preview()

plt.figure()

ax = plt.gca()
for i in range(0,len(array)):
    img.append(cv2.imread('img%s.jpg' % i))

#img1 = cv2.imread('img.jpg')
    r = cv2.selectROI(img[i])# select the region of the captured image to be analysed
    # Crop image
    imCrop = img[i][int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
    gray = cv2.cvtColor(imCrop,cv2.COLOR_BGR2GRAY)
## 
## Apply edge detection method on the image
    edges = cv2.Canny(gray,50,150,apertureSize = 3)
    linesP = cv2.HoughLinesP(edges,1,np.pi/180,10,10,1)
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            print(l)
            cv2.line(imCrop, (l[0], l[1]), (l[2], l[3]), (0,255,0), 3)
#    plt.subplot(121),plt.imshow(img[i],cmap = 'gray')
#    plt.title('lines'), plt.xticks([]), plt.yticks([])
#    plt.show()
    points = []
    for line in linesP:
     for x1,y1,x2,y2 in line:
        plt.plot([x1,x2], [y1,y2], 'ro')
        points.append((x1,y1))
        points.append((x2,y2))
    [vx1,vy1,x1,y1] = cv2.fitLine(np.array(points, dtype=np.int32), cv2.DIST_L2,0,0.01,0.01) # fit line along all the small lines detected
    lineRep.append([vx1[0],vy1[0]])
    ax.quiver(0,0,vx1,vy1,angles='xy',scale_units='xy',scale=1)# plotting the line
    plt.axis([-1, 1, -1, 1])
    plt.draw()
    plt.show()


imgAngles = []
for i in range(0,len(array)-1):
    print(lineRep[i])
    print(lineRep[i+1])
    cosang = np.dot(lineRep[i],(lineRep[i+1]))
    sinang = la.norm(np.cross(lineRep[i],(lineRep[i+1])))
    imgAngles.append(np.degrees(np.arctan2(sinang, cosang)))      
#    font = cv2.FONT_HERSHEY_SIMPLEX                #uncomment to print the angle on the image
#    cv2.putText(imCrop,str(angle),(20,25), font, 1,(0,255,0),2,cv2.LINE_AA)

#cv2.imwrite('linesDetectedTest.jpg', imCrop)  #uncomment to save the image with detected lines

   