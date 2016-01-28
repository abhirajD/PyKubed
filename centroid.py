
import cv2
import numpy as np


im=cv2.imread('hand.jpg')
im = np.array(im * 255, dtype = np.uint8)
ret,thresh = cv2.threshold(im,100,255,cv2.THRESH_BINARY)
image,contours,hierarchy = cv2.findContours(thresh, 1, 2)
#print type(contours)
if contours:

    cnt = contours[0]
    M = cv2.moments(cnt)
    print M
    if M['m00'] != 0:
        print ':'

        cx = int(M['m10']/M['m00'])
        print cx
        cy = int(M['m01']/M['m00'])
        print cy