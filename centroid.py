
import cv2
import numpy as np


img=cv2.imread('hand1.jpg',0)
cv2.imshow('image',img)
cv2.waitKey(0)

ret2,thresh = cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
kernel = np.ones((5,5),np.uint8)
thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
cv2.imshow('threshold',thresh)
cv2.waitKey(0)

image,contours,hierarchy = cv2.findContours(thresh, 1, 2)
print len(contours)
thresh = cv2.drawContours(thresh, contours, -1, (0,255,0), 10)
cv2.imshow('centroid',thresh)
cv2.waitKey(0)
# #print type(contours)
# if contours:

#     cnt = contours[1]
#     convex_Hull=cv2.convexHull(cnt)
#     convexity_Defects = cv2.convexityDefects(cnt,convex_Hull)
#     M = cv2.moments(cnt)
#     print M
#     if M['m00'] != 0:
#         print ':'

#         cx = int(M['m10']/M['m00'])
#         print cx
#         cy = int(M['m01']/M['m00'])
#         print cy
#         frame = cv2.circle(img,(cx,cy), 10,(255,0,0),5)
#         cv2.imshow('centroid',frame)
#         cv2.waitKey(0)