from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from matplotlib import pyplot as plt
from scipy import ndimage
from skimage.morphology import skeletonize,medial_axis
import numpy as np
import cv2
from os import system as cmd
import math
import time

while(1):
	im = cv2.imread("openhand.png")
	imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(imgray,127,255,0)
	img1,contours1, hierarchy1 = cv2.findContours(thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)

	drawing = np.zeros(thresh.shape,np.uint8)

	# cv2.drawContours(drawing, contours1, 0, 255, 1)
	# cv2.imshow("int",drawing)

	N = 5
	parts = np.array_split(contours1[0],N)
	for i in range(N-1):
		cnt = [parts[i]]
		print cnt
		cv2.drawContours(drawing, cnt, -1, 255, 1)
		cv2.imshow("int",drawing)
		# time.sleep(1)

	if cv2.waitKey(1) & 0xFF == ord('q'):
	            break