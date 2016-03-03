from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from matplotlib import pyplot as plt
from scipy import ndimage
import numpy as np
import cv2
from os import system as cmd
import math

while(True):
	img = cv2.imread('hand_body.jpg',0)
	img = np.array(img, dtype = np.uint8)
	cv2.imshow('oring',img)
	val = img[40,40]+3

	img[img>val] /=3

	
	cv2.imshow('orig2',img)
	ret, img = cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	cv2.imshow('thresh', img)
	# im2, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	# cv2.drawContours(img, contours, 3, 255, 3)
	# cv2.imshow('cont',img)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break