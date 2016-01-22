from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from matplotlib import pyplot as plt
from PIL import Image 
from skimage import data
from skimage.filters import threshold_otsu, threshold_adaptive
import scipy
import ctypes
import _ctypes
import pygame
import sys
import numpy
from scipy.misc import toimage
from scipy import ndimage 
import cv2

mindepth = 500
maxdepth = 65535
mapdepthtobyte = 8000/256;

class BodyGameRuntime(object):
    def __init__(self):
        # Loop until the user clicks the close button.
        self._done = False
        # Kinect runtime object, we want only depth and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth) 

    def subtract_frames(self,frame,previous_frame):

        #for i in range(0,424):
        #   for j in range(0,512):
        #       frame_foregnd=frame[i,j]-previous_frame[i,j]
        frame_foregnd = numpy.absolute(numpy.subtract(previous_frame,frame))

        # print numpy.shape(frame_foregnd)

        return frame_foregnd

 

    def max_hist_depth(self, frame):    

        #print 'FRAME_MAX = ' + str(frame.max())

        binaries = int(frame.max())

        if binaries <= 0:

            return 0

        histogram, bins = numpy.histogram(frame, bins = binaries)

        histogram = histogram.tolist(); bins = bins.tolist(); 

        histogram[0 : 1] = [0, 0]

        max_hist = bins[ histogram.index( max(histogram) ) ]

        return max_hist

    def run(self):
        exit = 0
        print 'IN_RUN'
        previous_frame = None

        while(True):

            if self._kinect.has_new_depth_frame():
                
                raw_frame = self._kinect.get_last_depth_frame()
                frame = raw_frame

                frame = frame.reshape(424,512)

                if previous_frame != None:
                    frame=frame.reshape(424,512)      
                    previous_frame=previous_frame.reshape(424,512)
                    frame_foregnd  = cv2.subtract(frame,previous_frame)
                    frame_denoised = scipy.ndimage.morphology.binary_erosion(frame_foregnd,iterations=3)                      
                    frame_foregnd=frame_foregnd.reshape(424,512)  
                     
                    cv2.imshow('display',frame_foregnd)

                previous_frame=frame

            if cv2.waitKey(1) & 0xFF ==ord('q'):
                break
            
            
 

        # Close our Kinect sensor, close the window and quit.

        self._kinect.close() 

__main__ = "Kinect v2 Body Game"

game = BodyGameRuntime();

game.run();