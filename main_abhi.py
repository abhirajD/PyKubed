from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

from matplotlib import pyplot as plt

import scipy
import numpy as np
import cv2

class HandGestureObjectClass(object):

    def __init__(self):

        # Kinect runtime object, we want only depth and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth) 

    def max_hist_depth(self, frame):    
        #print 'FRAME_MAX = ' + str(frame.max())
        binaries = int(frame.max())
        if binaries <= 0:
            return 0
        histogram, bins = np.histogram(frame, bins = binaries)
        histogram = histogram.tolist(); bins = bins.tolist(); 
        histogram[0 : 1] = [0, 0]
        max_hist = bins[histogram.index( max(histogram) )]
        return max_hist

    def run(self):
        print ':IN_RUN:Pulling Frames'
        previous_frame = None


        while(True):
            #Main event loop
            if self._kinect.has_new_depth_frame():

                frame = self._kinect.get_last_depth_frame()
                frame = frame.reshape(424,512)

                if previous_frame != None and not np.array_equal(frame,previous_frame):
                    
                    #Foreground Detection
                    frame_foregnd  = cv2.subtract(frame,previous_frame)
                    frame_denoised = np.where(frame_foregnd>=100,frame_foregnd,0)
                    
                    #Denoising by erosion
                    kernel = np.ones((5,5),np.uint8)                    
                    frame_denoised = cv2.erode(frame_denoised,kernel,iterations=1)
                    frame_denoised = cv2.dilate(frame_denoised,kernel,iterations=1)
                    
                    # Depth frame XOR Denoised Frame
                    frame_xored = np.where(frame_denoised != 0, previous_frame, 0)
                    
                    #Depth of the closest object
                    hand_depth = self.max_hist_depth(frame_xored)
                    print "Hand Depth: " + str(hand_depth)
                    hand_filtered_frame = np.where(frame> (hand_depth + 20),0 , frame)
                    hand_filtered_frame = np.where(hand_filtered_frame < (hand_depth - 20), 0 , hand_filtered_frame)

                    ret,thresh = cv2.threshold(hand_filtered_frame,32767,65535,cv2.THRESH_BINARY)
                    contours,hierarchy = cv2.findContours(hand_filtered_frame, 1, 2)
                    
                    cnt = contours[0]
                    M = cv2.moments(cnt)
                    print ':'
                    print M
                    cx = int(M['m10']/M['m00'])
                    print cx
                    cy = int(M['m01']/M['m00'])
                    print cy

                    #Printing Frame
                    hand_filtered_frame *= 32
                    cv2.imshow('Kinect',hand_filtered_frame)

                else:
                    print "Move your hand"
                
                previous_frame = frame

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()


HandGestureObject = HandGestureObjectClass();
HandGestureObject.run();