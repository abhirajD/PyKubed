from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

from matplotlib import pyplot as plt

import scipy
import numpy as np
import cv2

class HandGestureObjectClass(object):

    def __init__(self):

        # Kinect runtime object, we want only depth and body depth_frames
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
        previous_depth_frame = None
        cx=0
        cy=0


        while(True):
            #Main event loop
            if self._kinect.has_new_depth_frame() and self._kinect.has_new_color_frame:

                depth_frame = self._kinect.get_last_depth_frame()

                depth_frame = depth_frame.reshape(424,512)

                color_frame = self._kinect.get_last_color_frame()
                #print color_frame.shape()
                #color_frame = color_frame.reshape(424,512)

                if previous_depth_frame != None and not np.array_equal(depth_frame,previous_depth_frame):
                    
                    # Foreground Detection
                    depth_frame_foregnd  = cv2.subtract(depth_frame,previous_depth_frame)
                    depth_frame_denoised = np.where(depth_frame_foregnd>=100,depth_frame_foregnd,0)
                    
                    # Denoising by erosion
                    kernel = np.ones((5,5),np.uint8)                    
                    depth_frame_denoised = cv2.erode(depth_frame_denoised,kernel,iterations=1)
                    depth_frame_denoised = cv2.dilate(depth_frame_denoised,kernel,iterations=1)
                    
                    # Depth depth_frame XOR Denoised depth_frame
                    depth_frame_xored = np.where(depth_frame_denoised != 0, previous_depth_frame, 0)
                    
                    # Depth of the closest object
                    hand_depth = self.max_hist_depth(depth_frame_xored)
                    print "Hand Depth: " + str(hand_depth)
                    hand_filtered_depth_frame = np.where(depth_frame> (hand_depth + 20),0 , depth_frame)
                    hand_filtered_depth_frame = np.where(hand_filtered_depth_frame < (hand_depth - 20), 0 , hand_filtered_depth_frame)

                    im = np.array(hand_filtered_depth_frame * 255, dtype = np.uint8)

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
                            
                            
                    thresh = cv2.circle(thresh,(cx,cy), 10,(255,0,0),1)
                  

                    #Printing depth_frame
                    hand_filtered_depth_frame=thresh
                    hand_filtered_depth_frame *= 32
                    cv2.imshow('Kinect',hand_filtered_depth_frame)
                    # cv2.imshow('color_frame',color_frame)

                else:
                    print "Move your hand"
                
                previous_depth_frame = depth_frame

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()


HandGestureObject = HandGestureObjectClass();
HandGestureObject.run();