from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from matplotlib import pyplot as plt
import math
import scipy
import numpy as np
import cv2
import pygame
 

class HandGestureObjectClass(object):
    def __init__(self):
       
        # Kinect runtime object, we want only color and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Body)
 
       
        # here we will store skeleton data
        self._bodies = None
 
 
   
   
    def run(self):
        print_frame=None
 
        print ':IN_RUN:Pulling Frames'
 
 
 
        # -------- Main Program Loop -----------
        while (True):
            # --- Main event loop
           
            if self._kinect.has_new_depth_frame() or self._kinect.has_new_body_frame():
                print ':IN_RUN:depth_frame received'
 
                depth_frame = self._kinect.get_last_depth_frame()
                print_frame = 32*depth_frame.reshape(424,512)                
 
           
                self._bodies = self._kinect.get_last_body_frame()
 
                # --- draw skeletons to _frame_surface
                if self._bodies is not None:
                    print ':IN_RUN:body received'    
                    for i in range(0, self._kinect.max_body_count):
                        body = self._bodies.bodies[i]
                        if not body.is_tracked:
                            continue
                       
                        joints = body.joints
                        # convert joint coordinates to color space
                        joint_points = self._kinect.body_joints_to_depth_space(joints)
                        print ':'
                        rx=joint_points[PyKinectV2.JointType_HandRight].x
                        ry=joint_points[PyKinectV2.JointType_HandRight].y
                        lx=joint_points[PyKinectV2.JointType_HandLeft].x
                        ly=joint_points[PyKinectV2.JointType_HandLeft].y
                        rx=math.floor(rx)
                        ry=math.floor(ry)
                        lx=math.floor(lx)
                        ly=math.floor(ly)
                        print_frame=cv2.circle(print_frame,(int(rx),int(ry)), 10,(255,0,0),5)
                        print_frame=cv2.circle(print_frame,(int(lx),int(ly)), 10,(255,0,0),5)
                        #############
                        # Read image
                        im = cv2.imread("hand.bmp", cv2.IMREAD_GRAYSCALE)
                        #im=print_frame
                        # Setup SimpleBlobDetector parameters.
                        params = cv2.SimpleBlobDetector_Params()
                        # Change thresholds
                        params.minThreshold = 10
                        params.maxThreshold = 200
                        # Filter by Area.
                        #params.filterByArea = True
                        #params.minArea = 1500
                        # Filter by Circularity
                        #params.filterByCircularity = True
                        #params.minCircularity = 0.1
                        # Filter by Convexity
                        #params.filterByConvexity = True
                        #params.minConvexity = 0.87
                        # Filter by Inertia
                        #params.filterByInertia = True
                        #params.minInertiaRatio = 0.01
                        # Create a detector with the parameters
                        ver = (cv2.__version__).split('.')
                        if int(ver[0]) < 3 :
                            detector = cv2.SimpleBlobDetector(params)
                        else :
                            detector = cv2.SimpleBlobDetector_create(params)
                        # Detect blobs.
                        keypoints = detector.detect(im)
                        # Draw detected blobs as red circles.
                        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
                        # the size of the circle corresponds to the size of blob
                        im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                        # Show blobs
                        cv2.imshow("Keypoints", im_with_keypoints)
                        cv2.waitKey(0)
 
 
                if print_frame != None:
 
                    cv2.imshow('Depthimage',print_frame)
 
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
 
 
            # --- Limit to 60 frames per second
           
 
        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
 
 
 
HandGestureObject = HandGestureObjectClass();
HandGestureObject.run();