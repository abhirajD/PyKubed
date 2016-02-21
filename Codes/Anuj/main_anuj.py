from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from matplotlib import pyplot as plt
import scipy
import numpy as np
import cv2
from os import system as cmd

class HandGestureObjectClass(object):
    def __init__(self):
        
        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Body)

        # here we will store skeleton data 
        self._bodies = None    
    
    def get_hand_coordinates(self, joint_points):

        right_x=int(joint_points[PyKinectV2.JointType_HandRight].x)
        right_y=int(joint_points[PyKinectV2.JointType_HandRight].y)
        left_x=int(joint_points[PyKinectV2.JointType_HandLeft].x)
        left_y=int(joint_points[PyKinectV2.JointType_HandLeft].y)
       
        right_x = right_x if right_x < 424 else 423
        right_y = right_y if right_y < 512 else 511
        left_x = left_x if left_x < 424 else 423
        left_y = left_y if left_y < 512 else 511

        right_hand = [right_x,right_y]
        left_hand = [left_x,left_y]
        return [right_hand,left_hand]
        
    def neighbourhood(self, array, radius, seed):       
        neighbour = np.array(array)
        neighbour *= 0
        temp = np.array(array[seed[1]-radius:seed[1]+radius, seed[0]-radius:seed[0]+radius], dtype = np.uint8)
        ret,temp = cv2.threshold(temp,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        return temp

    def merge(self, array_big, array_small, seed ):
        [a,b] = np.shape(array_small)
        array_big[seed[1]-b/2:seed[1]+b/2, seed[0]-a/2:seed[0]+a/2] = array_small
        return array_big
    
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
    
    def max_area_contour(self, contours):
        max_area = 0
        ci = 0
        for i in range(len(contours)):
            cnt=contours[i]
            area = cv2.contourArea(cnt)
            if(area>max_area):
                max_area=area
                ci=i
        return contours[ci]
    
    def min_area_contour(self, contours):
        min_area = 0
        ci = 0
        for i in range(len(contours)):
            cnt=contours[i]
            area = cv2.contourArea(cnt)
            if(area<min_area):
                min_area=area
                ci=i
        return contours[ci]

    def min_depth(self, array):
        return np.amin(array)

    def plot_topview(self,depth_frame1):
        [a,b]=np.shape(depth_frame1)
        topview = np.zeros(1000*b)
        topview =topview.reshape(1000,b)
        for i in range(0,424):
            for j in range(0,b):
                q=depth_frame1[i,j]
                topview[q,j]=65536
        return topview

    def run(self):
        print_frame=None
        #Initialize variables
        depth_frame = np.zeros((424,512), dtype = np.uint16)
        initial = np.zeros((424,512), dtype = np.uint16)
        
        while (True):
            #Inits for every cycle
            cmd('cls')      #Clears the screen
            body_present = 0

            #Get depth frames
            if self._kinect.has_new_depth_frame():
                frame = self._kinect.get_last_depth_frame()
                frame = frame.reshape(424,512)
                frame_8 = np.array(frame/16, dtype= np.uint8)
                depth_frame_16 = np.array(frame)
                depth_frame_16 *= 8
                depth_frame_8 =np.array(frame_8)
                #cv2.imshow('raw',depth_frame_8)
                #cv2.imshow('raw16',depth_frame_16)
                frame = None
                frame_8 = None
            
            #Check if body frames are ready and take the latest one
            if self._kinect.has_new_body_frame():
                self._bodies = self._kinect.get_last_body_frame()
 

            if self._bodies is not None:                
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked:
                        print '::NO Body tracked'             
                        continue 
                        print ':: after_continue'
                    else:
                        print '::Body tracked'
                        body_present = 1
                        break

            if body_present == 1:
                joints = body.joints 
                print '::Body tracked'
                # convert joint coordinates to depth space 
                joint_points = self._kinect.body_joints_to_depth_space(joints)

                [right_hand, left_hand] = self.get_hand_coordinates(joint_points)
                #print depth_frame_8[right_hand]
                right_hand_depth = np.array(depth_frame_16[right_hand[0]-3:right_hand[0]+3,right_hand[1]-3:right_hand[1]+3])
                left_hand_depth = np.array(depth_frame_16[left_hand[0]-3:left_hand[0]+3,left_hand[1]-3:left_hand[1]+3])
                right_hand_depth = np.mean(right_hand_depth)
                left_hand_depth = np.mean(left_hand_depth)
                
                print 'ld:' + str(int(left_hand_depth))+'\trd:' + str(int(right_hand_depth))
                neighbour = np.array(depth_frame_8)
                neighbour *= 0
                d = 1.5*40
                print_frame = np.array(np.zeros(np.shape(depth_frame_16)),dtype=np.uint16)
                #circle = np.array(depth_frame_8)
                #circle = cv2.circle(circle,(right_hand[0],right_hand[1]),3,255,3)
                #cv2.imshow('cc',circle)
                if depth_frame_16 != None: 
                    right_hand_filtered = self.neighbourhood(depth_frame_8,d,right_hand)
                    left_hand_filtered = self.neighbourhood(depth_frame_8,d,left_hand)
                    if right_hand_filtered!=None:
                        right_hand_filtered_depth_frame = np.where(self.merge(neighbour, right_hand_filtered,right_hand) != 0,depth_frame_16,0)
                        neighbour *= 0                            
                        #ret,right_hand_filtered_depth_frame = cv2.threshold(right_hand_filtered_depth_frame,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                    if left_hand_filtered!=None:
                        left_hand_filtered_depth_frame = np.where(self.merge(neighbour, left_hand_filtered,left_hand) != 0,depth_frame_16,0)                            
                        #ret,left_hand_filtered_depth_frame = cv2.threshold(left_hand_filtered_depth_frame,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                        print_frame += left_hand_filtered_depth_frame
                        print_frame += right_hand_filtered_depth_frame
                        #print str(depth_frame_16.shape)+':'+str(print_frame.shape)
                            #print_frame=cv2.bitwise_and(depth_frame_16,print_frame)
                    cv2.imshow('final',print_frame)
                    topview=self.plot_topview(print_frame/66)
                    cv2.imshow('topview',topview)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()



HandGestureObject = HandGestureObjectClass();
HandGestureObject.run();