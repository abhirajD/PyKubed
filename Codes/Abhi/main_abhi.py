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
        
    def get_wrist_coordinates(self, joint_points):

        right_x=int(joint_points[PyKinectV2.JointType_WristRight].x)
        right_y=int(joint_points[PyKinectV2.JointType_WristRight].y)
        left_x=int(joint_points[PyKinectV2.JointType_WristLeft].x)
        left_y=int(joint_points[PyKinectV2.JointType_WristLeft].y)
       
        right_x = right_x if right_x < 424 else 423
        right_y = right_y if right_y < 512 else 511
        left_x = left_x if left_x < 424 else 423
        left_y = left_y if left_y < 512 else 511

        right_wrist = [right_x,right_y]
        left_wrist = [left_x,left_y]
        return [right_wrist,left_wrist]

    def neighbourhood(self, array, radius, seed):
        neighbour = np.array(array)
        neighbour *= 0
        temp = np.array(array[seed[1]-radius:seed[1]+radius, seed[0]-radius:seed[0]+radius], dtype = np.uint16)
        # cv2.imshow('hand',temp)
        # ret,temp = cv2.threshold(temp,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        return temp

    def merge(self, array_big, array_small, seed ):
        [a,b] = np.shape(array_small)
        if array_big != None:
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
        return contours[ci],max_area
            
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

    def get_radius(self, joint_points):

        radius_left =int( math.sqrt((int(joint_points[PyKinectV2.JointType_WristLeft].x)-int(joint_points[PyKinectV2.JointType_HandTipLeft].x))**2+(int(joint_points[PyKinectV2.JointType_WristLeft].y)-int(joint_points[PyKinectV2.JointType_HandTipLeft].y))**2))+1
        radius_right =int( math.sqrt((int(joint_points[PyKinectV2.JointType_WristRight].x)-int(joint_points[PyKinectV2.JointType_HandTipRight].x))**2+(int(joint_points[PyKinectV2.JointType_WristRight].y)-int(joint_points[PyKinectV2.JointType_HandTipRight].y))**2))+1
        # print radiuqs_right

        return max(radius_right,radius_left)

    def run(self):
        
        #Initialize variables

        print_frame = None
        depth_frame = np.zeros((424,512), dtype = np.uint16)
        initial = np.zeros((424,512), dtype = np.uint16)
        file = open('feature','w')

        while (True):
            #Inits per cycle
            cmd('cls')                  #Clears the screen
            body_present = 0

            #Get depth frames
            if self._kinect.has_new_depth_frame():
                frame = self._kinect.get_last_depth_frame()
                frame = np.array(frame*9, dtype= np.uint16)
                frame = frame.reshape(424,512)
                depth_frame = np.array(frame)
                cv2.imshow('raw',depth_frame)
                frame = None
            
            #Check if body frames are ready and take the latest one
            if self._kinect.has_new_body_frame():
                self._bodies = self._kinect.get_last_body_frame()

            #Extract Body
            if self._bodies is not None:                
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked:
                        print '::NO Body tracked'             
                        continue 
                        print ':: after_continue'
                    else:
                        #print '::Body tracked'
                        body_present = 1
                        break

            if body_present == 1:
                joints = body.joints 
                #print '::Body tracked'
                # convert joint coordinates to depth space 
                joint_points = self._kinect.body_joints_to_depth_space(joints)

                [right_hand, left_hand] = self.get_hand_coordinates(joint_points)
                [right_wrist, left_wrist] = self.get_wrist_coordinates(joint_points)
                
                # d = 40
                d = self.get_radius(joint_points)

                print_frame = np.zeros(np.shape(depth_frame))
# HAND FILTERING               
                if depth_frame != None: 
                    neighbour = np.array(depth_frame)
                    neighbour *= 0

                    right_hand_filtered = self.neighbourhood(depth_frame,d,right_hand)
                    left_hand_filtered = self.neighbourhood(depth_frame,d,left_hand)
                    
                    cv2.imshow('depth',depth_frame)

                    if right_hand_filtered != None:
                        right_hand_depth = right_hand_filtered[d,d] 
                        right_hand_filtered[right_hand_filtered > right_hand_depth + 1200] = 0
                        # right_hand_filtered[right_hand_filtered < right_hand_depth - 1200] = 0

                        right_hand_filtered_depth_frame = self.merge(neighbour, right_hand_filtered,right_hand)                     
                        neighbour = right_hand_filtered_depth_frame

                    if left_hand_filtered != None:
                        left_hand_depth = left_hand_filtered[d,d] 
                        left_hand_filtered[left_hand_filtered > left_hand_depth + 1200] = 0
                        # left_hand_filtered[left_hand_filtered < left_hand_depth - 1200] = 0
                        
                        left_hand_filtered_depth_frame = self.merge(neighbour, left_hand_filtered,left_hand)                         
                        # ret,left_hand_filtered_depth_frame = cv2.threshold(left_hand_filtered_depth_frame,0,255,cv2.THRESH_OTSU)
                                       
                    hand_filtered = left_hand_filtered_depth_frame
                    hand_filtered_8 = np.array(hand_filtered/255, dtype = np.uint8)
                    # hand_filtered += right_hand_filtered_depth_frame

                    cv2.imshow('final',hand_filtered)
                    cv2.circle(hand_filtered_8,tuple(right_wrist),5,[150,50,255],-1)
                    # cv2.imshow('8-bit', hand_filtered_8)


                    right = np.array(right_hand_filtered/255, dtype = np.uint8)
                    ret , right_binary = cv2.threshold(right,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

                    kernel = np.ones((10,10),np.uint8)
                    right_erosion = cv2.erode(right_binary,kernel,iterations = 1)
                    right_dilation = cv2.dilate(right_erosion,kernel,iterations = 1)
                    right_palm = np.array(right_dilation)
                    cv2.imshow("Right_erode", right_palm)
                    
# SKELETONIZATION
                    # ret, right11 = cv2.threshold(right,0,1,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                    # skel = skeletonize(right11).astype(np.uint8)
                    # skel = skel*255
                    # right[skel > 1] = 255
                    # cv2.imshow('skeleton',right)

                    
# FINGER DETECTION

                    img1,right_hand_contours, hierarchy1 = cv2.findContours(right_binary ,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
                    cnt, area = self.max_area_contour(right_hand_contours)
                    hull = cv2.convexHull(cnt,returnPoints = False)

                    img1, right_palm_contours, hierarchy1 = cv2.findContours(right_palm, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
                    cnt_palm, area_palm = self.max_area_contour(right_palm_contours)

                    

                    match = abs(area - area_palm)
                    match = int(match)
                    print "::MATCH\t"+str(match)

                    defects = cv2.convexityDefects(cnt,hull)
                    drawing = np.zeros(right_hand_filtered.shape,np.uint8)
                    drawing = cv2.cvtColor(drawing,cv2.COLOR_GRAY2RGB)
                    # print defects.shape[0]
                    fingers = 0
                    # s,e,f,d = defects[0,0]
                    # start = tuple(cnt[s][0])
                    # end = tuple(cnt[e][0])
                    # far = tuple(cnt[f][0])
                    # cv2.circle(drawing,start,5,[255,0,255],-1)
                    # cv2.circle(drawing,far,5,[0,0,255],-1)
                    # cv2.line(drawing,start,end,[0,255,0],2)
                    # cv2.circle(drawing,end,5,[0,255,255],-1)
                    avg = 0
                    for i in range(defects.shape[0]):
                        s,e,f,d=defects[i,0]
                        avg +=d
                    avg /= defects.shape[0]

                    for i in range(defects.shape[0]):
                        s,e,f,d = defects[i,0]
                        start = tuple(cnt[s][0])
                        end = tuple(cnt[e][0])
                        far = tuple(cnt[f][0])
                        # d = tuple(cnt[d][0])
                        # print  str(i) + ":" +str(d)
                        if d >= avg/2:
                            fingers = fingers+1
                            cv2.circle(drawing,start,5,[255,0,255],-1)
                            cv2.circle(drawing,far,5,[0,0,255],-1)
                            cv2.line(drawing,start,end,[0,255,0],2)
                            # cv2.circle(drawing,end,5,[0,255,255],-1)

                    # rect = cv2.minAreaRect(cnt)
                    # print rect
                    # box = cv2.boxPoints(rect)
                    # box = np.int0(box)
                    # cv2.drawContours(drawing,[box],0,(100,100,255),1q)
                    print fingers
                    features = [fingers,match]
                    file.write(str(features)+"\n")                    
                    # print right_wrist
                    drawing = cv2.drawContours(drawing,[cnt],-1,150,1)
                    cv2.imshow('right_hand_contours',drawing)
# AREA DETECTION
                    

                cv2.imshow
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                file.close()

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()



HandGestureObject = HandGestureObjectClass();
HandGestureObject.run();