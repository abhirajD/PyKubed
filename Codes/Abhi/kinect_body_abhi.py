from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from matplotlib import pyplot as plt
import scipy
import numpy as np
import cv2

class HandGestureObjectClass(object):
    def __init__(self):
        
        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Body)

        # here we will store skeleton data 
        self._bodies = None    
    

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
        # ci = 0
        for i in range(len(contours)):
            cnt=contours[i]
            area = cv2.contourArea(cnt)
            if(area>max_area):
                max_area=area
                ci=i
        return contours[ci]
    
    def run(self):
        print_frame=None

        # -------- Main Program Loop -----------
        while (True):
            # --- Main event loop
           
            if self._kinect.has_new_depth_frame() or self._kinect.has_new_body_frame():

                depth_frame = self._kinect.get_last_depth_frame()

                depth_frame = np.array(depth_frame/16, dtype= np.uint8)
                depth_frame = depth_frame.reshape(424,512)

                self._bodies = self._kinect.get_last_body_frame()

                if self._bodies is not None:
                    #first detected body taken
                    body = self._bodies.bodies[0]   
                    if not body.is_tracked: 
                        continue 
                    
                    joints = body.joints 

                    # convert joint coordinates to color space 
                    joint_points = self._kinect.body_joints_to_depth_space(joints)

                    right_x=int(joint_points[PyKinectV2.JointType_HandRight].x)
                    right_y=int(joint_points[PyKinectV2.JointType_HandRight].y)
                    left_x=int(joint_points[PyKinectV2.JointType_HandLeft].x)
                    left_y=int(joint_points[PyKinectV2.JointType_HandLeft].y)
                   
                    right_x = right_x if right_x < 424 else 423
                    right_y = right_y if right_y < 512 else 511
                    left_x = left_x if left_x < 424 else 423
                    left_y = left_y if left_y < 512 else 511

                    right_hand_depth = depth_frame[right_x,right_y]
                    left_hand_depth = depth_frame[left_x,left_y]
                    print 'ld:' + str(left_hand_depth)+'\trd:' + str(right_hand_depth)
                   
                    right_hand = [right_x,right_y]
                    left_hand = [left_x,left_y]

                    #print type(c)

                    d = 50
                    if depth_frame != None:
                        right_hand_filtered = self.neighbourhood(depth_frame,d,right_hand)
                        left_hand_filtered = self.neighbourhood(depth_frame,d,left_hand)

                        kernel = np.ones((3,3),np.uint8)
                        right_hand_filtered = cv2.erode(right_hand_filtered, kernel,iterations = 1)
                        left_hand_filtered = cv2.erode(left_hand_filtered, kernel,iterations = 1)

                        neighbour = np.array(depth_frame)
                        neighbour *= 0

                        print_frame = np.zeros(np.shape(depth_frame))



                        if right_hand_filtered != None:

                            img1,contours1, hierarchy1 = cv2.findContours(right_hand_filtered,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
                            cnt = self.max_area_contour(contours1)
                            hull = cv2.convexHull(cnt)
                            drawing = np.zeros(right_hand_filtered.shape,np.uint8)
                            drawing = cv2.drawContours(drawing,[cnt],0,150,1)
                            drawing = cv2.drawContours(drawing,[hull],0,200,1)
                            cv2.imshow('contours1',drawing)

                            right_hand_filtered_depth_frame = cv2.bitwise_and(self.merge(neighbour, right_hand_filtered,right_hand),depth_frame)
                            
                            ret,right_hand_filtered_depth_frame = cv2.threshold(right_hand_filtered_depth_frame,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                            print_frame += right_hand_filtered_depth_frame
                        
                        if left_hand_filtered != None:
                            

                            left_hand_filtered_depth_frame = cv2.bitwise_and(self.merge(neighbour, left_hand_filtered, left_hand),depth_frame)                            
                            ret,left_hand_filtered_depth_frame = cv2.threshold(left_hand_filtered_depth_frame,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                            print_frame += left_hand_filtered_depth_frame

            if print_frame != None:
                dpt = depth_frame
                cv2.imshow('Hand Filtered',print_frame)
            


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


            # --- Limit to 60 frames per second
           

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()



HandGestureObject = HandGestureObjectClass();
HandGestureObject.run();