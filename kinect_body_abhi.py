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
    
    def neighbourhood_old(self, array, radius, seed, depth):
        # temp = np.nditer(array, flags = ['multi_index'], op_flags = ['readwrite'])
        #cv2.imshow('neigh', array)
        # print 'in neighbour'
        temp = 0
        [a,b] = np.shape(array)
        neighbour = np.array(array)
        neighbour *= 0
        for i in range(seed[0]-radius, seed[0]+radius):
            for j in range(seed[1]-radius, seed[1]+radius):
                temp+=array[j,i]
                if array[j,i] < depth+3:

                    neighbour[j,i] = array[j,i]
                else:
                    neighbour[j,i] = 0

        # cv2.imshow('neigh', array)
        return neighbour,temp/(2*radius+1)^2

    def neighbourhood(self, array, radius, seed, depth):
        [a,b] = np.shape(array)
        neighbour = np.array(array)
        neighbour *= 0
        block = np.array(array[seed[1]-radius:seed[1]+radius, seed[0]-radius:seed[0]+radius], dtype = np.uint8)
        # blur = cv2.GaussianBlur(block,(5,5),0)
        ret,segmented = cv2.threshold(block,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        neighbour[seed[1]-radius:seed[1]+radius, seed[0]-radius:seed[0]+radius] = segmented
        return neighbour

    def max_hist_depth(self, frame):
        binaries = int(frame.max())
        if binaries <= 0:
            return 0
        histogram, bins = np.histogram(frame, bins = binaries)
        histogram = histogram.tolist(); bins = bins.tolist(); 
        histogram[0 : 1] = [0, 0]
        max_hist = bins[histogram.index( max(histogram) )]
        return max_hist

    def run(self):
        print_frame=None

        # -------- Main Program Loop -----------
        while (True):
            # --- Main event loop
           
            if self._kinect.has_new_depth_frame() or self._kinect.has_new_body_frame():

                depth_frame = self._kinect.get_last_depth_frame()
                depth_frame = np.array(depth_frame/16, dtype= np.uint8)
                print depth_frame.max()
                print '_'
                depth_frame = depth_frame.reshape(424,512)                
                self._bodies = self._kinect.get_last_body_frame()

                if self._bodies is not None:
                    body = self._bodies.bodies[0]
                    if not body.is_tracked: 
                        continue 
                    
                    joints = body.joints 
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

                    d = 50

                    if depth_frame != None:
                        right_hand_filtered_depth_frame = self.neighbourhood(depth_frame,d,right_hand,right_hand_depth)
                        left_hand_filtered_depth_frame = self.neighbourhood(depth_frame,d,left_hand,left_hand_depth)
                        
                        print_frame = right_hand_filtered_depth_frame+left_hand_filtered_depth_frame
                        print_frame = cv2.bitwise_and(print_frame,depth_frame)

            if print_frame != None:
                dpt = depth_frame
                cv2.imshow('Hand Filtered',print_frame)
                cv2.imshow('OG',depth_frame)


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self._kinect.close()



HandGestureObject = HandGestureObjectClass();
HandGestureObject.run();