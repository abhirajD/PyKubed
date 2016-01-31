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
        # temp = np.nditer(array, flags = ['multi_index'], op_flags = ['readwrite'])
        #cv2.imshow('neigh', array)
        # print 'in neighbour'
        temp = 0
        [a,b] = np.shape(array)
        neighbour = np.zeros(a*b)
        neighbour = neighbour.reshape(a,b)
        for i in range(seed[0]-radius, seed[0]+radius):
            for j in range(seed[1]-radius, seed[1]+radius):
                temp+=array[j,i]
                neighbour[j,i] = array[j,i]
        # cv2.imshow('neigh', array)
        return neighbour,temp/(2*radius+1)^2

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
        print_frame=None

        # -------- Main Program Loop -----------
        while (True):
            # --- Main event loop
           
            if self._kinect.has_new_depth_frame() or self._kinect.has_new_body_frame():

                depth_frame = self._kinect.get_last_depth_frame()
                depth_frame = np.array(depth_frame, dtype= np.uint8)
                print depth_frame.max()
                print '_'
                depth_frame = depth_frame.reshape(424,512)                
                print_frame = depth_frame
                self._bodies = self._kinect.get_last_body_frame()

                # --- draw skeletons to _frame_surface
                if self._bodies is not None:
                    #for i in range(0, self._kinect.max_body_count):
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
                    #print right_x
                    right_x = right_x if right_x < 424 else 423
                    right_y = right_y if right_y < 512 else 511
                    left_x = left_x if left_x < 424 else 423
                    left_y = left_y if left_y < 512 else 511

                    right_hand_depth = depth_frame[right_x,right_y]
                    left_hand_depth = depth_frame[left_x,left_y]
                    print 'ld:' + str(left_hand_depth)+'\trd:' + str(right_hand_depth)
                    
                    
                    # hand_filtered_depth_frame = np.where(depth_frame< (left_hand_depth + 20) ,0 , depth_frame)
                    # hand_filtered_depth_frame = np.where(depth_frame> (left_hand_depth - 20) ,0 ,  depth_frame)
                    # hand_filtered_depth_frame = np.where(hand_filtered_depth_frame>100, 65535, 0)


                    # print_frame=4*depth_frame
                    # print_frame=cv2.circle(print_frame,(right_x,right_y), 10,(255,0,0),5)
                    # print_frame=cv2.circle(print_frame,(left_x,left_y), 10,(255,0,0),5)

                    right_hand = [right_x,right_y]
                    left_hand = [left_x,left_y]

                    #print type(c)

                    d = 30
                    if print_frame != None:
                        right_hand_filtered_depth_frame,avg1 = self.neighbourhood(print_frame,d,right_hand)
                        left_hand_filtered_depth_frame,avg2 = self.neighbourhood(print_frame,d,left_hand)
                        #img_grey = cv2.cvtColor(hand_filtered_depth_frame, cv2.COLOR_BGR2GRAY)
                        right_hand_filtered_depth_frame = np.array(right_hand_filtered_depth_frame/16, dtype = np.uint8)
                        left_hand_filtered_depth_frame = np.array(left_hand_filtered_depth_frame/16, dtype = np.uint8)
                        
                        blur1 = cv2.GaussianBlur(right_hand_filtered_depth_frame,(5,5),0)
                        ret1,thresh1 = cv2.threshold(blur1,right_hand_depth-10,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                        # thresh1 = cv2.adaptiveThreshold(blur1,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,11,2)

                        # kernel = np.ones((3,3),np.uint8)
                        # opening1 = cv2.morphologyEx(thresh1,cv2.MORPH_OPEN,kernel, iterations = 2)

                        # dist_transform1 = cv2.distanceTransform(opening1,cv2.DIST_L2,5)
                        # ret1, sure_fg1 = cv2.threshold(dist_transform1,0.3*dist_transform1.max(),255,0)

                        blur2 = cv2.GaussianBlur(left_hand_filtered_depth_frame,(5,5),0)
                        ret2,thresh2 = cv2.threshold(blur2,avg2,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)


                        # ret1,thresh1 = cv2.threshold(right_hand_filtered_depth_frame,0,255,cv2.THRESH_BINARY)
                        # ret2,thresh2 = cv2.threshold(left_hand_filtered_depth_frame,0,255,cv2.THRESH_BINARY)

                        # thresh = cv2.adaptiveThreshold(img_grey, 0, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
                        print_frame = thresh1+thresh2

                    # it = np.nditer(print_frame, flags=['multi_index'],op_flags=['readwrite'])
                    # while not it.finished:
                    #     p=it.multi_index
                        
                    #     if (p[0]>c[0]+d or p[0]<c[0]-d) or (p[1]>c[1]+d or p[1]<c[1]-d) :
                        
                    #         it[0] = 0
                    #         #print "%d <%s>" % (it[0], it.multi_index),
                    #     it.iternext()

                    # hand_filtered_depth_frame = np.where(depth_frame < (left_hand_depth + 2), depth_frame, 0)
                    # hand_filtered_depth_frame = np.where(depth_frame > (left_hand_depth - 2), depth_frame, 0)
                    # # hand_filtered_depth_frame = np.where(hand_filtered_depth_frame > 0, 65535, 0)
                    # print_frame=cv2.circle(print_frame,(right_x,right_y), 10,(255,0,0),5)
                    # print_frame=cv2.circle(print_frame,(left_x,left_y), 10,(255,0,0),5)


            if print_frame != None:
                dpt = depth_frame
                cv2.imshow('Depthimage',print_frame)
                cv2.imshow('OG',dpt)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


            # --- Limit to 60 frames per second
           

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()



HandGestureObject = HandGestureObjectClass();
HandGestureObject.run();