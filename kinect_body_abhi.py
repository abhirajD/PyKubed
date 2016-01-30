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
        temp = np.nditer(array, flags = ['multi_index'], op_flags = ['readwrite'])
        [a,b] = np.shape(array)
        neighbour = np.zeros(a*b)
        neighbour = neighbour.reshape(a,b)
        for i in range(seed[0]-radius, seed[0]+radius):
            for j in range(seed[1]-radius, seed[1]+radius):
                neighbour[i,j] = array[i,j]
        return neighbour

    def depth_average(self, array, radius,seed):
        array = self.neighbourhood(array, radius, seed)
        average = sum(sum(array))/sum(sum(array > 0))
        return average
    
    def run(self):
        print_frame=None

        # -------- Main Program Loop -----------
        while (True):
            # --- Main event loop
           
            old_right_x = 0
            old_right_y = 0
            old_left_x = 0
            old_left_y = 0
            old_depth_right_hand = 0
            old_depth_left_hand = 0

            if self._kinect.has_new_depth_frame() or self._kinect.has_new_body_frame():

                depth_frame = self._kinect.get_last_depth_frame()
                print 'Max'+str(depth_frame.max())
                depth_frame = depth_frame.reshape(424,512)                
                print_frame = depth_frame
                self._bodies = self._kinect.get_last_body_frame()

                # --- draw skeletons to _frame_surface
                if self._bodies is not None:
                    for i in range(0, self._kinect.max_body_count):
                        body = self._bodies.bodies[i]
                        if not body.is_tracked: 
                            continue 
                        
                        joints = body.joints 
                        # convert joint coordinates to color space 
                        joint_points = self._kinect.body_joints_to_depth_space(joints)

                        right_x = int(joint_points[PyKinectV2.JointType_HandRight].x)
                        right_y = int(joint_points[PyKinectV2.JointType_HandRight].y)
                        left_x = int(joint_points[PyKinectV2.JointType_HandLeft].x)
                        left_y = int(joint_points[PyKinectV2.JointType_HandLeft].y)
                        
                        # Bounding coordinate values
                        right_x = right_x if right_x < 424 and right_x > 0 else old_right_x
                        right_y = right_y if right_y < 512 and right_y > 0 else old_right_y
                        left_x = left_x if left_x < 424 and left_x > 0 else old_left_x
                        left_y = left_y if left_y < 512 and left_y > 0 else old_left_y

                        old_right_x = right_x
                        old_right_y = right_y
                        old_left_x = left_x
                        old_left_y = left_y
                        
                        temp = depth_frame
                        tdepth_right_hand = self.depth_average(temp, 3, [right_x, right_y])
                        depth_right_hand = tdepth_right_hand if tdepth_right_hand > 1 else old_depth_right_hand
                        temp = depth_frame
                        tdepth_left_hand = self.depth_average(temp, 3, [left_x, left_y])
                        depth_left_hand = tdepth_left_hand if tdepth_left_hand > 1 else old_depth_left_hand
                        
                        old_depth_right_hand = depth_right_hand
                        old_depth_left_hand = depth_left_hand                        
                        
                        print 'ld['+str(left_x)+','+str(left_y)+']:'+str(depth_left_hand)+ '\trd['+str(right_x)+','+str(right_y)+']:'+str(depth_right_hand)

                        hand_filtered_depth_frame = np.where(depth_frame < (depth_right_hand + 20), depth_frame, 0)
                        hand_filtered_depth_frame = np.where(depth_frame > (depth_right_hand - 20), depth_frame, 0)
                        # hand_filtered_depth_frame = np.where(hand_filtered_depth_frame > 0, 65535, 0)

                        print_frame = depth_frame
                        print_frame = cv2.circle(print_frame,(right_x,right_y), 10,(255,0,0),5)
                        print_frame = cv2.circle(print_frame,(left_x,left_y), 10,(255,0,0),5)


                    dpt = depth_frame
                    cv2.imshow('Depthimage',32*print_frame)
                    cv2.imshow('OG',32*dpt)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


            # --- Limit to 60 frames per second
           

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()



HandGestureObject = HandGestureObjectClass();
HandGestureObject.run();