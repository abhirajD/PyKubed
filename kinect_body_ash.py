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

                # --- draw skeletons to _frame_surface
                if self._bodies is not None:
                    for i in range(0, self._kinect.max_body_count):
                        body = self._bodies.bodies[i]
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
                        
                        hand_filtered_depth_frame = np.where(depth_frame < (left_hand_depth + 2), depth_frame, 0)
                        hand_filtered_depth_frame = np.where(depth_frame > (left_hand_depth - 2), depth_frame, 0)
                        # hand_filtered_depth_frame = np.where(hand_filtered_depth_frame > 0, 65535, 0)

                        print_frame=hand_filtered_depth_frame
                        # print_frame=cv2.circle(print_frame,(right_x,right_y), 10,(255,0,0),5)
                        print_frame=cv2.circle(print_frame,(left_x,left_y), 10,(255,0,0),5)

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