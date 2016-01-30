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

    def convert_depthframe2cv(self, frame):
        frame = frame.reshape(424,512)
        frame = np.array(frame/16, dtype = np.uint8)


    def run(self):
        print_frame=None

        # -------- Main Program Loop -----------
        while (True):
            # --- Main event loop
           
            if self._kinect.has_new_depth_frame() or self._kinect.has_new_body_frame():

                depth_frame = self._kinect.get_last_depth_frame()
                print_frame = convert_depthframe2cv(depth_frame)
                print_frame = cv2.cvtColor(print_frame,cv2.COLOR_GRAY2BGR)

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

                        right_x = joint_points[PyKinectV2.JointType_HandRight].x
                        right_y = joint_points[PyKinectV2.JointType_HandRight].y
                        left_x = joint_points[PyKinectV2.JointType_HandLeft].x
                        left_y = joint_points[PyKinectV2.JointType_HandLeft].y


                        print_frame=cv2.circle(print_frame, (int(right_x), int(right_y)), 10, (255,44,255), 5)
                        print_frame=cv2.circle(print_frame, (int(left_x), int(left_y)), 10, (255,44,0), 5)


                if print_frame != None:

                    cv2.imshow('Depthimage',print_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


            # --- Limit to 60 frames per second
           

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()



HandGestureObject = HandGestureObjectClass();
HandGestureObject.run();