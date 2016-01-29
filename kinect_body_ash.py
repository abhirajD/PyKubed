from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

from matplotlib import pyplot as plt

import scipy
import numpy as np
import cv2
import pygame


# colors for drawing different bodies 
SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
                  pygame.color.THECOLORS["blue"], 
                  pygame.color.THECOLORS["green"], 
                  pygame.color.THECOLORS["orange"], 
                  pygame.color.THECOLORS["purple"], 
                  pygame.color.THECOLORS["yellow"], 
                  pygame.color.THECOLORS["violet"]]


class HandGestureObjectClass(object):
    def __init__(self):
        
        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Body)

       
        # here we will store skeleton data 
        self._bodies = None


    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1,print_frame):
        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good 
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)

        try:
            print ':drawing bone'

            print_frame = cv2.line(print_frame, start, end, color, 8)
        except: # need to catch it due to possible invalid positions (with inf)
            pass

    def draw_body(self, joints, jointPoints, color,print_frame):
        
        # Right Arm    
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight,print_frame);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight,print_frame);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight,print_frame);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight,print_frame);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_ThumbRight,print_frame);

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft,print_frame);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft,print_frame);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft,print_frame);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft,print_frame);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft,print_frame);

       
    
    def run(self):
        print_frame=None

        print ':IN_RUN:Pulling Frames'



        # -------- Main Program Loop -----------
        while (True):
            # --- Main event loop
           
            if self._kinect.has_new_depth_frame():
                depth_frame = self._kinect.get_last_depth_frame()
                print_frame = 32*depth_frame.reshape(424,512)
                


            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame(): 
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
                    self.draw_body(joints, joint_points, SKELETON_COLORS[i],print_frame)

            if print_frame != None:

                cv2.imshow('Depthimage',print_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


            # --- Limit to 60 frames per second
           

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()



HandGestureObject = HandGestureObjectClass();
HandGestureObject.run();