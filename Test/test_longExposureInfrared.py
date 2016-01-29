from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

from matplotlib import pyplot as plt

import scipy
import numpy as np
import cv2

class Kinect_LongExposureInfrared(object):

    def __init__(self):

        # Kinect runtime object, we want only depth and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_LongExposureInfrared) 

    def run(self):
        print ':IN_RUN:Pulling Frames'

        while(True):
            #Main event loop
            if self._kinect.has_new_long_exposure_infrared_frame():

                frame = self._kinect.get_last_long_exposure_infrared_frame()
                # frame = np.array(frame ,dtype = np.uint8)
                frame = frame.reshape(424,512)
                cv2.imshow('Long_Exposure_Infrared',frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()


HandGestureObject = Kinect_LongExposureInfrared();
HandGestureObject.run();