from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

from matplotlib import pyplot as plt

import scipy
import numpy as np
import cv2

class HandGestureObject(object):

    def __init__(self):

        # Kinect runtime object, we want only depth and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth) 

    def max_hist_depth(self, frame):    
        #print 'FRAME_MAX = ' + str(frame.max())
        binaries = int(frame.max())
        if binaries <= 0:
            return 0
        histogram, bins = numpy.histogram(frame, bins = binaries)
        histogram = histogram.tolist(); bins = bins.tolist(); 
        histogram[0 : 1] = [0, 0]
        max_hist = bins[ histogram.index( max(histogram) ) ]
        return max_hist

    def run(self):
        previous_frame = None

        while(True):
            #Main event loop
            if self._kinect.has_new_depth_frame():
                raw_frame = self._kinect.get_last_depth_frame()
                frame = raw_frame
                if previous_frame != None:
                    frame=frame.reshape(424,512)    

                    previous_frame=previous_frame.reshape(424,512)  

                    frame_foregnd  = cv2.subtract(frame,previous_frame)                     

                    frame_denoised = scipy.ndimage.morphology.binary_erosion(frame_foregnd,iterations=3)                      

                    frame_foregnd=frame_foregnd.reshape(424,512)   

                    # for i in range(0,200):

                    #    for j in range(0,200):

                    #        frame_foregnd[i,j]=100 
                            
                    cv2.imshow('display',frame_foregnd)

                    print_frame = frame_denoised

                    print_frame >>= 8

                    print_frame = frame.astype(numpy.uint8)

                    

                    self.draw_depth_frame(print_frame, self._frame_surface)

                previous_frame=frame

 

            #Copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio

            #(screen size may be different from Kinect's depth frame size)

            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()

            target_height = int(h_to_w * self._screen.get_width())

            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));

            self._screen.blit(surface_to_draw, (0,0))

            surface_to_draw = None

            pygame.display.update()

 

            # --- Go ahead and update the screen with what we've drawn.

            pygame.display.flip()

 

            # --- Limit to 60 frames per second

            self._clock.tick(60)

 

        # Close our Kinect sensor, close the window and quit.

        self._kinect.close()

        pygame.quit()

 

 

__main__ = "Kinect v2 Body Game"

game = HandGestureObject();

game.run();