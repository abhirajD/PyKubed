from direct.showbase.ShowBase import ShowBase
from panda3d.core import AmbientLight, DirectionalLight, LightAttrib
from panda3d.core import NodePath
from panda3d.core import LVector3
from direct.interval.IntervalGlobal import *  # Needed to use Intervals
from direct.gui.DirectGui import *
from direct.gui.OnscreenImage import OnscreenImage
from panda3d.core import TransparencyAttrib
# Kinect Libraries
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import sys
import os
import numpy as np
import cv2

# Importing math constants and functions
from math import pi, sin, cos


class CarouselDemo(ShowBase):

    def __init__(self):
        # Initialize the ShowBase class from which we inherit, which will
        # create a window and set up everything we need for rendering into it.
        ShowBase.__init__(self)

        base.disableMouse()  # Allow manual positioning of the camera
        camera.setPos(0, -10, 1)  # Set the cameras' position 
        camera.setHpr(0, 0, 0)                         # and orientation

        self.keyMap = {
            "left": 0, "right": 0, "up": 0, "down": 0}

        taskMgr.add(self.startCarousel, "moveTask")

        imageObject = OnscreenImage(image = 'env_sky.jpg', pos = (-10,0,-10))
        imageObject.setImage('env_sky.jpg')
        imageObject.setTransparency(TransparencyAttrib.MAlpha)

        self.loadModels()  # Load and position our models
        self.setupLights()  # Add some basic lighting

        
        self.accept("escape", sys.exit)
        self.accept("arrow_left", self.setKey, ["left", True])
        self.accept("arrow_right", self.setKey, ["right", True])
        self.accept("arrow_up", self.setKey, ["up", True])
        self.accept("arrow_down", self.setKey, ["down", True])
        self.accept("arrow_left-up", self.setKey, ["left", False])
        self.accept("arrow_right-up", self.setKey, ["right", False])
        self.accept("arrow_up-up", self.setKey, ["up", False])
        self.accept("arrow_down-up", self.setKey, ["down", False])


    def setKey(self, key, value):
        self.keyMap[key] = value

    def loadModels(self):
        # Load the carousel base
        self.carousel = loader.loadModel("models/carousel_base")
        self.carousel.reparentTo(render)  # Attach it to render


    def setupLights(self):
        ambientLight = AmbientLight("ambientLight")
        ambientLight.setColor((1,1,1, 1))
        render.setLight(render.attachNewNode(ambientLight))

    def startCarousel(self,task):
        h = self.carousel.getH()
        p = self.carousel.getP()

        if self.keyMap["left"]:
            self.carouselSpin = self.carousel.setH(h+1)
        if self.keyMap["right"]:
            self.carouselSpin = self.carousel.setH(h-1)
        if self.keyMap["up"]:
            self.carouselSpin = self.carousel.setP(p-1)
        if self.keyMap["down"]:
            self.carouselSpin = self.carousel.setP(p+1)
            
        return task.cont


    def startCarousels(self,task):
        if self.keyMap["left"]:
            angleDegrees = task.time * 100.0
            angleRadians = angleDegrees * (pi / 180.0)
            self.camera.setPos(10 * sin(angleRadians), -10.0 * cos(angleRadians), 2)
            self.camera.setHpr(angleDegrees, 0, 0)

        if self.keyMap["right"]:
            angleDegrees = task.time * 10.0
            angleRadians = angleDegrees * (pi / 180.0)
            self.camera.setPos(10 * sin(angleRadians), -10.0 * cos(angleRadians), 2)
            self.camera.setHpr(angleDegrees, 0, 0)
        return task.cont

demo = CarouselDemo()
demo.run()