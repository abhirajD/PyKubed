#!/usr/bin/env python

# Author: Shao Zhang, Phil Saltzman, and Eddie Canaan
# Last Updated: 2015-03-13
#
# This tutorial will demonstrate some uses for intervals in Panda
# to move objects in your panda world.
# Intervals are tools that change a value of something, like position,
# rotation or anything else, linearly, over a set period of time. They can be
# also be combined to work in sequence or in Parallel
#
# In this lesson, we will simulate a carousel in motion using intervals.
# The carousel will spin using an hprInterval while 4 pandas will represent
# the horses on a traditional carousel. The 4 pandas will rotate with the
# carousel and also move up and down on their poles using a LerpFunc interval.
# Finally there will also be lights on the outer edge of the carousel that
# will turn on and off by switching their texture with intervals in Sequence
# and Parallel

from direct.showbase.ShowBase import ShowBase
from panda3d.core import AmbientLight, DirectionalLight, LightAttrib
from panda3d.core import NodePath
from panda3d.core import LVector3
from direct.interval.IntervalGlobal import *  # Needed to use Intervals
from direct.gui.DirectGui import *
import sys
import os
# Importing math constants and functions
from math import pi, sin, cos


class CarouselDemo(ShowBase):

    def __init__(self):
        # Initialize the ShowBase class from which we inherit, which will
        # create a window and set up everything we need for rendering into it.
        ShowBase.__init__(self)

        # This creates the on screen title that is in every tutorial
        self.title = OnscreenText(text="Panda3D: Tutorial - Carousel",
                                  parent=base.a2dBottomCenter,
                                  fg=(1, 1, 1, 1), shadow=(0, 0, 0, .5),
                                  pos=(0, .1), scale=.1)

        base.disableMouse()  # Allow manual positioning of the camera
        camera.setPosHpr(0, -8, 2.5, 0, -9, 0)  # Set the cameras' position
                                                # and orientation

        self.keyMap = {
            "left": 0, "right": 0, "forward": 0, "cam-left": 0, "cam-right": 0}

        taskMgr.add(self.startCarousel, "moveTask")

        self.loadModels()  # Load and position our models
        self.setupLights()  # Add some basic lighting
        
        self.accept("escape", sys.exit)
        self.accept("arrow_left", self.setKey, ["left", True])
        self.accept("arrow_right", self.setKey, ["right", True])
        self.accept("arrow_up", self.setKey, ["forward", True])
        self.accept("arrow_left-up", self.setKey, ["left", False])
        self.accept("arrow_right-up", self.setKey, ["right", False])
        self.accept("arrow_up-up", self.setKey, ["forward", False])


    def setKey(self, key, value):
        self.keyMap[key] = value

    def loadModels(self):
        # Load the carousel base
        self.carousel = loader.loadModel("models/carousel_base")
        self.carousel.reparentTo(render)  # Attach it to render


    # Panda Lighting
    def setupLights(self):
        # Create some lights and add them to the scene. By setting the lights on
        # render they affect the entire scene
        # Check out the lighting tutorial for more information on lights
        ambientLight = AmbientLight("ambientLight")
        ambientLight.setColor((1,1,1, 1))
        # directionalLight = DirectionalLight("directionalLight")
        # directionalLight.setDirection(LVector3(0, 8, -2.5))
        # directionalLight.setColor((0.9, 0.8, 0.9, 1))
        # render.setLight(render.attachNewNode(directionalLight))
        render.setLight(render.attachNewNode(ambientLight))

    def startCarousel(self,task):
        if self.keyMap["left"]:
            

        if self.keyMap["right"]:
            
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
