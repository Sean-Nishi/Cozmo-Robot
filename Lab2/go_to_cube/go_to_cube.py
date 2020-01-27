#Lab 2 for Mobile Robotics
#by Sean Nishi and Stanley Chan
#go_to_cube.py with find_cube.py



#!/usr/bin/env python3
#!c:/Python35/python3.exe -u
import asyncio
import sys
import cv2
import numpy as np
import cozmo

#additional imported libraries
from cozmo.util import degrees, distance_mm, speed_mmps

import time
import os
from glob import glob

from find_cube import *

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')
def nothing(x):
    pass

#Stanley's lab1 values
YELLOW_LOWER = np.array([9, 175, 130])
YELLOW_UPPER = np.array([70, 255, 255])

#HSV = Hue, Saturation, Value
#Lab's original values
#YELLOW_LOWER = np.array([9, 115, 151])
#YELLOW_UPPER = np.array([179, 215, 255])

GREEN_LOWER = np.array([0,0,0])
GREEN_UPPER = np.array([179, 255, 60])

# Define a decorator as a subclass of Annotator; displays the keypoint
class BoxAnnotator(cozmo.annotate.Annotator):

    #set default to no cube detected
    cube = None

    #apply itself to an image (from camera) with the scale
    def apply(self, image, scale):
        #d = the frame we will be drawing on
        d = ImageDraw.Draw(image)
        #define the bounds for the image
        bounds = (0, 0, image.width, image.height)

        #if we already have a cube
        if BoxAnnotator.cube is not None:

            #double size of bounding box to match size of rendered image
            BoxAnnotator.cube = np.multiply(BoxAnnotator.cube,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BoxAnnotator.cube[0]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[1]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[2], BoxAnnotator.cube[2])
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)
            #above function puts a box outline around the part of the image we want
            #in the color green. Without any labeling text

            #reset the cube to None so we can detect a cube from a new image
            BoxAnnotator.cube = None


         #run() takes the interface to a cozmo robot as the arg and sets it to "robot"
async def run(robot: cozmo.robot.Robot):

    #disable all annotations from the interface
    robot.world.image_annotator.annotation_enabled = True
    #add your own annotation called 'box'
    robot.world.image_annotator.add_annotator('box', BoxAnnotator)

    #make sure the camera receives image data (enable the camera)
    robot.camera.image_stream_enabled = True
    #make sure you receive color images from the robot
    robot.camera.color_image_enabled = True
    #enabling auto_exposure to constantly update the exposure time and gain values on recent images
    robot.camera.enable_auto_exposure = True

    #current camera gain setting, current camera exposure in ms, and...?
    gain,exposure,mode = 390,3,1

    try:

        while True:
            #current event is the new raw image from the camera that lasts 30ms
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)   #get camera image
            #if we got an image (of course we will)
            if event.image is not None:
                #convert the image into into an array and change the color scheme
                image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)

                #make sure we refresh the image
                if mode == 1:
                    robot.camera.enable_auto_exposure = True
                else:
                    #force the specified exposure time and gain values to the camera
                    robot.camera.set_manual_exposure(exposure,fixed_gain)

                #find the cube (using the current image, and the lower and upper bounds of 'yellow'
                cube = find_cube(image, YELLOW_LOWER, YELLOW_UPPER)
                #print the coords in the terminal or None if no cube is detected
                print(cube)
                #set the returned cube value to BoxAnnotator's cube
                BoxAnnotator.cube = cube

                ################################################################
                # Todo: Add Motion Here
                ################################################################
                #Want to center the cube in cozmo's vision, move so it is close to the center of the image
                #Once cozmo's camera is centered on the cube, move forward.

                #if cozmo doesn't see the cube, rotate 45 degrees
                if BoxAnnotator.cube is None:
                    await robot.turn_in_place(degrees(45)).wait_for_completed()
                     
                if BoxAnnotator.cube is not None:
                    #check to see if we are close enough to the cube
                    if (BoxAnnotator.cube[2] > 90):#was 90 for quarter of the screen
                        print("I am close to the object")
                        robot.stop_all_motors()

                    #cozmo detects cube on the left of the screen, reorient itself
                    elif (BoxAnnotator.cube[0] < 103):
                        print("Cube on left half of screen, reorienting myself")
                        #orient yourself so cube is near the center of the screen
                        await robot.turn_in_place(degrees(15)).wait_for_completed()

                    #cozmo detects cube on right half of screen, reorient itself
                    elif (BoxAnnotator.cube[0] > 206):
                        print("Cube on right half of screen, reorienting myself")
                        #orient yourself so cube is in center of the screen
                        await robot.turn_in_place(degrees(-15)).wait_for_completed()

                    #cube is centered, move towards it
                    elif (BoxAnnotator.cube[0] > 103 and BoxAnnotator.cube[0] < 206):
                        #cube is in center of screen, move forward
                        print("Cube is near the center of the screen, moving forward")
                        await robot.drive_straight(distance_mm(25.4), speed_mmps(25.4), in_parallel = True).wait_for_completed()

                #We can also use cozmo.go_to_object()?
                #no? CUstomObject instances are not supported

                #any keyboard interrupt will exit the program
    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    #if cozmo is doing something, let him finish before doing the next task
    except cozmo.RobotBusy as e:
        print(e)
    #cv2.destroyAllWindows()


if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
