#!/usr/bin/env python
# encoding: utf-8


"""Module to connect to a kinect through ROS + OpenNI and access
the skeleton postures.

Author: Ryan
ITERO's original project used the Kinect V2 that only ran on windows. This
uses a regular kinect or asus xtion pro and runs entirely on linux.  

The kinect class was authored by: omangin
"""


import roslib
roslib.load_manifest('smartMimic')
import rospy
import tf
import signal
import numpy as np
from maestor.srv import *
from jsonConverter import jsonMaker
from AngleCalculator import generateAngles
import time

from useClassifier import *
#Include the dances
from dances.Maestor import maestor
from dances.ChickenDance import chickenDanceRobot
from dances.disco import stayAliveRobot
from dances.walkLikeAnEgyptian import walkLikeAnEgyptianRobot
from dances.YMCA import doTheYMCARobot



BASE_FRAME = '/openni_depth_frame'
FRAMES = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'left_hip',
        'left_knee',
        'left_foot',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        'right_hip',
        'right_knee',
        'right_foot',
        ]
LAST = rospy.Duration()

continuing = True




class Kinect:

    def __init__(self, user=1):
        rospy.init_node("Itero", anonymous=True)
        self.listener = tf.TransformListener()
        self.user = user

    
    def get_posture(self):
        """Returns a list of frames constituted by a translation matrix
        and a rotation matrix.

        Raises IndexError when a frame can't be found (which happens if
        the requested user is not calibrated).
        """
        try:
            frames = []
            for frame in FRAMES:
                #Block until there are enough tf frames in the buffer
                self.listener.waitForTransform(BASE_FRAME, "/%s_%d" % (frame, self.user), rospy.Time(0), rospy.Duration(4.0))

                trans, rot = self.listener.lookupTransform(BASE_FRAME,"/%s_%d" % (frame, self.user), rospy.Time(0))

                frames.append((frame, trans, rot))
            return frames
        except (tf.LookupException):
            print "User: " + str(self.user) + " not in frame"
        except (tf.ConnectivityException):
            print "Connectivity Exception"
        except (tf.ExtrapolationException):
            print "ExtrapolationException"
        except (tf.Exception):
            print "You done goofed"

class CopyCat:
    def __init__(self):
        #Wait for the service we want before we publish
        rospy.wait_for_service("setProperties")
        #[REB LEB RSY LSY RSR LSR RSP LSP]
        # Each of these arrays are values for each joint that it matches up with
        self.offsets = [3.14, 3.14, 1.52, 1.52, -.3, .3, 0, 0] #These current values work. Don't touch them
        self.upper = [0, 0, 3, 3, 0, 2.5, 2, 2]
        self.lower = [-2, -2, -3, -3, -2.5, 0, -2, -2]
        self.oldFilter = [0, 0, 0, 0, 0, 0, 0, 0]
        #Used for smoothing
        self.alpha = .3
        #Getting the angles and publishing to hubo
        self.kinect = Kinect()
        self.robot = maestor()
        #Window for dance gesture recognition
        self.slidingWindow = []
        self.capacity = 40

    def publishJointAngles(self, angles):

        #Offset the angles appropriately
        print angles
        stringNums = ""
        for index in xrange(0,8):
            newValue = float(angles[index]) - self.offsets[index]
            #Adjust to keep values in safe joint angles
            if newValue > self.upper[index]:
                newValue = self.upper[index]
            if newValue < self.lower[index]:
                newValue = self.lower[index]

            #Filter the value
            newValue = (self.alpha * newValue) + ((1-self.alpha) * self.oldFilter[index])
            self.oldFilter[index] = newValue

            stringNums += str(newValue) + " "
        #Discard the last space    
        stringNums = stringNums[:-1]
        #Publish the angles
        service = rospy.ServiceProxy("setProperties", setProperties)
        print stringNums
        self.robot.setProperties("RSP RSR RSY LSP LSR LSY REP LEP RWP LWP RWY LWY", "velocity velocity velocity velocity velocity velocity velocity velocity velocity velocity velocity velocity", "3 3 3 3 3 3 3 3 3 3 3 3")
        service("REB LEB RSY LSY RSR LSR RSP LSP", "position position position position position position position position", stringNums)
        #print stringNums


    def checkSlidingWindow(self, f):
        #Get a list of frames and turn it into a list of angles
        values = self.kinect.get_posture()
        if values is None:
            return None
        angleString = jsonMaker((values,))
        angles = generateAngles(angleString)

        f.write(str(angles) + "\n")

        #Add the angle list to the list of angles in the sliding 
        # window list. If the window if above capacity pop the oldest
        # set of angles.
        self.slidingWindow.append(angles)
        if len(self.slidingWindow) > self.capacity:
            self.slidingWindow.pop(0)

        #Let the window fill up before we start trying to classify
        if len(self.slidingWindow) != self.capacity:
            return angles
        #Get the average of the sliding window angles and run them
        # through the classifier
        avgAngles = self.getAvgAngles()

        #Classify the average angles
        result = classifySample(avgAngles)

        print "Result: " + str(result) 

        if result is None:
            return angles

        result = result[0]

        if result == "Disco":
            stayAliveRobot(self.robot)
        elif result == "ChickenDance":
            chickenDanceRobot(self.robot)
        elif result == "WalkLikeAnEgyptian":
            walkLikeAnEgyptianRobot(self.robot)
        elif result == "YMCA":
            doTheYMCARobot(self.robot)

        #Reset the sliding window after a dance was recognized
        self.slidingWindow = []

        return angles


    def getAvgAngles(self):
        #Go through the sliding window and calculate the average angles 
        # for this window
        npArray = np.array(self.slidingWindow)
        avgAngles = np.mean(npArray, axis=0)
        avgAngleList = avgAngles.tolist()

        return avgAngleList



def endDemo(signal, frame):
    global continuing
    continuing = False

def main():
    loadClassifier()
    signal.signal(signal.SIGINT, endDemo)
    mimic = CopyCat()
    f= open("log.log", "w")
    while continuing:
        angles = mimic.checkSlidingWindow(f)
        if angles is None:
            continue
        mimic.publishJointAngles(angles)
        time.sleep(.03)


if __name__ == '__main__':
    main()

 

