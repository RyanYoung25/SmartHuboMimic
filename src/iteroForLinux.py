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
roslib.load_manifest('IteroForLinux')
import rospy
import tf
import signal
import sys
from maestor.srv import *
from jsonConverter import jsonMaker
from AngleCalculator import generateAngles
import time

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

class ITERO:
    def __init__(self):
        #Wait for the service we want before we publish
        rospy.wait_for_service("setProperties")
        self.offsets = [3.14, 3.14, 0, 0, -.4, .4, 0, 0] 
        self.upper = [0, 0, 2, 2, 0, 2.5, 2, 2]
        self.lower = [-2, -2, -2, -2, -2.5, 0, -2, -2]
        self.oldFilter = [0, 0, 0, 0, 0, 0]
        self.alpha = .3
        self.kinect = Kinect()

    def publishJointAngles(self):
        #Get a list of frames and turn it into a list of angles
        values = self.kinect.get_posture()
        angleString = jsonMaker((values,))
        angles = generateAngles(angleString)

        #Offset the angles appropriately

        index = 0
        stringNums = ""
        print len(angles)
        for val in angles:
            newValue = float(val) - self.offsets[index]
            #Adjust to keep values in safe joint angles
            if newValue > self.upper[index]:
                newValue = self.upper[index]
            if newValue < self.lower[index]:
                newValue = self.lower[index]

            #Filter the value
            newValue = (self.alpha * newValue) + ((1-self.alpha) * self.oldFilter[index])
            self.oldFilter[index] = newValue

            stringNums += str(newValue) + " "
            index += 1
        stringNums = stringNums[:-1]
        #Publish the angles
        service = rospy.ServiceProxy("setProperties", setProperties)
        service("REB LEB RSY LSY RSR LSR", "position position position position position position", stringNums)
        print stringNums

def endDemo(signal, frame):
    global continuing
    continuing = False

def main():
    signal.signal(signal.SIGINT, endDemo)
    itero = ITERO()
    while continuing:
        itero.publishJointAngles()
        time.sleep(.03)


if __name__ == '__main__':
    main()

 

