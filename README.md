Smart Mimic: An interactive robot experience with a few smart surprises
======================================================================

Based off of Interactive Torso-tracking Enabling Robotic Operation, ITERO, this software allows a user 
to control a HUBO robot by moving their own body in front of a Kinect sensor. The interactivity goes a 
little bit further by having certian gestures that when held for a period of time trigger the robot to respond 
with a dance. HUBO's current favorite dances are the YMCA, Walking like an Egyptian, a disco point, and the Chicken Dance. 

The way that the program recongizes a dance is it runs it through a random forest classifier to attempt to classify the gesture. 
If the gesture is a known dance then the robot responds with the appropriate dance. The classifier is unique in that it was 
trained on and uses calculated joint angles for classification as opposed to normalized joint position data. This allows 
the classifier to be more robust to users that have never before been seen by the system and more reliably handles 
new users. 

This demonstration can be done in simulation at home if you install the necessary dependencies below, have a kinect sensor or any compatible
RGB-Depth sensor and run MAESTOR on your personal computer. 

We will be showing a live interactive demo of this at the Philly Tech Week Signature Event for 2015.

This is still under development. 


Dependencies
============

This project will require a few dependencies:

- MAESTOR
- OpenNi ROS package
- OpenNi_tracker ros package
- scikit learn 
