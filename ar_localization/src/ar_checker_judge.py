#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threading
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
import datetime
import mavros
from mavros_msgs.msg import State


MARKERS = [6, 13, 4, 1, 2, 5, 7, 8]

##########################################################################################################################
# CONSTANTS
##########################################################################################################################
class Constants:

    _Z_THRESH = 0.1
    PERIOD = 1 # Message will print every _ seconds
    RATE = 1 # hz

##########################################################################################################################
# ARDISTCHECKER
##########################################################################################################################
class ARDistChecker:

    def __init__(self):
        
        # rospy.loginfo("ARDistChecker Started!")

        self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb)

        # A subscriber to the topic '/mavros/state'. self.state is called when a message of type 'State' is recieved
        self.state_subscriber = rospy.Subscriber("/mavros/state", State, self.update_state)
        # State of the drone. self.state.mode = flight mode, self.state.armed = are motors armed (True or False), etc.
        self.state = State()

        # List of all the "captured" markers
        self.captured = []
        # The closest marker to the drone (in the z direction)
        self.marker = None
        # Points to the next marker in Constants.MARKERS that needs to be captured
        self.index = 0

        if len(MARKERS) > 0:
            # True if all markers have been captured, Fasle otherwise
            self.done = False
        else:
            rospy.loginfo("DONE!")
            self.done = True

    def update_state(self, state):
        """
        Callback function which is called when a new message of type State is recieved by self.state_subscriber

            Args:
                - state = mavros State message
        """
        self.state = state

    def ar_pose_cb(self,msg):
        """
        Given an AlvarMArkers message, sets self.current_marker to the closest observed marker (in the z direction) then calls
        the "check_dist" method.

            Args:
                - msg = AlvarMarkers message
        """
        # Check if challenge has been completed (all markers have been seen). If so, do nothing
        if self.done:
            return

        rospy.loginfo_throttle(Constants.PERIOD, 'Move to marker ' + str(MARKERS[self.index]))
        
        if len(msg.markers) > 0: 
            self.marker = min(msg.markers, key = lambda marker: marker.pose.pose.position.z)
            self.check_dist()

    def check_dist(self):
        """
        Determine whether the drone is within the goal distance of the marker and print the appropriate message the the log.

            - The drone has already "seen" the marker: "Marker <marker.id> has already been seen"
        """
        z_des = self.get_dist(self.marker.id)

        # Check if the closest marker is next marker that needs to be captured
        if self.marker.id == MARKERS[self.index]:

            # Drone is within the threshold
            if abs(self.marker.pose.pose.position.z - z_des) < Constants._Z_THRESH:
                rospy.loginfo("CAPTURED!")
                self.captured.append(self.marker.id)
                self.index += 1

                # Check if challenge has been completed
                if len(self.captured) == len(MARKERS):
                    timedelta = datetime.datetime.now() - start_time
                    rospy.loginfo("DONE!")
                    rospy.loginfo("Your time was: " + str(timedelta))
                    self.done = True

            else:
                rospy.loginfo_throttle(Constants.PERIOD, "NOT IN RANGE: move " + str(round(self.marker.pose.pose.position.z - z_des, 2)) + " meters to capture " + str(self.marker.id))



    def get_dist(self,id):
        return 1.0 # change later


if __name__ == '__main__':

    rospy.init_node('ar_checker_judge')
    a = ARDistChecker()

    while not rospy.is_shutdown() and a.state.mode != 'POSCTL':
        
        print('Waiting to enter POSCTL mode')
        
        # Publish at the desired rate
        rospy.Rate(Constants.RATE).sleep()

    start_time = datetime.datetime.now()

    rospy.spin()