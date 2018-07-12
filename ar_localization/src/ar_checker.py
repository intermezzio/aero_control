#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threading
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker


_Z_THRESH = 0.1

class ARDistChecker:
    def __init__(self):
        rospy.loginfo("ARDistChecker Started!")

        '''
        TODO: Determine how to initialize a subscriber for AR tracking
        '''
        self.ar_pose_sub = None 
        raise Exception("Delete this and fill-in subscriber intialization!")


        self.seen = {}
        self.current_marker = None

    def ar_pose_cb(self,msg):
        '''
        TODO: Filter incoming AR message to determine where drone is relative to tag
        '''
        if len(msg.markers) < 1: 
            return

        marker = None # <-- Fill in (Hint: consider python list filtering functions)
        raise Exception("Delete this and filter AR message!")

        self.current_marker = marker
        self.check_dist()

    def check_dist(self):

        '''
        TODO: Determine how to share marker data with check_dist
        '''
    	marker = None # <-- fill-in
        raise Exception("Delete this and fill-in marker definition!")

        z_des = self.get_dist(marker.id)


        '''
        TODO: Fill in conditionals appropriately to filter marker cases
        '''
        raise Exception("Delete this and fix these conditionals!")
        if False and False:
            rospy.loginfo("Already seen: marker "+ str(marker.id))

        elif False:
            rospy.loginfo("Got it: marker " + str(marker.id) + " captured")
            # MINI TODO: How can we track successful detects? (HINT: add something to this elif block)

        else:
            rospy.loginfo("Not there yet: move " + str(None) + " meters to capture " + str(marker.id))
            # MINI TODO: replace None with a method of calculating distance to AR tag

    def get_dist(self,id):
        return 1.0 # change later


if __name__ == '__main__':
    rospy.init_node('ar_checker')
    a = ARDistChecker()

    rospy.spin()



