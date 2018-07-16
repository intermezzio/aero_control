#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threading
import numpy as np
import functools
# from std_msgs.msg import String

from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker


_Z_THRESH = 0.1

class Drone:
    def __init__(self):
        rospy.loginfo("Drone Started!")

        self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker",AlvarMarkers, self.ar_pose_cb)
        self.seen = []
        self.current_marker = None
    def position(self,msg):
        '''
        TODO: Filter incoming AR message to determine where drone is relative to tag
        '''
        # rospy.loginfo("msg.markers: %s"%msg.markers)


        if len(msg.markers) < 1:
            return
        elif len(msg.markers) == 1:

            marker = msg.markers[0]

        else:
            marker = min(msg.markers, key=pose.pose.position.z)

        self.current_marker = marker
        # rospy.loginfo("should start check_dist")
        self.check_dist()

    def check_dist(self):
        '''
        TODO: Determine how to share marker data with check_dist
        '''
    	marker = self.current_marker # <-- fill-in
        #raise Exception("Delete this and fill-in marker definition!")

        z_des = marker.pose.pose.position.z
        # while z_des


        '''
        TODO: Fill in conditionals appropriately to filter marker cases
        '''
        #raise Exception("Delete this and fix these conditionals!")
        if marker.id in self.seen:
            if lastpass is not 1:
                rospy.loginfo("Already seen: marker "+ str(marker.id))
            lastpass=1
        elif abs(z_des-1) < _Z_THRESH:
            if lastpass is not 2:
                rospy.loginfo("Got it: marker " + str(marker.id) + " captured")
            self.seen += [marker.id]
            lastpass=2
            # MINI TODO: How can we track successful detects? (HINT: add something to this elif block)
        else:
            rospy.loginfo("Not there yet: move " +str(z_des-1) + " meters to capture " +str(marker.id))
            lastpass=3
            # MINI TODO: replace None with a method of calculating distance to AR tag

    def pos(self, orientation="x"):
        return self.pos.pos.position[orientation.__name__]


if __name__ == '__main__':
    rospy.init_node('ar_checker')
    a = ARDistChecker()

    rospy.spin()
