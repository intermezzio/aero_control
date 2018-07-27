#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threading
import numpy as np
import tf
from tf.transformations import * 
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, PoseArray
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from std_msgs.msg import String

import mavros
from mavros_msgs.msg import State



class ARObstacleController:
    def __init__(self, hz=60):
        rospy.loginfo("ARObstacleController Started!")
        mavros.set_namespace()
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_cb)
        self.local_pose_sp_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.local_vel_sp_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)

        self.ar_pose_sub = rospy.Subscriber("/ar_aero_pose", AlvarMarkers, self.ar_pose_cb)

        self.obstacles = {12 : 3, 24 : 2, 9: 4} # dict (marker -> mode)
        self.rate = rospy.Rate(hz)
        self.current_state = State()
        self.current_pose = None
        self.current_vel = None

        '''
         0 is hovering in space, 
         1 is flying to obstacle 
         2 is ring flythru (marker id: 24)
         3 is hurdle flyover (marker id: 12)
         4 is gate flyunder (marker id: 9)
        '''
        self.finite_state = 0 
        self.markers = []
        self.vel_hist = [[],[],[],[]]
        self.current_obstacle_seq = 0
        self.current_obstacle_tag = None
        self.t_marker_last_seen = None
        self.t_obstacle_start = None

        self.local_vel_sp = TwistStamped()
        self.local_pose_sp = None

        self.offboard_vel_streaming = False

        self.tl = tf.TransformListener()

    def state_cb(self, msg):
        self.current_state = msg

    def ar_pose_cb(self,msg):
        self.markers = msg.markers
    
        self.t_marker_last_seen = datetime.now()

        self.update_finite_state()

    def update_finite_state(self, mode=0, force=False): # updates current phase of avoidance 
        if force:
            self.finite_state = mode
            return


        if self.t_marker_last_seen is not None and self.finite_state < 2:
            self.td = datetime.now() - self.t_marker_last_seen
            if self.td.total_seconds() > 1: 
###########################################################################################################################
# TODO: Decide which finite state to enter when you've lost the AR tags
###########################################################################################################################

                mode = 0
                # raise Exception("Correct the finite state here!")

                self.finite_state = mode

        if mode == 0:
            self.finite_state = mode

        
        if any(marker.id in self.obstacles for marker in self.markers) and self.finite_state == 0:
###########################################################################################################################
# TODO: filter your detections for the best marker you can see (think about useful metrics here!)
###########################################################################################################################
            self.current_obstacle_tag = min(self.markers, key= marker.pose.pose.position.x)
