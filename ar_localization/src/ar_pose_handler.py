#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threading
import numpy as np
import tf
import tf.transformations as tft
from tf.transformations import *

from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, PoseArray, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

import mavros
from mavros_msgs.msg import State
 
_DEBUG = True


# body-up and body-down
R_bu2bd = tft.rotation_matrix(np.pi, (1,0,0))

# downward camera and body-down
R_dc2bd = tft.identity_matrix()

# forward camera and body-down
R_fc2bd = np.array([[0.0, 0.0, 1.0, 0.0],
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]])

# Find inverse rotation matrices 
R_bd2bu = R_bu2bd.T
R_bd2dc = R_dc2bd.T
R_bd2fc = R_fc2bd.T

# Find chained rotation matrices from downward-camera to forward-camera
# NOTE: 'concatenate_matrices' is actually doing a matrix multiplication, 'concatenate' is a 
# bad name for this function, but we didn't make it up. See here:
# https://github.com/davheld/tf/blob/master/src/tf/transformations.py
R_dc2fc = tft.concatenate_matrices(R_bd2fc, R_dc2bd)
R_fc2dc = R_dc2fc.T
R_dc2bu = tft.concatenate_matrices(R_bd2bu, R_dc2bd)
R_bu2dc = R_dc2bu.T
R_fc2bu = tft.concatenate_matrices(R_bd2bu, R_fc2bd)
R_bu2fc = R_fc2bu.T

class ARPoseHandler:
    def __init__(self, hz=60):
        rospy.loginfo("ARPoseHandler Started!")
        self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb)
        self.ar_pose_pub = rospy.Publisher("/ar_aero_pose", AlvarMarkers)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_cb)
        self.poses_msg = PoseArray()
        self.tl = tf.TransformListener()
        self.local_pose = None

    def ar_pose_cb(self,msg):
        self.poses_msg = AlvarMarkers()
        self.poses_msg.markers = [self.process_marker(i) for i in msg.markers]

        if len(self.poses_msg.markers) > 0: 
            if _DEBUG: rospy.loginfo("publishing poses!")

            self.ar_pose_pub.publish(self.poses_msg)
    def process_marker(self, marker):
        if _DEBUG: rospy.loginfo("i'm alive!!, pose generation!")

        p_bu_lenu__lenu = [self.local_pose.position.x,self.local_pose.position.y,self.local_pose.position.z,0.0]
        q_bu_lenu = [self.local_pose.orientation.x,self.local_pose.orientation.y,self.local_pose.orientation.z,self.local_pose.orientation.w]
        
        p_m_fc__fc = [marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z,0.0]
        q_m_fc = [marker.pose.pose.orientation.x,marker.pose.pose.orientation.y,marker.pose.pose.orientation.z,marker.pose.pose.orientation.w]

        p_m_bu__fc = p_m_fc__fc # body up origin is approx same as forward camera origin

        p_m_bu__bu = R_fc2bu.dot(p_m_bu__fc)

        q_fc_bu = quaternion_from_matrix(R_fc2bu)

        q_m_bu = quaternion_multiply(q_fc_bu,q_m_fc) # reverse ordering is intentional!!! (fc_bu * m_fc = m_bu) 
        # think rotation matrices (adjacent ref frames cancel)

        marker.pose.pose.position.x = p_m_bu__bu[0]
        marker.pose.pose.position.y = p_m_bu__bu[1]
        marker.pose.pose.position.z = p_m_bu__bu[2]

        marker.pose.pose.orientation.x = q_m_bu[0]
        marker.pose.pose.orientation.y = q_m_bu[1]
        marker.pose.pose.orientation.z = q_m_bu[2]
        marker.pose.pose.orientation.w = q_m_bu[3]

        return marker

    def local_pose_cb(self, msg):
        self.local_pose = msg.pose

if __name__ == '__main__':
    rospy.init_node('ar_pose_handler')
    a = ARPoseHandler()


    rospy.spin()
