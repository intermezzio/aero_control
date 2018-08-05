#!/usr/bin/env python

from __future__ import division, print_function

import rospy
import threading
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion, Point, Vector3
from sensor_msgs.msg import Image
from aero_control_staffonly.msg import Line
import cv2
import mavros
from mavros_msgs.msg import State
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy

# Variable Notation:
# v__x_y__z: velocity of "x" frame with respect to "y" frame expressed in "z" coordinates
#
# Frame Subscripts:
# m = marker frame (x-right, y-up, z-out when looking at marker)
# dc = downward-facing camera
# fc = forward-facing camera
# bu = body-up frame (x-forward, y-left, z-up, similar to ENU)
# lenu = local East-North-Up world frame ("local" implies that it may not be aligned with east and north, but z is up)

DEBUG = True
NO_ROBOT = False # allows testing on laptop
CONTROL_YAW = True

class LineTracker:
    _IMAGE_HEIGHT = 128
    _IMAGE_WIDTH = 128

    _P_X = 0.6 / _IMAGE_WIDTH * 2.
    _P_Y = 0.4 / _IMAGE_HEIGHT * 2.
    _P_YAW = np.pi / 8.0

    _MAX_SPEED = 1.0

    def __init__(self, rate=10):
        """ Initializes publishers and subscribers, sets initial values for vars
        :param rate: the rate at which the setpoint_velocity is published
        """
        assert rate > 2 # make sure rate is high enough, if new setpoint recieved within .5 seconds, robot will switch back to POSCTL
        mavros.set_namespace()

        # Variables for `self.line_param_cb`
        self.sub_line_param = rospy.Subscriber("/line/param", Line, self.line_param_cb)
        self.bridge = CvBridge()

        # Variables for more debug output
        if DEBUG:
            self.sub_img = rospy.Subscriber("/line/img", Image, self.img_cb)
            self.img = None
            self.pub_alg = rospy.Publisher("/line/alg", Image, queue_size=1)
            self.pub_error = rospy.Publisher("/line/error", Vector3, queue_size=1)

        # Setpoint field expressed as the desired velocity of the body-down frame
        #  with respect to the world frame parameterized in the body-down frame
        self.vsp__bu_lenu__bu = None

        # Variables for `self.state_cb` and start/stop offboard streaming
        self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.current_state = None
        self.offboard_point_streaming = False
        self.rate = rospy.Rate(rate)
        self.pub_local_velocity_setpoint = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",
                                                           TwistStamped, queue_size=1)

        while not rospy.is_shutdown() and self.current_state == None:
            pass  # Wait for connection

    def line_param_cb(self, line_params):
        mode = getattr(self.current_state, "mode", None)
        if (mode is not None and mode != "MANUAL") or NO_ROBOT:

            # Project center point onto line
            frame_center = np.asarray([self._IMAGE_WIDTH / 2.0, self._IMAGE_HEIGHT / 2.0]) # ctr of screen
            line_center = np.asarray([line_params.x, line_params.y]) # point on line
            slope = np.asarray([line_params.vx, line_params.vy]) # vector parallel to line
            slope = slope / np.linalg.norm(slope) # normalize
            closest = slope * np.dot(slope, frame_center - line_center) + line_center # point on line closest to center
            to_line = closest - frame_center # shortest vector from ctr to line
            line_dist = np.linalg.norm(to_line)  # dist from line to ctr

            # Extrapolate forwards to determine target
            if slope[0] < 0:
                slope *= -1
            target = closest + slope * (self._IMAGE_HEIGHT +self._IMAGE_WIDTH) / 4.0 # 64 pixels fwd
            to_target = target - frame_center

            # Publish distance to target
            error = Vector3()
            error.x = to_target[0]
            error.y = to_target[1]
            error.z = 0


            if DEBUG and self.img is not None:
                # Add additional information to debug image
                self.pub_error.publish(error)

                img = self.bridge.imgmsg_to_cv2(self.img, "rgb8")

                frame_center = (int(frame_center[0]), int(frame_center[1])) # tupleify
                closest = (int(closest[0]), int(closest[1])) # tupleify
                target = (int(target[0]), int(target[1]))  # tupleify

                cv2.line(img=img, pt1=(frame_center), pt2=(closest), color=(255, 0, 0), thickness=1) # projection line
                cv2.circle(img=img, center=(frame_center), radius=3, color=(255, 0, 0), thickness=-1) # center dot
                cv2.circle(img=img, center=(target), radius=4, color=(0, 0, 255), thickness=1) # target
                self.pub_alg.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))

            # Velocity Control
            vel = TwistStamped()

            # + x on camera ==> + x in body_down
            vel.twist.linear.x = self._P_X * to_target[0]

            # + y on camera ==> - y in body_down
            vel.twist.linear.y = -self._P_Y * to_target[1]

            # Yaw Control
            if CONTROL_YAW:
                vel.twist.angular.z = -self._P_Y * slope[1]

            self.vsp__bu_lenu__bu = vel

    def state_cb(self, state):
        """ Starts setpoint streamer when mode is "POSCTL" and disables it when mode is "MANUAL"
        :param state: Given by subscribed topic `/mavros/state`
        """
        self.current_state = state
        mode = getattr(state, "mode", None)
        if (mode == "POSCTL") and not self.offboard_point_streaming:
            rospy.loginfo("Setpoint stream ENABLED")
            self.start_streaming_offboard_points()
        elif mode == "MANUAL" and self.offboard_point_streaming:
            rospy.loginfo("Setpoint stream DISABLED")
            self.stop_streaming_offboard_points()

    def start_streaming_offboard_points(self):
        """ Starts thread that will publish yawrate at `rate` in Hz
        """
        def run_streaming():
            self.offboard_point_streaming = True
            while (not rospy.is_shutdown()) and self.offboard_point_streaming:
                # Publish commands
                if (self.vsp__bu_lenu__bu is not None):
                    # limit speed for safety
                    velocity_setpoint_limited = deepcopy(self.vsp__bu_lenu__bu)
                    speed = np.linalg.norm([velocity_setpoint_limited.twist.linear.x,
                                            velocity_setpoint_limited.twist.linear.y,
                                            velocity_setpoint_limited.twist.linear.z])
                    if speed > self._MAX_SPEED:
                        velocity_setpoint_limited.twist.linear.x *= self._MAX_SPEED / speed
                        velocity_setpoint_limited.twist.linear.y *= self._MAX_SPEED / speed
                        velocity_setpoint_limited.twist.linear.z *= self._MAX_SPEED / speed

                    # Publish limited setpoint
                    self.pub_local_velocity_setpoint.publish(velocity_setpoint_limited)
                self.rate.sleep()

        self.offboard_point_streaming_thread = threading.Thread(target=run_streaming)
        self.offboard_point_streaming_thread.start()

    def stop_streaming_offboard_points(self):
        """ Safely terminates offboard publisher
        """
        self.offboard_point_streaming = False
        try:
            self.offboard_point_streaming_thread.join()
        except AttributeError:
            pass

    def img_cb(self, msg):
        """ Updates debug img
        """
        self.img = msg


if __name__ == "__main__":
    rospy.init_node("line_tracker")
    d = LineTracker()
    d.start_streaming_offboard_points()
    rospy.spin()
d.stop_streaming_offboard_points()
