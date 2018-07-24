#!/usr/bin/env python

from __future__ import division, print_function

import rospy
import threading
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion, Point, Vector3
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler, quaternion_matrix
from aero_control_staffonly.msg import Line, TrackerParams
import cv2
import mavros
from mavros_msgs.msg import State
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy


WINDOW_HEIGHT = 128
WINDOW_WIDTH = 128
NO_ROBOT = False # set to True to test on laptop
MAX_SPEED = .5 # [m/s]
K_P_X = 0 # TODO: decide upon initial K_P_X
K_P_Y = 0 # TODO: decide upon initial K_P_Y
class LineTracker:
    def __init__(self, rate=10):
        """ Initializes publishers and subscribers, sets initial values for vars
        :param rate: the rate at which the setpoint_velocity is published
        """
        assert rate > 2 # make sure rate is high enough, if new setpoint recieved within .5 seconds, robot will switch back to POSCTL
        self.rate = rospy.Rate(rate)

        mavros.set_namespace()
        self.bridge = CvBridge()

        self.pub_local_velocity_setpoint = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
        self.sub_line_param = rospy.Subscriber("/line/param", Line, self.line_param_cb)
        self.pub_error = rospy.publisher("/line/error", Vector3, queue_size=1)


        # Variables dealing with publishing setpoint
        self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.current_state = None
        self.offboard_point_streaming = False

        # Setpoint field expressed as the desired velocity of the body-down frame
        #  with respect to the world frame parameterized in the body-down frame
        self.velocity_setpoint = None

        while not rospy.is_shutdown() and self.current_state == None:
            pass  # Wait for connection

    def line_param_cb(self, line_params):
        mode = getattr(self.current_state, "mode", None)
        if mode not in (None, "MANUAL") or NO_ROBOT:
            global WINDOW_HEIGHT, WINDOW_WIDTH
            """ Map line paramaterization to a velocity setpoint so the robot will approach and follow the LED strip
            
            Note: Recall the formatting of a Line message when dealing with line_params

            Recomended Steps: 
            
            Read the documentation at https://bwsi-uav.github.io/website/line_following.html

            After calculating your various control signals, place them in self.velocity_setpoint (which
                is a TwistStamped, meaning self.velocity_setpoint.twist.linear.x is x vel for example)

            Be sure to publish your error using self.pub_error.publish(Vector3(x_error,y_error,0))
    
            """
            x, y, vx, vy = line_params
            p_frame_center = np.array([WINDOW_WIDTH/2, WINDOW_HEIGHT/2])
            
            linevec = np.array([vx,vy])
            # TODO-START: Create velocity controller based on above specs
            raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
            # TODO-END

    def state_cb(self, state):
        """ Starts setpoint streamer when mode is "POSCTL" and disables it when mode is "MANUAL"
        :param state: Given by subscribed topic `/mavros/state`
        """
        self.current_state = state
        mode = getattr(state, "mode", None)
        if (mode == "POSCTL" or NO_ROBOT) and not self.offboard_point_streaming:
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
                if (self.vel_setpnt is not None):
                    # limit speed for safety
                    velocity_setpoint_limited = deepcopy(self.velocity_setpoint)
                    speed = np.linalg.norm([velocity_setpoint_limited.linear.x,
                                            velocity_setpoint_limited.linear.y,
                                            velocity_setpoint_limited.linear.z])
                    if speed > MAX_SPEED:
                        velocity_setpoint_limited.linear.x *= MAX_SPEED / speed
                        velocity_setpoint_limited.linear.y *= MAX_SPEED / speed
                        velocity_setpoint_limited.linear.z *= MAX_SPEED / speed

                    # Publish limited setpoint
                    self.vel_setpoint_pub.publish(velocity_setpoint_limited)
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


if __name__ == "__main__":
    rospy.init_node("line_tracker")
    d = LineTracker()
    rospy.spin()
d.stop_streaming_offboard_points()
