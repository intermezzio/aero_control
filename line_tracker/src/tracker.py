#!/usr/bin/env python

from __future__ import division, print_function

import rospy
import threading
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion, Point, Vector3
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler, quaternion_matrix
from aero_control.msg import Line
import cv2
import mavros
from mavros_msgs.msg import State
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy
from PID_control import PIDController as PID


WINDOW_HEIGHT = 128
WINDOW_WIDTH = 128
NO_ROBOT = True # set to True to test on laptop
MAX_SPEED =  0.5# [m/s]
# K_P_X = 60.0 # TODO: decide upon initial K_P_X
# K_P_Y = 60.0 # TODO: decide upon initial K_P_Y
# K_P_YAW = .25
num_unit_vecs = 50
_TIME_STEP = 0.1
_PTS_AHEAD = 50
class LineTracker:
    def __init__(self, rate=10):
        """ Initializes publishers and subscribers, sets initial values for vars
        :param rate: the rate at which the setpoint_velocity is published
        """
        assert rate > 2 # make sure rate is high enough, if new setpoint recieved within .5 seconds, robot will switch back to POSCTL
        self.rate = rospy.Rate(rate) # set rate for updating

        mavros.set_namespace()
        self.bridge = CvBridge()

        # self.pub_local_velocity_setpoint = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1) # send velocity
        self.sub_line_param = rospy.Subscriber("/line/param", Line, self.line_param_cb) # get camera data from this object
        self.pub_error = rospy.Publisher("/line/error", Vector3, queue_size=1) # send error with this object
        self.line_vel = rospy.Publisher("/line_vel", TwistStamped, queue_size=1)


        # Variables dealing with publishing setpoint
        self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_cb) # get drne state
        self.current_state = None
        self.offboard_point_streaming = False

        # Setpoint field expressed as the desired velocity of the body-down frame
        #  with respect to the world frame parameterized in the body-down frame
        self.velocity_setpoint = None # desired velocity

        # while not rospy.is_shutdown() and self.current_state == None:
        #     pass  # Wait for connection
        # create PID controllers


        self.controlX = PID(kp=0.5, ki=0, kd=0)
        self.controlY = PID(kp=0.5, ki=0, kd=0)
        self.controlYAW = PID(kp=1.0, ki=0, kd=0)


    def line_param_cb(self, line_params):
        global WINDOW_HEIGHT, WINDOW_WIDTH
        mode = getattr(self.current_state, "mode", None) # drone state
        if mode not in (None, "MANUAL") or NO_ROBOT:
            # if in pos ctrl or offboard:

            """ Map line paramaterization to a velocity setpoint so the robot will approach and follow the LED strip

            Note: Recall the formatting of a Line message when dealing with line_params

            Recomended Steps:

            Read the documentation at https://bwsi-uav.github.io/website/line_following.html

            After calculating your various control signals, place them in self.velocity_setpoint (which
                is a TwistStamped, meaning self.velocity_setpoint.twist.linear.x is x vel for example)

            Be sure to publish your error using self.pub_error.publish(Vector3(x_error,y_error,0))

            """

            # TODO-START: Create velocity controller based on above specs


            img_center_x = WINDOW_HEIGHT//2 # get image data
            img_center_y = WINDOW_WIDTH//2

            # assign variables for original data


            x = line_params.x
            y = line_params.y
            vx = line_params.vx
            vy = line_params.vy

            # switch to proper coordinates

            x -= img_center_x
            y -= img_center_y
            y *= -1
            
            vx = vx
            vy *= -1

            if vx == 0:
                vx = 0.01


            m = vy/vx
            b = y - m*x

            
            if m == 0:
                m = 0.00001
            if m == -1:
                m = -0.0001


            closeX = -b/(1+1/m)
            closeY = m*closeX + b

            if vx < 0: # change direction of vector if it's going the wrong way
                vx *= -1
                vy *= -1

            extX = closeX + num_unit_vecs * vx # new target x coord
            extY = closeY + num_unit_vecs * vy # new target y coord

            yaw_error = -np.arctan(vy/vx) # yaw angle error

            x_error = x - extX
            y_error = y - extY

            self.pub_error.publish(Vector3(x_error,y_error,0))


            self.control(x_error,y_error,yaw_error)

        # return x_err, y_err


    def actuate_acceleration_command(self, acc_cmd, dt=_TIME_STEP):
        self.__v += acc_cmd*dt
        self.__x += self.__v*dt

    def control(self,x_err,y_err,yaw_err):
        self.velocity_setpoint = TwistStamped() # create p controlled commands


        cmd_x = self.controlX.adjust(x_err)
        cmd_y = self.controlY.adjust(y_err)
        cmd_yaw = self.controlYAW.adjust(yaw_err)

        self.velocity_setpoint.twist.angular.z = cmd_yaw # execute vel commands

        self.velocity_setpoint.twist.linear.x = cmd_x
        self.velocity_setpoint.twist.linear.y = cmd_y
        self.velocity_setpoint.twist.linear.z = 0

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
                # Publish commands when on
                if (self.velocity_setpoint is not None):
                    # limit speed for safety
                    velocity_setpoint_limited = deepcopy(self.velocity_setpoint)
                    speed = np.linalg.norm([velocity_setpoint_limited.twist.linear.x,
                                            velocity_setpoint_limited.twist.linear.y,
                                            velocity_setpoint_limited.twist.linear.z])
                    if speed > MAX_SPEED:
                        velocity_setpoint_limited.twist.linear.x *= MAX_SPEED / speed
                        velocity_setpoint_limited.twist.linear.y *= MAX_SPEED / speed
                        velocity_setpoint_limited.twist.linear.z *= MAX_SPEED / speed

                    # Publish limited setpoint
                    self.line_vel.publish(velocity_setpoint_limited)
                    # self.pub_local_velocity_setpoint.publish(velocity_setpoint_limited.twist.angular.z)
                    rospy.loginfo(velocity_setpoint_limited)
                self.rate.sleep()

        self.offboard_point_streaming_thread = threading.Thread(target=run_streaming) # turns streaming on and adds above method
        self.offboard_point_streaming_thread.start() # start streaming

    def stop_streaming_offboard_points(self):
        """ Safely terminates offboard publisher
        """
        self.offboard_point_streaming = False
        try:
            self.offboard_point_streaming_thread.join()
        except AttributeError:
            pass


if __name__ == "__main__":

    rospy.init_node("line_tracker") # create tracker node
    d = LineTracker() # create line-making object
    d.start_streaming_offboard_points() # basically start code
    print("DONE")
    rospy.spin()
d.stop_streaming_offboard_points()
