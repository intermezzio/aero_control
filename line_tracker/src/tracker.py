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



WINDOW_HEIGHT = 128
WINDOW_WIDTH = 128
NO_ROBOT = True # set to True to test on laptop
MAX_SPEED = .5 # [m/s]
K_P_X = 1.0 # TODO: decide upon initial K_P_X
K_P_Y = 1.0 # TODO: decide upon initial K_P_Y
K_P_YAW = 0.1
num_unit_vecs = 50
_TIME_STEP = 0.1
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
        self.pub_error = rospy.Publisher("/line/error", Vector3, queue_size=1)



        # Variables dealing with publishing setpoint
        self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.current_state = None
        self.offboard_point_streaming = False

        # Setpoint field expressed as the desired velocity of the body-down frame
        #  with respect to the world frame parameterized in the body-down frame
        self.velocity_setpoint = None

        # while not rospy.is_shutdown() and self.current_state == None:
        #     pass  # Wait for connection

    def line_param_cb(self, line_params):
        mode = getattr(self.current_state, "mode", None)
        if mode not in (None, "MANUAL") or NO_ROBOT:
            

            """ Map line paramaterization to a velocity setpoint so the robot will approach and follow the LED strip
            
            Note: Recall the formatting of a Line message when dealing with line_params

            Recomended Steps: 
            
            Read the documentation at https://bwsi-uav.github.io/website/line_following.html

            After calculating your various control signals, place them in self.velocity_setpoint (which
                is a TwistStamped, meaning self.velocity_setpoint.twist.linear.x is x vel for example)

            Be sure to publish your error using self.pub_error.publish(Vector3(x_error,y_error,0))

            """

            # TODO-START: Create velocity controller based on above specs

            img_center_x = 64
            img_center_y = 64

            x = line_params.x
            y = line_params.y
            vx = line_params.vx
            vy = line_params.vy

            px1 = 127
            px2 = 0
            py1 = int(((128-x)*vy/vx)+y)
            py2 = int((-x*vy/vx) + y)

            p_line_center_x = (px1+px2)/2
            p_line_center_y = (py1+py2)/2

            r_line_unit = (vx,vy)

            m = vy/vx
            b = p_line_center_y - m*p_line_center_x

            distances = [20000]
            xs = []
            ys = []
            for x1 in range(0,px1):
                y1 = m*x1 + b
                dist = np.sqrt((x1 - img_center_x)**2 + (y1 - img_center_y)**2)
                if dist < distances[-1]:
                    distances.append(dist)
                    xs.append(x1)
                    ys.append(y1)

            if vy < 0: #axes are switched LOL
                yaw_angle = -1*(0 - np.arctan(vx/vy))
            if vy > 0:
                yaw_angle = 0 - np.arctan(vx/vy)

            print(np.arctan(vx/vy))



            if len(xs) > 0 and len(ys) > 0:

                p_line_closest_center = (xs[-1],ys[-1])
                p_line_closest_center_x = xs[-1]
                p_line_closest_center_y = ys[-1]

                p_target = (vx+p_line_closest_center_x,vy+p_line_closest_center_y)
                p_target_x = num_unit_vecs*vx+p_line_closest_center_x
                p_target_y = num_unit_vecs*vy+p_line_closest_center_y

                # r_to_target_x,r_to_target_y = (img_center_x + p_target_x, img_center_y + p_target_y) #<----------------------------use these for velocities

                x_err = (-1*img_center_x + p_target_x)   
                y_err = (-1*img_center_y + p_target_y) 

               # if x_err and y_err:
                #    m_thresh = 1000000
                 #   largest_int = 9223372036854775807
                  #  if -1*largest_int < m and m < -1*m_thresh: 
                   #     self.pub_error.publish(Vector3(1.0,y_err,0))
                    #if m_thresh < m and m <largest_int:
                     #   self.pub_error.publish(Vector3(1.0,y_err,0))

                self.pub_error.publish(Vector3(x_err,y_err,0))


                self.p_control(x_err,y_err,yaw_angle)

        # return x_err, y_err


    def actuate_acceleration_command(self, acc_cmd, dt=_TIME_STEP):
        self.__v += acc_cmd*dt
        self.__x += self.__v*dt
    
    def p_control(self,x_err,y_err,yaw_angle):
        self.velocity_setpoint = TwistStamped()
        cmd_x = x_err*(1*K_P_X)
        cmd_y = y_err*(-1*K_P_Y)
        if yaw_angle:
            cmd_yaw = yaw_angle*(-1*K_P_YAW)
            self.velocity_setpoint.twist.angular.z = cmd_yaw


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
                # Publish commands
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
                    self.pub_local_velocity_setpoint.publish(velocity_setpoint_limited)
                    # self.pub_local_velocity_setpoint.publish(velocity_setpoint_limited.twist.angular.z) 
                    rospy.loginfo(velocity_setpoint_limited)
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
    d.start_streaming_offboard_points()
    print("DONE")
    rospy.spin()
d.stop_streaming_offboard_points()
