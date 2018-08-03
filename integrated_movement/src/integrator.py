#!/usr/bin/env python

import numpy
import rospy
from geometry_msgs.msg import Twist, TwistStamped
import threading
import mavros
from mavros_msgs.msg import State

class Integrator:
    def __init__(self):
        rospy.loginfo("Integrator started")
        mavros.set_namespace()

        self.ar_vel = None
        self.line_vel = None

        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)

        self.local_vel_sp_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size = 1)
        self.obst_cmds = rospy.Subscriber("/ar_vel", TwistStamped, self.ar_cb)
        self.line_cmds = rospy.Subscriber("/line_vel", TwistStamped, self.line_cb)
        self.current_state = State()

        self.rate = rospy.Rate(60)
    def state_cb(self,msg):
        self.current_state = msg

    def ar_cb(self, msg): 
        self.ar_vel = msg

    def line_cb(self, msg):
        self.line_vel = msg

    def merge_cmds(self):
        vel = TwistStamped()
        if self.line_vel is not None:
		vel.twist.linear.x = self.line_vel.twist.linear.x
        	vel.twist.linear.y = self.line_vel.twist.linear.y
              	vel.twist.angular.z = self.line_vel.twist.angular.z
        if self.ar_vel is not None:
                vel.twist.linear.z = self.ar_vel.twist.linear.z

        return vel

    def start_streaming_offboard_vel(self):
        def run_streaming():
            self.offboard_vel_streaming = True
            while not rospy.is_shutdown() and self.current_state.mode != 'OFFBOARD':
                # Publish a "don't move" velocity command
                velocity_message = TwistStamped()
                self.local_vel_sp_pub.publish(velocity_message)
                rospy.loginfo('Waiting to enter offboard mode')
                rospy.Rate(60).sleep()

            # Publish at the desired rate
            while (not rospy.is_shutdown()) and self.offboard_vel_streaming:
                
                self.local_vel_sp_pub.publish(self.merge_cmds()) # merge commands and publish
                self.rate.sleep()

        self_offboard_vel_streaming_thread = threading.Thread(target=run_streaming)
        self_offboard_vel_streaming_thread.start()
        

    def stop_streaming_offboard_vel(self):
        self.offboard_vel_streaming = False
        try:
            self_offboard_vel_streaming_thread.join()
        except AttributeError:
            pass

if __name__ == '__main__':

   rospy.init_node('Integrator')
   a = Integrator()

   a.start_streaming_offboard_vel()

   rospy.spin()








