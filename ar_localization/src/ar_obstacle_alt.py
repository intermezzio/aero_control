#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threading
import numpy as np
import tf
from tf.transformations import * 
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, PoseArray, Vector3
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from std_msgs.msg import String



import mavros
from mavros_msgs.msg import State


_DEBUG = False

class ARObstacleController:
    def __init__(self, hz=60):
        rospy.loginfo("ARObstacleController Started!")
        mavros.set_namespace()
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_cb)
        self.local_pose_sp_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.local_vel_sp_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
        self.pub_error = rospy.Publisher("/obs_error", Twist, queue_size=1) # set data type to publish to error

        self.ar_pose_sub = rospy.Subscriber("/ar_aero_pose", AlvarMarkers, self.ar_pose_cb)

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
                return

        if mode == 0:
            self.finite_state = mode

        
        if len(self.markers) > 0 and self.finite_state == 0:
###########################################################################################################################
# TODO: filter your detections for the best marker you can see (think about useful metrics here!)
###########################################################################################################################


                self.current_obstacle_tag = min(self.markers, key=lambda marker: marker.pose.pose.position.x).id
                if marker.id % 2 == 0:
		            self.finite_state = 4
		        else:
		            self.finite_state = 3
                
                

            
    def generate_vel(self): # assesses course of action using finite state

        if self.finite_state == 0:
###########################################################################################################################
# TODO: fill in velocity commands for finite state 0
###########################################################################################################################

            self.vel_hist[0].insert(0,0.0)
            self.vel_hist[1].insert(0,0.0)
            self.vel_hist[2].insert(0,0.0)
            self.vel_hist[3].insert(0,0.0)

        elif self.finite_state == 3:
            rospy.logerr("avoiding hurdle")
            #self.current_obstacle_tag = 20
            if self.t_obstacle_start == None:
                self.clear_history(wipe=True) 
                self.t_obstacle_start = datetime.now()

            self.avoid_hurdle()

        elif self.finite_state == 4:
            #self.current_obstacle_tag = 9
            rospy.logerr("avoiding gate")
            if self.t_obstacle_start == None:
                self.clear_history(wipe=True) 
                self.t_obstacle_start = datetime.now()
            self.avoid_gate()

        self.smooth_vel()

        vel = self.local_vel_sp

        if _DEBUG: rospy.loginfo("vel cmd: x: " + "%.05f" % vel.twist.linear.x + " y: " + "%.05f" % vel.twist.linear.y + " z: " + "%.05f" % vel.twist.linear.z + " yaw: " + "%.05f" % vel.twist.angular.z)


    def avoid_hurdle(self): # commands vel such that hurdle can be avoided open-loop
        td = datetime.now()-self.t_obstacle_start

###########################################################################################################################
# TODO: decide how long / at what vel to go up/forward to avoid hurdle
###########################################################################################################################
        t_up = 1
        #t_forward = 1.5

        # raise Exception("hurdle avoid times!")
        if td.total_seconds() < t_up:
            # add to vel_hist here!! (insert at zero)
            dist_hurdle_up = 0.5
            vel_hurdle_up = self.local_vel_sp.twist.linear.z = dist_hurdle_up/t_up
            self.vel_hist[2].insert(0,vel_hurdle_up)
            if _DEBUG: rospy.loginfo("hurdle avoid: going up!")
            rospy.loginfo(current_vel)

        # elif td.total_seconds() < t_forward and td.total_seconds() > t_up:
        #     self.clear_history(z=True)
        #     # add to vel_hist here!! (insert at zero)
        #     dist_hurdle_forward = 2
        #     vel_hurdle_forward = self.local_vel_sp.twist.linear.x = 1
        #     self.vel_hist[0].insert(0,vel_hurdle_forward)
        #     if _DEBUG: rospy.loginfo("hurdle avoid: going forward!")
        #     rospy.loginfo(current_vel)

        else:
            self.clear_history(x=True, z=True)
            self.t_obstacle_start = None
            self.update_finite_state(force=True)

    def avoid_gate(self): # commands vel such that gate can be avoided open-loop
        td = datetime.now()-self.t_obstacle_start

###########################################################################################################################
# TODO: decide how long / at what vel to go down/forward to avoid gate
###########################################################################################################################
        t_down = 1
        if td.total_seconds() < t_down:
            # add to vel_hist here (insert at zero)
            dist_gate_down = -0.5
            vel_gate_down = self.local_vel_sp.twist.linear.z = -dist_gate_down/t_down
            self.vel_hist[2].insert(0,vel_gate_down)            
            if _DEBUG: rospy.loginfo("gate avoid: going down!")
            rospy.loginfo(current_vel)
        else:
            self.clear_history(x=True, z=True)
            self.t_obstacle_start = None
            self.update_finite_state(force=True)


    def smooth_vel(self): # running average to produce smoother movements 
        for i in range(len(self.vel_hist)-1):
            if len(self.vel_hist[i]) > 19:
                self.vel_hist[i].pop()

        if len(self.vel_hist[3]) > 5:
            self.vel_hist[3].pop()
    
    
        x_vel = sum(self.vel_hist[0])/len(self.vel_hist[0]) if len(self.vel_hist[0])>0 else 0

        y_vel = sum(self.vel_hist[1])/len(self.vel_hist[1]) if len(self.vel_hist[1])>0 else 0

        z_vel = sum(self.vel_hist[2])/len(self.vel_hist[2]) if len(self.vel_hist[2])>0 else 0

        yaw_vel = sum(self.vel_hist[3])/len(self.vel_hist[3]) if len(self.vel_hist[3]) > 0 else 0


        self.local_vel_sp.twist.linear.x = x_vel
        self.local_vel_sp.twist.linear.y = y_vel
        self.local_vel_sp.twist.linear.z = z_vel
        self.local_vel_sp.twist.angular.z = yaw_vel

    def clear_history(self,x=False, y=False, z=False, yaw=False, wipe=False): # clears vel hist (helpful at transitions)
        if wipe:
            self.vel_hist = [[],[],[],[]]
            return
        if x:
            self.vel_hist[0] = [0 for i in self.vel_hist[0]]
        if y:
            self.vel_hist[1] = [0 for i in self.vel_hist[1]]
        if z:
            self.vel_hist[2] = [0 for i in self.vel_hist[2]]
        if yaw:
            self.vel_hist[3] = [0 for i in self.vel_hist[3]]



###########################################################################################################################
# DO NOT MODIFY BELOW THIS COMMENT
###########################################################################################################################
               
    def start_streaming_offboard_vel(self):
        def run_streaming():
            self.offboard_vel_streaming = True
	    print(self.current_state.mode)
            while not rospy.is_shutdown() and self.current_state.mode == 'OFFBOARD':
        
        # Publish a "don't move" velocity command
                velocity_message = TwistStamped()
                self.local_vel_sp_pub.publish(velocity_message)
                rospy.loginfo('Waiting to enter offboard mode')
                rospy.Rate(60).sleep()

        # Publish at the desired rate
            while (not rospy.is_shutdown()) and self.offboard_vel_streaming:
                self.update_finite_state()
                vel = TwistStamped()
                self.generate_vel()
            # create a vel setpoint based on 1the vel setpoint member variable
                vel = self.local_vel_sp
           
            # Create a zero-velocity setpoint
            # vel = Twist()    
                self.local_vel_sp_pub.publish(vel)
                self.rate.sleep()

        self_offboard_vel_streaming_thread = threading.Thread(target=run_streaming)
        self_offboard_vel_streaming_thread.start()


    def stop_streaming_offboard_vel(self):
        self.offboard_vel_streaming = False
        try:
            self_offboard_vel_streaming_thread.join()
        except AttributeError:
            pass

    def local_pose_cb(self,msg):
        self.current_pose = msg

if __name__ == '__main__':

    rospy.init_node('ar_obstacle')
    a = ARObstacleController()

    a.start_streaming_offboard_vel()

    rospy.spin()
