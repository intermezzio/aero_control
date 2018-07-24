#bu vectors ([0,1, 0],[0,0,1],[0,-1,0],[0,0,-1])


#!/usr/bin/env python

import rospy
import threading
import mavros
import datetime
import math

import numpy as np 
import tf.transformations as tft

from copy import deepcopy
from mavros_msgs.msg import State
from geometry_msgs.msg import Twist, PoseStamped


# Maneuver inputs (placed at top for ease of modification)


velocities = [np.array([0.0,1.0, 0.0]),np.array([0.0,0.0,1.0]),np.array([0.0,-1.0, 0.0]),np.array([0.0,0.0, 1.0])]
maneuver_reference_Frame = ['bu','bu','bu','bu']
durations = [2.0,2.0,2.0,2.0]

#########################################################################################################################
# CONSTANTS (DON'T CHANGE)
#########################################################################################################################
class Constants:

    RATE = 10 # [hz]
    MAX_DISTANCE = 2 # [m]
    MAX_SPEED = .5 # [m/s]
    MIN_SPEED = 0.01 # [m/s]

class StaticTransforms():
    # Variable Notation:
    # v__x: vector expressed in "x" frame
    # q_x_y: quaternion of "x" frame with respect to "y" frame
    # p_x_y__z: position of "x" frame with respect to "y" frame expressed in "z" coordinates
    # v_x_y__z: velocity of "x" frame with respect to "y" frame expressed in "z" coordinates
    # R_x2y: rotation matrix that maps vector represented in frame "x" to representation in frame "y" (right-multiply column vec)
    #
    # Frame Subscripts:
    # m = marker frame (x-right, y-up, z-out when looking at marker)
    # dc = downward-facing camera
    # fc = forward-facing camera
    # bu = body-up frame (x-forward, y-left, z-up, similar to ENU)
    # bd = body-down frame (x-forward, y-right, z-down, similar to NED)
    # lenu = local East-North-Up world frame ("local" implies that it may not be aligned with east and north, but z is up)
    # lned = local North-East-Down world frame ("local" implies that it may not be aligned with north and east, but z is down)
    
    # local ENU and local NED
    R_lenu2lned = np.array([[0.0, 1.0, 0.0, 0.0],
                            [1.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, -1.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0]])

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
    R_lned2lenu = R_lenu2lned.T
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
    
    def __init__(self):
        pass #<-----------------------------------------------------------------------------------------------------------------------put rotation stuff here
    
    def coord_transform(self, v__fin, fin, fout):
        ''' transform vector v which is represented in frame fin into its representation in frame fout
        Args:
        - v__fin: 3D vector represented in fin coordinates
        - fin: string describing input coordinate frame (bd, bu, fc, dc, lned, lenu)
        - fout: string describing output coordinate frame (bd, bu, fc, dc, lned, lenu)
        Returns
        - v__fout: vector v represent in fout coordinates
        '''
        
        # trivial transform, checking input shape
        if fin==fout:
            v4__fin = list(v__fin)+[0.0]
            R = tft.identity_matrix()
            v4__fout = np.dot(R, v4__fin)
            v__fout = np.array(v4__fout[0:3])
            return v__fout
        
        # check for existence of rotation matrix
        R_str = 'R_{}2{}'.format(fin, fout)
        try:
            R_i2o = getattr(self, R_str)
        except AttributeError:
            err = 'No static transform exists from {} to {}.'.format(fin, fout)
            err += ' Are you sure these frames are not moving relative to each other?'
            raise AttributeError(err)
        
        # perform transform
        v4__fin = list(v__fin) + [0.0]
        v4__fout = np.dot(R_i2o, v4__fin)
        v__fout = np.array(v4__fout[0:3])
        return v__fout

def get_lenu_velocity(q_bu_lenu, v__fin, fin, static_transforms=None):
        '''tranforms a vector represented in fin frame to vector in lenu frame
        Args:
        - v__fin: 3D vector represented in input frame coordinates
        - fin: string describing input coordinate frame (bd, bu, fc, dc)
        Returns:
        - v__lenu: 3D vector v represented in local ENU world frame
        '''

        # create static transforms if none given
        if static_transforms is None:
            static_transforms = StaticTransforms()

        if fin=='lenu':
            v__lenu = v__fin

        elif fin=='lned':
            v__lenu = static_transforms.coord_transform(v__fin, 'lned', 'lenu')

        else:
            # create rotation matrix from quaternion
            R_bu2lenu = tft.quaternion_matrix(q_bu_lenu)
            
            # represent vector v in body-down coordinates
            v__bu = static_transforms.coord_transform(v__fin, fin, 'bu')
            
            # calculate lenu representation of v
            v__lenu = np.dot(R_bu2lenu, list(v__bu)+[0.0])

        v__lenu = np.array(v__lenu[0:3])
        return v__lenu
 
#########################################################################################################################
# TRANSLATION CONTROLLER
#########################################################################################################################
class TranslationController:

    def __init__(self, maneuver_velocity_setpoint,
                       maneuver_reference_frame,
                       maneuver_duration):
        """ Object that manages velocity commands for OFFBOARD mode
        Attributes:
        - vel_sepoint_pub: rospy publisher for cmd_vel_unstamped topic
        - vel_setpoint_bu_lenu__lenu: Twist message for desired velocity of quadrotor expressed in local ENU coords in m/s
        - state_sub: rospy subscriber for mavros/state topic
        - current_state: State() object to access current flight mode
        - prev_state: State() object to track most recent flight mode before current
        - rate: command rate
        - offboard_point_streaming: boolean for if offboard commands should stream or not
        - static_transforms: StaticTransforms object to hold relative orientation of different frames (bu, bd, fc, etc)
        - maneuver_velocity_setpoint: desired velocity vector for manuever [m/s], related to vel_setpoint_bu_lenu__lenu
        - maneuver_reference_frame: frame in which maneuver_velocity_setpoint is expressed
        - maneuver_duration: [s] duration of maneuver
        """

        # Create node with name 'translation_controller' and set update rate
        rospy.init_node('translation_controller')

        # A publisher which will publish the desired linear and anglar velocity to the topic '/.../cmd_vel_unstamped'
        self.vel_setpoint_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = 1)
        self.vel_setpoint_bu_lenu__lenu = Twist()

        # A subscriber to the topic '/mavros/state'. self.state is called when a message of type 'State' is recieved
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.current_state = State()
        self.prev_state = State()

        # A subscriber to the /mavros/local_position/pose topic that is used to access the transform between the body-up
        # and local ENU frames
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_sub_cb)
        self.q_bu_lenu = None

        self.rate = rospy.Rate(Constants.RATE)
        self.offboard_point_streaming = False
        self.static_transforms = StaticTransforms()
        self.maneuver_velocity_setpoint = maneuver_velocity_setpoint
        self.maneuver_reference_frame = maneuver_reference_frame
        self.maneuver_duration = maneuver_duration
  
    
    def execute_maneuver(self, velsp__fin, fin, duration):
        ''' move at given velocity, described in a given frame, for a given duration
        Args:
        - velsp__fin: [m/s] 3D vector of desired velocity expressed in input reference frame 'fin'
        - fin: [str] string describing reference frame in which velsp is expressed (e.g. 'bu', 'bd', 'lenu', etc)
        - duration: [s] time duration of manuever
        Notes:
        - This function only needs to update the variable self.vel_setpoint_bu_lenu__lenu, it does not actually call the
            publish command. The publish command is constantly being called in run_streaming with whatever
            is currently stored in self.vel_setpoint_bu_lenu__lenu.
        '''

        # timedelta (datetime.timedelta object) is the amount of time the velocity message will be published for
        timedelta = datetime.timedelta(seconds = duration)

        # Record the start time
        start_time = datetime.datetime.now()

        # Create velocity setpoint
        # Note difference: vsp_bu_lenu__lenu is an array, vel_setpoint_bu_lenu__lenu is a Twist message
        # They encode the same velocity information in different package





	'''TODO-START: FILL IN CODE HERE 
        Use the provided functions to calculate the desired velocity of the body-up frame with respect to the 
        local ENU frame, expressed in local ENU coordinates (i.e. vsp_bu_lenu__lenu).
        Encode this in the linear portion of the Twist message and assign to the member variable
        self.vel_setpoint_bu_lenu__lenu     
        '''

        # raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
        velsp_bu_lenu__lenu = get_lenu_velocity(self.q_bu_lenu, velsp__fin, fin, self.static_transforms)
        self.vel_setpoint_bu_lenu__lenu = Twist()
        self.vel_setpoint_bu_lenu__lenu.linear.x = velsp_bu_lenu__lenu[0]
        self.vel_setpoint_bu_lenu__lenu.linear.y = velsp_bu_lenu__lenu[1]
        self.vel_setpoint_bu_lenu__lenu.linear.z = velsp_bu_lenu__lenu[2]
        '''TODO-END '''

        # Publish command velocites for timedelta seconds
        while not rospy.is_shutdown() and datetime.datetime.now() - start_time < timedelta and self.current_state.mode == 'OFFBOARD':

            # Note: we don't actually have to call the publish command here.
            # Publish velocity command is automatically done in run_stremaing function which is running in parrellel
            # and just publishes whatever is stored in self.vel_setpoint_bu_lenu__lenu
            self.rate.sleep()

        # at end of maneuver, set setpoint back to zero
        self.vel_setpoint_bu_lenu__lenu = Twist()
    
    

    def hover(self):
        ''' change setpoint to zero velocity. streaming_offboard_points automatically sends info
        '''
        self.vel_setpoint_bu_lenu__lenu = Twist()

    #################################################################################################################################
    # DO NOT MODIFY
    #################################################################################################################################

    def start_streaming_offboard_points(self):
        """ Starts thread that will publish yawrate at `rate` in Hz
        """
        def run_streaming():
            self.offboard_point_streaming = True
            while (not rospy.is_shutdown()) and self.offboard_point_streaming:
                # Publish commands
                if (self.vel_setpoint_bu_lenu__lenu is not None):

                    # limit speed for safety
                    velsp_limited = deepcopy(self.vel_setpoint_bu_lenu__lenu)
                    speed = np.linalg.norm([velsp_limited.linear.x,
                                            velsp_limited.linear.y,
                                            velsp_limited.linear.z])
                    if speed > Constants.MAX_SPEED:
                        rospy.logwarn("Velocity setpoint too high! Limiting speed to {} m/s".format(Constants.MAX_SPEED))
                        velsp_limited.linear.x *= Constants.MAX_SPEED/speed
                        velsp_limited.linear.y *= Constants.MAX_SPEED/speed
                        velsp_limited.linear.z *= Constants.MAX_SPEED/speed


                    # Publish limited setpoint
                    self.vel_setpoint_pub.publish(velsp_limited)

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

    def pose_sub_cb(self, msg):
        """
        Callback function which is called when a new message of type PoseStamped is recieved by self.position_subscriber.
            Args: 
                - msg = ROS PoseStamped message
        """
        self.q_bu_lenu = [msg.pose.orientation.x,
                          msg.pose.orientation.y,
                          msg.pose.orientation.z,
                          msg.pose.orientation.w]

    def state_cb(self, msg):
        """ callback function for mavros/state messages
        Notes:
            - The purpose of this function to to manage the streaming of setpoints based on which flight mode
            the drone is currently operation. 
        """
        self.prev_state = deepcopy(self.current_state)
        self.current_state = msg

        if self.current_state.mode == "MANUAL":
            if self.offboard_point_streaming:
                rospy.loginfo("Setpoint stream DISABLED")
                self.stop_streaming_offboard_points()

        if self.current_state.mode == "POSCTL":
            if not self.offboard_point_streaming:
                rospy.loginfo("Setpoint stream ENABLED")
                self.start_streaming_offboard_points()
            if not self.prev_state.mode == "POSCTL":
                # just switched into POSCTL, call hover
                self.hover()

        if self.current_state.mode == "OFFBOARD":
            if not self.prev_state.mode == "OFFBOARD":
                # just switched to OFFBOARD, call move
                rospy.loginfo("Entering OFFBOARD Mode")
                for i in range(0,len(velocities)):
                    maneuver_velocity_setpoint=velocities[i]
                    maneuver_reference_frame = maneuver_reference_Frame
                    maneuver_duration=duration[i]
                    self.execute_maneuver(  self.maneuver_velocity_setpoint, 
                                            self.maneuver_reference_frame, 
                                            self.maneuver_duration)
            
    #################################################################################################################################
    #################################################################################################################################

if __name__ == '__main__':

    controller = TranslationController([velocities[i] for i in range(0,len(velocities))], [maneuver_reference_Frame[i] for i in range(0,len(velocities))], [durations[i] for i in range(0,len(velocities))])


    # In order to ter offboard mode, the drone must already be receiving commands
    # TODO: Write code that publishes "don't move" velocity commands until the drone is place into offboard mode
    #######################################
      
    #######################################


    # In order to enter offboard mode, the drone must already be receiving commands
    # TODO: Write code that publishes "don't move" velocity commands until the drone is place into offboard mode


    rospy.spin()

    controller.stop_streaming_offboard_points()
    print('DONE!')

    
