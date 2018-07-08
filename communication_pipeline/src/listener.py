#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from std_msgs.msg import String

'''
A python script to practice receiving ROS messages
'''

class Listener():
    ''' Subscribes to ROS messages
    '''
    def __init__(self):

        # subscription objects
        self.chatter_sub = rospy.Subscriber("/chatter", String, self.chatter_callback)


    def chatter_callback(self, msg):
        ''' Function to be run everytime a message is received on chatter topic
        '''

        '''TODO-START: FILL IN CODE HERE 
        * print out the message received to the terminal
        '''
        raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
        '''TODO-END '''

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('listener')
    l_obj = Listener()
    print("Listener node running")
    rospy.spin()
