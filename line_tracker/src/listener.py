#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from aero_control.msg import Line
# TODO-START: Import our custom message
# raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
# TODO-END


'''
A python script to practice receiving ROS messages
'''

class Listener():
    ''' Subscribes to ROS messages
    '''
    def __init__(self):
        # TODO-START: subscribe to our custom chatter topic, using chatter_callback as the callback
        self.chatter_sub = rospy.Subscriber("custom_chatter", Line, self.chatter_callback)

        # self.chatter_sub = rospy.Subscriber("/chatter", String, self.chatter_callback)
        # TODO-END

    def chatter_callback(self, msg):
        ''' Function to be run everytime a message is received on chatter topic
        '''
        # TODO-START: print the x, y, vx, and vy values in msg
        # msg = x,y,vx,vy
        print(msg)
        # raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
        # TODO-END

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('listener')
    l_obj = Listener()
    print("Listener node running")
    rospy.spin()
