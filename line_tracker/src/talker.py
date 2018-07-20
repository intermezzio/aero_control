#!/usr/bin/env python

from __future__ import division, print_function

import rospy
# TODO-START: Import our custom message
from aero_control.msg import Line
# raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
# TODO-END

'''
A python script to practice sending ROS messages
'''

class Talker():
    ''' Generates and publishes ROS messages
    '''
    def __init__(self, chat_frequency=1.0):

        
        # raise Exception("CODE INCOMPLETE! Delete this exception and complete the following lines")
        self.chatter_pub = rospy.Publisher("custom_chatter", Line, queue_size=1)
        # TODO subscribe to our custom chatter topic, using chatter_callback as the callback


        # rate of publishing
        self.chat_frequency = rospy.Rate(chat_frequency)

    def start_chatter(self):
        ''' send messages on chatter topic at regular rate
        '''
        i = 0
        while (not rospy.is_shutdown()):
            i = i + 1
            # TODO-START: create and publish a custom message [values can be anything]

            # raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")

            
            chat_msg = Line()
            chat_msg.x = 1
            chat_msg.y = 2
            chat_msg.vx = 3
            chat_msg.vy = 4
            print(chat_msg)

            self.chatter_pub.publish(chat_msg)
            self.chat_frequency.sleep()

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('talker')
    td = Talker()
    print("Talker node running")

    # start the chatter
    td.start_chatter()
    rospy.spin()
