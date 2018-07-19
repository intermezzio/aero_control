#!/usr/bin/env python

from __future__ import division, print_function

import rospy
# TODO-START: Import our custom message
raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
# TODO-END

'''
A python script to practice sending ROS messages
'''

class Talker():
    ''' Generates and publishes ROS messages
    '''
    def __init__(self, chat_frequency=1.0):

        
        raise Exception("CODE INCOMPLETE! Delete this exception and complete the following lines")
        self.chatter_pub = # TODO subscribe to our custom chatter topic, using chatter_callback as the callback

        # rate of publishing
        self.chat_frequency = rospy.Rate(chat_frequency)

    def start_chatter(self):
        ''' send messages on chatter topic at regular rate
        '''
        i = 0
        while (not rospy.is_shutdown()):
            i = i + 1
            # TODO-START: create and publish a custom message [values can be anything]
            raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
            # TODO-END
            
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
