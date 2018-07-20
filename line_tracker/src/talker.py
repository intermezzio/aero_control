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
        pub = rospy.Publisher('custom_chatter', Person)
        rospy.init_node('custom_talker', anonymous=True)

        msg = Line() # Object

        while not rospy.is_shutdown():
            rospy.loginfo(msg)
            pub.publish(msg)
            r.sleep()

        self.chatter_pub = rospy.Publisher("/custom_chatter", String, queue_size=1)
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
