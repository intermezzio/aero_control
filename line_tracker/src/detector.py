#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from aero_control.msg import Line
import sys

DEBUG = True

class LineDetector:
    def __init__(self):
        raise Exception("CODE INCOMPLETE! Delete this exception and complete the following lines")
        self.sub_cam = # TODO subscribe to downward facing camera, and set callaback to image_cb
        self.pub_param = # TODO create a publisher for line parameterizations
        
        # TODO-START: create a CvBridge instance
        raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
        # TODO-END

    def parameterizeLine(self, cv_image):
        """ Fit a line to LED strip
        :param cv_image: opencv image
        """
        # TODO-START: run linear regression to parameterize the line
        raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
        # TODO-END
        
        if DEBUG:
            # TODO-OPTIONAL-START: publish an image showing your line on top of the LED strip image
            pass # delete ths line and add your code if you want debug images
            # TODO-OPTIONAL-END

        # TODO-START: return x, y, vx, and vy in that order
        raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
        # TODO-END

    def image_cb(self, data):
        line = self.segmentLine(self.bridge.imgmsg_to_cv2(data, "8UC1"))
        if line is not None:
            x, y, vx, vy = line
            # TODO-START: create a line message and publish it with x, y, vx, and vy
            raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
            # TODO-END
            

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('line_detector', anonymous=True)

    ld = LineDetector()
    rospy.loginfo("Line detector initialized")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    except Exception as e:
        print(e)
