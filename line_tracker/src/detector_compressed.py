#!/usr/bin/env python

# Python libs
import sys, time
# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2
from aero_control.msg import Line
from copy import deepcopy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage, Image 



class LineDetector:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/aero_downward_camera/image/compressed", CompressedImage, self.image_cb)
        self.pub_param = rospy.Publisher("/line/param", Line, queue_size=1)
        self.image_pub = rospy.Publisher("/image_to_cv/processed", Image, queue_size=1)
        self.bridge = CvBridge()
        '''Initialize ros publisher, ros subscriber'''
         # topic where we publish


    def parameterizeLine(self, img):
        """ Fit a line to LED strip
        :param cv_image: opencv image
        """
        # TODO-START: run linear regression to parameterize the line



        #### direct conversion to CV2 ####
        # arr = self.bridge.imgmsg_to_cv2(img,"8UC1")
        np_arr = np.fromstring(img.data, np.uint8)
        d = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        # d = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)


        upper = 255
        lower = 255

        k_dim = 5
        kernel = np.ones((k_dim,k_dim),np.uint8)

        copy = deepcopy(d) #preserves raw images, but requires more time and processing power    
        copy2 = cv2.inRange(copy,lower,upper)
        copy3 = cv2.morphologyEx(copy2,cv2.MORPH_CLOSE,kernel)
        # d = cv2.resize(d, (0,0), fx=10, fy=10)
# 
        ret,thresh = cv2.threshold(copy3,0,255,0)
        im2,contours,hierarchy = cv2.findContours(thresh, 1, 2)#adds the pixels in the threshold to the list of possible contours
        
        if len(contours) > 0:

            max_contours = max(contours, key = lambda x: cv2.contourArea(x))

            # rows,cols = d.shape[:2]
            # img_center_x, img_center_y = rows/2, cols/2
            # p_frame_center = (img_center_x,img_center_y)

            [vx,vy,x,y] = cv2.fitLine(max_contours, cv2.DIST_L2,0,0.01,0.01)

            lefty = int((-x*vy/vx) + y)
            righty = int(((128-x)*vy/vx)+y)

            pt1 = (127,righty)
            pt2 = (0,lefty)

            regression = cv2.line(copy3,pt1,pt2,(0,255,0),2)
            regression = cv2.line(d,pt1,pt2,(0,255,0),2)


            # TODO-START: return x, y, vx, and vy in that order
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(copy3, "8UC1"))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(d, "8UC1"))
            print(x,y,vx,vy)
            return x,y,vx,vy

        else:
            print(d)

        # TODO-END

    def image_cb(self, data):
        line = self.parameterizeLine(data)
        if line is not None:
            x, y, vx, vy = line
            # TODO-START: create a line message and publish it with x, y, vx, and vy

            msg = Line()
            msg.x = x
            msg.y = y
            msg.vx = vx
            msg.vy = vy


            self.pub_param.publish(msg)

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


