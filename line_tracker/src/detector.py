#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy
# from shapely.geometry import LineString, Point
# from aero_control.msg import
from aero_control.msg import Line
import sys

DEBUG = True

class LineDetector:
    def __init__(self):
        self.sub_cam = rospy.Subscriber("/aero_downward_camera/image", Image, self.image_cb)
        self.pub_param = rospy.Publisher("/line/param", Line, queue_size=1)
        self.bridge = CvBridge() 
        # TODO create a publisher for line parameterizations

        # raise Exception("CODE INCOMPLETE! Delete this exception and complete the following lines")
        # self.sub_cam = # TODO: subscribe to downward facing camera, and set callaback to image_cb
        # self.pub_param = # TODO: create a publisher for line parameterizations
        
        # TODO-START: create a CvBridge instance
        # TODO-END

    # def image_cb(self, msg):
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(msg, "8UC1")

    #     except CvBridgeError as e:
    #         print(e)

        # cv2.imshow(cv_image) # shows the image, might be VERY slow (optional)
        # self.parameterizeLine(cv_image)

    def parameterizeLine(self, cv_image):
        """ Fit a line to LED strip
        :param cv_image: opencv image
        """
        # TODO-START: run linear regression to parameterize the line


        upper = 255
        lower = 255

        k_dim = 5
        kernel = np.ones((k_dim,k_dim),np.uint8)

        d = deepcopy(cv_image) #preserves raw images, but requires more time and processing power    
        d = cv2.inRange(d,lower,upper)
        d = cv2.morphologyEx(d,cv2.MORPH_CLOSE,kernel)
        d = cv2.resize(d, (0,0), fx=10, fy=10)

        ret,thresh = cv2.threshold(d,0,255,0)
        im2,contours,hierarchy = cv2.findContours(thresh, 1, 2)#adds the pixels in the threshold to the list of possible contours
        max_contours = max(contours, key = cv2.contourArea)

        rows,cols = d.shape[:2]
        img_center_x, img_center_y = rows/2, cols/2
        p_frame_center = (img_center_x,img_center_y)

        [vx,vy,x,y] = cv2.fitLine(max_contours, cv2.DIST_L2,0,0.01,0.01)
        lefty = int((-x*vy/vx) + y)
        righty = int(((cols-x)*vy/vx)+y)
        regression = cv2.line(d,(cols-1,righty),(0,lefty),(0,255,0),20)


        px1 = cols-1
        px2 = 0
        py1 = righty
        py2 = lefty



        ##||||||||||||||||||||||||||||||||||||||||||||||||||##
        ##VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV## I think this will work for tracker.py



        # print(p_line_closest_center,p_target,x_error,y_error)

        



        # TODO-END
        
        if DEBUG:
            # TODO-OPTIONAL-START: publish an image showing your line on top of the LED strip image
             # delete ths line and add your code if you want debug images

            try:
                # self.pub_param.publish(self.bridge.cv2_to_imgmsg(d, "8UC1"))
                pass

            except CvBridgeError as e:
                print(e)
            # TODO-OPTIONAL-END

        # TODO-START: return x, y, vx, and vy in that order
        return x,y,vx,vy
        # TODO-END

    def image_cb(self, data):
        line = self.parameterizeLine(self.bridge.imgmsg_to_cv2(data, "8UC1"))
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
