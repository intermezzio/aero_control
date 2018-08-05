#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from aero_control_staffonly.msg import Line
import sys

led_low = 254
led_hig = 255
LENGTH_THRESH = 40
DEBUG = True
USE_COMPRESSED = False
kernel  = np.ones((2,2),np.uint8)

class LineDetector:
    def __init__(self):
        self.bridge = CvBridge()
        if USE_COMPRESSED:
            rospy.loginfo("Using Compressed Image Topic")
            self.sub_cam = rospy.Subscriber("/aero_downward_camera/image/compressed", CompressedImage, self.image_cb)
        else:
            rospy.loginfo("Using Uncompressed Image Topic")
            self.sub_cam = rospy.Subscriber("/aero_downward_camera/image", Image, self.image_cb)
        self.pub_param = rospy.Publisher("/line/param", Line, queue_size=1)
        self.pub_img = rospy.Publisher("/line/img", Image, queue_size=1)
        self.switch_pub = rospy.Publisher("/switch", String, queue_size=1)
        self.ctr = 0

    def segmentLine(self, cv_image):
        """ Fit a line to biggest contour if it meets size requirements
        :param cv_image: opencv image
        """
        if DEBUG:
            colored = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        cv2.inRange(cv_image, led_low, led_hig, cv_image)
        #cv2.morphologyEx(cv_image, cv2.MORPH_OPEN, kernel, cv_image)
        cv2.morphologyEx(cv_image, cv2.MORPH_CLOSE, kernel, cv_image)
        _, contours, _ = cv2.findContours(cv_image, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # find the biggest area: https://stackoverflow.com/a/44591580
            maxc = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(maxc)

            rectw, recth = rect[1]
            if rectw < LENGTH_THRESH and recth < LENGTH_THRESH:
                return None
            if DEBUG:
                cv2.drawContours(colored, contours, -1, (0, 100, 100), -1)
                cv2.drawContours(colored, [maxc], -1, (0, 50, 100), -1)
                [vx,vy,x,y] = cv2.fitLine(maxc, distType=cv2.DIST_L2, param=0, reps=0.01, aeps=0.01)
                xy = np.array([x[0],y[0]])
                v = np.array([vx[0], vy[0]])
                bv = (v * 30) + xy
                if x and y and bv[0] > 0 and bv[1] > 0:
                    cv2.line(img=colored,pt1=(x, y),pt2=(bv[0], bv[1]),color=(255,0,255),thickness=1)
                    cv2.circle(img=colored, center=(x, y), radius=2, color=(0, 255, 0), thickness=-1)
                self.pub_img.publish(self.bridge.cv2_to_imgmsg(colored, "rgb8"))
            return x, y, vx, vy
        return None

    def image_cb(self, data):
        if USE_COMPRESSED:
            cv_image = cv2.imdecode(np.fromstring(data.data, np.uint8), 0)
        else:
            cv_image = self.bridge.imgmsg_to_cv2(data, "8UC1")
        line = self.segmentLine(cv_image)
        if line is not None:
            x, y, vx, vy = line
            msg = Line()
            msg.x = x
            msg.y = y
            msg.vx = vx
            msg.vy = vy
            self.switch_pub.publish("line")
            self.pub_param.publish(msg)
            self.ctr += 1

if __name__ == '__main__':
    rospy.init_node('line_detector', anonymous=True)
    rospy.loginfo("Using Default Threshs: {} - {}".format(led_low, led_hig))

    ld = LineDetector()
    rospy.loginfo("Line detector initialized")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    except Exception as e:
        print(e)
