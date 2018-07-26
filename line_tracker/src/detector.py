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
        self.img_view = rospy.Publisher("/image_to_cv/processed", Image, queue_size=1)
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
        # d = cv2.resize(d, (0,0), fx=10, fy=10)
# 
        ret,thresh = cv2.threshold(d,0,255,0)
        im2,contours,hierarchy = cv2.findContours(thresh, 1, 2)#adds the pixels in the threshold to the list of possible contours
        
        if len(contours) > 0:
            max_contours = max(contours, key = lambda x: cv2.contourArea(x))

            rows,cols = d.shape[:2]
            img_center_x, img_center_y = rows/2, cols/2
            p_frame_center = (img_center_x,img_center_y)

            [vx,vy,x,y] = cv2.fitLine(max_contours, cv2.DIST_L2,0,0.01,0.01)
        
            lefty = int((-x[0]*vy[0]/vx[0]) + y[0])
            righty = int(((cols-x)[0]*vy[0]/vx)[0]+y[0])
            print lefty, righty,vx[0],vy[0],x[0],y[0]
            pt1 = (cols-1,righty)
            pt2 = (0,lefty)
            try:
                regression = cv2.line(d,pt1,pt2,(0,255,0),20)
            except OverflowError:
                print 'Overflow!!!!'    
                regression = cv2.line(d,(img_center_x,0),(img_center_x,rows),(255,255,255),5)
            x_axis = cv2.line(d,(0,img_center_y),(cols,img_center_y),(255,255,255),5)
            y_axis = cv2.line(d,(img_center_x,0),(img_center_x,rows),(255,255,255),5)


        ##||||||||||||||||||||||||||||||||||||||||||||||||||##
        ##VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV## I think this will work for tracker.py
            px1 = cols-1
            px2 = 0
            py1 = righty
            py2 = lefty

            p_line_center_x = (px1+px2)/2
            p_line_center_y = (py1+py2)/2

            r_line_unit = (vx[0],vy[0])

            m = vy/vx
            b = p_line_center_y - m*p_line_center_x

            distances = [20000]
            xs = []
            ys = []
            for x1 in range(0,d.shape[0]):
                y1 = m*x1 + b
                dist = np.sqrt((x1 - img_center_x)**2 + (y1 - img_center_y)**2)
                if dist < distances[-1]:
                    distances.append(dist)
                    xs.append(x1)
                    ys.append(y1)

            if len(xs) > 0 and len(ys) > 0:

                p_line_closest_center = (xs[-1],ys[-1])
                p_line_closest_center_x = xs[-1]
                p_line_closest_center_y = ys[-1]

                p_target = (vx[0]+p_line_closest_center_x,vy[0]+p_line_closest_center_y)
                p_target_x = vx[0]+p_line_closest_center_x
                p_target_y = vy[0]+p_line_closest_center_y

                # r_to_target_x,r_to_target_y = (img_center_x + p_target_x, img_center_y + p_target_y) #<----------------------------use these for velocities

                num_unit_vecs = 30

                x_err = num_unit_vecs*(img_center_x - p_target_x)   
                y_err = num_unit_vecs*(img_center_y - p_target_y) 

                new_line = cv2.line(d,(img_center_x,img_center_y),(p_line_closest_center_x,p_line_closest_center_y),(255,255,255),10)


        # print(p_line_closest_center,p_target,x_error,y_error)

        



        # TODO-END
        
            if DEBUG:
                # TODO-OPTIONAL-START: publish an image showing your line on top of the LED strip image
                 # delete ths line and add your code if you want debug images

                try:
                    self.img_view.publish(self.bridge.cv2_to_imgmsg(d, "8UC1"))
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
