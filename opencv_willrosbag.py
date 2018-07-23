import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import numpy as np
from scipy.stats import linregress
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt



_DEBUG = True
class ImageToCV:
    def __init__(self):
        rospy.Subscriber("/aero_downward_camera/image", Image, self.image_cb)
        self.image_pub = rospy.Publisher("/image_to_cv/processed", Image, queue_size=1)
        self.bridge = CvBridge()

    def image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "8UC1")

        except CvBridgeError as e:
            print(e)

        # cv2.imshow(cv_image) # shows the image, might be VERY slow (optional)
        self.process(cv_image)

        
    def process(self, img):
        # cv2 processing goes here <--------------------


        white_thresh = 255
        black_thresh = 255

        k_dim = 10
        kernel = np.ones((k_dim,k_dim),np.uint8)

        d = np.copy(img) #preserves raw images, but requires more time and processing power    
        d = cv2.inRange(d,black_thresh,white_thresh)
        d = cv2.dilate(d,kernel)
        d = cv2.resize(d, (0,0), fx=10, fy=10)

        # ret,thresh = cv2.threshold(d,0,255,0)
        # im2,contours,hierarchy = cv2.findContours(thresh, 1, 2)#adds the pixels in the threshold to the list of possible contours
        # cntrs = contours[0] #takes a random pixel, in this case the first
        
        # rows,cols = d.shape[:2]
        # [vx,vy,x,y] = cv2.fitLine(cntrs, cv2.DIST_L2,0,0.01,0.01)
        # lefty = int((-x*vy/vx) + y)
        # righty = int(((cols-x)*vy/vx)+y)
        # regression = cv2.line(d,(cols-1,righty),(0,lefty),(255,255,255),2)



        ###matplotlib regression###

        def find_inliers(m,b,shape):
            for x1 in range(shape[0]):
                y1 = m * x1 + b
                if y1 < 0 or y1 > shape[1]:
                    continue ##jump back to the beginning of the for loop
                break
                
            for x2 in range(shape[0])[::-1]:
                y2 = m * x2 + b
                if y2 < 0 or y1 > shape[1]:
                    continue
                break
            return(x1,y1,x2,y2)

        aw = np.argwhere(d)
        m,b,_,_,_ = linregress(aw[:,1], aw[:,0])
        
        x1,y1,x2,y2 = find_inliers(m,b,d.shape)
        
        # regression = Line2D([x1,x2],[y1,y2], color="red",linewidth=10)
        regression  = cv2.line(d, (int(x1),int(y1)),(int(x2),int(y2)), (255,255,255),2)    




        # cv2.imshow('image', regression)

        if _DEBUG:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(d, "8UC1"))
                # self.image_pub.publish(self.bridge.cv2_to_imgmsg(regression, "8UC1"))


            except CvBridgeError as e:
                print(e)


if __name__ == "__main__":
    rospy.init_node("image_to_cv")
    a = ImageToCV()
    print("this is running")

    # rospy.on_shutdown(cv2.destroyAllWindows())

    rospy.spin()


