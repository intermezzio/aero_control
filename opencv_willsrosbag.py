import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image


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
        # cv2 processing goes here

        if _DEBUG:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "8UC1"))

            except CvBridgeError as e:
                print(e)

if __name__ == "__main__":
    rospy.init_node("image_to_cv")
    a = ImageToCV()

    #rospy.on_shutdown(cv2.destroyAllWindows())

    rospy.spin()
