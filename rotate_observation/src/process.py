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
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        except CvBridgeError as e:
            print(e)

        cv2.imshow(cv_image) # shows the image, might be VERY slow (optional)
        self.process(cv_image)

    def process(self, img):
        # cv2 processing goes here
        HSV = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
        mins = np.array([0,0,200])
        maxs = np.array([255,30,255])
        validHSV = cv2.inRange(HSV,mins,maxs)
        m, b = calculate_regression(validHSV)
        coords = find_inliers(m, b, img.shape)
        lineImg = cv2.line(img, coords[::2], coords[1::2], (0,255,0), thickness=5)
        if _DEBUG:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

            except CvBridgeError as e:
                print(e)

def calculate_regression(points): # input is the result of np.argwhere
    # convert points to float
    points = points.astype(float)
    #TODO (see astype, https://docs.scipy.org/doc/numpy/reference/generated/numpy.ndarray.astype.html)

    xs = points[:,1]
    ys = points[:,0]
    x_mean = np.mean(xs)
    y_mean = np.mean(ys)

    xy_mean = np.mean(xs * ys)

    x_squared_mean = np.mean(np.square(xs))

    m = (x_mean * y_mean - xy_mean)/(x_mean ** 2 - x_squared_mean)

    b = y_mean - (m * x_mean)

    return (m,b)

def find_inliers(m, b, shape):
    height = shape[0]
    width = shape[1]
    coords = []
    for i in range(height + 1):
        if int(b) == i:
            coords += [0,int(b)]
        if int(m * width + b) == i:
            coords += [width,i]
    for i in range(width + 1):
        if int(m * i + b) == 0:
            coords += [i, 0]
        if int(m * i + b) == height:
            coords += [i, height]
    return coords

if __name__ == "__main__":
    rospy.init_node("image_to_cv")
    a = ImageToCV()

    rospy.on_shutdown(cv2.destroyAllWindows())

    rospy.spin()
