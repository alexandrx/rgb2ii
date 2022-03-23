import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

"""
   Implements the Illumination Invariant transform 
   based on Maddern et al. "Illumination Invariant Imaging: Applications in Robust Vision-based
Localisation, Mapping and Classification for Autonomous Vehicles"
   https://www.robots.ox.ac.uk/~mobile/Papers/2014ICRA_maddern.pdf 
"""
__author__ = "Alexander Carballo"
__email__ = "alexander@g.sp.m.is.nagoya-u.ac.jp"

class IIImage(object):
    def __init__(self):
        self.ros_init()

    def ros_init(self):
        rospy.init_node('rgb2ii', anonymous=True)
        self.img_topic = rospy.get_param("~image", "/image_raw")
        self.alpha = rospy.get_param("~alpha", 0.394)
        self.pub_ii = rospy.Publisher('/ii_image', Image, queue_size=10)
        rospy.Subscriber(self.img_topic, Image, self.img_callback)
        self.bridge = CvBridge()

    def rgb2ii(self, img, alpha):
        """Convert RGB image to illumination invariant image."""
        imgf = np.array(img, dtype=np.float32) / 255.0
        ii_image = 0.5 + np.log(img[:, :, 1]) - alpha * np.log(img[:, :, 0])  - (1 - alpha) * np.log(img[:, :, 2]) 
        return np.array(ii_image*255.0, dtype=np.uint8)

    def img_callback(self, image_msg):
        self.header = image_msg.header
        camera_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        self.publish(self.rgb2ii(camera_img, self.alpha))

    def publish(self, image):
        image_msg = self.bridge.cv2_to_imgmsg(image, "mono8")
        image_msg.header = self.header
        self.pub_ii.publish(image_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    image_convert = IIImage()
    image_convert.run()


