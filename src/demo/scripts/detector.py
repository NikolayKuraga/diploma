#!/usr/bin/python3

import cv2
import numpy as np

import rospy
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Detector:

    def __init__(self):
        self.bridge = CvBridge()
        self.redLower = np.array([0, 50, 20])
        self.redUpper = np.array([5, 255, 255])
        self.msg = Float32MultiArray()
        self.publisher = rospy.Publisher('/detector/pos_image', Float32MultiArray, queue_size=1)

    # detects red sphere and returns its center coordinates and bounding box width and height
    def detect(self, img):
        # Adding a mask to extract only the red ball
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.redLower, self.redUpper)

        # Detecting edges by adding Gaussian Blur and then using canny edge detection
        gray_img = cv2.GaussianBlur(mask, (5, 5), 0)
        edges = cv2.Canny(gray_img, 35, 125)
        M = cv2.moments(mask)
        x, y, w, h = cv2.boundingRect(mask)
        cx = cy = 0
        try:
            cx = int(M['m10'] // M['m00'])
            cy = int(M['m01'] // M['m00'])
        except ZeroDivisionError:
            pass
        return M, cx, cy, x, y, w, h

    def start(self):
        rospy.Subscriber('/webcam/image_raw', Image, self.img_cb)

    def img_cb(self, data):
        try:
            # Converting ROS image topic to CV format
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # M, cx, cy, w, h = self.detect(img)
            # self.msg.data = [cx, cy, w, h, 1.0]
            self.msg.data = self.detect(img)[1:]
            self.publisher.publish(self.msg)
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    rospy.init_node('detector_node', anonymous=True)
    detector = Detector()
    detector.start()
    while not rospy.is_shutdown():
        rospy.spin()
