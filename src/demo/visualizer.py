#!/usr/bin/python3

import time

import cv2
import numpy as np
import math

import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from utils import *
from vmo import VMO


class Visualizer:

    def __init__(self):
        self.fl = [1117.7832090530542, 1117.7832090530542]
        self.pp = [640.5098521801531, 360.5]
        self.K = np.array([
            [self.fl[0], 0., self.pp[0]],
            [0., self.fl[1], self.pp[1]],
            [0., 0., 1.]
        ])
        self.vmo = VMO(self.fl, self.pp)

        self.bridge = CvBridge()

        self.mav_vel = [0.0] * 3
        self.mav_ang_vel = [0.0] * 3

        self.pos_image = [0.0] * 6
        self.pos_filter = [0.0] * 2
        self.tracker_bb = [0.0] * 4

        self.ball_state = np.zeros(3)

        self.R_ae = None
        self.R_ba = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        self.mav_R = None
        self.euler = [0.0] * 3
        self.mav_pos = None
        self.mav_roll = None
        self.mav_pitch = None
        self.mav_yaw = None

    def start(self):
        rospy.Subscriber('/detector/pos_image', Float32MultiArray, self.detector_cb)
        rospy.Subscriber('/filter/pos_image', Float32MultiArray, self.filter_cb)
        rospy.Subscriber('/tracker/pos_image', Float32MultiArray, self.tracker_cb)
        rospy.Subscriber('/ball/state', Float32MultiArray, self.ball_state_cb)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.mav_vel_cb)
        rospy.Subscriber('/webcam/image_raw', Image, self.image_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mav_pose_cb)

    def mav_pose_cb(self, msg):
        self.mav_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        # self.mav_yaw = self.euler[0] = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
        # self.mav_pitch = self.euler[1] = math.asin(2 * (q0 * q2 - q1 * q3))
        # self.mav_roll = self.euler[2] = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        self.euler = quaternion_to_euler_angle(q0, q1, q2, q3) # rpy
        self.R_ae = np.array([[q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
                         [2 * (q1 * q2 + q0 * q3), q0 ** 2 - q1 ** 2 + q2 ** 2 - q3 ** 2, 2 * (q2 * q3 - q0 * q1)],
                         [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2]])
        self.mav_pos = self.R_ba.dot(np.array(self.mav_pos))
        self.mav_R = self.R_ae.dot(self.R_ba)

    def mav_vel_cb(self, msg):
        self.mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

    def detector_cb(self, msg):
        self.pos_image = msg.data

    def filter_cb(self, msg):
        self.pos_filter = msg.data

    def tracker_cb(self, msg):
        self.tracker_bb = msg.data

    def ball_state_cb(self, msg):
        self.ball_state = np.array(msg.data)

    def image_cb(self, msg):
        try:
            # Converting ROS image topic to CV format
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            mag = np.linalg.norm(self.mav_vel)
            cx, cy, x, y, w, h = self.pos_image

            cv2.circle(img, (int(self.pos_image[0]), int(self.pos_image[1])), 6, (0, 100, 100), -1)
            cv2.circle(img, (int(self.pos_filter[0]), int(self.pos_filter[1])), 3, (255, 0, 0), -1)

            cv2.circle(img, (int(self.pos_image[0]),
                             int(self.pos_image[1] + h/2)), 3, (0, 255, 0), -1)
            cv2.circle(img, (int(self.pos_image[0] + w/2),
                             int(self.pos_image[1])), 3, (0, 255, 0), -1)
            cv2.circle(img, (int(self.pos_image[0]),
                             int(self.pos_image[1] - h/2)),3, (0, 255, 0), -1)
            cv2.circle(img, (int(self.pos_image[0] - w/2),
                             int(self.pos_image[1])),3, (0, 255, 0), -1)

            cv2.line(img, (640, 0), (640, 720), (255, 0, 255))
            cv2.line(img, (0, 360), (1280, 360), (255, 0, 255))

            cv2.putText(img, f"{mag}", (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))
            cv2.imshow('window', img)
            cv2.waitKey(2)

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    rospy.init_node('visualizer_node', anonymous=True)
    visualizer = Visualizer()
    visualizer.start()
    while not rospy.is_shutdown():
        rospy.spin()
