#!/usr/bin/python3

import time

import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
from std_msgs.msg import Float32MultiArray, UInt64

from detector import Detector
from movement import MovementController
from strategy import *


################################################################################################

# fov 1.57

# TODO create packages for detector, visualizer. Organize pkg structure, add options to launch file. Optimization?
class Controller:

    def __init__(self):

        self.mav_R = None
        self.mav_pos = None
        self.mav_roll = None
        self.mav_pitch = None
        self.mav_yaw = None
        self.mav_vel = None
        self.R_ba = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])  # body to enu
        self.initialized = False

        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()
        self.detector = Detector()
        self.paused = True

        self.img_size = (1280, 720)
        self.img_center = (640, 360)  # image center coordinates

        # self.movement_controller = MovementController(AngleHighSpeedStrategy(self.img_size[0], self.img_size[1]))
        self.movement_controller = MovementController(HighSpeedStrategy(self.img_size[0], self.img_size[1]))
        # self.movement_controller = MovementController(AccStrategyVelCmd(self.img_size[0], self.img_size[1]))
        # self.movement_controller = MovementController(TestStrategy())
        self.reset = False

        # Subscribers to drone pos info
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.mav_pose_cb)
        rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, self.mav_vel_cb)

        # Image subscriber to read data from camera
        # self.img_sub = rospy.Subscriber('/webcam/image_raw', Image, self.img_cb)

        # Subscribe to detector
        rospy.Subscriber('/filter/pos_image', Float32MultiArray, self.detector_cb)

    def arm_and_takeoff(self, target_alt):
        while not self.initialized:
            print("Waiting for initialization")
            time.sleep(1)
        # TODO use better condition, subscribe to mavros state topic
        if self.mav_pos[2] > 0.5:
            return
        rospy.wait_for_service('/mavros/set_mode')
        try:
            change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = change_mode(custom_mode="GUIDED")
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Set mode failed: %s" % e)

        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arming_cl(value=True)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Arming failed: %s" % e)

        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            response = takeoff_cl(altitude=target_alt, latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Takeoff failed: %s" % e)

        while True:
            print(" Altitude: ", self.mav_pos[2])
            # Break and return from function just below target altitude.
            if self.mav_pos[2] >= target_alt * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def start(self):
        if self.paused:
            self.paused = False

    def mav_pose_cb(self, msg):
        self.mav_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        self.mav_yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
        self.mav_pitch = math.asin(2 * (q0 * q2 - q1 * q3))
        self.mav_roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        R_ae = np.array([[q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
                         [2 * (q1 * q2 + q0 * q3), q0 ** 2 - q1 ** 2 + q2 ** 2 - q3 ** 2, 2 * (q2 * q3 - q0 * q1)],
                         [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2]])
        self.mav_R = R_ae.dot(self.R_ba)
        self.initialized = True

    def mav_vel_cb(self, msg):
        self.mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        self.initialized = True

    def detector_cb(self, msg):

        if self.paused or not self.initialized:
            return

        pos_info = {"mav_pos": self.mav_pos, "mav_vel": self.mav_vel, "mav_R": self.mav_R,
                    "R_bc": np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]), "mav_yaw": self.mav_yaw,
                    "mav_pitch": self.mav_pitch}

        self.movement_controller.update(pos_info, msg.data, self.img_center)
        self.reset = False


if __name__ == '__main__':
    print("Initializing system")
    rospy.init_node('controller_node', anonymous=True)
    controller = Controller()
    print("Arming")
    controller.arm_and_takeoff(20)
    print("Starting detection")
    controller.start()
    while not rospy.is_shutdown():
        rospy.spin()
