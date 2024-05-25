#!/usr/bin/env python3

import math
import numpy as np
import operator
import time

import cv_bridge
import geometry_msgs
import mavros_msgs.srv
import rospy

import movement
import strategy

class Ctrler:
    '''FCU controller class'''

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
        self.bridge = cv_bridge.CvBridge()
        self.paused = True

        self.img_size = (1280, 720)
        self.img_center = tuple(x / 2 for  x in self.img_size)

        # self.movement_controller = movement.MovementController(
        #         AngleHighSpeedStrategy(self.img_size[0], self.img_size[1]))
        # self.movement_controller = movement.MovementController(
        #         AccStrategyVelCmd(self.img_size[0], self.img_size[1]))
        # self.movement_controller = movement.MovementController(TestStrategy())
        self.movement_controller = movement.MovementController(
                strategy.HighSpeedStrategy(self.img_size[0], self.img_size[1]))
        self.reset = False

        # Subscribers to drone pos info

        def cb_mav_pose(msg):
            self.mav_pos = np.array(operator.attrgetter('x', 'y', 'z')(msg.pose.position))
            q0, q1, q2, q3 = operator.attrgetter('w', 'x', 'y', 'z')(msg.pose.orientation)

            self.mav_yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
            self.mav_pitch = math.asin(2 * (q0 * q2 - q1 * q3))
            self.mav_roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
            R_ae = np.array([
                [q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
                [2 * (q1 * q2 + q0 * q3), q0 ** 2 - q1 ** 2 + q2 ** 2 - q3 ** 2, 2 * (q2 * q3 - q0 * q1)],
                [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2]])
            self.mav_R = R_ae.dot(self.R_ba)
            self.initialized = True

        def cb_mav_vel(msg):
            self.mav_vel = np.array(operator.attrgetter('x', 'y', 'z')(msg.twist.linear))
            self.initialized = True

        rospy.Subscriber(
                "mavros/local_position/pose",
                geometry_msgs.msg.PoseStamped,
                cb_mav_pose)
        rospy.Subscriber(
                "mavros/local_position/velocity_local",
                geometry_msgs.msg.TwistStamped,
                cb_mav_vel)

    def arm_and_takeoff(self, target_alt):
        while not self.initialized:
            print("Waiting for initialization")
            time.sleep(1)

        # TODO use better condition, subscribe to mavros state topic
        if self.mav_pos[2] > 0.5:
            return
        rospy.wait_for_service('/mavros/set_mode')
        try:
            change_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            response = change_mode(custom_mode="GUIDED")
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Set mode failed: %s" % e)

        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            response = arming_cl(value=True)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Arming failed: %s" % e)

        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
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

    def detector_cb(self, msg):

        if self.paused or not self.initialized:
            return

        pos_info = {"mav_pos": self.mav_pos, "mav_vel": self.mav_vel, "mav_R": self.mav_R,
                    "R_bc": np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]), "mav_yaw": self.mav_yaw,
                    "mav_pitch": self.mav_pitch}

        self.movement_controller.update(pos_info, msg.data, self.img_center)
        self.reset = False

