#!/usr/bin/python3

import logging
from datetime import datetime
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
# from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float32MultiArray


class Logger:

    def __init__(self):
        # log_session_folder = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_session_folder = datetime.now().strftime("%Y%m%d-%H%M%S")
        log_session_folder_path = Path.home().joinpath('Work').joinpath(
            'balloon_ws').joinpath('src').joinpath('stat').joinpath(log_session_folder)
        log_session_folder_path.mkdir()

        self.log_drone = self.setup_logger(
            'logger_drone', log_session_folder_path.joinpath('drone.txt'))
        self.log_ball = self.setup_logger(
            'logger_ball', log_session_folder_path.joinpath('ball.txt'))
        # for debug purpose
        self.log_debug = self.setup_logger(
            'log_debug', log_session_folder_path.joinpath('debug.txt'))

    def start(self):
        rospy.Subscriber('/mavros/local_position/pose',
                         PoseStamped, self.log_drone_position)
        rospy.Subscriber('/ball/state', Float32MultiArray,
                         self.log_ball_position)

    def setup_logger(self, name, log_file, level=logging.INFO, format='%(message)s', clear_if_exist=True):
        if clear_if_exist:
            with open(log_file, 'w'):
                pass

        handler = logging.FileHandler(log_file)
        formatter = logging.Formatter(format)
        handler.setFormatter(formatter)

        logger = logging.getLogger(name)
        logger.setLevel(level)
        logger.addHandler(handler)

        return logger

    def log_drone_position(self, msg):
        log_message = f"{str(msg.header.stamp.secs)}.{str(msg.header.stamp.nsecs)},{str(msg.pose.position.x)},{str(msg.pose.position.y)},{str(msg.pose.position.z)}"
        self.log_drone.info(log_message)

    def log_ball_position(self, msg):
        log_message = f"{str(msg.data[0])},{str(msg.data[1])},{str(msg.data[2])}"
        self.log_ball.info(log_message)


if __name__ == '__main__':
    rospy.init_node('logger_node', anonymous=True)
    logger = Logger()
    logger.start()

    while not rospy.is_shutdown():
        rospy.spin()
