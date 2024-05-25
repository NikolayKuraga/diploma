#!/usr/bin/python3

import logging
import os
from datetime import datetime
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from logger import *
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float32MultiArray


class Statistic:

    def __init__(self):
        stat_directory = Path.home().joinpath('Work').joinpath(
            'balloon_ws').joinpath('src').joinpath('stat')
        nested_dirs = [d for d in os.listdir(stat_directory)
                       if os.path.isdir(stat_directory.joinpath(d))]
        sorted_dirs = sorted(nested_dirs, key=lambda x: os.path.getctime(
            stat_directory.joinpath(x)), reverse=True)[:1]

        latest_stat_directory = None
        if len(sorted_dirs) > 0:
            latest_stat_directory = stat_directory.joinpath(sorted_dirs[0])

        print(latest_stat_directory)
        self.log_distance = Logger.setup_logger(
            self, name='distance', log_file=latest_stat_directory.joinpath('distance.txt'))
        self.log_speed_approach = Logger.setup_logger(
            self, name='speed_approach', log_file=latest_stat_directory.joinpath('speed_approach.txt'))

        self.min_distance = None
        self.prev_distance = None
        self.prev_time_secs = 0
        self.prev_time_nsecs = 0

    def start(self):
        self.drone_ball_pair_subs = [Subscriber("/mavros/local_position/pose", PoseStamped),
                                     Subscriber("/ball/state", Float32MultiArray)]
        self.tss = ApproximateTimeSynchronizer(
            self.drone_ball_pair_subs, queue_size=16, slop=0.2, allow_headerless=True)
        self.tss.registerCallback(self.got_drone_and_ball)

    def got_drone_and_ball(self, dronePose, ballCoords):
        drone_pos = np.array((dronePose.pose.position.x,
                             dronePose.pose.position.y, dronePose.pose.position.z))
        ball_pos = np.array((
            ballCoords.data[0], ballCoords.data[1], ballCoords.data[2]))
        distance = np.linalg.norm(drone_pos - ball_pos)

        if self.min_distance == None or distance < self.min_distance:
            self.min_distance = distance

        log_message = f"{str(dronePose.header.stamp.secs)}.{str(dronePose.header.stamp.nsecs)},{str(distance)},{str(self.min_distance)}"
        self.log_distance.info(log_message)

        speed = 0
        if self.prev_distance != None:
            speed = abs(distance - self.prev_distance) / abs((dronePose.header.stamp.secs +
                                                              dronePose.header.stamp.nsecs * 10**-len(str(dronePose.header.stamp.nsecs))) - (self.prev_time_secs + self.prev_time_nsecs * 10**-len(str(self.prev_time_nsecs))))
        log_message = f"{str(dronePose.header.stamp.secs)}.{str(dronePose.header.stamp.nsecs)},{str(speed)}"
        self.log_speed_approach.info(log_message)

        self.prev_distance = distance
        self.prev_time_secs = dronePose.header.stamp.secs
        self.prev_time_nsecs = dronePose.header.stamp.nsecs


if __name__ == '__main__':
    rospy.init_node('statistic_node', anonymous=True)
    statistic = Statistic()
    statistic.start()

    while not rospy.is_shutdown():
        rospy.spin()
