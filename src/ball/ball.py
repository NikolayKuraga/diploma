#!/usr/bin/python3

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Float32MultiArray
from trajectory import *


class BallController:

    def __init__(self, traj):
        self.rate = rospy.Rate(50)
        self.pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.publisher = rospy.Publisher('/ball/state', Float32MultiArray, queue_size=1)
        self.traj = traj

        self.msg = Float32MultiArray()
        self.state_msg = ModelState()
        self.state_msg.model_name = 'unit_sphere'

    def start(self):
        while not rospy.is_shutdown():
            point = self.traj.next()
            self.publish(point)
            self.rate.sleep()

    def publish(self, data):
        self.msg.data = list(data)
        self.publisher.publish(self.msg)
        self.state_msg.pose.position.x = data[0]
        self.state_msg.pose.position.y = data[1]
        self.state_msg.pose.position.z = data[2]
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(self.state_msg)
        except rospy.ServiceException:
            print("Service call failed: ")


if __name__ == '__main__':
    rospy.init_node('model_service_node', anonymous=True)
    x = y = 0
    z = 10
    trajectory = CircleTrajectory(x, y, z, 10, lambda _: 10)
    # trajectory = LemniscateBernoulliTrajectory(x, y, z, 10, lambda _: 10)
    # trajectory = LineTrajectory(x, y, z, [50, 30, 10], lambda _: 10) # with constant speed = 10
    # trajectory = LineTrajectory(x, y, z,[50,30,10],lambda t: t) # with acceleration
    # trajectory = PointTrajectory(x, y, z)

    # trajectory = CircleTrajectory3D(x, y, z, 50, lambda _: 10)
    # trajectory = LemniscateBernoulliTrajectory3D(x, y, z, 10, lambda _: 10)
    # trajectory = LineTrajectory3D(x, y, z,[50,30,25],lambda _: 10)


    ball_controller = BallController(trajectory)
    ball_controller.start()

