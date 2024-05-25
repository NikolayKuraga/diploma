#!/usr/bin/env python3


import math
import operator

import geometry_msgs
import rospy

import ctrler


points = [(200, 200), (500, 200), (500, 500), (200, 500)]

def cb(msg):
    '''Callback that sets waypoints for FCU to fly to.'''

    def error(x_expected, y_expected, x_real, y_real):
        return math.sqrt(
            ((x_real - x_expected) ** 2) + ((y_real - y_expected) ** 2))

    print('distance to next waypoint: %.2f' % error(
       posemsg.pose.position.x,
       posemsg.pose.position.y,
       msg.pose.position.x,
       msg.pose.position.y))

    for i in range(len(points)):
        if 2 > error(*points[i - 1], msg.pose.position.x, msg.pose.position.y):
            posemsg.pose.position.x, posemsg.pose.position.y = points[i]

    pub_obj.publish(posemsg)


if __name__ == '__main__':

    rospy.init_node('square')

    height = 100

    ctrler = ctrler.Ctrler()
    print('going to arm and takeoff')
    ctrler.arm_and_takeoff(height)
    print('armed and takeoff???')

    # used in callback
    posemsg = geometry_msgs.msg.PoseStamped()
    posemsg.pose.position.x = points[0][0]
    posemsg.pose.position.y = points[0][1]
    posemsg.pose.position.z = height

    # subscriber to local_position/setpoint, used in callback
    sub_obj = rospy.Subscriber(
            '/mavros/local_position/pose',
            geometry_msgs.msg.PoseStamped,
            cb)

    # publisher object, used in callback
    pub_obj = rospy.Publisher(
            '/mavros/setpoint_position/local',
            geometry_msgs.msg.PoseStamped,
            queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()

    exit()
