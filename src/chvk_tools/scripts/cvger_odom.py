#!/usr/bin/env python3


import operator as op
import typing

import rospy

import geometry_msgs.msg
import mrs_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

import topic_cvger


def main():

    rospy.init_node('cvger_odom', disable_signals=True)

    # Just notes.
    # topicsNames = ('/chvk/optic_flow/velocity', '/mavros/imu/data')
    # topicsTypes = (
    #     'geometry_msgs.msg.TwistWithCovarianceStamped', 'sensor_msgs.msg.Imu')
    # topicsNamesAndTypes = list(zip(topicsNames, topicsTypes))

    def convertData(data: typing.Tuple[
            geometry_msgs.msg.TwistWithCovarianceStamped, sensor_msgs.msg.Imu]
        ) -> nav_msgs.msg.Odometry:
        '''Convert from le first function argument's type to le output
        function argument's type (see le annotation) and then return
        it.'''

        twistStamped, Imu = data

        odom = nav_msgs.msg.Odometry()

        ### odom.header
        odom.header.seq = 0
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom_ned'

        ### odom.child_frame_id
        odom.child_frame_id = 'base_link_frd'

        ### odom.pose
        odom.pose = geometry_msgs.msg.PoseWithCovariance()
        odom.pose.pose.orientation = Imu.orientation
        odom.pose.pose.orientation.z *= -1.

        ### odom.twist
        odom.twist = twistStamped.twist

        x, y, z = op.attrgetter('x', 'y', 'z')(odom.twist.twist.linear)
        odom.twist.twist.linear.x = y
        odom.twist.twist.linear.y = -x
        odom.twist.twist.linear.z = z

        x, y, z = op.attrgetter('x', 'y', 'z')(odom.twist.twist.angular)
        odom.twist.twist.angular.x = y
        odom.twist.twist.angular.y = -x
        odom.twist.twist.angular.z = z

        return odom

    topicCvger = topic_cvger.TopicCvger(
        from_topics_and_types=[
            ('/chvk/optic_flow/velocity',
             geometry_msgs.msg.TwistWithCovarianceStamped),
            ('/imu/data',
             sensor_msgs.msg.Imu)],
        to_topic_and_type=('/mavros/odometry/out', nav_msgs.msg.Odometry),
        converter=convertData,
        period=rospy.Duration(0.016))

    topicCvger.start()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()

